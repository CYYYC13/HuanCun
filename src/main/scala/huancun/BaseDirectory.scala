/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  *          http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

// See LICENSE.SiFive for license details.

package huancun

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import chisel3.util.random.LFSR
import freechips.rocketchip.tilelink.TLMessages
import freechips.rocketchip.util.{Pow2ClockDivider, ReplacementPolicy}
import huancun.utils._
import utility.{Code}

trait BaseDirResult extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W) // which mshr the result should be sent to
}
trait BaseDirWrite extends HuanCunBundle
trait BaseTagWrite extends HuanCunBundle

class DirRead(implicit p: Parameters) extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W)
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val replacerInfo = new ReplacerInfo()
  val source = UInt(sourceIdBits.W)
  val wayMode = Bool()
  val way = UInt(log2Ceil(maxWays).W)
}

abstract class BaseDirectoryIO[T_RESULT <: BaseDirResult, T_DIR_W <: BaseDirWrite, T_TAG_W <: BaseTagWrite](
  implicit p: Parameters)
    extends HuanCunBundle {
  val read:    DecoupledIO[DirRead]
  val result:  Valid[T_RESULT]
  val dirWReq: DecoupledIO[T_DIR_W]
  val tagWReq:  DecoupledIO[T_TAG_W]
}

abstract class BaseDirectory[T_RESULT <: BaseDirResult, T_DIR_W <: BaseDirWrite, T_TAG_W <: BaseTagWrite](
  implicit p: Parameters)
    extends HuanCunModule {
  val io: BaseDirectoryIO[T_RESULT, T_DIR_W, T_TAG_W]
}

class SubDirectory[T <: Data](
  wports:      Int,
  sets:        Int,
  ways:        Int,
  tagBits:     Int,
  dir_init_fn: () => T,
  dir_hit_fn: T => Bool,
  invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
  replacement: String)(implicit p: Parameters)
    extends Module {

  val setBits = log2Ceil(sets)
  val wayBits = log2Ceil(ways)
  val dir_init = dir_init_fn()

  val io = IO(new Bundle() {
    val read = Flipped(DecoupledIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val replacerInfo = new ReplacerInfo()
      val wayMode = Bool()
      val way = UInt(wayBits.W)
    }))
    val resp = ValidIO(new Bundle() {
      val hit = Bool()
      val way = UInt(wayBits.W)
      val tag = UInt(tagBits.W)
      val dir = dir_init.cloneType
      val error = Bool()
      val channel = UInt(3.W)
      val opcode = UInt(3.W)
      val set = UInt(setBits.W)
      val bypass = Bool()
      val TC = UInt(2.W)
      val UC = UInt(2.W)
      val repl_state = UInt(32.W)
      val next_state = UInt(32.W)
      val isSample = Bool()
      val new_TCUC_s2 = UInt(4.W)
      val Dvec = Vec(16, UInt(20.W))
      val Lvec = Vec(16, UInt(20.W))
      val DLvec = Vec(16, UInt(20.W))
      val DLcond1 = Bool()
      val DLcond2 = Bool()
      val DLcond3 = Bool()
    })
    val tag_w = Flipped(DecoupledIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val way = UInt(wayBits.W)
    }))
    val dir_w = Flipped(DecoupledIO(new Bundle() {
      val set = UInt(setBits.W)
      val way = UInt(wayBits.W)
      val dir = dir_init.cloneType
    }))
  })

  val clk_div_by_2 = p(HCCacheParamsKey).sramClkDivBy2
  val resetFinish = RegInit(false.B)
  val resetIdx = RegInit((sets - 1).U)
  val metaArray = Module(new SRAMTemplate(chiselTypeOf(dir_init), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
//  val binArray = if (!tubins_repl) None
//                 else Some(Module(new SRAMTemplate(UInt(40.W), 1, 16, singlePort = true, shouldReset = true)))  //29-15:D;14-0:L
//  val TCUCArray = if (!tubins_repl) None
//                  else Some(Module(new SRAMTemplate(UInt(4.W), sets, ways, singlePort = true, shouldReset = true)))

  val clkGate = Module(new STD_CLKGT_func)
  val clk_en = RegInit(false.B)
  clk_en := ~clk_en
  clkGate.io.TE := false.B
  clkGate.io.E := clk_en
  clkGate.io.CK := clock
  val masked_clock = clkGate.io.Q

  val tag_wen = io.tag_w.valid
  val dir_wen = io.dir_w.valid
  val replacer_wen_old = RegInit(false.B)
  val replacer_wen = WireInit(false.B)
  val binWen = WireInit(false.B)
  val tcucWen = WireInit(false.B)
  io.tag_w.ready := true.B
  io.dir_w.ready := true.B
  io.read.ready := !tag_wen && !dir_wen && !replacer_wen && resetFinish

  def tagCode: Code = Code.fromString(p(HCCacheParamsKey).tagECC)

  val eccTagBits = tagCode.width(tagBits)
  val eccBits = eccTagBits - tagBits
  println(s"Tag ECC bits:$eccBits")
  val tagRead = Wire(Vec(ways, UInt(tagBits.W)))
  val eccRead = Wire(Vec(ways, UInt(eccBits.W)))
  val tagArray = Module(new SRAMTemplate(UInt(tagBits.W), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
  if(eccBits > 0){
    val eccArray = Module(new SRAMTemplate(UInt(eccBits.W), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
    eccArray.io.w(
      io.tag_w.fire,
      tagCode.encode(io.tag_w.bits.tag).head(eccBits),
      io.tag_w.bits.set,
      UIntToOH(io.tag_w.bits.way)
    )
    if (clk_div_by_2) {
      eccArray.clock := masked_clock
    }
    eccRead := eccArray.io.r(io.read.fire, io.read.bits.set).resp.data
  } else {
    eccRead.foreach(_ := 0.U)
  }

  tagArray.io.w(
    io.tag_w.fire,
    io.tag_w.bits.tag,
    io.tag_w.bits.set,
    UIntToOH(io.tag_w.bits.way)
  )
  tagRead := tagArray.io.r(io.read.fire, io.read.bits.set).resp.data

  if (clk_div_by_2) {
    metaArray.clock := masked_clock
    tagArray.clock := masked_clock
  }

  val reqValidReg = RegInit(false.B)
  val reqReg = RegEnable(io.read.bits, io.read.fire)
  val req_s1 = RegNext(reqReg)
  val req_s2 = RegEnable(req_s1, reqValidReg)
  val reqSource = req_s2.replacerInfo.reqSource
//  val req_prefetch = (reqSource === 5.U || reqSource === 6.U || reqSource === 8.U || reqSource === 9.U || reqSource === 10.U || reqSource === 11.U || reqSource === 12.U || reqSource === 13.U || reqSource === 14.U)
  val req_prefetch = req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.Hint)
  if (clk_div_by_2) {
    reqValidReg := RegNext(io.read.fire)
  } else {
    reqValidReg := io.read.fire
  }

  val hit_s1 = Wire(Bool())
  val way_s1 = Wire(UInt(wayBits.W))

  val repl = ReplacementPolicy.fromString(replacement, ways)
  val replaceWay = WireInit(UInt(wayBits.W), 0.U)
  val random_repl = replacement == "random"
  val tubins_repl = replacement == "tubins"
  val replacer_sram = if (random_repl) None else
    Some(Module(new SRAMTemplate(UInt(repl.nBits.W), sets, 1, singlePort = true, shouldReset = true)))

  io.resp.valid := reqValidReg
  val metas = metaArray.io.r(io.read.fire, io.read.bits.set).resp.data
  val tagMatchVec = tagRead.map(_(tagBits - 1, 0) === reqReg.tag)
  val metaValidVec = metas.map(dir_hit_fn)
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val hitWay = OHToUInt(hitVec)
  val (inv, invalidWay) = invalid_way_sel(metas, replaceWay)
  val chosenWay = Mux(inv, invalidWay, replaceWay)

  /* stage 0: io.read.fire
     stage #: wait for sram
     stage 1: generate hit/way, io.resp.valid = TRUE (will latch into MSHR)
     stage 2: output latched hit/way, output dir/tag
  */
  hit_s1 := Cat(hitVec).orR
  way_s1 := Mux(reqReg.wayMode, reqReg.way, Mux(hit_s1, hitWay, chosenWay))

  val hit_s2 = RegEnable(hit_s1, false.B, reqValidReg)
  val way_s2 = RegEnable(way_s1, 0.U, reqValidReg)
  val metaAll_s2 = RegEnable(metas, reqValidReg)
  val tagAll_s2 = RegEnable(tagRead, reqValidReg)
  val meta_s2 = metaAll_s2(way_s2)
  val tag_s2 = tagAll_s2(way_s2)

  val errorAll_s1 = VecInit(eccRead.zip(tagRead).map{x => tagCode.decode(x._1 ## x._2).error})
  val errorAll_s2 = RegEnable(errorAll_s1, reqValidReg)
  val error_s2 = errorAll_s2(way_s2)

  io.resp.bits.hit := hit_s2
  io.resp.bits.way := way_s2
  io.resp.bits.dir := meta_s2
  io.resp.bits.tag := tag_s2
  io.resp.bits.error := io.resp.bits.hit && error_s2


  metaArray.io.w(
    !resetFinish || dir_wen,
    Mux(resetFinish, io.dir_w.bits.dir, dir_init),
    Mux(resetFinish, io.dir_w.bits.set, resetIdx),
    Mux(resetFinish, UIntToOH(io.dir_w.bits.way), Fill(ways, true.B))
  )

  /* ======!! Replacement logic !!====== */
  /* ====== Read, choose replaceWay ====== */
  val repl_state = if (replacement == "random") {
    when(io.tag_w.fire) {
      repl.miss
    }
    0.U
  } else if (replacement == "tubins") {
    val repl_sram_r = replacer_sram.get.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))
    repl_state_hold
  } else {  // plru
    val repl_sram_r = replacer_sram.get.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))

    repl_state_hold
  }

  replaceWay := repl.get_replace_way(repl_state)

  /* ====== Update ====== */
  // PLRU: update replacer only when releaseData or Hint, at stage 2
  // TUBINS:  update replacer when when A hit or C req, at stage 2
  val updateAHit = RegNext(reqValidReg) && req_s2.replacerInfo.channel(0) && hit_s2 &&(req_s2.replacerInfo.opcode === TLMessages.AcquireBlock || req_s2.replacerInfo.opcode === TLMessages.AcquirePerm || req_s2.replacerInfo.opcode === TLMessages.Hint)
  val updateC = RegNext(reqValidReg) && req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.ReleaseData || req_s2.replacerInfo.opcode === TLMessages.Release)
  val updateHintMiss = RegNext(reqValidReg) && req_s2.replacerInfo.channel(0) && !hit_s2 &&(req_s2.replacerInfo.opcode === TLMessages.Hint) && req_s2.replacerInfo.preferCache

//  replacer_wen := RegNext(updateAHit) || RegNext(updateC)
  replacer_wen := updateAHit || updateC || updateHintMiss

  if(replacement == "tubins") {

    val binArray =  Module(new SRAMTemplate(UInt(40.W), 1, 16, singlePort = true, shouldReset = true))  //39-20:D;19-0:L
    val TCUCArray = Module(new SRAMTemplate(UInt(4.W), sets, ways, singlePort = true, shouldReset = true))
    val binRead = Wire(Vec(16, UInt(40.W)))
    val TCUCRead = Wire(Vec(ways, UInt(4.W)))

    // TCUC R/W for TUBINS
    TCUCRead := TCUCArray.io.r(io.read.fire, io.read.bits.set).resp.data
//    val TCUCAll_s2 = RegEnable(TCUCRead, 0.U.asTypeOf(TCUCRead), reqValidReg)
    val TCUCAll_s2 = WireInit(0.U.asTypeOf(TCUCRead))
    TCUCAll_s2 := HoldUnless(TCUCRead, RegNext(io.read.fire, false.B))
    val TCUC_s2 = TCUCAll_s2(way_s2)
    val TC_s2 = WireInit(0.U(2.W))
    TC_s2 := TCUC_s2(3, 2)
    val UC_s2 = WireInit(0.U(2.W))
    UC_s2 := TCUC_s2(1, 0)
    tcucWen := updateAHit || updateC || updateHintMiss
    val new_TC = WireInit(0.U(2.W))
    new_TC := Mux(hit_s2 && req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.AcquirePerm || req_s2.replacerInfo.opcode === TLMessages.AcquireBlock), Mux(TC_s2 === 3.U, 3.U, TC_s2 + 1.U),
      Mux(req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.Release || req_s2.replacerInfo.opcode === TLMessages.ReleaseData), req_s2.replacerInfo.TC,
        Mux(!hit_s2 && req_s2.replacerInfo.channel(0) && req_s2.replacerInfo.opcode === TLMessages.Hint && req_s2.replacerInfo.preferCache, 0.U, TC_s2)))
    val new_UC = WireInit(0.U(2.W))
    new_UC := Mux(req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.Release || req_s2.replacerInfo.opcode === TLMessages.ReleaseData), req_s2.replacerInfo.UC,
      Mux(!hit_s2 && req_s2.replacerInfo.channel(0) && req_s2.replacerInfo.opcode === TLMessages.Hint && req_s2.replacerInfo.preferCache, 0.U, UC_s2))
    val new_TCUC = Cat(new_TC, new_UC)
    val TCUC_init = Wire(Vec(ways, UInt(4.W)))
    TCUC_init.foreach(_ := 0.U(4.W))
    TCUCArray.io.w(
      !resetFinish || tcucWen,
      Mux(resetFinish, new_TCUC, TCUC_init.asUInt),
      Mux(resetFinish, req_s2.set, resetIdx),
      Mux(resetFinish, UIntToOH(way_s2), Fill(ways, true.B))
    )

    // Bin R/W for TUBINS
    val update_TCUC_s2 = WireInit(0.U(4.W))
    val new_TCUC_s2 = WireInit(0.U(4.W))
    update_TCUC_s2 := Mux(req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.Release || req_s2.replacerInfo.opcode === TLMessages.ReleaseData), 4.U * new_TC + new_UC, 4.U * TC_s2 + UC_s2)
    new_TCUC_s2 := 4.U * new_TC + new_UC
//    val isSampleSets = Mux((req_s2.set(9, 5) + req_s2.set(4, 0)) === 31.U, true.B, false.B) // choose 64 sample from 4096 sets
//    val isSampleSets = Mux((req_s2.set(11, 6) + req_s2.set(5, 0)) === 63.U, true.B, false.B) // choose 64 sample from 4096 sets
  val isSampleSets = Mux((req_s2.set(11, 6) + req_s2.set(5, 0)) > 0.U, true.B, false.B)
  binWen := (updateAHit || updateC || updateHintMiss) && isSampleSets
    binRead := binArray.io.r(io.read.fire, 0.U).resp.data
//    val binDL_all_s2 = RegEnable(binRead, 0.U.asTypeOf(binRead), reqValidReg)
    val binDL_all_s2 = WireInit(0.U.asTypeOf(binRead))
    binDL_all_s2 := HoldUnless(binRead, RegNext(io.read.fire, false.B))
    val Lvec = Wire(Vec(16, UInt(20.W)))
    val Dvec = Wire(Vec(16, UInt(20.W)))
    val DLvec = Wire(Vec(16, UInt(20.W)))
    Lvec.zipWithIndex.foreach {
      case (m, i) =>
        m := binDL_all_s2(i)(19, 0)
    }
    Dvec.zipWithIndex.foreach {
      case (m, i) =>
        m := binDL_all_s2(i)(39, 20)
    }
    DLvec.zipWithIndex.foreach {
      case (m, i) =>
        m := Mux(Dvec(i) >= Lvec(i), Dvec(i) - Lvec(i), 0.U)
    }
    val Lvec_low = Wire(Vec(8, UInt(20.W)))
    val Dvec_low = Wire(Vec(8, UInt(20.W)))
    val DLvec_low = Wire(Vec(8, UInt(20.W)))
    Lvec_low.zipWithIndex.foreach {
      case (m, i) =>
        m := binDL_all_s2(i)(19, 0)
    }
    Dvec_low.zipWithIndex.foreach {
      case (m, i) =>
        m := binDL_all_s2(i)(39, 20)
    }
    DLvec_low.zipWithIndex.foreach {
      case (m, i) =>
        m := Mux(Dvec(i) >= Lvec(i), Dvec(i) - Lvec(i), 0.U)
    }

    val binDL = binDL_all_s2(update_TCUC_s2)
    val binD = binDL(39, 20)
    val binL = binDL(19, 0)
    val sumL  =  Lvec(1)  +  Lvec(2)  +  Lvec(3)  +  Lvec(5)  +  Lvec(6)  +  Lvec(7)  +  Lvec(9)  +  Lvec(10)  +  Lvec(11)  +  Lvec(13)  +  Lvec(14)  +  Lvec(15)
    val sumD  =  Dvec(1)  +  Dvec(2)  +  Dvec(3)  +  Dvec(5)  +  Dvec(6)  +  Dvec(7)  +  Dvec(9)  +  Dvec(10)  +  Dvec(11)  +  Dvec(13)  +  Dvec(14)  +  Dvec(15)
    val sumDL =  DLvec(1) +  DLvec(2) +  DLvec(3) +  DLvec(5) +  DLvec(6) +  DLvec(7) +  DLvec(9) +  DLvec(10) +  DLvec(11) +  DLvec(13) +  DLvec(14) +  DLvec(15)
    val sumL_low =  Lvec(1)  +  Lvec(2)  +  Lvec(3)  +  Lvec(5)  +  Lvec(6)  +  Lvec(7)
    val sumD_low =  Dvec(1)  +  Dvec(2)  +  Dvec(3)  +  Dvec(5)  +  Dvec(6)  +  Dvec(7)
    val sumDL_low = DLvec(1) +  DLvec(2) +  DLvec(3) +  DLvec(5) +  DLvec(6) +  DLvec(7)
    val maxL_low = Lvec_low.reduce((a, b) => Mux(a > b, a, b))
    val minL_low = Lvec_low.reduce((a, b) => Mux(a < b, a, b))
    val maxD_low = Dvec_low.reduce((a, b) => Mux(a > b, a, b))
    val minD_low = Dvec_low.reduce((a, b) => Mux(a < b, a, b))
    val DLcond1 = Mux(Dvec(new_TCUC_s2) > 7.U * Lvec(new_TCUC_s2) && (Dvec(new_TCUC_s2) > maxD_low/4.U) && Lvec(new_TCUC_s2) =/= 0.U, true.B, false.B)
    val DLcond2 = Mux((Dvec(new_TCUC_s2) > 3.U * Lvec(new_TCUC_s2)) && (Dvec(new_TCUC_s2) > sumD_low/4.U) && Lvec(new_TCUC_s2) =/= 0.U, true.B, false.B)
    val DLcond3 = Mux((DLvec(new_TCUC_s2) > sumDL_low/2.U) && (Lvec(new_TCUC_s2) < (maxL_low + minL_low)/2.U) && Lvec(new_TCUC_s2) =/= 0.U, true.B, false.B)

    val new_binD = WireInit(0.U(20.W))
//    new_binD := Mux(isSampleSets, Mux(hit_s2 && req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.AcquirePerm || req_s2.replacerInfo.opcode === TLMessages.AcquireBlock || req_prefetch),
//      Mux(binD >= 2.U, binD - 2.U, 0.U),
//      Mux(req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.Release || req_s2.replacerInfo.opcode === TLMessages.ReleaseData), binD + 1.U,
//        binD)), binD)
    new_binD := Mux(isSampleSets,
                  Mux(req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.Release || req_s2.replacerInfo.opcode === TLMessages.ReleaseData), binD + 1.U,
                    Mux(!hit_s2 && req_s2.replacerInfo.channel(0) && req_s2.replacerInfo.opcode === TLMessages.Hint && req_s2.replacerInfo.preferCache, binD + 1.U, binD)), binD)
    val new_binL = WireInit(0.U(20.W))
    new_binL := Mux(isSampleSets, Mux(hit_s2 && req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.AcquirePerm || req_s2.replacerInfo.opcode === TLMessages.AcquireBlock || req_prefetch),
      binL + 1.U, binL), binL)
//    val bypass = ((new_binD >= sumD / 2.U) && (new_binL <= (maxL + minL) / 2.U)) || (new_binD >= (sumD - (sumD/4.U)).asTypeOf(new_binD))
    val bypass = DLcond3
    val new_binDL = Cat(new_binD, new_binL)
    val binDL_init = Wire(Vec(16, UInt(40.W)))
    binDL_init.foreach(_ := 0.U(40.W))
    binArray.io.w(
      !resetFinish || binWen,
      Mux(resetFinish, new_binDL, binDL_init.asUInt),
      0.U,
      Mux(resetFinish, UIntToOH(update_TCUC_s2), Fill(16, true.B))
    )

    // req_type[2]: release(1); req_type[1]: acq(1), hint(0); req_type[0]: hit(1), miss(0)
    // 101: release hit
    // 100: release miss
    // 011: acq_hit
    // 001: hint_hit
    // 010: acq_miss(do not update age)
    // 000: hint_miss(if prefercache, will save!!)
    val req_type = WireInit(0.U(3.W))
    req_type := Mux(!hit_s2 && req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.ReleaseData || req_s2.replacerInfo.opcode === TLMessages.Release), 5.U,
      Mux(hit_s2 && req_s2.replacerInfo.channel(2) && (req_s2.replacerInfo.opcode === TLMessages.ReleaseData || req_s2.replacerInfo.opcode === TLMessages.Release), 4.U,
      Mux(hit_s2 && req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.AcquireBlock || req_s2.replacerInfo.opcode === TLMessages.AcquirePerm), 3.U,
        Mux(hit_s2 && req_prefetch, 1.U,
          Mux(!hit_s2 && req_s2.replacerInfo.channel(0), 2.U,
            Mux(!hit_s2 && req_prefetch && req_s2.replacerInfo.preferCache, 0.U, 7.U))))))
    //    override def get_next_state(state: UInt, touch_way: UInt, req_type: UInt, TC: UInt, UC: UInt, binD: Vec[UInt], binL: Vec[UInt]): UInt = {
    //    val next_state_s3 = repl.get_next_state(repl_state_s3, touch_way_s3, req_type, new_TC, new_UC, binD, binL, sumL, maxL, minL, maxD, minD)
    val next_state_s2 = repl.get_next_state(repl_state, way_s2, inv, bypass, req_type, new_TC, new_UC, DLcond1, DLcond2, DLcond3)
    val repl_init = Wire(Vec(ways, UInt(2.W)))
    repl_init.foreach(_ := 3.U(2.W))
    replacer_sram.get.io.w(
      !resetFinish || replacer_wen,
      Mux(resetFinish, next_state_s2, repl_init.asUInt),
      Mux(resetFinish, req_s2.set, resetIdx),
      1.U
    )

//    io.resp.bits.bypass := bypass && !inv && !hit_s2
    io.resp.bits.bypass := false.B
    io.resp.bits.TC := Mux(req_s2.replacerInfo.channel(0) && (req_s2.replacerInfo.opcode === TLMessages.AcquirePerm || req_s2.replacerInfo.opcode === TLMessages.AcquireBlock), Mux(hit_s2, Mux(TC_s2 === 3.U, 3.U, TC_s2 + 1.U), 1.U), 0.U)
    io.resp.bits.UC := new_UC
    io.resp.bits.repl_state := repl_state
    io.resp.bits.next_state := next_state_s2
    io.resp.bits.isSample := isSampleSets
    io.resp.bits.new_TCUC_s2 := new_TCUC_s2
    io.resp.bits.Dvec := Dvec
    io.resp.bits.Lvec := Lvec
    io.resp.bits.DLvec := DLvec
    io.resp.bits.DLcond1 := DLcond1
    io.resp.bits.DLcond2 := DLcond2
    io.resp.bits.DLcond3 := DLcond3
    io.resp.bits.channel := req_s2.replacerInfo.channel
    io.resp.bits.opcode := req_s2.replacerInfo.opcode
    io.resp.bits.set := req_s2.set

  } else if(replacement == "plru") {
    val next_state = repl.get_next_state(repl_state, way_s1)
    replacer_sram.get.io.w(
      !resetFinish ||  replacer_wen_old,
      Mux(resetFinish,  RegNext(next_state), 0.U),
      Mux(resetFinish,  RegNext(reqReg.set), resetIdx),
      1.U)
    io.resp.bits.bypass := false.B
    io.resp.bits.TC := 0.U
    io.resp.bits.UC := 0.U
    io.resp.bits.repl_state := 0.U
    io.resp.bits.next_state := 0.U
    io.resp.bits.isSample := false.B
    io.resp.bits.new_TCUC_s2 := 0.U
    io.resp.bits.Dvec := 0.U.asTypeOf(io.resp.bits.Dvec)
    io.resp.bits.Lvec := 0.U.asTypeOf(io.resp.bits.Lvec)
    io.resp.bits.DLvec := 0.U.asTypeOf(io.resp.bits.DLvec)
    io.resp.bits.DLcond1 := false.B
    io.resp.bits.DLcond2 := false.B
    io.resp.bits.DLcond3 := false.B
    io.resp.bits.channel := req_s2.replacerInfo.channel
    io.resp.bits.opcode := req_s2.replacerInfo.opcode
    io.resp.bits.set := req_s2.set
  } else {
    io.resp.bits.bypass := false.B
    io.resp.bits.TC := 0.U
    io.resp.bits.UC := 0.U
    io.resp.bits.repl_state := 0.U
    io.resp.bits.next_state := 0.U
    io.resp.bits.isSample := false.B
    io.resp.bits.new_TCUC_s2 := 0.U
    io.resp.bits.Dvec := 0.U.asTypeOf(io.resp.bits.Dvec)
    io.resp.bits.Lvec := 0.U.asTypeOf(io.resp.bits.Lvec)
    io.resp.bits.DLvec := 0.U.asTypeOf(io.resp.bits.DLvec)
    io.resp.bits.DLcond1 := false.B
    io.resp.bits.DLcond2 := false.B
    io.resp.bits.DLcond3 := false.B
    io.resp.bits.channel := req_s2.replacerInfo.channel
    io.resp.bits.opcode := req_s2.replacerInfo.opcode
    io.resp.bits.set := req_s2.set
  }


  val cycleCnt = Counter(true.B, 2)
  val resetMask = if (clk_div_by_2) cycleCnt._1(0) else true.B
  when(resetIdx === 0.U && resetMask) {
    resetFinish := true.B
  }
  when(!resetFinish && resetMask) {
    resetIdx := resetIdx - 1.U
  }

}

trait HasUpdate {
  def doUpdate(info: ReplacerInfo): Bool
}

trait UpdateOnRelease extends HasUpdate {
  override def doUpdate(info: ReplacerInfo) = {
    info.channel(2) && info.opcode === TLMessages.ReleaseData
  }
}

trait UpdateOnAcquire extends HasUpdate {
  override def doUpdate(info: ReplacerInfo) = {
    info.channel(0) && (info.opcode === TLMessages.AcquirePerm || info.opcode === TLMessages.AcquireBlock)
  }
}

abstract class SubDirectoryDoUpdate[T <: Data](
  wports:      Int,
  sets:        Int,
  ways:        Int,
  tagBits:     Int,
  dir_init_fn: () => T,
  dir_hit_fn:  T => Bool,
  invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
  replacement: String)(implicit p: Parameters)
    extends SubDirectory[T](
      wports, sets, ways, tagBits,
      dir_init_fn, dir_hit_fn, invalid_way_sel,
      replacement
    ) with HasUpdate {

  val update = doUpdate(reqReg.replacerInfo)
  when(reqValidReg && update) {
    replacer_wen_old := true.B
  }.otherwise {
    replacer_wen_old := false.B
  }
}
