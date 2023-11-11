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
import huancun.noninclusive.SelfDirEntry
import huancun.utils._
// import sun.management.AgentConfigurationError
import utility.Code

trait BaseDirResult extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W) // which mshr the result should be sent to
}
trait BaseDirWrite extends HuanCunBundle
trait BaseTagWrite extends HuanCunBundle
// trait BaseBinCounterWrite extends HuanCunBundle

class DirRead(implicit p: Parameters) extends HuanCunBundle {
  val idOH = UInt(mshrsAll.W)
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val replacerInfo = new ReplacerInfo()
  val source = UInt(sourceIdBits.W)
  val wayMode = Bool()
  val way = UInt(log2Ceil(maxWays).W)
  val tripCount = UInt(1.W)
  val useCount = UInt(2.W)
}

class AgeEntry(implicit p: Parameters) extends HuanCunBundle {
  // for L3-replacement
  // bin number - (tripCount, useCount)
  // 0-(0,0), 1-(0,1), 2-(0,2), 3-(0,3),
  // 4-(1,0), 5-(1,1), 6-(1,2), 7-(1,3)
  val age = UInt(2.W)
  val binNumber = UInt(3.W)
}

// meta + age, for selfDir's invalidWaySel function
class SelfDirAgeEntry(implicit p: Parameters) extends SelfDirEntry {
  val meta = new SelfDirEntry()
  val age = UInt(2.W)
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
  self_invalidWay_sel: (Seq[SelfDirAgeEntry], UInt) => (Bool, UInt),
  client_invalidWay_sel: (Seq[T], UInt) => (Bool, UInt),
  SelfDir_flag: Boolean = false,    // only selfDir needs ageArray
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
      // for L3-replacement
      val repl_msg = if(SelfDir_flag == true) Some(Vec(ways, new AgeEntry)) else None
      val old_age = if(SelfDir_flag == true) Some(new AgeEntry) else None
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
    val age_w = Flipped(DecoupledIO(new Bundle() {
      val set = UInt(setBits.W)
      val way = UInt(wayBits.W)
      val repl_msg = Vec(ways, new AgeEntry)
    }))
  })

  val clk_div_by_2 = p(HCCacheParamsKey).sramClkDivBy2
  val resetFinish = RegInit(false.B)
  val resetIdx = RegInit((sets - 1).U)
  val metaArray = Module(new SRAMTemplate(chiselTypeOf(dir_init), sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))

  val clkGate = Module(new STD_CLKGT_func)
  val clk_en = RegInit(false.B)
  clk_en := ~clk_en
  clkGate.io.TE := false.B
  clkGate.io.E := clk_en
  clkGate.io.CK := clock
  val masked_clock = clkGate.io.Q

  val tag_wen = io.tag_w.valid
  val dir_wen = io.dir_w.valid
  val replacer_wen = RegInit(false.B)
  io.tag_w.ready := true.B
  io.dir_w.ready := true.B
  io.age_w.ready := true.B
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

  // ageArray in selfDir, using for L3-replacement
  val ageRead = Wire(Vec(ways, new AgeEntry))
  if (SelfDir_flag == true) {
    val ageArray = Module(new SRAMTemplate(new AgeEntry, sets, ways, singlePort = true, input_clk_div_by_2 = clk_div_by_2))
    val age_wen = io.age_w.valid
    if (clk_div_by_2) {
      ageArray.clock := masked_clock
    }
    ageArray.io.w(
      !resetFinish || age_wen,
      Mux(resetFinish, io.age_w.bits.repl_msg, 0.U.asTypeOf(io.age_w.bits.repl_msg)),
      Mux(resetFinish, io.age_w.bits.set, resetIdx),
      Fill(ways, true.B)
    )
    ageRead := ageArray.io.r(io.read.fire, io.read.bits.set).resp.data
  } else {
    ageRead := DontCare
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

  val reqReg = RegEnable(io.read.bits, io.read.fire)
  val reqValidReg = RegInit(false.B)
  if (clk_div_by_2) {
    reqValidReg := RegNext(io.read.fire)
  } else {
    reqValidReg := io.read.fire
  }

  val hit_s1 = Wire(Bool())
  val way_s1 = Wire(UInt(wayBits.W))

  val repl = ReplacementPolicy.fromString(replacement, ways)
  val repl_state = if(replacement == "random"){
    when(io.tag_w.fire){
      repl.miss
    }
    0.U
  } else {
    val replacer_sram = Module(new SRAMTemplate(UInt(repl.nBits.W), sets, singlePort = true, shouldReset = true))
    val repl_sram_r = replacer_sram.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))
    val next_state = repl.get_next_state(repl_state_hold, way_s1)
    replacer_sram.io.w(replacer_wen, RegNext(next_state), RegNext(reqReg.set), 1.U)
    repl_state_hold
  }

  io.resp.valid := reqValidReg
  val metas = metaArray.io.r(io.read.fire, io.read.bits.set).resp.data
  val metas_ages_init = 0.U.asTypeOf(Vec(ways, new SelfDirAgeEntry()))
  val metas_ages =WireInit(metas_ages_init)
  if(SelfDir_flag == true) {
    metas_ages.zipWithIndex.foreach {
      case (m, i) =>
        m.meta := metas(i)
        m.age := ageRead(i).age
    }
  } else {
    metas_ages.zipWithIndex.foreach {
      case (m, i) =>
        m.meta := 0.U.asTypeOf(m.meta)
        m.age := 0.U // ignored
    }
  }

  val tagMatchVec = tagRead.map(_(tagBits - 1, 0) === reqReg.tag)
  val metaValidVec = metas.map(dir_hit_fn)
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val hitWay = OHToUInt(hitVec)
  val replaceWay = repl.get_replace_way(repl_state)
  val (inv, invalidWay) = if(SelfDir_flag == true) {
   self_invalidWay_sel(metas_ages, replaceWay)
  } else {
    client_invalidWay_sel(metas, replaceWay)
  }

  val chosenWay = Wire(UInt(wayBits.W))
  if(SelfDir_flag == true) {
    chosenWay := invalidWay // selfDir
  } else {
    chosenWay := Mux(inv, invalidWay, replaceWay) // clientDir
  }

  // val chosenWay = Mux(meta_allways_flag,
  //                   invalidWay, //selfDir
  //                   Mux(inv, invalidWay, replaceWay)  // clientDir
  //
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

  val ages_s1 = ageRead
  val ages_s2 = RegEnable(ages_s1, reqValidReg)

  val errorAll_s1 = VecInit(eccRead.zip(tagRead).map{x => tagCode.decode(x._1 ## x._2).error})
  val errorAll_s2 = RegEnable(errorAll_s1, reqValidReg)
  val error_s2 = errorAll_s2(way_s2)

  io.resp.bits.hit := hit_s2
  io.resp.bits.way := way_s2
  io.resp.bits.dir := meta_s2
  io.resp.bits.tag := tag_s2
  io.resp.bits.error := io.resp.bits.hit && error_s2
  if(SelfDir_flag == true) {
    io.resp.bits.repl_msg.foreach(_ := ages_s2)
    io.resp.bits.old_age.foreach(_ := ages_s2(way_s2))
  }

  metaArray.io.w(
    !resetFinish || dir_wen,
    Mux(resetFinish, io.dir_w.bits.dir, dir_init),
    Mux(resetFinish, io.dir_w.bits.set, resetIdx),
    Mux(resetFinish, UIntToOH(io.dir_w.bits.way), Fill(ways, true.B))
  )


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
  self_invalidWay_sel: (Seq[SelfDirAgeEntry], UInt) => (Bool, UInt),
  client_invalidWay_sel: (Seq[T], UInt) => (Bool, UInt),
  SelfDir_flag: Boolean = false,
  replacement: String)(implicit p: Parameters)
    extends SubDirectory[T](
      wports, sets, ways, tagBits,
      dir_init_fn, dir_hit_fn, self_invalidWay_sel, client_invalidWay_sel, SelfDir_flag,
      replacement
    ) with HasUpdate {

  val update = doUpdate(reqReg.replacerInfo)
  when(reqValidReg && update) {
    replacer_wen := true.B
  }.otherwise {
    replacer_wen := false.B
  }
}

// for inclusive directory
abstract class SubDirectoryDoUpdate_inclusive[T <: Data](
                                                wports:      Int,
                                                sets:        Int,
                                                ways:        Int,
                                                tagBits:     Int,
                                                dir_init_fn: () => T,
                                                dir_hit_fn:  T => Bool,
                                                invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
                                                SelfDir_flag: Boolean = false,
                                                replacement: String)(implicit p: Parameters)
  extends SubDirectory_inclusive[T](
    wports, sets, ways, tagBits,
    dir_init_fn, dir_hit_fn, invalid_way_sel, SelfDir_flag,
    replacement
  ) with HasUpdate {

  val update = doUpdate(reqReg.replacerInfo)
  when(reqValidReg && update) {
    replacer_wen := true.B
  }.otherwise {
    replacer_wen := false.B
  }
}

class SubDirectory_inclusive[T <: Data](
                               wports:      Int,
                               sets:        Int,
                               ways:        Int,
                               tagBits:     Int,
                               dir_init_fn: () => T,
                               dir_hit_fn: T => Bool,
                              invalid_way_sel: (Seq[T], UInt) => (Bool, UInt),
                               SelfDir_flag: Boolean = false,    // only selfDir needs ageArray
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

  val clkGate = Module(new STD_CLKGT_func)
  val clk_en = RegInit(false.B)
  clk_en := ~clk_en
  clkGate.io.TE := false.B
  clkGate.io.E := clk_en
  clkGate.io.CK := clock
  val masked_clock = clkGate.io.Q

  val tag_wen = io.tag_w.valid
  val dir_wen = io.dir_w.valid
  val replacer_wen = RegInit(false.B)
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

  val reqReg = RegEnable(io.read.bits, io.read.fire)
  val reqValidReg = RegInit(false.B)
  if (clk_div_by_2) {
    reqValidReg := RegNext(io.read.fire)
  } else {
    reqValidReg := io.read.fire
  }

  val hit_s1 = Wire(Bool())
  val way_s1 = Wire(UInt(wayBits.W))

  val repl = ReplacementPolicy.fromString(replacement, ways)
  val repl_state = if(replacement == "random"){
    when(io.tag_w.fire){
      repl.miss
    }
    0.U
  } else {
    val replacer_sram = Module(new SRAMTemplate(UInt(repl.nBits.W), sets, singlePort = true, shouldReset = true))
    val repl_sram_r = replacer_sram.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))
    val next_state = repl.get_next_state(repl_state_hold, way_s1)
    replacer_sram.io.w(replacer_wen, RegNext(next_state), RegNext(reqReg.set), 1.U)
    repl_state_hold
  }

  io.resp.valid := reqValidReg
  val metas = metaArray.io.r(io.read.fire, io.read.bits.set).resp.data
  val tagMatchVec = tagRead.map(_(tagBits - 1, 0) === reqReg.tag)
  val metaValidVec = metas.map(dir_hit_fn)
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val hitWay = OHToUInt(hitVec)
  val replaceWay = repl.get_replace_way(repl_state)
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

  val cycleCnt = Counter(true.B, 2)
  val resetMask = if (clk_div_by_2) cycleCnt._1(0) else true.B
  when(resetIdx === 0.U && resetMask) {
    resetFinish := true.B
  }
  when(!resetFinish && resetMask) {
    resetIdx := resetIdx - 1.U
  }



}