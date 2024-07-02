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
import chisel3.util.{RegEnable, _}
import chisel3.util.random.LFSR
import freechips.rocketchip.tilelink.TLMessages
import freechips.rocketchip.util.{Pow2ClockDivider, ReplacementPolicy}
import huancun.utils._
import utility.{Code, MemReqSource}

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

class EQentry(implicit p: Parameters) extends HuanCunBundle {
  val addr_tag = UInt(tagBits.W)  // addr's tag bits
  // val state = UInt(26.W)
  val pc = UInt(13.W) // low 13 bits
  val pn = UInt(13.W) // paddr's high 13 bits
  val action = UInt(4.W)
  val trigger = Bool() // hit or miss
  val reward = SInt(20.W) // use the highest bit to indicates sign(positive or negative)
  val Q_value = SInt(20.W)
}

object Actions {
  def HitEPV0    = 0.U
  def HitPEV1    = 1.U
  def HitEPV2    = 2.U
  def HitEPV3    = 3.U
  def MissEPV0   = 4.U
  def MissEPV1   = 5.U
  def MissEPV2   = 6.U
  def MissEPV3   = 7.U
  def MissBypass = 8.U
}

object Rewards {
  def Demand_hit = 20000.S
  def Prefetch_hit = 5000.S
  def Evict_pos = 10000.S
  def Demand_miss = -20000.S
  def Prefetch_miss = -5000.S
  def Evict_neg = -10000.S
}

object HyperParam {
  def alpha: SInt = 50.S
  def garma: SInt = 3.S
  def div_param: SInt = 1000.S
  def AcqEPV: UInt = 3.U
}

class ChromeInfo(implicit p: Parameters) extends HuanCunBundle {
  val channel = UInt(3.W)
  val opcode = UInt(3.W)
  val tag = UInt(tagBits.W)
  val sset = UInt(setBits.W)
  val pc = UInt(39.W)
  val pn = UInt(13.W)
  val reqSource = UInt(MemReqSource.reqSourceBits.W)
  val way = UInt(wayBits.W)
  val hit = Bool()
  // r/w EQ
  val issampleset = Bool()
  val write_ptr = UInt(5.W)
  val evict_ptr = UInt(5.W)
  val head_ptr = UInt(5.W)
  // for A request
  val update_EQ_wen = Bool()
  val match_EQ = new EQentry()  // old
  val update_EQ = new EQentry() // new
  // for C request
  val insert_EQ_wen = Bool()
  val evict_EQ = new EQentry()  // old
  val insert_EQ = new EQentry() // new
  // for update Q_Table
  val head_Qvalue = SInt(20.W)

  // read Q_Table and search for action
  val Qpc_Value = Vec(9, SInt(20.W))  // QPCread
  val Qpn_Value = Vec(9, SInt(20.W))  // QPNread
  val exp_cnt = UInt(10.W)
  val action = UInt(4.W)
  // write Qpc
  val Qpc_wen = Bool()
  val Qpc_w_set = UInt(13.W)
  val Qpc_w_way = UInt(4.W)
  val Qpc_w_value = SInt(20.W)
  // write Qpc
  val Qpn_wen = Bool()
  val Qpn_w_set = UInt(13.W)
  val Qpn_w_way = UInt(4.W)
  val Qpn_w_value = SInt(20.W)
  // replacer
  val repl_state = UInt(32.W)
  val next_state = UInt(32.W)
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
      val chromeInfo = new ChromeInfo()
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
  val chrome_replacer_wen = RegInit(false.B)
  val QPC_wen = WireInit(false.B)
  val QPN_wen = WireInit(false.B)
  val EQ_wen = WireInit(false.B)
  io.tag_w.ready := true.B
  io.dir_w.ready := true.B
  io.read.ready := !tag_wen && !dir_wen && resetFinish && !chrome_replacer_wen
//  io.read.ready := !tag_wen && !dir_wen && resetFinish && !chrome_replacer_wen && !QPC_wen && !QPN_wen && !EQ_wen

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
  val req_s1 = RegNext(reqReg)
  val req_s2 = RegEnable(req_s1, reqValidReg)
  val reqSource_s1 = req_s1.replacerInfo.reqSource
  val reqSource_s2 = req_s2.replacerInfo.reqSource
  if (clk_div_by_2) {
    reqValidReg := RegNext(io.read.fire)
  } else {
    reqValidReg := io.read.fire
  }
  val req_valid_s2 = RegNext(reqValidReg)
  val hit_s1 = Wire(Bool())
  val way_s1 = Wire(UInt(wayBits.W))

  val repl = ReplacementPolicy.fromString(replacement, ways)
  val replacer_sram = if (replacement == "random") None else
    Some(Module(new SRAMTemplate(UInt(repl.nBits.W), sets, 1, singlePort = true, shouldReset = true)))

  val repl_state = if(replacement == "random"){
    when(io.tag_w.fire){
      repl.miss
    }
    0.U
  } else if (replacement == "chrome"){
    val repl_sram_r = replacer_sram.get.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))
    repl_state_hold
  } else {  // plru
    val repl_sram_r = replacer_sram.get.io.r(io.read.fire, io.read.bits.set).resp.data(0)
    val repl_state_hold = WireInit(0.U(repl.nBits.W))
    repl_state_hold := HoldUnless(repl_sram_r, RegNext(io.read.fire, false.B))
    val next_state = repl.get_next_state(repl_state_hold, way_s1)
    replacer_sram.get.io.w(replacer_wen, RegNext(next_state), RegNext(reqReg.set), 1.U)
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
  io.resp.bits.chromeInfo := 0.U.asTypeOf(io.resp.bits.chromeInfo)

  metaArray.io.w(
    !resetFinish || dir_wen,
    Mux(resetFinish, io.dir_w.bits.dir, dir_init),
    Mux(resetFinish, io.dir_w.bits.set, resetIdx),
    Mux(resetFinish, UIntToOH(io.dir_w.bits.way), Fill(ways, true.B))
  )

  /* stage 0: io.read.fire
     stage #: wait for sram
     stage 1: generate hit/way, io.resp.valid = TRUE (will latch into MSHR), update EQ
     stage 2: output latched hit/way, output dir/tag; update replacer_sram and Q_Table
  */

  if(replacement == "chrome") {
    val EQ = Module(new SRAMTemplate(new EQentry, 64, 28, singlePort = true, shouldReset = true)) // 64sampleset, 28entry
    val Q_PC = Module(new SRAMTemplate(SInt(20.W), 8192, 9, singlePort = true, shouldReset = true))
    val Q_PN = Module(new SRAMTemplate(SInt(20.W), 8192, 9, singlePort = true, shouldReset = true))

    val isSampleSets = (io.read.bits.set(11, 6) + io.read.bits.set(5, 0) === 63.U)
    val pc_index = io.read.bits.replacerInfo.pc(12, 0)
    val pn_index = io.read.bits.tag(15,3)

    val req_A_s1 = req_s1.replacerInfo.channel(0) && (req_s1.replacerInfo.opcode === TLMessages.AcquireBlock || req_s1.replacerInfo.opcode === TLMessages.AcquirePerm)
    val req_Hint_s1 = req_s1.replacerInfo.channel(0) && req_s1.replacerInfo.opcode === TLMessages.Hint
    val req_Release_s1 = req_s1.replacerInfo.channel(2) && (req_s1.replacerInfo.opcode === TLMessages.ReleaseData || req_s1.replacerInfo.opcode === TLMessages.Release)
    val req_Acquire_s1 = req_A_s1 && (reqSource_s1 === 2.U || reqSource_s1 === 3.U || reqSource_s1 === 7.U)
    val req_prefetch_s1 = req_A_s1 && reqSource_s1 >= 5.U && reqSource_s1 <= 14.U && reqSource_s1 =/= 7.U
    val req_Acquire_Release_s1 = req_Release_s1 && (reqSource_s1 === 2.U || reqSource_s1 === 3.U) // demand access
    val req_prefetch_Release_s1 = req_Release_s1 && reqSource_s1 >= 5.U && reqSource_s1 <= 14.U && reqSource_s1 =/= 7.U
    val req_PTW_Release_s1 = req_Release_s1 && reqSource_s1 === 7.U
    val isSampleSets_s1 = (req_s1.set(11, 6) +req_s1.set(5, 0) === 63.U)

    val EQread = Wire(Vec(28, new EQentry()))
    val QPCread =  Wire(Vec(9, SInt(20.W)))
    val QPNread =  Wire(Vec(9, SInt(20.W)))

    val EQread_hold = WireInit(0.U.asTypeOf(EQread))
    val QPCread_hold = WireInit(0.U.asTypeOf(QPCread))
    val QPNread_hold = WireInit(0.U.asTypeOf(QPNread))

    EQread := Mux(isSampleSets, EQ.io.r(io.read.fire, io.read.bits.set(5,0)).resp.data, 0.U.asTypeOf(EQread))
    QPCread := Q_PC.io.r(io.read.fire, pc_index).resp.data
    QPNread := Q_PN.io.r(io.read.fire, pn_index).resp.data

    EQread_hold := HoldUnless(EQread, RegNext(io.read.fire, false.B))
    QPCread_hold := HoldUnless(QPCread, RegNext(io.read.fire, false.B))
    QPNread_hold := HoldUnless(QPNread, RegNext(io.read.fire, false.B))

    def vec_max_index(vec: Seq[SInt], num: Int): (UInt, SInt) = {
      val max_value_vec = Wire(Vec(num, Bool()))
      max_value_vec.zipWithIndex.map {
       case(e, i) =>
         val isLarger = Wire(Vec(num, Bool()))
          for(j <- 0 until num) {
            isLarger(j) := vec(j) > vec(i)
          }
          e := !(isLarger.contains(true.B))
     }
      val vec_in = VecInit(vec)
      val max_index = PriorityEncoder(max_value_vec)
      val max_value = vec_in(max_index)
      (max_index, max_value)
    }

    //  return the latest same addr entry's way id
    def addr_match(tag: UInt, tag_vec: Seq[UInt], evict_ptr: UInt): (Bool, UInt) = {
      val wayid = RegInit(0.U(5.W))
      val wayid_low = RegInit(0.U(5.W))
      val wayid_high = RegInit(0.U(5.W))
      val tag_match_vec = tag_vec.map(_ === tag)
      val ismatch = Cat(tag_match_vec).orR
      tag_match_vec.zipWithIndex.foreach {
        case(m, i) =>
          wayid_low := Mux(m && i.U < evict_ptr, i.U, wayid_low)
          wayid_high := Mux(m && i.U > evict_ptr, i.U, wayid_high)
          wayid := Mux(wayid_low === 0.U, wayid_high, wayid_low)
      }
      (ismatch, wayid)
    }

    val QPC_hitvec = Wire(Vec(4, SInt(20.W)))
    val QPC_missvec = Wire(Vec(5, SInt(20.W)))
    val QPN_hitvec = Wire(Vec(4, SInt(20.W)))
    val QPN_missvec = Wire(Vec(5, SInt(20.W)))

    QPC_hitvec.zipWithIndex.foreach {
      case(m, i) =>
        m := QPCread_hold(i)
    }
    QPC_missvec.zipWithIndex.foreach {
      case(m, i) =>
        m := QPCread_hold(i + 4)
    }
    QPN_hitvec.zipWithIndex.foreach {
      case (m, i) =>
        m := QPNread_hold(i)
    }
    QPN_missvec.zipWithIndex.foreach {
      case (m, i) =>
        m := QPNread_hold(i + 4)
    }

    // (0,hit0), (1,hit1), (2,hit2), (3,hit3)
    // (4,miss0), (5,miss1), (6,miss2), (7,miss3), (8,missbypass)
    // only useful for release
    val action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val pn_action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val explore_action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val explore_pn_action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val exploit_action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val exploit_pn_action_s1 = WireInit(0.U.asTypeOf(UInt(4.W)))
    val hit_action = WireInit(0.U.asTypeOf(UInt(4.W)))
    val miss_action = WireInit(0.U.asTypeOf(UInt(4.W)))

    val Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))
    val pn_Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))  // for prefetch/PTW release
    val explore_Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))
    val explore_pn_Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))
    val exploit_Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))
    val exploit_pn_Q_value_s1 = WireInit(0.S.asTypeOf(SInt(20.W)))
    val hit_Q_value = WireInit(0.S.asTypeOf(SInt(20.W)))
    val miss_Q_value = WireInit(0.S.asTypeOf(SInt(20.W)))

    val (hit_QPC_index, hit_QPC_max) = vec_max_index(QPC_hitvec, 4)
    val (miss_QPC_index, miss_QPC_max) = vec_max_index(QPC_missvec, 5)
    val (hit_QPN_index, hit_QPN_max) = vec_max_index(QPN_hitvec, 4)
    val (miss_QPN_index, miss_QPN_max) = vec_max_index(QPN_missvec, 5)
    hit_action := Mux(hit_QPC_max >= hit_QPN_max, hit_QPC_index, hit_QPN_index)
    miss_action := Mux(miss_QPC_max >= miss_QPN_max, miss_QPC_index, miss_QPN_index)
    hit_Q_value := Mux(hit_QPC_max >= hit_QPN_max, hit_QPC_max, hit_QPN_max)
    miss_Q_value := Mux(miss_QPC_max >= miss_QPN_max, miss_QPC_max, miss_QPN_max)

    val exp_cnt = RegInit(0.U(10.W))
    exp_cnt := Mux(req_Release_s1, exp_cnt + 1.U, exp_cnt)

    exploit_pn_action_s1 := Mux(hit_s1, hit_QPN_index, miss_QPN_index)
    exploit_action_s1 := Mux(hit_s1, hit_action, miss_action)
    explore_pn_action_s1 := Mux(hit_s1, req_s1.set(1,0), req_s1.set(1,0) + 4.U)
    explore_action_s1 := Mux(hit_s1, req_s1.set(1,0), req_s1.set(1,0) + 4.U)
    pn_action_s1 := Mux(exp_cnt === 1023.U, explore_pn_action_s1, exploit_pn_action_s1)
    action_s1 := Mux(exp_cnt === 1023.U, explore_action_s1, exploit_action_s1)

    exploit_pn_Q_value_s1 := Mux(hit_s1, hit_QPN_max, miss_QPN_max)
    exploit_Q_value_s1 := Mux(hit_s1, hit_Q_value, miss_Q_value)
    explore_pn_Q_value_s1 := Mux(hit_s1, QPN_hitvec(req_s1.set(1,0)), QPN_missvec(req_s1.set(1,0)))
    explore_Q_value_s1 := Mux(hit_s1, QPC_hitvec(req_s1.set(1,0)), QPC_missvec(req_s1.set(1,0)))
    pn_Q_value_s1 := Mux(exp_cnt === 1023.U, explore_pn_Q_value_s1, exploit_pn_Q_value_s1)
    Q_value_s1 := Mux(exp_cnt === 1023.U, explore_Q_value_s1, exploit_Q_value_s1)

    val write_ptr = RegInit(VecInit(Seq.fill(64)(0.U(5.W))))
    val evict_ptr = RegInit(VecInit(Seq.fill(64)(0.U(5.W))))
    val head_ptr = RegInit(VecInit(Seq.fill(64)(0.U(5.W))))

    // Acquire or Prefetch: update EQ_entry(addr && latest)
    // Acquire_Release && sampledset: insert new entry
    val EQ_tag_vec = Wire(Vec(28, UInt(tagBits.W)))
    EQ_tag_vec.zipWithIndex.foreach {
      case(m, i) =>
        m := EQread_hold(i).addr_tag
    }

    val (has_match_EQ, eq_match_idx) = addr_match(req_s1.tag, EQ_tag_vec, evict_ptr(req_s1.set(5,0)))
    val EQ_match_entry = EQread_hold(eq_match_idx)
    val EQ_update_entry = EQ_match_entry
    EQ_update_entry.reward := Mux(req_Acquire_s1, Mux(hit_s1, Rewards.Demand_hit, Rewards.Demand_miss), Mux(req_prefetch_s1 || req_Hint_s1, Mux(hit_s1, Rewards.Prefetch_hit, Rewards.Prefetch_miss), 0.S))

    val EQ_evict_entry = EQread_hold(evict_ptr(req_s1.set(5,0)))
    val EQ_evict_entry_reward = WireInit(0.S(20.W))
    EQ_evict_entry_reward := Mux(EQ_evict_entry.reward === 0.S, Mux(EQ_evict_entry.action === Actions.HitEPV3 || EQ_evict_entry.action === Actions.MissBypass, Rewards.Evict_pos, Rewards.Evict_neg), EQ_evict_entry.reward)


    val EQ_head_entry = EQread_hold(head_ptr(req_s1.set(5,0)))
    val EQ_head_Q_value = EQ_head_entry.Q_value

    val EQ_new_entry = WireInit(0.U.asTypeOf(new EQentry()))
    EQ_new_entry.addr_tag := req_s1.tag
    EQ_new_entry.pc := req_s1.replacerInfo.pc(12, 0)
    EQ_new_entry.pn := req_s1.tag(15, 3)
    EQ_new_entry.trigger := hit_s1
    EQ_new_entry.action := action_s1
    EQ_new_entry.reward := 0.S
    EQ_new_entry.Q_value := Q_value_s1

    val EQ_update_A = isSampleSets_s1 && has_match_EQ && (req_Acquire_s1 || req_prefetch_s1 || req_Hint_s1)
    val EQ_insert_C = isSampleSets_s1 && req_Acquire_Release_s1
    val EQ_evict = EQ_insert_C && (write_ptr(req_s1.set(5,0)) === 28.U)
    EQ_wen := reqValidReg && (EQ_update_A || EQ_insert_C)
    val EQ_w_set = req_s1.set(5,0)
    val EQ_w_way = Mux(EQ_update_A, eq_match_idx, Mux(write_ptr(req_s1.set(5,0)) < 28.U, write_ptr(req_s1.set(5,0)), evict_ptr(req_s1.set(5,0))))
//    val EQ_w_entry = Mux(EQ_update_A, EQ_update_entry, EQ_new_entry)
    val EQ_w_entry = Mux(EQ_update_A, EQ_update_entry, EQ_new_entry)
    val EQ_w_init = WireInit(0.U.asTypeOf(new EQentry))
    EQ.io.w(
      !resetFinish || EQ_wen,
      Mux(resetFinish, EQ_w_entry, EQ_w_init),
      Mux(resetFinish, EQ_w_set, resetIdx),
      Mux(resetFinish, UIntToOH(EQ_w_way), Fill(28, true.B))
    )
    write_ptr(req_s1.set(5,0)) := Mux(isSampleSets_s1 && req_Acquire_Release_s1 && write_ptr(req_s1.set(5,0)) < 28.U, write_ptr(req_s1.set(5,0)) + 1.U, write_ptr(req_s1.set(5,0)))
    evict_ptr(req_s1.set(5,0)) := Mux(isSampleSets_s1 && req_Acquire_Release_s1 && write_ptr(req_s1.set(5,0)) === 28.U, Mux(evict_ptr(req_s1.set(5,0)) < 27.U, evict_ptr(req_s1.set(5,0)) + 1.U, 0.U), evict_ptr(req_s1.set(5,0)))
    head_ptr(req_s1.set(5,0)) := Mux(evict_ptr(req_s1.set(5,0)) < 27.U, evict_ptr(req_s1.set(5,0)) + 1.U, 0.U)


    // stage 2: SARSA and write Q_Table/replacer_sram
    val pc_index_s2 = req_s2.replacerInfo.pc(12, 0)
    val pn_index_s2 = req_s2.tag(15, 3)
    val EQ_evict_entry_s2 = RegEnable(EQ_evict_entry, reqValidReg)
    val action_s2 = RegEnable(action_s1, reqValidReg)
    val pn_action_s2 = RegEnable(pn_action_s1, reqValidReg)
    val EQ_head_Q_value_s2 = RegEnable(EQ_head_Q_value, reqValidReg)
    val EQ_evict_Q_value_s2 = RegEnable(Q_value_s1, reqValidReg)
    val EQ_evict_reward_s2 = RegEnable(EQ_evict_entry_reward, reqValidReg)
    val EQ_evict_s2 = RegEnable(EQ_evict, false.B, reqValidReg)
    val isSampleSets_s2 = RegEnable(isSampleSets_s1, reqValidReg)

    // SARSA algorithm(may have negative results)
    val R = WireInit(0.S(20.W))
    R := EQ_evict_reward_s2
    val Q1 = WireInit(0.S(20.W))
    Q1 := EQ_evict_Q_value_s2
    val Q2 = WireInit(0.S(20.W))
    Q2 := EQ_head_Q_value_s2
    val RminusEvictedQ1 = WireInit(0.S(20.W))
    val df_headQ2 = WireInit(0.S(20.W))
    val biasPlus = WireInit(0.S(20.W))
    val LR_times = WireInit(0.S(26.W))
    val division_by_1000 = WireInit(0.S(20.W))
    val new_Q1_value_s2 = WireInit(0.S(20.W))
    RminusEvictedQ1 := R - Q1
    df_headQ2 := Q2 / HyperParam.garma
    biasPlus := RminusEvictedQ1 + df_headQ2
    LR_times := HyperParam.alpha * biasPlus
    division_by_1000 := LR_times / HyperParam.div_param
    new_Q1_value_s2 := Q1 + division_by_1000

    // write Q_Table
    QPC_wen := EQ_evict_s2
    QPN_wen := EQ_evict_s2
    val QPC_w = new_Q1_value_s2
    val QPN_w = new_Q1_value_s2
    val QPC_w_init = WireInit(42000.S.asTypeOf(QPC_w))
    val QPN_w_init = WireInit(42000.S.asTypeOf(QPN_w))
    val QPC_w_set = pc_index_s2
    val QPN_w_set = pn_index_s2
    val QPC_w_way = EQ_evict_entry_s2.action
    val QPN_w_way = EQ_evict_entry_s2.action
    Q_PC.io.w(
      !resetFinish || QPC_wen,
      Mux(resetFinish, QPC_w, QPC_w_init),
      Mux(resetFinish, QPC_w_set, resetIdx),
      Mux(resetFinish, UIntToOH(QPC_w_way), Fill(9, true.B))
    )
    Q_PN.io.w(
      !resetFinish || QPN_wen,
      Mux(resetFinish, QPN_w, QPN_w_init),
      Mux(resetFinish, QPN_w_set, resetIdx),
      Mux(resetFinish, UIntToOH(QPN_w_way), Fill(9, true.B))
    )

    // write replacer_sram

//    val req_A_s1 = req_s1.replacerInfo.channel(0) && (req_s1.replacerInfo.opcode === TLMessages.AcquireBlock || req_s1.replacerInfo.opcode === TLMessages.AcquirePerm)
//    val req_Hint_s1 = req_s1.replacerInfo.channel(0) && req_s1.replacerInfo.opcode === TLMessages.Hint
//    val req_Release_s1 = req_s1.replacerInfo.channel(2) && (req_s1.replacerInfo.opcode === TLMessages.ReleaseData || req_s1.replacerInfo.opcode === TLMessages.Release)
//    val req_Acquire_s1 = req_A_s1 && (reqSource_s1 === 2.U || reqSource_s1 === 3.U || reqSource_s1 === 7.U)
//    val req_prefetch_s1 = (req_A_s1 && reqSource_s1 >= 5.U && reqSource_s1 <= 14.U && reqSource_s1 =/= 7.U)
//    val req_Acquire_Release_s1 = req_Release_s1 && (reqSource_s1 === 2.U || reqSource_s1 === 3.U) // demand access
//    val req_prefetch_Release_s1 = req_Release_s1 && reqSource_s1 >= 5.U && reqSource_s1 <= 14.U && reqSource_s1 =/= 7.U
//    val req_PTW_Release_s1 = req_Release_s1 && reqSource_s1 === 7.U

    val req_A_s2 = RegEnable(req_A_s1, false.B, reqValidReg)
    val req_Hint_s2 = RegEnable(req_Hint_s1, false.B, reqValidReg)
    val req_Acquire_s2 = RegEnable(req_Acquire_s1, false.B, reqValidReg)
    val req_prefetch_s2 = RegEnable(req_prefetch_s1, false.B, reqValidReg)
    val req_Acquire_Release_s2 = RegEnable(req_Acquire_Release_s1, false.B, reqValidReg)
    val req_prefetch_Release_s2 = RegEnable(req_prefetch_Release_s1, false.B, reqValidReg)
    val req_PTW_Release_s2 = RegEnable(req_PTW_Release_s1, false.B, reqValidReg)
    chrome_replacer_wen := reqValidReg && ((req_A_s1 && hit_s1) || req_Hint_s1 || req_Acquire_Release_s1 || req_prefetch_Release_s1 || req_PTW_Release_s1)
    val next_state_s2 = repl.get_next_state(repl_state, way_s2, hit_s2, req_Acquire_s2, req_prefetch_s2, req_Hint_s2, req_Acquire_Release_s2, req_prefetch_Release_s2 || req_PTW_Release_s2, action_s2, pn_action_s2)
    val repl_init = Wire(Vec(ways, UInt(2.W)))
    repl_init.foreach(_ := 3.U(2.W))
    replacer_sram.get.io.w(
      !resetFinish || chrome_replacer_wen,
      Mux(resetFinish, next_state_s2, repl_init.asUInt),
      Mux(resetFinish, req_s2.set, resetIdx),
      1.U
    )

    val write_ptr_s2 = RegEnable(write_ptr(req_s1.set(5,0)), 0.U, reqValidReg)
    val evict_ptr_s2 = RegEnable(evict_ptr(req_s1.set(5,0)), 0.U, reqValidReg)
    val head_ptr_s2 = RegEnable(head_ptr(req_s1.set(5,0)), 0.U, reqValidReg)
    val update_EQ_wen = RegEnable(reqValidReg && EQ_update_A, false.B, reqValidReg)
    val insert_EQ_wen = RegEnable(reqValidReg && EQ_insert_C, false.B, reqValidReg)
    val match_EQ_s2 = RegEnable(EQ_match_entry, reqValidReg)
    val update_EQ_s2 = RegEnable(EQ_update_entry, reqValidReg)
    val evict_EQ_s2 = RegEnable(EQ_evict_entry, reqValidReg)
    val insert_EQ_s2 = RegEnable(EQ_new_entry, reqValidReg)
    val head_Qvalue_s2 = RegEnable(EQ_head_Q_value, 0.S, reqValidReg)
    val exp_cnt_s2 = RegEnable(exp_cnt, 0.U, reqValidReg)

    val resp_chrome = io.resp.bits.chromeInfo
    resp_chrome.channel := req_s2.replacerInfo.channel
    resp_chrome.opcode := req_s2.replacerInfo.opcode
    resp_chrome.tag := req_s2.tag
    resp_chrome.sset := req_s2.set
    resp_chrome.pc := req_s2.replacerInfo.pc
    resp_chrome.reqSource := req_s2.replacerInfo.reqSource
    resp_chrome.way := way_s2
    resp_chrome.hit := hit_s2
    resp_chrome.issampleset := isSampleSets_s2
    resp_chrome.write_ptr := write_ptr_s2
    resp_chrome.evict_ptr := evict_ptr_s2
    resp_chrome.head_ptr := head_ptr_s2
    resp_chrome.update_EQ_wen := update_EQ_wen
    resp_chrome.match_EQ := match_EQ_s2
    resp_chrome.update_EQ := update_EQ_s2
    resp_chrome.insert_EQ_wen := insert_EQ_wen
    resp_chrome.evict_EQ := evict_EQ_s2
    resp_chrome.insert_EQ := insert_EQ_s2
    resp_chrome.head_Qvalue := head_Qvalue_s2
    resp_chrome.Qpc_Value := QPCread_hold
    resp_chrome.Qpn_Value := QPNread_hold
    resp_chrome.exp_cnt := exp_cnt_s2
    resp_chrome.action := action_s2
    resp_chrome.Qpc_wen := QPC_wen
    resp_chrome.Qpc_w_set := QPC_w_set
    resp_chrome.Qpc_w_way := QPC_w_way
    resp_chrome.Qpc_w_value := QPC_w
    resp_chrome.Qpn_wen := QPN_wen
    resp_chrome.Qpn_w_set := QPN_w_set
    resp_chrome.Qpn_w_way := QPN_w_way
    resp_chrome.Qpn_w_value := QPN_w
    resp_chrome.repl_state := repl_state
    resp_chrome.next_state := next_state_s2
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
    replacer_wen := true.B
  }.otherwise {
    replacer_wen := false.B
  }
}
