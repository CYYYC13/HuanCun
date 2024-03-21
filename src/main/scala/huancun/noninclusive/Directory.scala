package huancun.noninclusive

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import firrtl.transforms.DontTouchAnnotation
import freechips.rocketchip.tilelink.TLMessages
import huancun.MetaData._
import huancun._
import huancun.debug.{DirectoryLogger, TypeId}
import huancun.utils._
import utility.{ChiselDB, ParallelMin, ParallelPriorityMux}

trait HasClientInfo { this: HasHuanCunParameters =>
  // assume all clients have same params
  // TODO: check client params to ensure they are same
  val clientCacheParams = cacheParams.clientCaches.head
  val aliasBits = aliasBitsOpt.getOrElse(0)

  val clientSets = clientCacheParams.sets
  val clientWays = clientCacheParams.ways
  val clientSetBits = log2Ceil(clientSets)
  val clientWayBits = log2Ceil(clientWays)
  val clientTagBits = addressBits - clientSetBits - offsetBits
}

class replBundle(implicit p: Parameters) extends HuanCunBundle {
  val channel = UInt(3.W)
  val opcode = UInt(3.W)
  val param = UInt(2.W)
  //val addr = UInt((tagBits + setBits).W)
  val tag = UInt(tagBits.W)
  val sset = UInt(setBits.W)  //'set' is C++ common word, use 'sset' instead
  val bank = UInt(2.W)
  val tripCount = UInt(1.W)
  val useCount = UInt(2.W)
  val L = Vec(8, UInt(10.W))
  val D_L = Vec(8, UInt(18.W))
  val L_sum = UInt(20.W)
  val selectedWay = UInt(wayBits.W)
  val hit = UInt(1.W)
  val hitway = UInt(wayBits.W)
  val age = Vec(cacheParams.ways, UInt(2.W))
  val wayCnt = Vec(cacheParams.ways, UInt(30.W))
}

class DLCounterEntry(implicit p: Parameters) extends HuanCunBundle {
  val D_L = UInt(20.W)
  val L = UInt(20.W)
  val L_sum = UInt(20.W)
}

/*
class BinCounterEntry(implicit p: Parameters) extends HuanCunBundle {
  val DLCounter = Vec(8, new DLCounterEntry())
}
*/

class SelfDirEntry(implicit p: Parameters) extends HuanCunBundle {
  val dirty = Bool()
  val state = UInt(stateBits.W)
  val clientStates = Vec(clientBits, UInt(stateBits.W))
  val prefetch = if (hasPrefetchBit) Some(Bool()) else None // whether the block is prefetched
}

class ClientDirEntry(implicit p: Parameters) extends HuanCunBundle {
  val state = UInt(stateBits.W)
  val alias = aliasBitsOpt.map(bits => UInt(bits.W))
}

class SelfDirResult(implicit p: Parameters) extends SelfDirEntry {
  val hit = Bool()
  val way = UInt(wayBits.W)
  val tag = UInt(tagBits.W)
  val error = Bool()
  // for L3-replacement
  val repl_msg = Vec(cacheParams.ways, new AgeEntry())
  val oldAge = new AgeEntry // save old age to directory
}

class ClientDirResult(implicit p: Parameters) extends HuanCunBundle with HasClientInfo {
  val states = Vec(clientBits, new ClientDirEntry {
    val hit = Bool()
  })
  val tag_match = Bool()
  val tag = UInt(clientTagBits.W)
  val way = UInt(clientWayBits.W)
  val error = Bool()

  def parseTag(lineAddr: UInt): UInt = {
    lineAddr(clientSetBits + clientTagBits - 1, clientSetBits)
  }
}

class DirResult(implicit p: Parameters) extends BaseDirResult with HasClientInfo {
  val self = new SelfDirResult
  val clients = new ClientDirResult
  val binCounter = Vec(8, new DLCounterEntry())
  val sourceId = UInt(sourceIdBits.W)
  val set = UInt(setBits.W)
  val replacerInfo = new ReplacerInfo
}

class SelfTagWrite(implicit p: Parameters) extends BaseTagWrite {
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)
  val tag = UInt(tagBits.W)
}

class ClientTagWrite(implicit p: Parameters) extends HuanCunBundle with HasClientInfo {
  val set = UInt(clientSetBits.W)
  val way = UInt(clientWayBits.W) // log2
  val tag = UInt(clientTagBits.W)

  def apply(lineAddr: UInt, way: UInt) = {
    this.set := lineAddr(clientSetBits - 1, 0)
    this.way := way
    this.tag := lineAddr(clientSetBits + clientTagBits - 1, clientSetBits)
  }
}

class SelfDirWrite(implicit p: Parameters) extends BaseDirWrite {
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)
  val data = new SelfDirEntry
  // val oldAge = UInt(2.W)
}

class ClientDirWrite(implicit p: Parameters) extends HuanCunBundle with HasClientInfo {
  val set = UInt(clientSetBits.W)
  val way = UInt(clientWayBits.W)
  val data = Vec(clientBits, new ClientDirEntry())

  def apply(lineAddr: UInt, way: UInt, data: Vec[ClientDirEntry]) = {
    this.set := lineAddr(clientSetBits - 1, 0)
    this.way := way
    this.data := data
  }
}

class BinCounterWrite(implicit p: Parameters) extends HuanCunBundle {
  val binNumber = UInt(3.W)
  val DLCounter = Vec(8, new DLCounterEntry)
}

class AgeWrite(implicit p: Parameters) extends HuanCunBundle {
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)
  val repl_msg = Vec(cacheParams.ways, new AgeEntry)
}

trait NonInclusiveCacheReplacerUpdate { this: HasUpdate =>
  override def doUpdate(info: ReplacerInfo): Bool = {
    val release_update = info.channel(2) && info.opcode === TLMessages.ReleaseData
    val prefetch_update = info.channel(0) && info.opcode === TLMessages.Hint
    release_update | prefetch_update
  }
}

class DirectoryIO(implicit p: Parameters) extends BaseDirectoryIO[DirResult, SelfDirWrite, SelfTagWrite] {
  val read = Flipped(DecoupledIO(new DirRead))
  val sliceId = Input(UInt(2.W))
  val result = ValidIO(new DirResult)
  val dirWReq = Flipped(DecoupledIO(new SelfDirWrite))
  val tagWReq = Flipped(DecoupledIO(new SelfTagWrite))
  val clientDirWReq = Flipped(DecoupledIO(new ClientDirWrite))
  val clientTagWreq = Flipped(DecoupledIO(new ClientTagWrite))
  val binWReq = Flipped(DecoupledIO(new BinCounterWrite))
  val ageWReq = Flipped(DecoupledIO(new AgeWrite))
}

class Directory(implicit p: Parameters)
    extends BaseDirectory[DirResult, SelfDirWrite, SelfTagWrite]
    with HasClientInfo {
  val io = IO(new DirectoryIO())

  val stamp = GTimer()
  val selfDirW = io.dirWReq
  val sliceId = WireInit(0.U)
  sliceId := io.sliceId
  dontTouch(sliceId)
  // dump self dir
  DirectoryLogger(cacheParams.name, TypeId.self_dir)(
    selfDirW.bits.set,
    selfDirW.bits.way,
    0.U,
    selfDirW.bits.data,
    stamp,
    selfDirW.fire,
    this.clock,
    this.reset
  )
  // dump self tag
  DirectoryLogger(cacheParams.name, TypeId.self_tag)(
    io.tagWReq.bits.set,
    io.tagWReq.bits.way,
    io.tagWReq.bits.tag,
    0.U,
    stamp,
    io.tagWReq.fire,
    this.clock,
    this.reset
  )

  // dump client dir
  DirectoryLogger(cacheParams.name, TypeId.client_dir)(
    io.clientDirWReq.bits.set,
    io.clientDirWReq.bits.way,
    0.U,
    io.clientDirWReq.bits.data,
    stamp,
    io.clientDirWReq.fire,
    this.clock,
    this.reset
  )
  // dump client tag
  DirectoryLogger(cacheParams.name, TypeId.client_tag)(
    io.clientTagWreq.bits.set,
    io.clientTagWreq.bits.way,
    io.clientTagWreq.bits.tag,
    0.U,
    stamp,
    io.clientTagWreq.fire,
    this.clock,
    this.reset
  )

  def client_invalid_way_fn(metaVec: Seq[Vec[ClientDirEntry]], repl: UInt): (Bool, UInt) = {
    val invalid_vec = metaVec.map(states => Cat(states.map(_.state === INVALID)).andR)
    val has_invalid_way = Cat(invalid_vec).orR
    val way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(clientWayBits.W)))
    (has_invalid_way, way)
  }
  // empty function, just for clientDir module instantiation
  def client_self_invalid_way_sel(metaVec: Seq[SelfDirAgeEntry], repl: UInt): (Bool, UInt) = {
    (false.B, 0.U)
  }

  val clientDir = Module(
    new SubDirectory[Vec[ClientDirEntry]](
      wports = mshrsAll,
      sets = clientSets,
      ways = clientWays,
      tagBits = clientTagBits,
      dir_init_fn = () => {
        val init = Wire(Vec(clientBits, new ClientDirEntry))
        init.foreach(_.state := MetaData.INVALID)
        init.foreach(_.alias.foreach(_ := DontCare))
        init
      },
      dir_hit_fn = dirs => Cat(dirs.map(_.state =/= MetaData.INVALID)).orR,
      client_invalidWay_sel = client_invalid_way_fn,
      self_invalidWay_sel = client_self_invalid_way_sel,
      SelfDir_flag = false,
      replacement = "random"
    )
  )

  def selfHitFn(dir: SelfDirEntry): Bool = dir.state =/= MetaData.INVALID
  /*
  def self_invalid_way_sel(metaVec: Seq[SelfDirEntry], repl: UInt): (Bool, UInt) = {
    // 1.try to find a invalid way
    val invalid_vec = metaVec.map(_.state === MetaData.INVALID)
    val has_invalid_way = Cat(invalid_vec).orR
    val invalid_way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // 2.if there is no invalid way, then try to find a TRUNK to replace
    // (we are non-inclusive, if we are trunk, there must be a TIP in our client)
    val trunk_vec = metaVec.map(_.state === MetaData.TRUNK)
    val has_trunk_way = Cat(trunk_vec).orR
    val trunk_way = ParallelPriorityMux(trunk_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    val repl_way_is_trunk = VecInit(metaVec)(repl).state === MetaData.TRUNK
    (
      has_invalid_way || has_trunk_way,
      Mux(has_invalid_way, invalid_way, Mux(repl_way_is_trunk, repl, trunk_way))
      )
  }
   */

  // add L3-replacement
    // do not use "repl" args, which is made by old replacement in SubDirectory
  def self_invalid_way_sel(metaVec: Seq[SelfDirAgeEntry], repl: UInt): (Bool, UInt) = {
    // 1.try to find a invalid way
    val invalid_vec = metaVec.map(_.meta.state === MetaData.INVALID)
    val has_invalid_way = Cat(invalid_vec).orR
    val invalid_way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // 2.if there is no invalid way, then try to find a way with minimum age
    // 3. if there are two or more ways with the same minimum age, chose the way with minimum way_id
    val age_vec = metaVec.map(_.age)
    val min_age = ParallelMin(age_vec)
    val min_age_way = WireInit(invalid_way)
    age_vec.zipWithIndex.reverse.foreach {
      case(m, i) =>
        when(m === min_age) {
          min_age_way := i.U
        }
    }
    (
      has_invalid_way,
      Mux(has_invalid_way, invalid_way, min_age_way)
    )
  }
  // empty function, just for selfDir module instantiation
  def self_client_invalid_way_sel(metaVec: Seq[SelfDirEntry], repl: UInt): (Bool, UInt) = {
    (false.B, 0.U)
  }



  val selfDir = Module(
    new SubDirectoryDoUpdate[SelfDirEntry](
      wports = mshrsAll,
      sets = cacheParams.sets,
      ways = cacheParams.ways,
      tagBits = tagBits,
      dir_init_fn = () => {
        val init = Wire(new SelfDirEntry())
        init := DontCare
        init.state := MetaData.INVALID
        init
      },
      dir_hit_fn = selfHitFn,
      self_invalidWay_sel = self_invalid_way_sel,
      client_invalidWay_sel = self_client_invalid_way_sel,
      SelfDir_flag = true,
      replacement = cacheParams.replacement
    ) with NonInclusiveCacheReplacerUpdate
  )

  // val DL = new DLCounterEntry()
  val binCounterArray_init = 0.U.asTypeOf(Vec(8, new DLCounterEntry()))
  val binCounterArray = WireInit(binCounterArray_init)

  def addrConnect(lset: UInt, ltag: UInt, rset: UInt, rtag: UInt) = {
    assert(lset.getWidth + ltag.getWidth == rset.getWidth + rtag.getWidth)
    val addr = Cat(rtag, rset)
    lset := addr.tail(ltag.getWidth)
    ltag := addr.head(ltag.getWidth)
  }

  val rports = Seq(clientDir.io.read, selfDir.io.read)
  val req = io.read
  rports.foreach { p =>
    p.valid := req.valid
    addrConnect(p.bits.set, p.bits.tag, req.bits.set, req.bits.tag)
    p.bits.replacerInfo := req.bits.replacerInfo
    p.bits.wayMode := req.bits.wayMode
    p.bits.way := req.bits.way
    when(req.fire && req.bits.wayMode){
      assert(req.bits.idOH(1, 0) === "b11".U)
    }
  }

  val clk_div_by_2 = p(HCCacheParamsKey).sramClkDivBy2
  val cycleCnt = Counter(true.B, 2)
  val readyMask = if (clk_div_by_2) cycleCnt._1(0) else true.B
  req.ready := Cat(rports.map(_.ready)).andR && readyMask
  val reqValidReg = RegNext(req.fire, false.B)
  val reqIdOHReg = RegEnable(req.bits.idOH, req.fire) // generate idOH in advance to index MSHRs
  // delay 2 cycles and hold until next valid coming
  val sourceIdReg = RegEnable(RegEnable(req.bits.source, req.fire), reqValidReg)
  val setReg = RegEnable(RegEnable(req.bits.set, req.fire), reqValidReg)
  val channelReg = RegEnable(RegEnable(req.bits.channel, req.fire), reqValidReg)
  val opcodeReg = RegEnable(RegEnable(req.bits.opcode, req.fire), reqValidReg)
  val paramReg = RegEnable(RegEnable(req.bits.param, req.fire), reqValidReg)
  val tagReg = RegEnable(RegEnable(req.bits.tag, req.fire), reqValidReg)
  val tripCountReg = RegEnable(RegEnable(req.bits.tripCount, req.fire), reqValidReg)
  val useCountReg = RegEnable(RegEnable(req.bits.useCount, req.fire), reqValidReg)
  val replacerInfoReg = RegEnable(RegEnable(req.bits.replacerInfo, req.fire), reqValidReg)
  val resp = io.result
  val clientResp = clientDir.io.resp
  val selfResp = selfDir.io.resp
  resp.valid := selfResp.valid
  val valids = Cat(clientResp.valid, selfResp.valid)
  assert(valids.andR || !valids.orR, "valids must be all 1s or 0s")
  resp.bits.idOH := reqIdOHReg
  resp.bits.sourceId := sourceIdReg
  resp.bits.set := setReg
  resp.bits.replacerInfo := replacerInfoReg
  resp.bits.self.hit := selfResp.bits.hit
  resp.bits.self.way := selfResp.bits.way
  resp.bits.self.tag := selfResp.bits.tag
  resp.bits.self.dirty := selfResp.bits.dir.dirty
  resp.bits.self.state := selfResp.bits.dir.state
  resp.bits.self.error := selfResp.bits.error
  resp.bits.self.clientStates := selfResp.bits.dir.clientStates
  resp.bits.self.prefetch.foreach(p => p := selfResp.bits.dir.prefetch.get)
  resp.bits.self.repl_msg := selfResp.bits.repl_msg.getOrElse(0.U.asTypeOf(resp.bits.self.repl_msg))
  resp.bits.self.oldAge := selfResp.bits.old_age.getOrElse(0.U.asTypeOf(resp.bits.self.oldAge))
  resp.bits.clients.way := clientResp.bits.way
  resp.bits.clients.tag := clientResp.bits.tag
  resp.bits.clients.error := Cat(resp.bits.clients.states.map(_.hit)).orR && clientResp.bits.error
  resp.bits.clients.states.zip(clientResp.bits.dir).foreach{
    case (s, dir) =>
      s.state := dir.state
      s.hit := clientResp.bits.hit && dir.state =/= INVALID
      s.alias.foreach(_ := dir.alias.get)
  }
  resp.bits.clients.tag_match := clientResp.bits.hit

  val binNumber = req.bits.tripCount * 4.U + req.bits.useCount
  resp.bits.binCounter.zipWithIndex.foreach {
    case (m, i) =>
      m.L := binCounterArray(i).L
      m.D_L := binCounterArray(i).D_L
      m.L_sum := binCounterArray(1).L +  binCounterArray(2).L + binCounterArray(3).L
  }

  // Self Tag Write
  selfDir.io.tag_w.valid := io.tagWReq.valid
  selfDir.io.tag_w.bits.tag := io.tagWReq.bits.tag
  selfDir.io.tag_w.bits.set := io.tagWReq.bits.set
  selfDir.io.tag_w.bits.way := io.tagWReq.bits.way
  io.tagWReq.ready := selfDir.io.tag_w.ready && readyMask
  // Clients Tag Write
  clientDir.io.tag_w.valid := io.clientTagWreq.valid
  clientDir.io.tag_w.bits.tag := io.clientTagWreq.bits.tag
  clientDir.io.tag_w.bits.set := io.clientTagWreq.bits.set
  clientDir.io.tag_w.bits.way := io.clientTagWreq.bits.way
  io.clientTagWreq.ready := clientDir.io.tag_w.ready && readyMask

  // Self Dir Write
   selfDir.io.dir_w.valid := io.dirWReq.valid
   selfDir.io.dir_w.bits.set := io.dirWReq.bits.set
   selfDir.io.dir_w.bits.way := io.dirWReq.bits.way
   selfDir.io.dir_w.bits.dir := io.dirWReq.bits.data
//  val selfDir_dirW_valid_reg = RegNext(io.dirWReq.valid)
//  val selfDir_dirW_set_reg = RegNext(io.dirWReq.bits.set)
//  val selfDir_dirW_way_reg = RegNext(io.dirWReq.bits.way)
//  val selfDir_dirW_dir_reg = RegNext(io.dirWReq.bits.data)
  // for L3-replacement
//  val new_meta_allways = RegInit(io.dirWReq.bits.meta_allways)
  // when(io.dirWReq.valid) {
  //   new_meta_allways := io.dirWReq.bits.meta_allways
  // }
//  when(io.dirWReq.valid) {  // delay selfDir_dirW for 1 cycle, waiting for new_meta_allways
//     new_meta_allways := io.dirWReq.bits.meta_allways
//    // selfDir_dirW_valid_reg := io.dirWReq.valid
//    // selfDir_dirW_set_reg := io.dirWReq.bits.set
//    // selfDir_dirW_way_reg := io.dirWReq.bits.way
//    // selfDir_dirW_dir_reg := io.dirWReq.bits.data
//  }
//  selfDir.io.dir_w.valid := selfDir_dirW_valid_reg
//  selfDir.io.dir_w.bits.set := selfDir_dirW_set_reg
//  selfDir.io.dir_w.bits.way := selfDir_dirW_way_reg
//  selfDir.io.dir_w.bits.dir := selfDir_dirW_dir_reg

  //val tagWReq_valid_hold = RegInit(false.B)

  // when(io.tagWReq.valid) { tagWReq_valid_hold := tagWReq_valid_hold | true.B }
  /*
  new_meta_allways.zipWithIndex.foreach {
    case(m, i) =>
      when(i.U =/= io.dirWReq.bits.way) {
        // other ways in the same set as the chosen way
        when(io.tagWReq.valid) {
          // the chosen way will be replaced
          m.replAge := Mux(
            m.replAge >= io.dirWReq.bits.oldAge,
            m.replAge - io.dirWReq.bits.oldAge,
            0.U
          )
        }
      }.otherwise {
        // the chosen way
        m.replAge := io.dirWReq.bits.data.replAge
      }
  }
  */
//  when(io.tagWReq.valid) {
//    new_meta_allways.zipWithIndex.foreach {
//      case (m, i) =>
//        when(i.U =/= io.dirWReq.bits.way) {
//          m.replAge := Mux(
//            m.replAge >= io.dirWReq.bits.oldAge,
//            m.replAge - io.dirWReq.bits.oldAge,
//            0.U
//          )
//        }.otherwise {
//            m.replAge := io.dirWReq.bits.data.replAge
//          }
//
//    }
//  }
//
//  dontTouch(new_meta_allways)
//  selfDir.io.dir_w.bits.meta_allways.zipWithIndex.foreach {
//    // final correct meta-allways
//    case(m, i) =>
//      m := Mux(
//        i.U === selfDir.io.dir_w.bits.way,
//        selfDir.io.dir_w.bits.dir,
//        //io.dirWReq.bits.meta_allways(i)
//        new_meta_allways(i)
//      )
//  }
  io.dirWReq.ready := selfDir.io.dir_w.ready && readyMask

  // Self Age Write
  selfDir.io.age_w.valid := io.ageWReq.valid
  selfDir.io.age_w.bits.set := io.ageWReq.bits.set
  selfDir.io.age_w.bits.way := io.ageWReq.bits.way
  selfDir.io.age_w.bits.repl_msg := io.ageWReq.bits.repl_msg
  io.ageWReq.ready := selfDir.io.age_w.ready && readyMask

  // Clients Dir Write
  clientDir.io.dir_w.valid := io.clientDirWReq.valid
  clientDir.io.dir_w.bits.set := io.clientDirWReq.bits.set
  clientDir.io.dir_w.bits.way := io.clientDirWReq.bits.way
  clientDir.io.dir_w.bits.dir := io.clientDirWReq.bits.data
  io.clientDirWReq.ready := clientDir.io.dir_w.ready && readyMask

  // dontcare Clients Dir's age_w port
  clientDir.io.age_w.valid := false.B
  clientDir.io.age_w.bits.set := 0.U
  clientDir.io.age_w.bits.way := 0.U
  clientDir.io.age_w.bits.repl_msg := 0.U.asTypeOf(clientDir.io.age_w.bits.repl_msg)

  // Bin Counter Write
  when(io.binWReq.valid){
    binCounterArray := io.binWReq.bits.DLCounter
  }
  io.binWReq.ready := selfDir.io.tag_w.ready && readyMask

  val wayCnt = RegInit(Vec(cacheParams.ways, UInt(30.W)), 0.U.asTypeOf(Vec(cacheParams.ways, UInt(30.W))))
  when(RegNext(io.result.valid)) {
    wayCnt(io.result.bits.self.way) := wayCnt(io.result.bits.self.way) + 1.U
  }
  val replDB = ChiselDB.createTable("l3_repl", new replBundle(), basicDB = true)
  val replInfo = Wire(new replBundle())
  replInfo.channel := channelReg
  replInfo.opcode := opcodeReg
  replInfo.param := paramReg
  replInfo.tag := tagReg
  replInfo.sset := setReg
  replInfo.bank := io.sliceId
  replInfo.tripCount := tripCountReg
  replInfo.useCount := useCountReg
  replInfo.L.zipWithIndex.foreach {
    case(m, i) =>
      m := resp.bits.binCounter(i).L
  }
  replInfo.D_L.zipWithIndex.foreach {
    case (m, i) =>
      m := resp.bits.binCounter(i).D_L
  }
  replInfo.L_sum := resp.bits.binCounter(1).L_sum
  replInfo.selectedWay := io.result.bits.self.way
  replInfo.hit := Mux(selfResp.bits.hit, 1.U, 0.U)
  replInfo.hitway := selfResp.bits.hitway.getOrElse(0.U.asTypeOf(replInfo.hitway))
  replInfo.age.zipWithIndex.foreach {
    case(m, i) =>
      m := resp.bits.self.repl_msg(i).age
  }
  replInfo.wayCnt := wayCnt
  replDB.log(replInfo, RegNext(io.result.valid), s"L3_repl_${sliceId}", clock, reset)

  assert(dirReadPorts == 1)
  val req_r = RegEnable(req.bits, req.fire)
  XSPerfAccumulate(cacheParams, "selfdir_A_req", req_r.replacerInfo.channel(0) && resp.valid)
  XSPerfAccumulate(cacheParams, "selfdir_A_hit", req_r.replacerInfo.channel(0) && resp.valid && resp.bits.self.hit)
  XSPerfAccumulate(cacheParams, "selfdir_B_req", req_r.replacerInfo.channel(1) && resp.valid)
  XSPerfAccumulate(cacheParams, "selfdir_B_hit", req_r.replacerInfo.channel(1) && resp.valid && resp.bits.self.hit)
  XSPerfAccumulate(cacheParams, "selfdir_C_req", req_r.replacerInfo.channel(2) && resp.valid)
  XSPerfAccumulate(cacheParams, "selfdir_C_hit", req_r.replacerInfo.channel(2) && resp.valid && resp.bits.self.hit)

  XSPerfAccumulate(cacheParams, "selfdir_dirty", resp.valid && resp.bits.self.dirty)
  XSPerfAccumulate(cacheParams, "selfdir_TIP", resp.valid && resp.bits.self.state === TIP)
  XSPerfAccumulate(cacheParams, "selfdir_BRANCH", resp.valid && resp.bits.self.state === BRANCH)
  XSPerfAccumulate(cacheParams, "selfdir_TRUNK", resp.valid && resp.bits.self.state === TRUNK)
  XSPerfAccumulate(cacheParams, "selfdir_INVALID", resp.valid && resp.bits.self.state === INVALID)
  //val perfinfo = IO(new Bundle(){
  //  val perfEvents = Output(new PerfEventsBundle(numPCntHcDir))
  //})
  val perfinfo = IO(Output(Vec(numPCntHcDir, (UInt(6.W)))))
  val perfEvents = Seq(
    ("selfdir_A_req     ", req_r.replacerInfo.channel(0) && resp.valid                      ),
    ("selfdir_A_hit     ", req_r.replacerInfo.channel(0) && resp.valid && resp.bits.self.hit),
    ("selfdir_B_req     ", req_r.replacerInfo.channel(1) && resp.valid                      ),
    ("selfdir_B_hit     ", req_r.replacerInfo.channel(1) && resp.valid && resp.bits.self.hit),
    ("selfdir_C_req     ", req_r.replacerInfo.channel(2) && resp.valid                      ),
    ("selfdir_C_hit     ", req_r.replacerInfo.channel(2) && resp.valid && resp.bits.self.hit),
    ("selfdir_dirty     ", resp.valid && resp.bits.self.dirty                               ),
    ("selfdir_TIP       ", resp.valid && resp.bits.self.state === TIP                       ),
    ("selfdir_BRANCH    ", resp.valid && resp.bits.self.state === BRANCH                    ),
    ("selfdir_TRUNK     ", resp.valid && resp.bits.self.state === TRUNK                     ),
    ("selfdir_INVALID   ", resp.valid && resp.bits.self.state === INVALID                   ),
  )

  for (((perf_out,(perf_name,perf)),i) <- perfinfo.zip(perfEvents).zipWithIndex) {
    perf_out := RegNext(perf)
  }
}
