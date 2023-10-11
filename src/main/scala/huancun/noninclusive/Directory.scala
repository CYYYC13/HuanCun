package huancun.noninclusive

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink.TLMessages
import huancun.MetaData._
import huancun._
import huancun.debug.{DirectoryLogger, TypeId}
import huancun.utils._
import utility.{ParallelMax, ParallelMin, ParallelPriorityMux}

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

class DLCounterEntry(implicit p: Parameters) extends HuanCunBundle {
  val D_L = UInt(10.W)
  val L = UInt(10.W)
  val L_sum = UInt(10.W)
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
  // for L3-replacement
  // bin number - (tripCount, useCount)
  // 0-(0,0), 1-(0,1), 2-(0,2), 3-(0,3),
  // 4-(1,0), 5-(1,1), 6-(1,2), 7-(1,3)
  val binNumber = UInt(3.W)
  val replAge = UInt(2.W)
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
  val meta_allways = Vec(cacheParams.ways, new SelfDirEntry())
  val oldAge = UInt(2.W) // save old age to directory
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
  val binCounter = new DLCounterEntry
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
  val meta_allways = Vec(cacheParams.ways, new SelfDirEntry)
  val oldAge = UInt(2.W)
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
  val DLCounter = new DLCounterEntry
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
  val result = ValidIO(new DirResult)
  val dirWReq = Flipped(DecoupledIO(new SelfDirWrite))
  val tagWReq = Flipped(DecoupledIO(new SelfTagWrite))
  val clientDirWReq = Flipped(DecoupledIO(new ClientDirWrite))
  val clientTagWreq = Flipped(DecoupledIO(new ClientTagWrite))
  val binWReq = Flipped(DecoupledIO(new BinCounterWrite))
}

class Directory(implicit p: Parameters)
    extends BaseDirectory[DirResult, SelfDirWrite, SelfTagWrite]
    with HasClientInfo {
  val io = IO(new DirectoryIO())

  val stamp = GTimer()
  val selfDirW = io.dirWReq
  // dump self dir
  DirectoryLogger(cacheParams.name, TypeId.self_dir)(
    selfDirW.bits.set,
    selfDirW.bits.way,
    0.U,
    selfDirW.bits.data,
    stamp,
    selfDirW.fire(),
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
    io.tagWReq.fire(),
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
    io.clientDirWReq.fire(),
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
    io.clientTagWreq.fire(),
    this.clock,
    this.reset
  )

  def client_invalid_way_fn(metaVec: Seq[Vec[ClientDirEntry]], repl: UInt): (Bool, UInt) = {
    val invalid_vec = metaVec.map(states => Cat(states.map(_.state === INVALID)).andR())
    val has_invalid_way = Cat(invalid_vec).orR()
    val way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(clientWayBits.W)))
    (has_invalid_way, way)
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
      invalid_way_sel = client_invalid_way_fn,
      meta_allways_flag = false,
      replacement = "random"
    )
  )

  def selfHitFn(dir: SelfDirEntry): Bool = dir.state =/= MetaData.INVALID
  /*
  def self_invalid_way_sel(metaVec: Seq[SelfDirEntry], repl: UInt): (Bool, UInt) = {
    // 1.try to find a invalid way
    val invalid_vec = metaVec.map(_.state === MetaData.INVALID)
    val has_invalid_way = Cat(invalid_vec).orR()
    val invalid_way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // 2.if there is no invalid way, then try to find a TRUNK to replace
    // (we are non-inclusive, if we are trunk, there must be a TIP in our client)
    val trunk_vec = metaVec.map(_.state === MetaData.TRUNK)
    val has_trunk_way = Cat(trunk_vec).orR()
    val trunk_way = ParallelPriorityMux(trunk_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    val repl_way_is_trunk = VecInit(metaVec)(repl).state === MetaData.TRUNK
    (
      has_invalid_way || has_trunk_way,
      Mux(has_invalid_way, invalid_way, Mux(repl_way_is_trunk, repl, trunk_way))
      )
  }
   */

  // add L3-replacement
    // do not use repl, which is made by old replacement in SubDirectory
  def self_invalid_way_sel(metaVec: Seq[SelfDirEntry], repl: UInt): (Bool, UInt) = {
    // 1.try to find a invalid way
    val invalid_vec = metaVec.map(_.state === MetaData.INVALID)
    val has_invalid_way = Cat(invalid_vec).orR()
    val invalid_way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    // 2.if there is no invalid way, then try to find a way with minimum age
    // 3. if there are two or more ways with the same minimum age, chose the way with minimum way_id
    val age_vec = metaVec.map(_.replAge)
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
      self_invalid_way_sel,
      meta_allways_flag = true,
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
    when(req.fire() && req.bits.wayMode){
      assert(req.bits.idOH(1, 0) === "b11".U)
    }
  }

  val clk_div_by_2 = p(HCCacheParamsKey).sramClkDivBy2
  val cycleCnt = Counter(true.B, 2)
  val readyMask = if (clk_div_by_2) cycleCnt._1(0) else true.B
  req.ready := Cat(rports.map(_.ready)).andR() && readyMask
  val reqValidReg = RegNext(req.fire(), false.B)
  val reqIdOHReg = RegEnable(req.bits.idOH, req.fire()) // generate idOH in advance to index MSHRs
  val sourceIdReg = RegEnable(RegEnable(req.bits.source, req.fire()), reqValidReg)
  val setReg = RegEnable(RegEnable(req.bits.set, req.fire()), reqValidReg)
  val replacerInfoReg = RegEnable(RegEnable(req.bits.replacerInfo, req.fire()), reqValidReg)
  val resp = io.result
  val clientResp = clientDir.io.resp
  val selfResp = selfDir.io.resp
  resp.valid := selfResp.valid
  val valids = Cat(clientResp.valid, selfResp.valid)
  assert(valids.andR() || !valids.orR(), "valids must be all 1s or 0s")
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
  resp.bits.self.replAge := selfResp.bits.dir.replAge
  resp.bits.self.binNumber := selfResp.bits.dir.binNumber
  resp.bits.self.meta_allways := selfResp.bits.meta_allways.getOrElse(0.U.asTypeOf(resp.bits.self.meta_allways))
  resp.bits.self.oldAge := selfResp.bits.dir.replAge
  resp.bits.clients.way := clientResp.bits.way
  resp.bits.clients.tag := clientResp.bits.tag
  resp.bits.clients.error := Cat(resp.bits.clients.states.map(_.hit)).orR() && clientResp.bits.error
  resp.bits.clients.states.zip(clientResp.bits.dir).foreach{
    case (s, dir) =>
      s.state := dir.state
      s.hit := clientResp.bits.hit && dir.state =/= INVALID
      s.alias.foreach(_ := dir.alias.get)
  }
  resp.bits.clients.tag_match := clientResp.bits.hit

  val binNumber = req.bits.tripCount * 4.U + req.bits.useCount
  resp.bits.binCounter.L := binCounterArray(binNumber).L
  resp.bits.binCounter.D_L := binCounterArray(binNumber).L
  resp.bits.binCounter.L_sum := binCounterArray(1).L +  binCounterArray(2).L + binCounterArray(3).L

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
  // for L3-replacement
  val new_meta_allways = RegInit(io.dirWReq.bits.meta_allways)
  // val tagWReq_valid_hold = RegInit(false.B)
  // when(io.tagWReq.valid) { tagWReq_valid_hold := tagWReq_valid_hold | true.B }
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
  selfDir.io.dir_w.bits.meta_allways.zipWithIndex.foreach {
    // final correct meta-allways
    case(m, i) =>
      m := Mux(
        i.U === io.dirWReq.bits.way,
        io.dirWReq.bits.data,
        io.dirWReq.bits.meta_allways(i)
      )
  }
  io.dirWReq.ready := selfDir.io.dir_w.ready && readyMask
  // Clients Dir Write
  clientDir.io.dir_w.valid := io.clientDirWReq.valid
  clientDir.io.dir_w.bits.set := io.clientDirWReq.bits.set
  clientDir.io.dir_w.bits.way := io.clientDirWReq.bits.way
  clientDir.io.dir_w.bits.dir := io.clientDirWReq.bits.data
  clientDir.io.dir_w.bits.meta_allways := 0.U.asTypeOf(clientDir.io.dir_w.bits.meta_allways)  // DontCare
  io.clientDirWReq.ready := clientDir.io.dir_w.ready && readyMask

  // Bin Counter Write
  when(io.binWReq.valid){
    binCounterArray(io.binWReq.bits.binNumber) := io.binWReq.bits.DLCounter
  }
  io.binWReq.ready := selfDir.io.tag_w.ready && readyMask

  assert(dirReadPorts == 1)
  val req_r = RegEnable(req.bits, req.fire())
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
