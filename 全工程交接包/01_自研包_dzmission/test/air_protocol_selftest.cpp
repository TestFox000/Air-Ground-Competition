// =============================================================================
// air_protocol_selftest —— C++ 协议层与 Python 参考实现的逐字节对撞
// -----------------------------------------------------------------------------
// 这个程序刻意**不依赖 ROS**（air_protocol.cpp 是纯逻辑，也不依赖 ROS），
// 所以可以用最朴素的方式编译：
//
//   g++ -std=c++17 -I include src/air_protocol.cpp test/air_protocol_selftest.cpp
//       -o /tmp/air_selftest && /tmp/air_selftest
//
//   ★ 注意：注释行末尾**不要**写反斜杠 —— C++ 里行注释以 \ 结尾会续行，
//     把下一行也吞进注释，编译器报 warning: multi-line comment [-Wcomment]。
//
// 输出格式：每行 "KEY <hex>"，由 test/air_protocol_cpp_check.py 读入并与
// Python 参考实现（air_protocol.py）的输出逐字节比对。
//
// 覆盖范围：
//   · CRC 标准向量
//   · 帧编解码往返 + 组帧字节
//   · 帧同步状态机的容错（A5 A5 C3 / 截断 / LEN 非法 / 垃圾前缀）
//   · 全部 14 种消息载荷
//   · 边界值（0 / 0xFF / 负数 / 最大格数）
// =============================================================================

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "dzmission/air_protocol.hpp"

using namespace dzmission::air;

static int g_fail = 0;

static void hexOut(const char *key, const std::vector<uint8_t> &v)
{
  std::string s;
  char buf[4];
  for (uint8_t b : v)
  {
    std::snprintf(buf, sizeof(buf), "%02x", b);
    s += buf;
  }
  std::printf("%s %s\n", key, s.c_str());
}

static void check(const char *name, bool ok, const std::string &extra = "")
{
  std::printf("%s %s%s\n", ok ? "[ OK ]" : "[FAIL]", name,
              ok ? "" : ("  -> " + extra).c_str());
  if (!ok)
  {
    ++g_fail;
  }
}

int main()
{
  std::printf("# air_protocol C++ selftest\n");

  // ---------------------------------------------------------------------------
  // 1. CRC 标准向量
  // ---------------------------------------------------------------------------
  {
    const char *s = "123456789";
    const uint16_t c = crc16CcittFalse(reinterpret_cast<const uint8_t *>(s), 9);
    check("CRC '123456789' == 0x29B1", c == 0x29B1,
          "got " + std::to_string(c));
    // 空输入应为 init 值
    check("CRC empty == 0xFFFF", crc16CcittFalse(nullptr, 0) == 0xFFFF);
    // 单字节 0x00
    const uint8_t z = 0x00;
    check("CRC {0x00} == 0xE1F0", crc16CcittFalse(&z, 1) == 0xE1F0);
  }

  // ---------------------------------------------------------------------------
  // 2. 组帧 / 解帧
  // ---------------------------------------------------------------------------
  {
    const auto f = encodeFrame(kAck, 7, enc::ack(7, 0, 0));
    check("ACK frame len == 10+8+2 == 20", f.size() == 20,
          "got " + std::to_string(f.size()));
    check("ACK frame SOF", f[0] == 0xA5 && f[1] == 0xC3);
    check("ACK frame VER", f[2] == kVer);
    check("ACK frame TYP", f[3] == kAck);
    hexOut("fr_ack", f);

    const auto d = decodeFrame(f.data(), f.size());
    check("decode ACK ok", d.ok, d.error);
    check("decode ACK typ/seq", d.typ == kAck && d.seq == 7);
    check("decode ACK payload == 8", d.payload.size() == 8,
          "got " + std::to_string(d.payload.size()));
  }

  // 空载荷帧
  {
    const auto f = encodeFrame(kHeartbeat, 0, nullptr, 0);
    check("empty payload frame len == 12", f.size() == 12,
          "got " + std::to_string(f.size()));
    const auto d = decodeFrame(f.data(), f.size());
    check("decode empty payload ok", d.ok && d.payload.empty(), d.error);
    hexOut("fr_empty", f);
  }

  // 最大载荷
  {
    std::vector<uint8_t> big(kMaxPayload, 0xA5);
    const auto f = encodeFrame(kRoute, 0xFFFFFFFFu, big);
    check("max payload frame len == 1036", f.size() == kMaxFrameLen,
          "got " + std::to_string(f.size()));
    const auto d = decodeFrame(f.data(), f.size());
    check("decode max payload ok", d.ok && d.payload.size() == kMaxPayload,
          d.error);
  }

  // 超长载荷必须拒绝
  {
    std::vector<uint8_t> over(kMaxPayload + 1, 0);
    const auto f = encodeFrame(kRoute, 1, over);
    check("payload > 1024 rejected (empty)", f.empty(),
          "got " + std::to_string(f.size()));
  }

  // 损坏帧必须被拒
  {
    auto good = encodeFrame(kAck, 1, enc::ack(1, 0, 0));

    auto bad_crc = good;
    bad_crc.back() ^= 0xFF;
    check("crc error rejected", !decodeFrame(bad_crc.data(), bad_crc.size()).ok);

    auto bad_magic = good;
    bad_magic[0] = 0x00;
    check("bad magic rejected",
          !decodeFrame(bad_magic.data(), bad_magic.size()).ok);

    auto bad_len = good;
    bad_len[8] = 0xFF;   // LEN 低字节改大
    bad_len[9] = 0x7F;
    check("len mismatch rejected",
          !decodeFrame(bad_len.data(), bad_len.size()).ok);

    check("too short rejected", !decodeFrame(good.data(), 5).ok);

    auto bad_ver = good;
    bad_ver[2] = 0x03;
    check("version mismatch rejected",
          !decodeFrame(bad_ver.data(), bad_ver.size()).ok);
  }

  // ---------------------------------------------------------------------------
  // 3. 帧同步状态机容错
  // ---------------------------------------------------------------------------
  {
    const auto f = encodeFrame(kHeartbeat, 42, enc::heartbeat(1000));
    std::vector<Decoded> got;
    auto cb = [&](const Decoded &d) { got.push_back(d); };

    // 干净输入
    {
      FrameSync fs;
      got.clear();
      fs.feed(f.data(), f.size(), cb);
      check("sync: clean frame", got.size() == 1 && got[0].seq == 42,
            "got " + std::to_string(got.size()));
    }
    // ★ A5 A5 C3 ... 第二个 A5 必须当新起点
    {
      FrameSync fs;
      got.clear();
      std::vector<uint8_t> junk = {0xA5, 0xA5};
      fs.feed(junk.data(), junk.size(), cb);
      fs.feed(f.data(), f.size(), cb);
      check("sync: A5 A5 then valid frame", got.size() == 1,
            "got " + std::to_string(got.size()));
    }
    // 垃圾前缀
    {
      FrameSync fs;
      got.clear();
      std::vector<uint8_t> junk = {0x00, 0xFF, 0x12, 0xA5, 0x00};
      fs.feed(junk.data(), junk.size(), cb);
      fs.feed(f.data(), f.size(), cb);
      check("sync: garbage prefix", got.size() == 1,
            "got " + std::to_string(got.size()));
    }
    // 一帧被拆成多次 feed
    {
      FrameSync fs;
      got.clear();
      for (size_t i = 0; i < f.size(); ++i)
      {
        fs.feed(&f[i], 1, cb);
      }
      check("sync: byte-by-byte", got.size() == 1,
            "got " + std::to_string(got.size()));
    }
    // 两帧连发
    {
      FrameSync fs;
      got.clear();
      const auto g = encodeFrame(kAck, 43, enc::ack(43, 0, 0));
      std::vector<uint8_t> both = f;
      both.insert(both.end(), g.begin(), g.end());
      fs.feed(both.data(), both.size(), cb);
      check("sync: two frames in one feed", got.size() == 2,
            "got " + std::to_string(got.size()));
    }
    // LEN 非法后仍能恢复
    {
      FrameSync fs;
      got.clear();
      std::vector<uint8_t> evil = {0xA5, 0xC3, 0x02, 0x01,
                                   0x00, 0x00, 0x00, 0x00,
                                   0xFF, 0xFF};   // LEN = 0xFFFF > 1024
      fs.feed(evil.data(), evil.size(), cb);
      check("sync: illegal LEN yields no frame", got.empty(),
            "got " + std::to_string(got.size()));
      got.clear();
      fs.feed(f.data(), f.size(), cb);
      check("sync: recovers after illegal LEN", got.size() == 1,
            "got " + std::to_string(got.size()));
    }
  }

  // ---------------------------------------------------------------------------
  // 4. cell 位域
  // ---------------------------------------------------------------------------
  {
    uint8_t o = 0;
    uint8_t c = 0;
    uint8_t a = 0;
    enc::cellDecode(enc::cellEncode(kOccMine, 15, 0), o, c, a);
    check("cell roundtrip MINE/15/0", o == kOccMine && c == 15 && a == 0);
    enc::cellDecode(enc::cellEncode(kOccGateUnk, 0, 15), o, c, a);
    check("cell roundtrip GATE_UNK/0/15",
          o == kOccGateUnk && c == 0 && a == 15);
    check("enc::cellEncode(MINE,15,0) == 0x007B",
          enc::cellEncode(kOccMine, 15, 0) == 0x007B,
          "got " + std::to_string(enc::cellEncode(kOccMine, 15, 0)));
  }

  // ---------------------------------------------------------------------------
  // 5. 全部消息载荷
  // ---------------------------------------------------------------------------
  {
    hexOut("m_heartbeat", enc::heartbeat(1000, 2, 95, 0x05));
    hexOut("m_team_info", enc::teamInfo(1, 14, 11, 35));
    hexOut("m_field_fixed", enc::fieldFixed());
    hexOut("m_field_fixed2", enc::fieldFixed(5, 3, 13, 10, 2, 4, 7, 9));

    // MAP_FULL：构造一个有内容的图，才能验出位域
    {
      std::vector<uint16_t> cells(14 * 11, 0);
      // ★★ 关键：先把**全部**格子编成"未知格"的规范编码 ★★
      //   cellEncode(kOccUnknown=0, conf=15, age=0) == 0x0078。
      //   若只填裸 0（不编码），载荷里会出现 0x0000 —— 那不是合法的
      //   "未知格"编码（conf/age 位全为 0，语义变成"置信度 0、最旧"），
      //   既不符合协议，也让本用例失去意义（测不出位域是否正确）。
      //
      //   ★ 这里曾漏掉这一步，golden 对撞报：
      //       m_map_full 首个差异: 第 10 字节 期望 78 实际 00
      for (size_t i = 0; i < cells.size(); ++i)
      {
        cells[i] = enc::cellEncode(kOccUnknown, 15, 0);
      }
      // K 列（col 10）除门 K4(row3)/K9(row8) 外都是 GATE_UNK
      for (int r = 0; r < 11; ++r)
      {
        cells[r * 14 + 10] = enc::cellEncode(kOccGateUnk, 15, 0);
      }
      cells[3 * 14 + 10] = enc::cellEncode(kOccGateOpen, 15, 0);
      cells[8 * 14 + 10] = enc::cellEncode(kOccGateOpen, 15, 0);
      // 水平隔离线 D..N 在 row5
      for (int c = 3; c <= 13; ++c)
      {
        cells[5 * 14 + c] = enc::cellEncode(kOccBlocked, 15, 0);
      }
      // 两颗雷 + 一个标靶
      cells[2 * 14 + 3] = enc::cellEncode(kOccMine, 15, 0);
      cells[4 * 14 + 6] = enc::cellEncode(kOccMine, 15, 0);
      cells[6 * 14 + 6] = enc::cellEncode(kOccTarget, 15, 0);
      const auto mf = enc::mapFull(5, 1000, cells);
      check("MAP_FULL len == 318", mf.size() == 318,
            "got " + std::to_string(mf.size()));
      hexOut("m_map_full", mf);
    }

    hexOut("m_mine_list",
           enc::mineList(1000, {{3, 2, 90}, {6, 4, 88}, {11, 1, 85},
                                {4, 7, 92}, {9, 8, 80}, {0, 3, 95},
                                {8, 9, 70}, {5, 2, 88}}));
    hexOut("m_mine_list_empty", enc::mineList(1000, {}));

    // BRIDGE_GATE：两条消息，每条一个门
    hexOut("m_gate_a", enc::bridgeGate(1, 3, 100, 0x01));
    hexOut("m_gate_b", enc::bridgeGate(2, 8, 100, 0x01));
    hexOut("m_gate_lo", enc::bridgeGate(1, 0, 0, 0x00));

    hexOut("m_target_list",
           enc::targetList(1000, {{6, 4, 2, 90}, {6, 5, 2, 90}}));
    hexOut("m_target_list_noface",
           enc::targetList(1000, {{6, 4, 0, 0}, {6, 5, 0, 0}}));

    hexOut("m_obstacle_list",
           enc::obstacleList({{1, 1, 1}, {7, 5, 2}, {13, 10, 1}, {2, 6, 2},
                              {10, 3, 1}, {6, 8, 2}, {0, 9, 1}, {9, 4, 2}}));
    hexOut("m_obstacle_list_empty", enc::obstacleList({}));

    // TASK_CMD（★ 以参考实现为准：STOP 是 8 不是 1）
    hexOut("m_cmd_nop", enc::taskCmd(kCmdNop, 0));
    hexOut("m_cmd_stop", enc::taskCmd(kCmdStop, 7));
    hexOut("m_cmd_nav", enc::navToCell(3, 12, 10, 60));
    hexOut("m_cmd_bridge", enc::bridgeCross(4, 1));
    hexOut("m_cmd_engage", enc::engage(5, 6, 4, 2));
    hexOut("m_cmd_mine_all", enc::taskCmd(kCmdMineClearAll, 6));
    hexOut("m_cmd_return", enc::taskCmd(kCmdReturnBase, 8));

    hexOut("m_route", enc::route(7, 2, {{12, 10, 0}, {10, 8, 1}, {13, 5, 0}}));

    hexOut("m_veh_status",
           enc::vehStatus(90000, 245, 175, -450, 30, -12, kStMineClear,
                          kFaultNone, 3, 1, 100, 7, 82, 95, 8, 0x01, 7, 5,
                          12345));
    hexOut("m_veh_status_neg",
           enc::vehStatus(0, -1, -1, -1800, -30, 0, kStIdle, kFaultBlocked,
                          0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0));

    hexOut("m_event", enc::event(1000, kEvGateConfirmed, 0, 1, 0));
    hexOut("m_event_mine", enc::event(2000, kEvMineCleared, 3, 2, 0));
    hexOut("m_event_invalid", enc::event(3000, kEvGateInvalid, 0, 0, 0));

    hexOut("m_ack", enc::ack(7, 0, 0));
    hexOut("m_ack_err", enc::ack(0xFFFFFFFFu, 3, 9));
  }

  // ---------------------------------------------------------------------------
  // 6. 长度自检（防止手改字段后长度漂移）
  // ---------------------------------------------------------------------------
  {
    struct
    {
      const char *name;
      size_t got;
      size_t want;
    } lens[] = {
        {"HEARTBEAT", enc::heartbeat(1).size(), 8},
        {"TEAM_INFO", enc::teamInfo(1).size(), 4},
        {"FIELD_FIXED", enc::fieldFixed().size(), 8},
        {"BRIDGE_GATE", enc::bridgeGate(1, 3).size(), 4},
        {"TASK_CMD_NOP", enc::taskCmd(kCmdNop, 0).size(), 2},
        {"NAV_TO_CELL", enc::navToCell(1, 2, 3).size(), 5},
        {"BRIDGE_CROSS", enc::bridgeCross(1, 1).size(), 3},
        {"ENGAGE", enc::engage(1, 2, 3).size(), 5},
        {"VEH_STATUS", enc::vehStatus(0, 0, 0, 0, 0, 0, 0, 0).size(), 32},
        {"EVENT", enc::event(0, 0).size(), 8},
        {"ACK", enc::ack(0).size(), 8},
        {"MINE_LIST(8)", enc::mineList(0, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0},
                                           {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
                                           {0, 0, 0}, {0, 0, 0}}).size(), 29},
        {"OBSTACLE_LIST(8)", enc::obstacleList({{0, 0, 0}, {0, 0, 0}, {0, 0, 0},
                                                {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
                                                {0, 0, 0}, {0, 0, 0}}).size(), 25},
        {"TARGET_LIST(2)", enc::targetList(0, {{0, 0, 0, 0}, {0, 0, 0, 0}}).size(), 13},
    };
    for (const auto &l : lens)
    {
      check(("len " + std::string(l.name)).c_str(), l.got == l.want,
            "got " + std::to_string(l.got) + " want " + std::to_string(l.want));
    }
  }

  std::printf("# failures: %d\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
