#ifndef DZMISSION_AIR_PROTOCOL_HPP
#define DZMISSION_AIR_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace dzmission
{
namespace air
{

// =============================================================================
// 空地协同协议 v2 帧层与载荷层（纯逻辑，无 ROS / 无 IO，可独立单测）
// -----------------------------------------------------------------------------
// ★ 本文件的布局必须与 test/air_protocol.py **逐字节一致**。
//   无人机端已按该参考实现完成字节级实现并通过 37 条双向对撞，
//   所以任何偏差都会导致跨端不通。
//
// 帧结构（共 10 + LEN + 2 字节）：
//   偏移  长度  字段
//    0     2    SOF        固定 A5 C3
//    2     1    VER        0x02
//    3     1    TYP        消息类型
//    4     4    SEQ        uint32 小端
//    8     2    LEN        uint16 小端，payload 长度，**上限 1024**
//   10     N    PAYLOAD
//  10+N   2    CRC16      CRC-16/CCITT-FALSE，覆盖 VER..PAYLOAD（不含 SOF 与 CRC 自身）
//
// 与旧文档的差异（已在无人机端审计中确认）：
//   · 旧文档写 LEN ≤ 256 / 最大帧 272 —— **错**，实现是 1024 / 1036
//     MAP_FULL 载荷 318 字节，装不进 256/272
//   · 旧文档的 TASK_CMD 码表与实现完全不同（0x01 在旧版是 STOP，
//     实现里是 NAV_TO_CELL）—— 属安全隐患级差异，本文件以实现为准
// =============================================================================

// ---- 帧常量 ----
constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0xC3;
constexpr uint8_t kVer = 0x02;
constexpr size_t kHeaderLen = 10;      // SOF(2)+VER(1)+TYP(1)+SEQ(4)+LEN(2)
constexpr size_t kCrcLen = 2;
constexpr size_t kMaxPayload = 1024;
constexpr size_t kMaxFrameLen = kHeaderLen + kMaxPayload + kCrcLen;   // 1036

// ---- 消息类型（与 air_protocol.py TYPE_NAMES 一致）----
enum Typ : uint8_t
{
  kHeartbeat = 0x01,
  kTeamInfo = 0x10,
  kFieldFixed = 0x11,
  kMapFull = 0x12,
  kMapDelta = 0x13,      // ★ 保留不使用：payload 有缺陷（见 §2），只发 MAP_FULL
  kMineList = 0x14,
  kBridgeGate = 0x15,
  kTargetList = 0x16,
  kObstacleList = 0x17,
  kTaskCmd = 0x20,
  kRoute = 0x21,
  kVehStatus = 0x30,
  kEvent = 0x31,
  kAck = 0x7F,
};

// ---- 占用类型（MAP_FULL 的 occupancy 3 bit）----
enum Occ : uint8_t
{
  kOccUnknown = 0,
  kOccFree = 1,
  kOccBlocked = 2,
  kOccMine = 3,
  kOccTarget = 4,
  kOccGateUnk = 5,
  kOccGateOpen = 6,
};

// ---- TASK_CMD 指令码（★ 以参考实现为准，旧文档 §4.12 整节作废）----
enum TaskCmd : uint8_t
{
  kCmdNop = 0,
  kCmdNavToCell = 1,
  kCmdNavToPose = 2,
  kCmdMineClearAll = 3,
  kCmdMineClearOne = 4,
  kCmdBridgeCross = 5,
  kCmdEngage = 6,
  kCmdHold = 7,
  kCmdStop = 8,           // ★ 注意：是 8，不是旧文档写的 1
  kCmdReturnBase = 9,
  kCmdSafeRetreat = 10,
};

// ---- EVENT 事件码 ----
enum EventCode : uint8_t
{
  kEvNop = 0,
  kEvMineCleared = 1,
  kEvBridgePassed = 2,
  kEvTargetHit = 3,
  kEvTargetDestroyed = 4,
  kEvArrived = 5,
  kEvStuckEscaped = 6,
  kEvSafeRetreatStart = 7,
  kEvMapStale = 8,
  kEvFriendlyDetected = 9,
  // 以下为车端新增（旧文档缺失，无人机端审计指出）
  kEvGateInvalid = 10,
  kEvGateConfirmed = 11,
  kEvNeedGate = 12,
};

// ---- 车端任务状态 / 故障 ----
enum TaskState : uint8_t
{
  kStIdle = 0, kStNav = 1, kStMineClear = 2, kStBridgeCross = 3,
  kStEngage = 4, kStReturn = 5, kStEstop = 6, kStDone = 7,
};

enum Fault : uint8_t
{
  kFaultNone = 0, kFaultNoAmmo = 1, kFaultLowBattery = 2,
  kFaultNoLocalization = 3, kFaultBlocked = 4, kFaultRefereeLinkLost = 5,
  kFaultCmdLinkLost = 6, kFaultStuckEscalated = 7, kFaultStuckUnrecoverable = 8,
};

// ---- 面向对象的颜色 ----
enum ObjColor : uint8_t { kColorUnknown = 0, kColorRed = 1, kColorBlue = 2 };

// =============================================================================
// CRC-16/CCITT-FALSE：poly 0x1021，init 0xFFFF，不反射，不异或输出
//   标准校验串 "123456789" -> 0x29B1
// =============================================================================
uint16_t crc16CcittFalse(const uint8_t *data, size_t len);

// 组帧：返回整帧字节。payload 超过 kMaxPayload 时返回空 vector。
std::vector<uint8_t> encodeFrame(uint8_t typ, uint32_t seq,
                                 const uint8_t *payload, size_t len);

inline std::vector<uint8_t> encodeFrame(uint8_t typ, uint32_t seq,
                                        const std::vector<uint8_t> &payload)
{
  return encodeFrame(typ, seq, payload.data(), payload.size());
}

// 解帧结果
struct Decoded
{
  bool ok{false};
  uint8_t ver{0};
  uint8_t typ{0};
  uint32_t seq{0};
  std::vector<uint8_t> payload;
  std::string error;
};

// 解帧（要求 frame 恰好是一整帧，长度自洽）。失败时 ok=false 且 error 有原因。
Decoded decodeFrame(const uint8_t *frame, size_t len);

// =============================================================================
// 流式帧同步状态机
// -----------------------------------------------------------------------------
// 容错要求（无人机端审计特别强调）：遇到 A5 A5 C3 时，
// **第二个 A5 必须被当作新的起始字节**，不能因为第一个匹配失败就跳过它。
// 否则会丢掉一帧有效数据。
// =============================================================================
class FrameSync
{
public:
  // 喂入字节流；每解出一整帧就通过回调返回（不含 SOF 的整帧，即从 VER 到 CRC 末尾）
  // 返回成功解出的帧数。
  template <typename Fn>
  int feed(const uint8_t *data, size_t len, Fn on_frame)
  {
    int n = 0;
    for (size_t i = 0; i < len; ++i)
    {
      const uint8_t b = data[i];
      switch (state_)
      {
        case 0:                       // 找 SOF[0]
          if (b == kMagic0) state_ = 1;
          break;

        case 1:                       // 找 SOF[1]
          if (b == kMagic1)
          {
            buf_.clear();
            state_ = 2;
          }
          else if (b == kMagic0)
          {
            // ★ A5 A5 ... 第二个 A5 是新起点，留在状态 1
            state_ = 1;
          }
          else
          {
            state_ = 0;
          }
          break;

        default:                      // 收 body：需要 8 字节头 + LEN + 2 字节 CRC
        {
          buf_.push_back(b);
          // 头未收齐
          if (buf_.size() < 8)
          {
            break;
          }
          // 解析 LEN（buf_[6], buf_[7]，对应帧内偏移 8..9）
          const uint16_t ln = static_cast<uint16_t>(
              buf_[6] | (static_cast<uint16_t>(buf_[7]) << 8));
          if (ln > kMaxPayload)
          {
            // ★ LEN 非法：丢弃并**重新检查当前字节**是否为新 SOF[0]
            resetTo(b);
            break;
          }
          const size_t want = 8 + static_cast<size_t>(ln) + 2;
          if (buf_.size() < want)
          {
            break;
          }
          // buf_ 已满一帧（VER..CRC）
          std::vector<uint8_t> frame;
          frame.reserve(2 + want);
          frame.push_back(kMagic0);
          frame.push_back(kMagic1);
          frame.insert(frame.end(), buf_.begin(), buf_.end());
          const Decoded d = decodeFrame(frame.data(), frame.size());
          if (d.ok)
          {
            ++n;
            on_frame(d);
          }
          state_ = 0;
          buf_.clear();
          break;
        }
      }
    }
    return n;
  }

  void reset() { state_ = 0; buf_.clear(); }
  size_t buffered() const { return buf_.size(); }

private:
  void resetTo(uint8_t b)
  {
    state_ = (b == kMagic0) ? 1 : 0;
    buf_.clear();
  }

  int state_{0};
  std::vector<uint8_t> buf_;
};

// =============================================================================
// 载荷编码器（与 air_protocol.py 的 hb_* 函数一一对应）
// =============================================================================
namespace enc
{
// 单格 uint16 位域：occupancy 3bit | confidence 4bit | age 4bit
uint16_t cellEncode(uint8_t occ, uint8_t conf = 15, uint8_t age = 0);
void cellDecode(uint16_t v, uint8_t &occ, uint8_t &conf, uint8_t &age);

std::vector<uint8_t> heartbeat(uint32_t ts_ms, uint16_t map_version = 0,
                               uint8_t quality = 100, uint8_t flags = 0);
std::vector<uint8_t> teamInfo(uint8_t our_color, uint8_t cols = 14,
                              uint8_t rows = 11, uint8_t cell_cm = 35);
// 场地固定结构（车端已固化大部分，无人机端上线时可选下发）
//
// ★ 版本演进（v2.1 追加了无人机起降点）：
//   v2.0  8 字节：up_row col_lo col_hi bridge_col a_lo a_hi b_lo b_hi
//   v2.1 12 字节：以上 8 字节 + 无人机起点(col,row) + 无人机终点(col,row)
//
// ★ 为什么需要无人机起降点：
//   无人机**起飞前与降落后**都停在场地上的固定格。地面车若规划路径经过
//   这两格就会撞机 —— 而"撞固定场地持续超 10s 裁判切手动"是硬规则。
//   所以车端必须把它们视为**禁入格**。
//
// ★ 向后兼容：这是**追加**扩展。车端按 payload.size() 解释，
//   旧车端收到 12 字节会忽略后 4 字节，不会报错。
std::vector<uint8_t> fieldFixed(uint8_t up_row = 5, uint8_t col_lo = 3,
                                uint8_t col_hi = 13, uint8_t bridge_col = 10,
                                uint8_t a_lo = 0, uint8_t a_hi = 5,
                                uint8_t b_lo = 6, uint8_t b_hi = 10,
                                // 无人机起点/终点（★ v2.1 新增）
                                //   默认值与现场图一致：起点 M9=(12,8)、终点 N2=(13,1)
                                uint8_t drone_start_col = 12,
                                uint8_t drone_start_row = 8,
                                uint8_t drone_end_col = 13,
                                uint8_t drone_end_row = 1);
std::vector<uint8_t> mapFull(uint32_t map_version, uint32_t ts_ms,
                             const std::vector<uint16_t> &cells);
std::vector<uint8_t> mineList(uint32_t ts_ms,
                              const std::vector<std::array<uint8_t, 3>> &mines);
std::vector<uint8_t> bridgeGate(uint8_t bridge_id, uint8_t gate_row,
                                uint8_t confidence = 100, uint8_t flags = 0x01);
std::vector<uint8_t> targetList(
    uint32_t ts_ms, const std::vector<std::array<uint8_t, 4>> &targets);
std::vector<uint8_t> obstacleList(
    const std::vector<std::array<uint8_t, 3>> &obstacles);
std::vector<uint8_t> taskCmd(uint8_t cmd, uint8_t cmd_seq,
                             const std::vector<uint8_t> &extra = {});
std::vector<uint8_t> navToCell(uint8_t cmd_seq, uint8_t col, uint8_t row,
                               uint8_t timeout_s = 60);
std::vector<uint8_t> bridgeCross(uint8_t cmd_seq, uint8_t bridge_id);
std::vector<uint8_t> engage(uint8_t cmd_seq, uint8_t col, uint8_t row,
                            uint8_t target_face = 2);
std::vector<uint8_t> route(uint32_t route_version, uint8_t task_id,
                           const std::vector<std::array<uint8_t, 3>> &wps);
std::vector<uint8_t> vehStatus(
    uint32_t ts_ms, int16_t x_cm, int16_t y_cm, int16_t yaw_c10, int16_t v_cms,
    int16_t w_c100, uint8_t task_state, uint8_t fault,
    uint8_t mines_cleared = 0, uint8_t bridges_passed = 0, uint8_t self_hp = 0,
    uint8_t ammo = 0, uint8_t battery_pct = 100, uint8_t pose_confidence = 100,
    uint8_t last_cmd_seq = 0, uint8_t flags = 0, int16_t blocked_col = -1,
    int16_t blocked_row = -1, uint16_t exec_ms = 0);
std::vector<uint8_t> event(uint32_t ts_ms, uint8_t ev, uint8_t a0 = 0,
                           uint8_t a1 = 0, uint8_t a2 = 0);
std::vector<uint8_t> ack(uint32_t ack_seq, uint8_t status = 0,
                         uint8_t detail = 0);
}  // namespace enc

}  // namespace air
}  // namespace dzmission

#endif  // DZMISSION_AIR_PROTOCOL_HPP
