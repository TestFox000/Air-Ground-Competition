#ifndef DZMISSION_MISSION_COMMAND_HPP
#define DZMISSION_MISSION_COMMAND_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace dzmission
{

// =============================================================================
// 任务指令协议 v1  ——  无人机 / 上位机  <->  地面车
// -----------------------------------------------------------------------------
// 帧格式（小端）:
//   | 0xAA | 0x55 | LEN | CMD | PAYLOAD[LEN-1] | CRC16(lo) | CRC16(hi) |
//   LEN   = 1(CMD) + payload 长度         取值范围 1..64
//   CRC16 = CRC-16/MODBUS，计算范围从 LEN 到 PAYLOAD 末字节（即整帧去掉帧头与 CRC 本身）
//
// 上行（无人机 -> 地面车），见下方 kCmd* 常量
// 下行（地面车 -> 无人机），见 kReport* 常量
// =============================================================================

constexpr uint8_t kFrameHead0 = 0xAA;
constexpr uint8_t kFrameHead1 = 0x55;
constexpr size_t kCmdFrameMaxLen = 96;   // 整帧最大字节数（含帧头与 CRC）
constexpr size_t kMaxPayloadLen = 64; // CMD + payload 上限

// ---- 上行指令集（无人机 / 上位机 -> 地面车） ----
constexpr uint8_t kCmdOpNop = 0x00;         // 心跳，维持链路
constexpr uint8_t kCmdNavTo = 0x01;       // 前往地图坐标点
constexpr uint8_t kCmdOpStop = 0x02;        // 立即停车（导航与交战都停）
constexpr uint8_t kCmdOpEngage = 0x10;      // 进入交战：锁定/击打指定目标
constexpr uint8_t kCmdCeaseFire = 0x11;   // 停火但保持交战姿态
constexpr uint8_t kCmdOpReturnBase = 0x20;  // 返航到 home 点
constexpr uint8_t kCmdOpHold = 0x21;        // 原地待命（保持当前航向）
constexpr uint8_t kCmdTargetReport = 0x30;// 无人机视觉目标上报（相机系相对量）
constexpr uint8_t kCmdSetHome = 0x40;     // 设置 home 点
constexpr uint8_t kCmdHeartbeat = 0x7F;   // 链路保活（等同于 Nop，但会刷新链路时间）

// =============================================================================
// ★★ 无人机侦察结果：四类批量消息（规则 image5/image6）★★
// -----------------------------------------------------------------------------
// 规则原文：无人机负责"地图构建 · 识别地雷准确位置 · 识别敌我打击区位置 ·
//           识别障碍物 · 判断桥洞可通行通道"，结果下发给小车供其规划。
//
// ★ 关键：这四类的载荷结构差得很远（列表 vs 单条），且**无人机端尚未实现**，
//   所以这里采取"**透传 + 领域层校验**"的策略：
//     · decodeMissionCommand 只做**边界检查**（长度是否够、条数是否自洽），
//       并把原始载荷存进 MissionCommand::payload
//     · 真正的领域校验（格子越界、门位区段、去重等）放在 MissionManager，
//       因为那里才知道场地规格（FieldGrid::Spec）
//   这样做的理由：协议层不应该知道场地有几行几列；而且当无人机端将来
//   改动字段时，只需改领域层，不必动协议编解码。
//
// 载荷布局（与 air_protocol.cpp 的编码函数一一对应）：
//   kCmdMineList      ts(u32) count(u8) [col(u8) row(u8) conf(u8)] * count
//   kCmdObstacleList  ts(u32) count(u8) [col(u8) row(u8) conf(u8)] * count
//   kCmdTargetList    ts(u32) count(u8) [col(u8) row(u8) face(u8) conf(u8)] * count
//   kCmdBridgeGate    bridge_id(u8) gate_row(u8) conf(u8) flags(u8)   ← 固定 4 字节
//   kCmdMapFull       map_version(u32) ts(u32) cols(u8) rows(u8) [cell(u16)] *
//                     cols*rows（313/318 字节量级，见 doc 一致性检查）
// =============================================================================
constexpr uint8_t kCmdMineList = 0x14;      // 地雷位置列表
constexpr uint8_t kCmdObstacleList = 0x17;  // 障碍物列表
constexpr uint8_t kCmdTargetList = 0x16;    // 敌我识别打击区列表
constexpr uint8_t kCmdBridgeGate = 0x15;    // 桥洞可通行通道
constexpr uint8_t kCmdMapFull = 0x12;       // 全图占据栅格（可选）
// ★ 场地固定结构（v2.1 起含**无人机起降点**）
//   无人机起飞前/降落后停在场上两格，地面车经过会撞机 —— 车端须设为禁入格。
//   载荷布局（12 字节）：up row/col_lo/col_hi/bridge_col/a_lo/a_hi/b_lo/b_hi
//                        + drone_start(col,row) + drone_end(col,row)
//
//   ★★ 取值必须避开已有操作码 ★★
//     这里曾误用 0x11（那是 **air 协议的类型码** kFieldFixed），
//     而本命名空间里 0x11 已归 kCmdCeaseFire，造成 switch 的
//     "duplicate case value" 编译错误。
//     注意：本文件的 kCmd* 里，**侦察类那几个**（0x12/0x14..0x17）
//     是**有意镜像 air 类型码**的；但 0x11 已被 CeaseFire 占用，
//     故这里取 **0x18**（紧邻侦察块，语义最近，且可用）。
constexpr uint8_t kCmdFieldFixed = 0x18;

// 桥洞区段编号（与规则"上方/下方桥洞"对应，v5 拓扑在 K 列）
constexpr uint8_t kBridgeIdA = 1;   // 区段一：K1..K6（row 0..5）
constexpr uint8_t kBridgeIdB = 2;   // 区段二：K7..K11（row 6..10）

// 打击区可见面（与 rknn_pt/det_node 的 [8] 字段一致）
constexpr uint8_t kFaceUnknown = 0;
constexpr uint8_t kFaceRed = 1;     // 友方（打了扣分）
constexpr uint8_t kFaceBlue = 2;    // 敌方（可打）

// ---- 下行上报集（地面车 -> 无人机 / 上位机） ----
constexpr uint8_t kReportStatus = 0x81;     // 车辆状态
constexpr uint8_t kReportTargetAck = 0x82;  // 目标确认 / 打击结果
constexpr uint8_t kReportFault = 0x83;      // 故障上报

// ---- 目标 ID ----
constexpr uint8_t kTargetNone = 0;
constexpr uint8_t kTargetEnemy = 1;
constexpr uint8_t kTargetMaterial = 2; // 物资点，不打击

// ---- 打击结论 ----
constexpr uint8_t kHitUnknown = 0;
constexpr uint8_t kHitSuccess = 1;
constexpr uint8_t kHitMissed = 2;
constexpr uint8_t kHitAborted = 3;

// ---- 故障码 ----
constexpr uint8_t kFaultNone = 0;
constexpr uint8_t kFaultNoAmmo = 1;
constexpr uint8_t kFaultLowBattery = 2;
constexpr uint8_t kFaultNoLocalization = 3;
constexpr uint8_t kFaultObstacleBlocked = 4;
constexpr uint8_t kFaultRefereeLinkLost = 5;
constexpr uint8_t kFaultCmdLinkLost = 6;
constexpr uint8_t kFaultVisionLost = 7;

// -----------------------------------------------------------------------------
// 上行指令结构体
// -----------------------------------------------------------------------------
struct MissionCommand
{
  uint8_t cmd{kCmdOpNop};
  // kCmdNavTo / kCmdSetHome
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool has_yaw{false};
  // kCmdOpEngage / kCmdTargetReport
  uint8_t target_id{kTargetNone};
  // kCmdOpEngage: 正数表示相对车头方向的方位角(度)，负数为左
  double bearing_deg{0.0};
  double range_m{0.0};
  // 无人机视觉置信度 0..100
  uint8_t confidence{0};
  // kCmdOpStop: 0 解除, 1 停车
  uint8_t stop{1};
  // 原始帧序号，用于回执
  uint8_t seq{0};

  // ★ 无人机侦察结果的原始载荷（见文件上方 kCmdMineList 等注释）
  //   协议层只做长度/自洽性检查，领域校验在 MissionManager
  std::vector<uint8_t> payload;
  // kCmdMineList / kCmdObstacleList / kCmdTargetList：载荷里的条数
  uint8_t item_count{0};
  // 这几类共有的时间戳（毫秒），用于判断新鲜度
  uint32_t ts_ms{0};

  bool valid{false};
  std::string error;
};

// -----------------------------------------------------------------------------
// 下行状态结构体
// -----------------------------------------------------------------------------
struct VehicleStatus
{
  uint8_t seq{0};
  uint8_t mission_state{0};  // 见 MissionState
  uint8_t fault{0};
  uint8_t self_hp{0};
  uint8_t ammo{0};
  uint8_t hit_dir{0x10};     // 0x01左 0x02前 0x03右 0x04后 0x10未中弹
  uint8_t shoot_count{0};
  uint8_t battery_pct{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double w{0.0};
};

struct TargetAck
{
  uint8_t seq{0};
  uint8_t target_id{kTargetNone};
  uint8_t result{kHitUnknown};
  uint8_t ammo_left{0};
  double aim_x{0.0};
  double aim_y{0.0};
};

// -----------------------------------------------------------------------------
// 编解码
// -----------------------------------------------------------------------------
uint16_t crc16_modbus(const uint8_t *data, size_t len);

// 把 cmd 编码成整帧，返回字节数；缓冲区不足返回 0。
size_t encodeFrame(uint8_t cmd, const std::vector<uint8_t> &payload,
                   uint8_t *out, size_t out_cap);

size_t encodeMissionCommand(const MissionCommand &m, uint8_t *out, size_t out_cap);
size_t encodeVehicleStatus(const VehicleStatus &s, uint8_t *out, size_t out_cap);
size_t encodeTargetAck(const TargetAck &a, uint8_t *out, size_t out_cap);
size_t encodeFault(uint8_t fault, uint8_t detail, uint8_t *out, size_t out_cap);

// 解析单帧（不含帧头，in 指向 LEN 字节）。成功返回 true。
bool decodeFrame(const uint8_t *in, size_t in_len, uint8_t &cmd,
                 std::vector<uint8_t> &payload, std::string &error);

// 解析并语义化上行指令。
MissionCommand decodeMissionCommand(const uint8_t *in, size_t in_len);

// 便捷读小端
inline uint16_t rdU16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline int16_t rdI16(const uint8_t *p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
inline int32_t rdI32(const uint8_t *p)
{
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}
// ★ 无符号 32 位读：数据面消息里的时间戳 ts_ms 用它。
//   （原先只有 rdI32，而 ts_ms 语义上必须无符号 —— 2024 年毫秒数超过
//     INT32_MAX 会变负数，所以不能用 rdI32 代替。）
inline uint32_t rdU32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline float rdBearingDeg(const uint8_t *p) { return static_cast<float>(rdI16(p)) * 0.1f; }
inline float rdRangeCm(const uint8_t *p) { return static_cast<float>(rdU16(p)) * 0.01f; }

void wrU16(std::vector<uint8_t> &v, uint16_t x);
void wrI16(std::vector<uint8_t> &v, int16_t x);
void wrI32(std::vector<uint8_t> &v, int32_t x);

const char *cmdName(uint8_t cmd);

}  // namespace dzmission

#endif  // DZMISSION_MISSION_COMMAND_HPP
