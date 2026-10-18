#include "dzmission/mission_command.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace dzmission
{

// 整帧最大长度必须容纳 LEN 字段的最大取值
static_assert(kCmdFrameMaxLen >= 3 + kMaxPayloadLen + 2,
              "kCmdFrameMaxLen 必须 >= 帧头(2) + LEN(1) + LEN最大值 + CRC(2)");
static_assert(kMaxPayloadLen <= 255, "LEN 字段只有一个字节");

// -----------------------------------------------------------------------------
// CRC-16/MODBUS: poly 0x8005 reflected => 0xA001, init 0xFFFF
// -----------------------------------------------------------------------------
uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x0001)
      {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      }
      else
      {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

void wrU16(std::vector<uint8_t> &v, uint16_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}

void wrI16(std::vector<uint8_t> &v, int16_t x)
{
  wrU16(v, static_cast<uint16_t>(x));
}

void wrI32(std::vector<uint8_t> &v, int32_t x)
{
  uint32_t u = static_cast<uint32_t>(x);
  v.push_back(static_cast<uint8_t>(u & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
}

// -----------------------------------------------------------------------------
// 组帧
// -----------------------------------------------------------------------------
size_t encodeFrame(uint8_t cmd, const std::vector<uint8_t> &payload,
                   uint8_t *out, size_t out_cap)
{
  if (out == nullptr)
  {
    return 0;
  }
  const size_t len_field = 1 + payload.size();  // CMD + payload
  if (len_field > kMaxPayloadLen)
  {
    return 0;
  }
  const size_t total = 3 + len_field + 2;  // head0 head1 len ... crc_lo crc_hi
  if (total > out_cap || total > kCmdFrameMaxLen)
  {
    return 0;
  }

  size_t i = 0;
  out[i++] = kFrameHead0;
  out[i++] = kFrameHead1;
  out[i++] = static_cast<uint8_t>(len_field);
  out[i++] = cmd;
  for (uint8_t b : payload)
  {
    out[i++] = b;
  }

  const uint16_t crc = crc16_modbus(out + 2, len_field + 1);  // 从 LEN 到 payload 末（共 len_field+1 字节）
  out[i++] = static_cast<uint8_t>(crc & 0xFF);
  out[i++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return i;
}

// -----------------------------------------------------------------------------
// 解码（in 指向 LEN 字节，in_len = 1 + payload_len）
// -----------------------------------------------------------------------------
bool decodeFrame(const uint8_t *in, size_t in_len, uint8_t &cmd,
                 std::vector<uint8_t> &payload, std::string &error)
{
  if (in == nullptr || in_len < 3)
  {
    error = "frame too short";
    return false;
  }
  const uint8_t len_field = in[0];
  if (len_field < 1 || len_field > kMaxPayloadLen)
  {
    error = "bad len field";
    return false;
  }
  if (in_len != static_cast<size_t>(len_field) + 3)
  {
    error = "len mismatch";
    return false;
  }

  const uint16_t crc_calc = crc16_modbus(in, static_cast<size_t>(len_field) + 1);
  const uint16_t crc_recv = rdU16(in + static_cast<size_t>(len_field) + 1);
  if (crc_calc != crc_recv)
  {
    error = "crc mismatch";
    return false;
  }

  cmd = in[1];
  payload.assign(in + 2, in + 1 + len_field);
  error.clear();
  return true;
}

// -----------------------------------------------------------------------------
// 上行指令语义化
// -----------------------------------------------------------------------------
MissionCommand decodeMissionCommand(const uint8_t *in, size_t in_len)
{
  MissionCommand m;
  uint8_t cmd = 0;
  std::vector<uint8_t> p;
  std::string err;

  if (!decodeFrame(in, in_len, cmd, p, err))
  {
    m.valid = false;
    m.error = err;
    return m;
  }

  m.cmd = cmd;
  const size_t n = p.size();

  auto need = [&](size_t k) -> bool {
    if (n < k)
    {
      m.valid = false;
      m.error = "payload too short";
      return false;
    }
    return true;
  };

  switch (cmd)
  {
    case kCmdOpNop:
    case kCmdHeartbeat:
      m.valid = true;
      break;

    case kCmdNavTo:
      // x(i32,cm) y(i32,cm) yaw(i16,0.1deg) flags(u8: bit0=has_yaw) seq(u8)
      if (!need(12)) return m;
      m.x = static_cast<double>(rdI32(&p[0])) * 0.01;
      m.y = static_cast<double>(rdI32(&p[4])) * 0.01;
      m.yaw = static_cast<double>(rdI16(&p[8])) * 0.1 * M_PI / 180.0;
      m.has_yaw = (p[10] & 0x01) != 0;
      m.seq = p[11];
      m.valid = true;
      break;

    case kCmdSetHome:
      // x(i32,cm) y(i32,cm) seq(u8)   —— home 航向由车辆到达后自动记录
      if (!need(9)) return m;
      m.x = static_cast<double>(rdI32(&p[0])) * 0.01;
      m.y = static_cast<double>(rdI32(&p[4])) * 0.01;
      m.seq = p[8];
      m.valid = true;
      break;

    case kCmdOpStop:
      // stop(u8) seq(u8)
      if (!need(2)) return m;
      m.stop = p[0];
      m.seq = p[1];
      m.valid = true;
      break;

    case kCmdOpHold:
    case kCmdOpReturnBase:
    case kCmdCeaseFire:
      // seq(u8)
      if (!need(1)) return m;
      m.seq = p[0];
      m.valid = true;
      break;

    case kCmdOpEngage:
      // target_id(u8) conf(u8) flags(u8,bit0=距离开火解算) bearing(i16,0.1deg)
      // range(u16,cm) seq(u8)
      if (!need(7)) return m;
      m.target_id = p[0];
      m.confidence = p[1];
      m.bearing_deg = static_cast<double>(rdBearingDeg(&p[3]));
      m.range_m = static_cast<double>(rdRangeCm(&p[5]));
      m.seq = p[6];
      m.valid = true;
      break;

    // =========================================================================
    // ★★ 无人机侦察结果四类（规则 image5/image6）★★
    // -------------------------------------------------------------------------
    // 协议层只做**边界检查**：长度是否够、条数是否与长度自洽。
    // 领域校验（格子越界、门位区段、去重）在 MissionManager，因为那里才
    // 知道场地规格。这样协议层不必知道"场地有几行几列"。
    //
    // ★ 为什么要严格检查"条数与长度自洽"：
    //   无人机端尚未实现，将来字段若有偏差，最可能的后果是**静默截断**
    //   （只解析前半段），表现为"地雷少了几颗"—— 极难现场定位。
    //   这里把不自洽直接判为无效帧，让它**显式失败**。
    // =========================================================================
    case kCmdMineList:
    case kCmdObstacleList:
    case kCmdTargetList:
    {
      // 头部：ts(u32) count(u8)；每条 record 大小见下
      if (!need(5)) return m;
      m.ts_ms = rdU32(&p[0]);
      m.item_count = p[4];
      const size_t rec = (cmd == kCmdTargetList) ? 4u : 3u;  // face 只在目标列表里
      const size_t expect = 5u + static_cast<size_t>(m.item_count) * rec;
      if (n < expect)
      {
        m.valid = false;
        m.error = "list length mismatch: need " + std::to_string(expect) +
                  " got " + std::to_string(n);
        return m;
      }
      m.payload.assign(p.begin(), p.begin() + expect);
      m.seq = 0;          // 列表类没有 seq 字段，回执用 0
      m.valid = true;
      break;
    }

    case kCmdBridgeGate:
      // bridge_id(u8) gate_row(u8) conf(u8) flags(u8) —— 固定 4 字节
      if (!need(4)) return m;
      m.payload.assign(p.begin(), p.begin() + 4);
      m.seq = 0;
      m.valid = true;
      break;

    case kCmdMapFull:
      // map_version(u32) ts(u32) cols(u8) rows(u8) [cell(u16)] * cols*rows
      if (!need(10)) return m;
      m.ts_ms = rdU32(&p[4]);
      {
        const size_t cols = p[8];
        const size_t rows = p[9];
        const size_t expect = 10u + cols * rows * 2u;
        if (cols == 0 || rows == 0 || n < expect)
        {
          m.valid = false;
          m.error = "mapFull length mismatch: need " +
                    std::to_string(expect) + " got " + std::to_string(n);
          return m;
        }
        m.payload.assign(p.begin(), p.begin() + expect);
      }
      m.seq = 0;
      m.valid = true;
      break;

    case kCmdTargetReport:
      // target_id(u8) conf(u8) bearing(i16,0.1deg) range(u16,cm) seq(u8)
      if (!need(6)) return m;
      m.target_id = p[0];
      m.confidence = p[1];
      m.bearing_deg = static_cast<double>(rdBearingDeg(&p[2]));
      m.range_m = static_cast<double>(rdRangeCm(&p[4]));
      m.seq = p[5];
      m.valid = true;
      break;

    default:
      m.valid = false;
      m.error = "unknown cmd";
      break;
  }

  return m;
}

// -----------------------------------------------------------------------------
// 下行组帧
// -----------------------------------------------------------------------------
size_t encodeMissionCommand(const MissionCommand &m, uint8_t *out, size_t out_cap)
{
  // 仅用于回环测试：把 MissionCommand 再编码回 NavTo 形态
  std::vector<uint8_t> p;
  wrI32(p, static_cast<int32_t>(std::lround(m.x * 100.0)));
  wrI32(p, static_cast<int32_t>(std::lround(m.y * 100.0)));
  wrI16(p, static_cast<int16_t>(std::lround(m.yaw * 180.0 / M_PI * 10.0)));
  p.push_back(static_cast<uint8_t>(m.has_yaw ? 0x01 : 0x00));
  p.push_back(m.seq);
  return encodeFrame(m.cmd, p, out, out_cap);
}

size_t encodeVehicleStatus(const VehicleStatus &s, uint8_t *out, size_t out_cap)
{
  std::vector<uint8_t> p;
  p.push_back(s.seq);                                   // 0
  p.push_back(s.mission_state);                         // 1
  p.push_back(s.fault);                                 // 2
  p.push_back(s.self_hp);                               // 3
  p.push_back(s.ammo);                                  // 4
  p.push_back(s.hit_dir);                               // 5
  p.push_back(s.shoot_count);                           // 6
  p.push_back(s.battery_pct);                           // 7
  wrI32(p, static_cast<int32_t>(std::lround(s.x * 100.0)));      // 8..11  (cm)
  wrI32(p, static_cast<int32_t>(std::lround(s.y * 100.0)));      // 12..15 (cm)
  wrI16(p, static_cast<int16_t>(std::lround(s.yaw * 180.0 / M_PI * 10.0)));  // 16..17 (0.1deg)
  wrI16(p, static_cast<int16_t>(std::lround(s.v * 100.0)));      // 18..19 (cm/s)
  wrI16(p, static_cast<int16_t>(std::lround(s.w * 100.0)));      // 20..21 (0.01rad/s)
  p.push_back(s.seq);                                   // 22 兼容位
  p.push_back(0);                                       // 23 保留
  return encodeFrame(kReportStatus, p, out, out_cap);
}

size_t encodeTargetAck(const TargetAck &a, uint8_t *out, size_t out_cap)
{
  std::vector<uint8_t> p;
  p.push_back(a.seq);
  p.push_back(a.target_id);
  p.push_back(a.result);
  p.push_back(a.ammo_left);
  wrI32(p, static_cast<int32_t>(std::lround(a.aim_x * 100.0)));
  wrI32(p, static_cast<int32_t>(std::lround(a.aim_y * 100.0)));
  return encodeFrame(kReportTargetAck, p, out, out_cap);
}

size_t encodeFault(uint8_t fault, uint8_t detail, uint8_t *out, size_t out_cap)
{
  std::vector<uint8_t> p;
  p.push_back(fault);
  p.push_back(detail);
  return encodeFrame(kReportFault, p, out, out_cap);
}

const char *cmdName(uint8_t cmd)
{
  switch (cmd)
  {
    case kCmdOpNop: return "NOP";
    case kCmdNavTo: return "NAV_TO";
    case kCmdOpStop: return "STOP";
    case kCmdOpEngage: return "ENGAGE";
    case kCmdCeaseFire: return "CEASE_FIRE";
    case kCmdOpReturnBase: return "RETURN_BASE";
    case kCmdOpHold: return "HOLD";
    case kCmdTargetReport: return "TARGET_REPORT";
    case kCmdSetHome: return "SET_HOME";
    case kCmdHeartbeat: return "HEARTBEAT";
    case kReportStatus: return "STATUS";
    case kReportTargetAck: return "TARGET_ACK";
    case kReportFault: return "FAULT";
    default: return "UNKNOWN";
  }
}

}  // namespace dzmission
