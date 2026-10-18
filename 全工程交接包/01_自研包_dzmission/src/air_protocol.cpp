#include "dzmission/air_protocol.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace dzmission
{
namespace air
{

// =============================================================================
// CRC-16/CCITT-FALSE：poly 0x1021，init 0xFFFF，不反射，不异或输出
//   逐位实现与 air_protocol.py 的 crc16_ccitt() 完全一致
//   标准校验串 "123456789" -> 0x29B1
// =============================================================================
uint16_t crc16CcittFalse(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
    for (int b = 0; b < 8; ++b)
    {
      if ((crc & 0x8000) != 0)
      {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      }
      else
      {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// =============================================================================
// 组帧
//   body = VER | TYP | SEQ(4) | LEN(2) | PAYLOAD
//   crc 覆盖 body（即 VER..PAYLOAD），不含 SOF 与 CRC 自身
// =============================================================================
std::vector<uint8_t> encodeFrame(uint8_t typ, uint32_t seq,
                                 const uint8_t *payload, size_t len)
{
  std::vector<uint8_t> out;
  if (len > kMaxPayload)
  {
    return out;   // 空表示失败（与 Python 抛异常对应）
  }
  out.reserve(kHeaderLen + len + kCrcLen);

  // SOF
  out.push_back(kMagic0);
  out.push_back(kMagic1);
  // body 起始位置（也是 CRC 覆盖范围的起点）
  const size_t body_at = out.size();
  out.push_back(kVer);
  out.push_back(typ);
  // SEQ 小端 uint32
  out.push_back(static_cast<uint8_t>(seq & 0xFF));
  out.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((seq >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((seq >> 24) & 0xFF));
  // LEN 小端 uint16
  const uint16_t ln = static_cast<uint16_t>(len);
  out.push_back(static_cast<uint8_t>(ln & 0xFF));
  out.push_back(static_cast<uint8_t>((ln >> 8) & 0xFF));
  // PAYLOAD
  if (len > 0 && payload != nullptr)
  {
    out.insert(out.end(), payload, payload + len);
  }
  // CRC 覆盖 out[body_at .. end)
  const uint16_t crc = crc16CcittFalse(out.data() + body_at, out.size() - body_at);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return out;
}

// =============================================================================
// 解帧
// =============================================================================
Decoded decodeFrame(const uint8_t *frame, size_t len)
{
  Decoded d;
  if (len < kHeaderLen + kCrcLen)
  {
    d.error = "frame too short";
    return d;
  }
  if (frame[0] != kMagic0 || frame[1] != kMagic1)
  {
    d.error = "bad magic";
    return d;
  }
  d.ver = frame[2];
  d.typ = frame[3];
  d.seq = static_cast<uint32_t>(frame[4]) |
          (static_cast<uint32_t>(frame[5]) << 8) |
          (static_cast<uint32_t>(frame[6]) << 16) |
          (static_cast<uint32_t>(frame[7]) << 24);
  const uint16_t ln = static_cast<uint16_t>(
      frame[8] | (static_cast<uint16_t>(frame[9]) << 8));

  if (ln > kMaxPayload)
  {
    d.error = "bad len " + std::to_string(ln);
    return d;
  }
  const size_t expect = kHeaderLen + static_cast<size_t>(ln) + kCrcLen;
  if (len != expect)
  {
    d.error = "len mismatch: expect " + std::to_string(expect) +
              " got " + std::to_string(len);
    return d;
  }

  // CRC 覆盖 frame[2 .. 10+ln)
  const uint16_t crc_recv = static_cast<uint16_t>(
      frame[10 + ln] | (static_cast<uint16_t>(frame[10 + ln + 1]) << 8));
  const uint16_t crc_calc = crc16CcittFalse(frame + 2, 8 + ln);
  if (crc_recv != crc_calc)
  {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "crc mismatch: recv 0x%04X calc 0x%04X",
                  crc_recv, crc_calc);
    d.error = buf;
    return d;
  }
  if (d.ver != kVer)
  {
    d.error = "version mismatch: " + std::to_string(d.ver) +
              " != " + std::to_string(kVer);
    return d;
  }

  d.payload.assign(frame + kHeaderLen, frame + kHeaderLen + ln);
  d.ok = true;
  return d;
}

// =============================================================================
// 载荷编码
// =============================================================================
namespace enc
{

uint16_t cellEncode(uint8_t occ, uint8_t conf, uint8_t age)
{
  return static_cast<uint16_t>((occ & 0x07) |
                               ((static_cast<uint16_t>(conf) & 0x0F) << 3) |
                               ((static_cast<uint16_t>(age) & 0x0F) << 7));
}

void cellDecode(uint16_t v, uint8_t &occ, uint8_t &conf, uint8_t &age)
{
  occ = static_cast<uint8_t>(v & 0x07);
  conf = static_cast<uint8_t>((v >> 3) & 0x0F);
  age = static_cast<uint8_t>((v >> 7) & 0x0F);
}

namespace
{
inline void put32(std::vector<uint8_t> &v, uint32_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}
inline void put16(std::vector<uint8_t> &v, uint16_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}
}  // namespace

std::vector<uint8_t> heartbeat(uint32_t ts_ms, uint16_t map_version,
                               uint8_t quality, uint8_t flags)
{
  std::vector<uint8_t> v;
  v.reserve(8);
  put32(v, ts_ms);
  put16(v, map_version);
  v.push_back(quality);
  v.push_back(flags);
  return v;   // 固定 8 字节
}

std::vector<uint8_t> teamInfo(uint8_t our_color, uint8_t cols, uint8_t rows,
                              uint8_t cell_cm)
{
  return {our_color, cols, rows, cell_cm};   // 4 字节
}

std::vector<uint8_t> fieldFixed(uint8_t up_row, uint8_t col_lo, uint8_t col_hi,
                                uint8_t bridge_col, uint8_t a_lo, uint8_t a_hi,
                                uint8_t b_lo, uint8_t b_hi,
                                uint8_t drone_start_col, uint8_t drone_start_row,
                                uint8_t drone_end_col, uint8_t drone_end_row)
{
  // ★ 版本演进：
  //   v2.0  8 字节 = up_row col_lo col_hi bridge_col a_lo a_hi b_lo b_hi
  //   v2.1 12 字节 = 上面 8 字节 + 无人机起点(col,row) + 无人机终点(col,row)
  //
  // ★ 为什么**追加**而不是新开一个消息类型：
  //   · 车端对 FIELD_FIXED 是"原样收下"，按 `payload.size()` 解释，
  //     旧车端收到 12 字节只会忽略后 4 字节 —— **不会崩**，向后兼容。
  //   · 无人机起降点与场地固定结构同属"本局不变的静态信息"，
  //     放在同一条消息里，无人机端只需发一次。
  //
  // ★ 为什么需要这两个格：无人机**起飞前、降落后**都停在场上，
  //   地面车必须避开这两个格，否则会撞机（撞固定场地超 10s 裁判切手动）。
  return {up_row, col_lo, col_hi, bridge_col, a_lo, a_hi, b_lo, b_hi,
          drone_start_col, drone_start_row, drone_end_col, drone_end_row};
}

std::vector<uint8_t> mapFull(uint32_t map_version, uint32_t ts_ms,
                             const std::vector<uint16_t> &cells)
{
  std::vector<uint8_t> v;
  if (cells.size() != 14u * 11u)
  {
    return v;   // 必须正好 154 格
  }
  v.reserve(10 + cells.size() * 2);
  put32(v, map_version);
  put32(v, ts_ms);
  v.push_back(14);
  v.push_back(11);
  for (uint16_t c : cells)
  {
    put16(v, c);
  }
  return v;   // 10 + 308 = 318 字节
}

std::vector<uint8_t> mineList(uint32_t ts_ms,
                              const std::vector<std::array<uint8_t, 3>> &mines)
{
  std::vector<uint8_t> v;
  v.reserve(5 + mines.size() * 3);
  put32(v, ts_ms);
  v.push_back(static_cast<uint8_t>(mines.size()));
  for (const auto &m : mines)
  {
    v.push_back(m[0]);
    v.push_back(m[1]);
    v.push_back(m[2]);
  }
  return v;
}

std::vector<uint8_t> bridgeGate(uint8_t bridge_id, uint8_t gate_row,
                                uint8_t confidence, uint8_t flags)
{
  return {bridge_id, gate_row, confidence, flags};   // 固定 4 字节
}

std::vector<uint8_t> targetList(
    uint32_t ts_ms, const std::vector<std::array<uint8_t, 4>> &targets)
{
  std::vector<uint8_t> v;
  v.reserve(5 + targets.size() * 4);
  put32(v, ts_ms);
  v.push_back(static_cast<uint8_t>(targets.size()));
  for (const auto &t : targets)
  {
    v.push_back(t[0]);
    v.push_back(t[1]);
    v.push_back(t[2]);
    v.push_back(t[3]);
  }
  return v;
}

std::vector<uint8_t> obstacleList(
    const std::vector<std::array<uint8_t, 3>> &obstacles)
{
  // ★ 注意：本条**没有 ts_ms**（与 MINE_LIST / TARGET_LIST 不一致，但以参考实现为准）
  std::vector<uint8_t> v;
  v.reserve(1 + obstacles.size() * 3);
  v.push_back(static_cast<uint8_t>(obstacles.size()));
  for (const auto &o : obstacles)
  {
    v.push_back(o[0]);
    v.push_back(o[1]);
    v.push_back(o[2]);
  }
  return v;
}

std::vector<uint8_t> taskCmd(uint8_t cmd, uint8_t cmd_seq,
                             const std::vector<uint8_t> &extra)
{
  std::vector<uint8_t> v;
  v.reserve(2 + extra.size());
  v.push_back(cmd);
  v.push_back(cmd_seq);
  v.insert(v.end(), extra.begin(), extra.end());
  return v;
}

std::vector<uint8_t> navToCell(uint8_t cmd_seq, uint8_t col, uint8_t row,
                               uint8_t timeout_s)
{
  return taskCmd(kCmdNavToCell, cmd_seq, {col, row, timeout_s});
}

std::vector<uint8_t> bridgeCross(uint8_t cmd_seq, uint8_t bridge_id)
{
  return taskCmd(kCmdBridgeCross, cmd_seq, {bridge_id});
}

std::vector<uint8_t> engage(uint8_t cmd_seq, uint8_t col, uint8_t row,
                            uint8_t target_face)
{
  return taskCmd(kCmdEngage, cmd_seq, {col, row, target_face});
}

std::vector<uint8_t> route(uint32_t route_version, uint8_t task_id,
                           const std::vector<std::array<uint8_t, 3>> &wps)
{
  std::vector<uint8_t> v;
  v.reserve(6 + wps.size() * 3);
  put32(v, route_version);
  v.push_back(static_cast<uint8_t>(wps.size()));
  v.push_back(task_id);
  for (const auto &w : wps)
  {
    v.push_back(w[0]);
    v.push_back(w[1]);
    v.push_back(w[2]);
  }
  return v;
}

std::vector<uint8_t> vehStatus(
    uint32_t ts_ms, int16_t x_cm, int16_t y_cm, int16_t yaw_c10, int16_t v_cms,
    int16_t w_c100, uint8_t task_state, uint8_t fault, uint8_t mines_cleared,
    uint8_t bridges_passed, uint8_t self_hp, uint8_t ammo, uint8_t battery_pct,
    uint8_t pose_confidence, uint8_t last_cmd_seq, uint8_t flags,
    int16_t blocked_col, int16_t blocked_row, uint16_t exec_ms)
{
  std::vector<uint8_t> v;
  v.reserve(32);
  put32(v, ts_ms);
  put16(v, static_cast<uint16_t>(x_cm));
  put16(v, static_cast<uint16_t>(y_cm));
  put16(v, static_cast<uint16_t>(yaw_c10));
  put16(v, static_cast<uint16_t>(v_cms));
  put16(v, static_cast<uint16_t>(w_c100));
  v.push_back(task_state);
  v.push_back(fault);
  v.push_back(mines_cleared);
  v.push_back(bridges_passed);
  v.push_back(self_hp);
  v.push_back(ammo);
  v.push_back(battery_pct);
  v.push_back(pose_confidence);
  v.push_back(last_cmd_seq);
  v.push_back(flags);
  put16(v, static_cast<uint16_t>(blocked_col));
  put16(v, static_cast<uint16_t>(blocked_row));
  put16(v, exec_ms);
  put16(v, 0);   // reserved
  return v;      // 固定 32 字节
}

std::vector<uint8_t> event(uint32_t ts_ms, uint8_t ev, uint8_t a0, uint8_t a1,
                           uint8_t a2)
{
  std::vector<uint8_t> v;
  v.reserve(8);
  put32(v, ts_ms);
  v.push_back(ev);
  v.push_back(a0);
  v.push_back(a1);
  v.push_back(a2);
  return v;   // 固定 8 字节
}

std::vector<uint8_t> ack(uint32_t ack_seq, uint8_t status, uint8_t detail)
{
  std::vector<uint8_t> v;
  v.reserve(8);
  put32(v, ack_seq);
  v.push_back(status);
  v.push_back(detail);
  put16(v, 0);   // reserved
  return v;      // 固定 8 字节
}

}  // namespace enc

}  // namespace air
}  // namespace dzmission
