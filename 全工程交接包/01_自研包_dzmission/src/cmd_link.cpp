#include "dzmission/cmd_link.hpp"
// ★ air 协议 v2：CmdLink 现在**同时**支持两套帧格式（见下方 feedBytes 说明）：
//     · 空地协同协议 v2（A5 C3 + VER/TYP/SEQ/LEN + CRC-16/CCITT-FALSE）
//       —— 无人机下发的四类侦察数据 + 任务指令走这套
//     · 串口任务指令 v1（AA 55 + LEN/CMD + CRC-16/MODBUS）
//       —— 早期上位机/旧固件走这套，保留兼容
//   air 帧的同步、CRC 与载荷解码**直接复用 air_protocol**，
//   不自己实现，避免与无人机端产生逐字节差异。
#include "dzmission/air_protocol.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "serial/serial.h"

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace dzmission
{

// total = 3 + len_field + 2，len_field <= kMaxPayloadLen
static_assert(kCmdFrameMaxLen >= 3 + kMaxPayloadLen + 2,
              "接收缓冲区必须能容纳最大帧");

namespace
{
constexpr size_t kRxCapacity = 512;
}

CmdLink::CmdLink(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  const std::string p = ns.empty() ? "" : ns + ".";

  node_->declare_parameter(p + "cmd.transport", transport_);
  node_->declare_parameter(p + "cmd.serial_port", serial_port_);
  node_->declare_parameter(p + "cmd.serial_baud", serial_baud_);
  node_->declare_parameter(p + "cmd.udp_bind_host", udp_bind_host_);
  node_->declare_parameter(p + "cmd.udp_bind_port", udp_bind_port_);
  node_->declare_parameter(p + "cmd.udp_peer_host", udp_peer_host_);
  node_->declare_parameter(p + "cmd.udp_peer_port", udp_peer_port_);
  node_->declare_parameter(p + "cmd.loopback_hz", loopback_hz_);
  node_->declare_parameter(p + "cmd.link_timeout_ms", link_timeout_ms_);

  node_->get_parameter(p + "cmd.transport", transport_);
  node_->get_parameter(p + "cmd.serial_port", serial_port_);
  node_->get_parameter(p + "cmd.serial_baud", serial_baud_);
  node_->get_parameter(p + "cmd.udp_bind_host", udp_bind_host_);
  node_->get_parameter(p + "cmd.udp_bind_port", udp_bind_port_);
  node_->get_parameter(p + "cmd.udp_peer_host", udp_peer_host_);
  node_->get_parameter(p + "cmd.udp_peer_port", udp_peer_port_);
  node_->get_parameter(p + "cmd.loopback_hz", loopback_hz_);
  node_->get_parameter(p + "cmd.link_timeout_ms", link_timeout_ms_);

  pub_cmd_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("mission_cmd", 20);
  pub_raw_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("mission_cmd_raw", 20);

  rx_.reserve(kRxCapacity);
  last_rx_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(), "[CmdLink] transport=%s", transport_.c_str());
}

CmdLink::~CmdLink() { stop(); }

void CmdLink::start()
{
  if (running_.exchange(true))
  {
    return;
  }

  if (transport_ == "serial")
  {
    worker_ = std::thread(&CmdLink::serialLoop, this);
  }
  else if (transport_ == "udp")
  {
    worker_ = std::thread(&CmdLink::udpLoop, this);
  }
  else
  {
    if (transport_ != "loopback")
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[CmdLink] unknown transport '%s', falling back to loopback",
                  transport_.c_str());
      transport_ = "loopback";
    }
    worker_ = std::thread(&CmdLink::loopbackLoop, this);
  }
}

void CmdLink::stop()
{
  if (!running_.exchange(false))
  {
    return;
  }
  if (worker_.joinable())
  {
    worker_.join();
  }

  std::lock_guard<std::mutex> lk(tx_mutex_);
  if (serial_handle_ != nullptr)
  {
    auto *ser = static_cast<serial::Serial *>(serial_handle_);
    try
    {
      if (ser->isOpen())
      {
        ser->close();
      }
    }
    catch (...)
    {
    }
    // 先把句柄置空，再用锁内替换的指针删除，避免发送线程访问已释放对象
    serial_handle_ = nullptr;
    delete ser;
  }
#ifndef _WIN32
  if (udp_fd_ >= 0)
  {
    ::close(udp_fd_);
    udp_fd_ = -1;
  }
#endif
  ready_.store(false);
}

// -----------------------------------------------------------------------------
// 串口接收
// -----------------------------------------------------------------------------
void CmdLink::serialLoop()
{
  auto *ser = new serial::Serial();
  // 句柄在锁内发布，保证 sendFrame() 看到的永远是"已构造完成"的对象
  {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    serial_handle_ = ser;
  }

  try
  {
    ser->setPort(serial_port_);
    ser->setBaudrate(static_cast<uint32_t>(serial_baud_));
    ser->setTimeout(serial::Timeout::simpleTimeout(20));
    ser->open();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[CmdLink] open %s failed: %s",
                 serial_port_.c_str(), e.what());
    {
      std::lock_guard<std::mutex> lk(tx_mutex_);
      if (serial_handle_ == ser)
      {
        serial_handle_ = nullptr;
      }
    }
    delete ser;
    ready_.store(false);
    return;
  }

  if (!ser->isOpen())
  {
    RCLCPP_ERROR(node_->get_logger(), "[CmdLink] %s not open", serial_port_.c_str());
    {
      std::lock_guard<std::mutex> lk(tx_mutex_);
      if (serial_handle_ == ser)
      {
        serial_handle_ = nullptr;
      }
    }
    delete ser;
    ready_.store(false);
    return;
  }

  ready_.store(true);
  RCLCPP_INFO(node_->get_logger(), "[CmdLink] serial %s@%d opened",
              serial_port_.c_str(), serial_baud_);

  std::vector<uint8_t> chunk(256);
  while (running_.load() && rclcpp::ok())
  {
    size_t avail = 0;
    size_t got = 0;
    {
      // 读操作与 sendFrame 的写操作共用同一把锁：并发读写同一个 fd 会导致
      // 帧错位（最坏情况是永远凑不齐长度）。
      std::lock_guard<std::mutex> lk(tx_mutex_);
      try
      {
        avail = ser->available();
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(node_->get_logger(), "[CmdLink] serial available() failed: %s", e.what());
        break;
      }

      if (avail == 0)
      {
        // 锁外 sleep，避免长时间占用发送路径
      }
      else
      {
        const size_t want = std::min(avail, chunk.size());
        try
        {
          got = ser->read(chunk.data(), want);
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(node_->get_logger(), "[CmdLink] serial read failed: %s", e.what());
          break;
        }
      }
    }

    if (got > 0)
    {
      feedBytes(chunk.data(), got);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[CmdLink] serial loop exit");
  ready_.store(false);
}

// -----------------------------------------------------------------------------
// UDP 接收
// -----------------------------------------------------------------------------
void CmdLink::udpLoop()
{
#ifdef _WIN32
  RCLCPP_ERROR(node_->get_logger(), "[CmdLink] udp transport unsupported on this platform");
  ready_.store(false);
  return;
#else
  udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_fd_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[CmdLink] socket() failed: %s", std::strerror(errno));
    ready_.store(false);
    return;
  }

  int reuse = 1;
  ::setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(udp_bind_port_));
  addr.sin_addr.s_addr = (udp_bind_host_ == "0.0.0.0")
                             ? htonl(INADDR_ANY)
                             : ::inet_addr(udp_bind_host_.c_str());

  if (::bind(udp_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[CmdLink] bind %s:%d failed: %s",
                 udp_bind_host_.c_str(), udp_bind_port_, std::strerror(errno));
    ::close(udp_fd_);
    udp_fd_ = -1;
    ready_.store(false);
    return;
  }

  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 50000;  // 50ms
  ::setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ready_.store(true);
  RCLCPP_INFO(node_->get_logger(), "[CmdLink] udp bound %s:%d -> peer %s:%d",
              udp_bind_host_.c_str(), udp_bind_port_,
              udp_peer_host_.c_str(), udp_peer_port_);

  std::vector<uint8_t> chunk(1024);
  while (running_.load() && rclcpp::ok())
  {
    const ssize_t n = ::recv(udp_fd_, chunk.data(), chunk.size(), 0);
    if (n > 0)
    {
      feedBytes(chunk.data(), static_cast<size_t>(n));
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      RCLCPP_WARN(node_->get_logger(), "[CmdLink] recv failed: %s", std::strerror(errno));
      break;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[CmdLink] udp loop exit");
  ready_.store(false);
#endif
}

// -----------------------------------------------------------------------------
// 离线自测：周期性注入一条 NavTo 指令（走完整编解码路径）
// -----------------------------------------------------------------------------
void CmdLink::loopbackLoop()
{
  ready_.store(true);
  RCLCPP_WARN(node_->get_logger(),
              "[CmdLink] LOOPBACK mode: no real drone link. %.2f Hz synthetic NavTo.",
              loopback_hz_);

  const double period = (loopback_hz_ > 0.01) ? (1.0 / loopback_hz_) : 5.0;
  uint8_t seq = 0;
  double acc = period;  // 立即发第一条

  while (running_.load() && rclcpp::ok())
  {
    const auto t0 = std::chrono::steady_clock::now();
    if (acc >= period)
    {
      acc = 0.0;
      std::vector<uint8_t> p;
      // 一个 1.5m 见方的正方形巡游
      const double phase = static_cast<double>(seq % 4);
      const double xs[4] = {1.5, 1.5, 0.0, 0.0};
      const double ys[4] = {0.0, 1.5, 1.5, 0.0};
      wrI32(p, static_cast<int32_t>(xs[static_cast<int>(phase)] * 100.0));
      wrI32(p, static_cast<int32_t>(ys[static_cast<int>(phase)] * 100.0));
      wrI16(p, 0);
      p.push_back(0x00);  // no yaw
      p.push_back(seq);

      uint8_t frame[32];
      const size_t n = encodeFrame(kCmdNavTo, p, frame, sizeof(frame));
      if (n > 0)
      {
        feedBytes(frame, n);
      }
      ++seq;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    acc += 0.05;
    (void)t0;
  }

  ready_.store(false);
}

// -----------------------------------------------------------------------------
// 帧提取状态机
// -----------------------------------------------------------------------------
// ★★ 本函数**同时**处理两套帧格式，靠帧头字节区分 ★★
// -----------------------------------------------------------------------------
// 为什么需要两套：
//   赛题规范（doc/AIR_GROUND_INTERFACE.md §3）要求车端用于**空地协同协议 v2**：
//       A5 C3 | VER | TYP | SEQ(4) | LEN(2) | PAYLOAD | CRC16(CCITT)
//     无人机的侦察数据（地雷/障碍/打击区/门位）与任务指令都走这套。
//   但项目里还留着**串口任务指令 v1**：
//       AA 55 | LEN | CMD | PAYLOAD | CRC16(MODBUS)
//     早期上位机/旧固件用它，为兼容而保留。
//
// 两套帧头不同（0xA5 0xC3 vs 0xAA 0x55），命令码也**互不重叠**
//   （air 任务码 0x00..0x0A，串口指令码 >= 0x10），所以可以安全共存。
//
// ★ 修复记录：此前 feedBytes **只认 v1**（同步找 AA 55、LEN 取第 3 字节、
//   CRC 算 MODBUS），于是无人机发的 v2 帧永远同步不上、CRC 必然失败，
//   四类侦察数据全部收不到 —— 排雷(40分)与桥洞(20分)完全失效。
// -----------------------------------------------------------------------------
namespace
{
constexpr uint8_t kV1Head0 = 0xAA;
constexpr uint8_t kV1Head1 = 0x55;

// ★ 格边长（米）——用于把 v2 的**格坐标**换算成 v1 的**米坐标**
//   必须与 FieldGrid 的 spec_.cell_size 一致（field_grid.hpp 默认 0.35 m）。
//   这里不 include field_grid.hpp 是为了不让传输层依赖场地模型；
//   若将来场地格边长改了，**两处都要改**（topology_v5_check.py 会校验场规格，
//   但不会校验这个常量，所以改场地时请一并搜索 kCellSizeM）。
constexpr double kCellSizeM = 0.35;

inline uint32_t rdU32le(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline uint16_t rdU16le(const uint8_t *p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

void CmdLink::feedBytes(const uint8_t *data, size_t len)
{
  if (data == nullptr || len == 0)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(rx_mutex_);
    rx_.insert(rx_.end(), data, data + len);
  }

  for (;;)
  {
    // 帧缓冲要能装下 air v2 的最大帧（1036B），v1 的 96B 只是它的子集
    uint8_t frame[air::kMaxFrameLen];
    size_t flen = 0;
    bool is_air = false;

    {
      std::lock_guard<std::mutex> lk(rx_mutex_);

      // ---- 1) 同步：找最近的一个帧头（两种都找，取更靠前的）----
      size_t sync_v2 = rx_.size();
      size_t sync_v1 = rx_.size();
      for (size_t i = 0; i + 1 < rx_.size(); ++i)
      {
        if (sync_v2 == rx_.size() && rx_[i] == air::kMagic0 && rx_[i + 1] == air::kMagic1)
        {
          sync_v2 = i;
        }
        if (sync_v1 == rx_.size() && rx_[i] == kV1Head0 && rx_[i + 1] == kV1Head1)
        {
          sync_v1 = i;
        }
        if (sync_v2 != rx_.size() && sync_v1 != rx_.size())
        {
          break;
        }
      }
      const size_t sync = std::min(sync_v2, sync_v1);

      if (sync == rx_.size())
      {
        // 没找到完整帧头：只保留最后一个**可能是帧头首字节**的字节
        // （A5 或 AA 都可能是下一帧的开头）
        if (!rx_.empty() &&
            (rx_.back() == air::kMagic0 || rx_.back() == kV1Head0))
        {
          const uint8_t last = rx_.back();
          rx_.clear();
          rx_.push_back(last);
        }
        else
        {
          rx_.clear();
        }
        return;
      }
      if (sync > 0)
      {
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(sync));
      }

      is_air = (rx_.size() >= 2 && rx_[0] == air::kMagic0 && rx_[1] == air::kMagic1);

      if (is_air)
      {
        // ---- v2：A5 C3 | VER TYP SEQ(4) LEN(2) | PAYLOAD | CRC(2) ----
        if (rx_.size() < air::kHeaderLen)
        {
          return;   // 头部还没到齐
        }
        if (rx_[2] != air::kVer)
        {
          // 版本不符：丢掉这两个字节重新同步（不要清空整个缓冲，
          // 规范 §5 明确要求"丢弃该帧，继续找下一个 A5 C3"）
          rx_.erase(rx_.begin(), rx_.begin() + 2);
          continue;
        }
        const size_t plen = rdU16le(&rx_[8]);
        if (plen > air::kMaxPayload)
        {
          rx_.erase(rx_.begin(), rx_.begin() + 2);
          continue;
        }
        const size_t total = air::kHeaderLen + plen + air::kCrcLen;
        if (rx_.size() < total)
        {
          return;   // 整帧还没到齐
        }
        std::memcpy(frame, rx_.data(), total);
        flen = total;
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(total));
      }
      else
      {
        // ---- v1：AA 55 | LEN | CMD PAYLOAD | CRC(2) ----
        if (rx_.size() < 3)
        {
          return;
        }
        const size_t len_field = rx_[2];
        if (len_field < 1 || len_field > kMaxPayloadLen)
        {
          // 非法 LEN：丢弃帧头，重新同步
          rx_.erase(rx_.begin(), rx_.begin() + 2);
          continue;
        }
        const size_t total = 3 + len_field + 2;
        if (rx_.size() < total)
        {
          return;
        }
        std::memcpy(frame, rx_.data(), total);
        flen = total;
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(total));
      }
    }

    if (is_air)
    {
      dispatchAirFrame(frame, flen);
    }
    else
    {
      dispatchFrame(frame, flen);
    }
  }
}

void CmdLink::dispatchFrame(const uint8_t *frame_with_head, size_t len)
{
  if (len < 5)
  {
    return;
  }

  std_msgs::msg::UInt8MultiArray raw;
  raw.data.assign(frame_with_head, frame_with_head + len);
  pub_raw_->publish(raw);

  const MissionCommand m =
      decodeMissionCommand(frame_with_head + 2, len - 2);  // 跳过帧头

  if (!m.valid)
  {
    rx_error_count_.fetch_add(1);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[CmdLink] bad frame: %s", m.error.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lk(time_mutex_);
    last_rx_time_ = node_->now();
  }
  rx_frame_count_.fetch_add(1);

  publishCommand(m, frame_with_head, len);
}

// -----------------------------------------------------------------------------
// v2（空地协同协议）帧 -> MissionCommand -> /mission_cmd
// -----------------------------------------------------------------------------
// ★ 为什么要做这一层转换：
//   mission_manager 只认 MissionCommand（帧头已剥离、字段已语义化）。
//   把 v2 解析放在 CmdLink 里，mission_manager 就**完全不需要知道**
//   数据是 v1 还是 v2 来的 —— 它那 13 个 handle* 一行都不用改。
//
// ★ 侦察数据（四类）怎么传：
//   v2 的载荷布局与 v1 的**完全一致**（ts(u32)+count(u8) / bridge_id 等），
//   所以直接 m.payload = 解码后的载荷，再让 publishCommand 的尾部扩展
//   原样透传给 mission_manager（它才做领域校验）。
//   —— 这条通路是排雷(40分)与桥洞(20分)的数据来源。
void CmdLink::dispatchAirFrame(const uint8_t *frame_with_head, size_t len)
{
  const air::Decoded d = air::decodeFrame(frame_with_head, len);

  // 原始帧照发，便于 ros2 topic echo /mission_cmd_raw 抓包排查
  {
    std_msgs::msg::UInt8MultiArray raw;
    raw.data.assign(frame_with_head, frame_with_head + len);
    pub_raw_->publish(raw);
  }

  if (!d.ok)
  {
    rx_error_count_.fetch_add(1);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[CmdLink] bad air frame: %s", d.error.c_str());
    return;
  }

  MissionCommand m;
  const uint8_t *p = d.payload.data();
  const size_t n = d.payload.size();

  switch (static_cast<air::Typ>(d.typ))
  {
    // ---------------- 任务指令 ----------------
    case air::kTaskCmd:
    {
      if (n < 2)
      {
        rx_error_count_.fetch_add(1);
        return;
      }
      const uint8_t op = p[0];
      m.seq = p[1];
      switch (op)
      {
        case air::kCmdNop:
          m.cmd = kCmdOpNop;
          break;

        case air::kCmdNavToCell:
          // 载荷：col(u8) row(u8) timeout_s(u8)
          if (n < 5)
          {
            rx_error_count_.fetch_add(1);
            return;
          }
          // ★ 格坐标 -> 米：与 v1 的 (i32 cm) 语义统一，mission_manager 不必区分来源
          m.cmd = kCmdNavTo;
          m.x = (static_cast<double>(p[2]) + 0.5) * kCellSizeM;
          m.y = (static_cast<double>(p[3]) + 0.5) * kCellSizeM;
          m.has_yaw = false;
          break;

        case air::kCmdEngage:
          // 载荷：col(u8) row(u8) target_face(u8)
          if (n < 5)
          {
            rx_error_count_.fetch_add(1);
            return;
          }
          m.cmd = kCmdOpEngage;
          m.x = (static_cast<double>(p[2]) + 0.5) * kCellSizeM;
          m.y = (static_cast<double>(p[3]) + 0.5) * kCellSizeM;
          m.target_id = static_cast<uint8_t>(p[4] + 1);   // face(0/1/2) -> target_id(1/2/3)
          break;

        case air::kCmdHold:
          m.cmd = kCmdOpHold;
          break;

        case air::kCmdStop:
          m.cmd = kCmdOpStop;
          m.stop = 1;
          break;

        case air::kCmdReturnBase:
          m.cmd = kCmdOpReturnBase;
          break;

        // ---- 以下 v2 任务码在本车端没有对应处理，只记日志 ----
        //   BRIDGE_CROSS 的过门判定由 BridgeTracker 依据自身位置完成，
        //   不依赖上位机下发；MINE_CLEAR_* 目前由 MinePlanner 自主排雷。
        case air::kCmdMineClearAll:
        case air::kCmdMineClearOne:
        case air::kCmdBridgeCross:
        case air::kCmdSafeRetreat:
        case air::kCmdNavToPose:
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                               "[CmdLink] 收到 v2 任务码 0x%02X —— 本车端暂不处理该码"
                               "（自主完成，无需上位机下发）", op);
          return;

        default:
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                               "[CmdLink] 未知 v2 任务码 0x%02X", op);
          rx_error_count_.fetch_add(1);
          return;
      }
      break;
    }

    // ---------------- 侦察结果（四类，排雷/桥洞的数据来源）----------------
    case air::kMineList:
    case air::kObstacleList:
    case air::kTargetList:
    {
      // 载荷：ts(u32) count(u8) [records]  —— 与 v1 完全一致
      if (n < 5)
      {
        rx_error_count_.fetch_add(1);
        return;
      }
      m.cmd = (d.typ == air::kMineList)     ? kCmdMineList
              : (d.typ == air::kObstacleList) ? kCmdObstacleList
                                              : kCmdTargetList;
      m.ts_ms = rdU32le(&p[0]);
      m.item_count = p[4];
      m.seq = 0;                                  // 列表类无 seq，回执用 0
      m.payload.assign(d.payload.begin(), d.payload.end());
      break;
    }

    case air::kBridgeGate:
      // 载荷：bridge_id(u8) gate_row(u8) conf(u8) flags(u8) —— 固定 4 字节
      if (n < 4)
      {
        rx_error_count_.fetch_add(1);
        return;
      }
      m.cmd = kCmdBridgeGate;
      m.seq = 0;
      m.payload.assign(d.payload.begin(), d.payload.end());
      break;

    case air::kFieldFixed:
      // ★ 场地固定结构 —— 必须转成指令！
      //   曾经把 kFieldFixed 归入"不产生指令"的忽略列表，于是 v2.1 的
      //   无人机起降点永远到不了 mission_manager，两格禁入形同虚设。
      //   载荷（v2.1 共 12 字节）：
      //     up col_lo col_hi bcol a_lo a_hi b_lo b_hi
      //     + drone_start(col,row) + drone_end(col,row)
      if (n < 8)
      {
        rx_error_count_.fetch_add(1);
        return;
      }
      m.cmd = kCmdFieldFixed;
      m.seq = 0;
      m.payload.assign(d.payload.begin(), d.payload.end());
      break;

    case air::kMapFull:
      // 载荷：map_version(u32) ts(u32) cols(u8) rows(u8) [cell(u16)]*
      if (n < 10)
      {
        rx_error_count_.fetch_add(1);
        return;
      }
      m.cmd = kCmdMapFull;
      m.ts_ms = rdU32le(&p[4]);
      m.seq = 0;
      m.payload.assign(d.payload.begin(), d.payload.end());
      break;

    // ---------------- 其余类型不需要转成 MissionCommand ----------------
    //   HEARTBEAT / TEAM_INFO —— 仅用于链路保活与颜色告知
    //   VEH_STATUS / EVENT / ACK / ROUTE —— 方向相反或不处理
    //   ★ 注意 kFieldFixed **不在这里** —— 它承载无人机起降点，必须下发
    case air::kHeartbeat:
    case air::kTeamInfo:
    case air::kMapDelta:
    case air::kRoute:
    case air::kVehStatus:
    case air::kEvent:
    case air::kAck:
      // 心跳类只刷新链路时间，不产生指令
      break;

    default:
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[CmdLink] 未知 v2 帧类型 0x%02X（%zu B）", d.typ, n);
      return;
  }

  m.valid = true;

  {
    std::lock_guard<std::mutex> lk(time_mutex_);
    last_rx_time_ = node_->now();
  }
  rx_frame_count_.fetch_add(1);

  publishCommand(m, frame_with_head, len);
}

// -----------------------------------------------------------------------------
// 指令 -> ROS 话题
// -----------------------------------------------------------------------------
void CmdLink::publishCommand(const MissionCommand &m, const uint8_t *raw, size_t raw_len)
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = cmdName(m.cmd);
  msg.layout.dim[0].size = 0;
  msg.layout.dim[0].stride = 1;
  msg.layout.data_offset = 0;

  // 紧凑编码：cmd, seq, target_id, confidence, stop, has_yaw, bearing_deg, range_m, x, y, yaw
  msg.data.resize(16);
  msg.data[0] = m.cmd;
  msg.data[1] = m.seq;
  msg.data[2] = m.target_id;
  msg.data[3] = m.confidence;
  msg.data[4] = m.stop;
  msg.data[5] = static_cast<uint8_t>(m.has_yaw ? 1 : 0);
  msg.data[6] = 0;
  msg.data[7] = 0;
  msg.data[8] = 0;
  msg.data[9] = 0;
  msg.data[10] = 0;
  msg.data[11] = 0;
  msg.data[12] = 0;
  msg.data[13] = 0;
  msg.data[14] = 0;
  msg.data[15] = 0;

  const int16_t bearing = static_cast<int16_t>(m.bearing_deg * 10.0);
  msg.data[6] = static_cast<uint8_t>(bearing & 0xFF);
  msg.data[7] = static_cast<uint8_t>((bearing >> 8) & 0xFF);
  const uint16_t range_cm = static_cast<uint16_t>(m.range_m * 100.0);
  msg.data[8] = static_cast<uint8_t>(range_cm & 0xFF);
  msg.data[9] = static_cast<uint8_t>((range_cm >> 8) & 0xFF);

  msg.layout.dim[0].size = static_cast<int>(msg.data.size());

  // ===========================================================================
  // ★★ 批量侦察消息的载荷透传（向后兼容的尾部扩展）★★
  // ---------------------------------------------------------------------------
  // 背景：本话题（mission_cmd）用的是**扁平紧凑编码**（前 16 字节放标量字段），
  //       而无人机下发的四类侦察消息（地雷/障碍/打击区列表）是**变长列表**，
  //       塞不进固定字段。若不透传，这些数据会在 CmdLink 层被**静默丢弃**。
  //
  // ★ 为什么用"尾部扩展"而不是改前 16 字节的布局：
  //     mission_manager.cpp 的 onMissionCommand 按固定下标读 [0]..[9]，
  //     并且有 `msg->data.size() < 16` 的守卫。尾部追加**不影响**任何
  //     既有下标，老代码/老订阅者完全不受影响 —— 这是最安全的扩展方式。
  //
  // 追加布局（只在列表类消息上出现）：
  //     [16..19] ts_ms (u32, 小端)
  //     [20]     item_count (u8)
  //     [21..]   payload（与协议帧里的载荷一字不差）
  //
  // ★ 注意：这里**不做裁剪** —— payload 原样透传，领域校验交给
  //   MissionManager（只有它知道场地规格）。少了这层透传，本赛题的
  //   排雷(40分)与桥洞(20分)全都拿不到数据。
  // ===========================================================================
  if (!m.payload.empty())
  {
    const size_t base = msg.data.size();
    msg.data.resize(base + 5 + m.payload.size(), 0);
    msg.data[base + 0] = static_cast<uint8_t>(m.ts_ms & 0xFF);
    msg.data[base + 1] = static_cast<uint8_t>((m.ts_ms >> 8) & 0xFF);
    msg.data[base + 2] = static_cast<uint8_t>((m.ts_ms >> 16) & 0xFF);
    msg.data[base + 3] = static_cast<uint8_t>((m.ts_ms >> 24) & 0xFF);
    msg.data[base + 4] = m.item_count;
    for (size_t i = 0; i < m.payload.size(); ++i)
    {
      msg.data[base + 5 + i] = m.payload[i];
    }
    msg.layout.dim[0].size = static_cast<int>(msg.data.size());
  }

  (void)raw;
  (void)raw_len;
  pub_cmd_->publish(msg);
}

// -----------------------------------------------------------------------------
bool CmdLink::sendFrame(const uint8_t *frame, size_t len)
{
  if (frame == nullptr || len == 0)
  {
    return false;
  }
  std::lock_guard<std::mutex> lk(tx_mutex_);

  if (transport_ == "serial")
  {
    if (serial_handle_ == nullptr)
    {
      return false;
    }
    auto *ser = static_cast<serial::Serial *>(serial_handle_);
    try
    {
      return ser->write(frame, len) == len;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[CmdLink] serial write failed: %s", e.what());
      return false;
    }
  }

  if (transport_ == "udp")
  {
#ifdef _WIN32
    return false;
#else
    if (udp_fd_ < 0)
    {
      return false;
    }
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(static_cast<uint16_t>(udp_peer_port_));
    peer.sin_addr.s_addr = ::inet_addr(udp_peer_host_.c_str());
    const ssize_t n = ::sendto(udp_fd_, frame, len, 0,
                               reinterpret_cast<sockaddr *>(&peer), sizeof(peer));
    return n == static_cast<ssize_t>(len);
#endif
  }

  // loopback：记录在日志里，便于离线观察状态回传是否正常
  RCLCPP_DEBUG(node_->get_logger(), "[CmdLink] loopback drop %zu byte report frame", len);
  return true;
}

bool CmdLink::linkAlive() const
{
  if (transport_ == "loopback")
  {
    return ready_.load();
  }
  return lastRxAge() * 1000.0 < static_cast<double>(link_timeout_ms_);
}

double CmdLink::lastRxAge() const
{
  std::lock_guard<std::mutex> lk(time_mutex_);
  return (node_->now() - last_rx_time_).seconds();
}

}  // namespace dzmission
