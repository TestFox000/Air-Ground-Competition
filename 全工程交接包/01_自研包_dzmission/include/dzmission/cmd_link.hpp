#ifndef DZMISSION_CMD_LINK_HPP
#define DZMISSION_CMD_LINK_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "dzmission/mission_command.hpp"

namespace dzmission
{

// =============================================================================
// CmdLink —— 任务指令链路
// -----------------------------------------------------------------------------
// 职责：
//   1. 从无人机/上位机接收指令帧（串口 或 UDP），解析后发布 /mission_cmd
//   2. 提供 sendFrame() 供状态回传使用（同一条链路）
//   3. 维护链路心跳时间，供任务管理器判断链路是否中断
//
// transport 取值：
//   "serial"  —— 使用 serial_port / serial_baud
//   "udp"     —— 绑定 udp_bind_host:udp_bind_port，向 udp_peer_host:udp_peer_port 发送
//   "loopback"—— 无外部链路，按 loopback_hz 周期注入一条 NavTo 指令（离线自测用）
//
// ★★ 支持两套帧格式（靠帧头区分，见 .cpp 的 feedBytes）★★
//   · 空地协同协议 v2（A5 C3 + VER/TYP/SEQ/LEN + CRC-16/CCITT-FALSE）
//     —— 无人机下发的四类侦察数据与任务指令走这套（赛题规范 §3）
//   · 串口任务指令 v1（AA 55 + LEN/CMD + CRC-16/MODBUS）
//     —— 早期上位机/旧固件走这套，保留兼容
//   两套的命令码互不重叠（air 0x00..0x0A vs 串口 >= 0x10），可安全共存。
// =============================================================================
class CmdLink
{
public:
  using FrameCallback = std::function<void(const uint8_t *, size_t)>;

  CmdLink(rclcpp::Node *node, const std::string &ns);
  ~CmdLink();

  CmdLink(const CmdLink &) = delete;
  CmdLink &operator=(const CmdLink &) = delete;

  // 启动接收线程（构造后调用，参数已从节点读入）
  void start();
  void stop();

  // 发送整帧（已含帧头与 CRC）。返回是否成功。
  bool sendFrame(const uint8_t *frame, size_t len);

  // 链路是否存活（最近 link_timeout_ms 内收到过合法帧）
  bool linkAlive() const;

  // 链路是否已建立（串口打开成功 / UDP 绑定成功 / loopback 模式）
  bool linkReady() const { return ready_.load(); }

  // 最近一次接收的时间戳（秒，steady 相对值）
  double lastRxAge() const;

  const std::string &transport() const { return transport_; }

private:
  void serialLoop();
  void udpLoop();
  void loopbackLoop();

  // 帧提取状态机：把字节流喂进来，按帧头分派给下面两个函数
  void feedBytes(const uint8_t *data, size_t len);
  // v1（串口任务指令 AA 55 / CRC-MODBUS）
  void dispatchFrame(const uint8_t *frame_with_head, size_t len);
  // v2（空地协同协议 A5 C3 / CRC-CCITT）
  void dispatchAirFrame(const uint8_t *frame_with_head, size_t len);

  void publishCommand(const MissionCommand &m, const uint8_t *raw, size_t raw_len);

  rclcpp::Node *node_{nullptr};

  std::string transport_{"loopback"};
  std::string serial_port_{"/dev/ttyACM1"};
  int serial_baud_{115200};

  std::string udp_bind_host_{"0.0.0.0"};
  int udp_bind_port_{9100};
  std::string udp_peer_host_{"192.168.1.50"};
  int udp_peer_port_{9101};

  double loopback_hz_{0.2};
  int link_timeout_ms_{1000};

  // 接收缓冲
  std::vector<uint8_t> rx_;
  std::mutex rx_mutex_;

  // 发送锁
  std::mutex tx_mutex_;

  std::atomic<bool> running_{false};
  std::atomic<bool> ready_{false};
  std::thread worker_;

  // 链路时间
  mutable std::mutex time_mutex_;
  rclcpp::Time last_rx_time_;
  std::atomic<uint64_t> rx_frame_count_{0};
  std::atomic<uint64_t> rx_error_count_{0};

  // 串口句柄（前向声明，实现里用 void* 避免头文件泄漏 serial.h）
  void *serial_handle_{nullptr};

  // UDP 句柄
  int udp_fd_{-1};

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_raw_;
};

}  // namespace dzmission

#endif  // DZMISSION_CMD_LINK_HPP
