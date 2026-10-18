#ifndef DZMISSION_RAW_TRANSPORT_HPP
#define DZMISSION_RAW_TRANSPORT_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace dzmission
{

// =============================================================================
// RawTransport —— 纯字节收发（**不做任何帧解析**）
// -----------------------------------------------------------------------------
// 为什么要把传输与协议分开：
//   原来的 CmdLink 把"传输 + v1 帧校验 + 解析 + 发 ROS 话题"揉在一个类里。
//   接入 v2 协议（A5C3）时必须换掉帧校验，但传输骨架（串口/UDP 线程、
//   打开/重连、TX 锁）是好用的、值得保留。
//   所以拆开：传输只负责字节进出，帧同步与协议交给 air::FrameSync / AirLink。
//
// transport 取值：
//   "serial"   —— serial_port / serial_baud；读**带超时**，非阻塞轮询以响应 stop()
//   "udp"      —— bind udp_bind_host:udp_bind_port，回发到 udp_peer_host:udp_peer_port
//   "loopback" —— 无外部链路：TX 打日志，RX 需外部调用 injectRx() 喂数据
//                 （离线自测时由 AirLink 注入合成帧，不再在此注入 v1 指令）
//
// 线程模型：一个接收线程（serial 或 udp）；发送在调用者线程，受 tx_mutex_ 保护。
// =============================================================================
class RawTransport
{
public:
  using RxCallback = std::function<void(const uint8_t *, size_t)>;

  RawTransport(rclcpp::Node *node, const std::string &ns);
  ~RawTransport();

  RawTransport(const RawTransport &) = delete;
  RawTransport &operator=(const RawTransport &) = delete;

  // 每收到一段字节就回调（可能是半帧、多帧、含垃圾）
  void setRxCallback(RxCallback cb) { rx_cb_ = std::move(cb); }

  // 是否把 TX 打成 hex 日志（调试用）
  void setLogTx(bool on) { log_tx_ = on; }

  void start();
  void stop();

  // 发送原始字节。返回是否全部写出。
  bool send(const uint8_t *data, size_t len);

  bool ready() const { return ready_.load(); }
  bool running() const { return running_.load(); }
  const std::string &transport() const { return transport_; }

  // 供离线自测注入字节（loopback 或任何模式下都可用）
  void injectRx(const uint8_t *data, size_t len);

  // 统计
  uint64_t txBytes() const { return tx_bytes_.load(); }
  uint64_t rxBytes() const { return rx_bytes_.load(); }
  // 最近一次实际收到字节的时间（秒，wall clock）；从未收到返回 -1
  double lastRxWall() const { return last_rx_wall_.load(); }

  // ---- 参数（由 MissionManager 从节点读取后填入，或内部用默认值）----
  std::string transport_{"loopback"};
  std::string serial_port_{"/dev/ttyACM1"};
  int serial_baud_{115200};

  std::string udp_bind_host_{"0.0.0.0"};
  int udp_bind_port_{9100};
  std::string udp_peer_host_{"192.168.1.50"};
  int udp_peer_port_{9101};

  // 串口读超时（毫秒）。设小一点以便及时响应 stop()
  int serial_read_timeout_ms_{50};
  // UDP 接收超时（毫秒）
  int udp_recv_timeout_ms_{100};

private:
  void serialLoop();
  void udpLoop();
  void loopbackIdleLoop();

  void deliver(const uint8_t *data, size_t len);

  rclcpp::Node *node_{nullptr};
  RxCallback rx_cb_;

  std::mutex tx_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> ready_{false};
  std::thread worker_;

  std::atomic<uint64_t> tx_bytes_{0};
  std::atomic<uint64_t> rx_bytes_{0};
  std::atomic<double> last_rx_wall_{-1.0};

  // 串口句柄（前向声明为 void*，避免头文件泄漏 serial.h）
  void *serial_handle_{nullptr};
  int udp_fd_{-1};

  bool log_tx_{false};
};

}  // namespace dzmission

#endif  // DZMISSION_RAW_TRANSPORT_HPP
