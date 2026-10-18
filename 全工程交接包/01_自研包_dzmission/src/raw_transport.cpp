#include "dzmission/raw_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include <serial/serial.h>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace dzmission
{

namespace
{
constexpr size_t kChunk = 1024;
constexpr size_t kRxCap = 2048;   // 允许的最大 UDP 数据报

double wallNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void logHex(rclcpp::Logger lg, const char *tag, const uint8_t *d, size_t n)
{
  std::string s;
  char b[4];
  const size_t lim = std::min<size_t>(n, 64);
  for (size_t i = 0; i < lim; ++i)
  {
    std::snprintf(b, sizeof(b), "%02X ", d[i]);
    s += b;
  }
  if (n > lim)
  {
    s += "...";
  }
  RCLCPP_INFO(lg, "[Transport] %s %zu B: %s", tag, n, s.c_str());
}
}  // namespace

RawTransport::RawTransport(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  (void)ns;
}

RawTransport::~RawTransport() { stop(); }

void RawTransport::start()
{
  if (running_.load())
  {
    return;
  }
  running_.store(true);

  if (transport_ == "serial")
  {
    worker_ = std::thread(&RawTransport::serialLoop, this);
  }
  else if (transport_ == "udp")
  {
    worker_ = std::thread(&RawTransport::udpLoop, this);
  }
  else
  {
    // loopback：不产生任何合成数据，RX 只能靠 injectRx()
    worker_ = std::thread(&RawTransport::loopbackIdleLoop, this);
  }
}

void RawTransport::stop()
{
  if (!running_.exchange(false))
  {
    return;
  }
  if (worker_.joinable())
  {
    worker_.join();
  }
  // 关闭句柄（loop 内也会做，这里兜底）
  {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    if (serial_handle_ != nullptr)
    {
      auto *ser = static_cast<serial::Serial *>(serial_handle_);
      serial_handle_ = nullptr;
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
      delete ser;
    }
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
void RawTransport::deliver(const uint8_t *data, size_t len)
{
  if (len == 0)
  {
    return;
  }
  rx_bytes_.fetch_add(len);
  last_rx_wall_.store(wallNow());
  if (rx_cb_)
  {
    rx_cb_(data, len);
  }
}

void RawTransport::injectRx(const uint8_t *data, size_t len)
{
  deliver(data, len);
}

// -----------------------------------------------------------------------------
// 串口：读**带超时**，所以循环能及时看到 running_ 变 false 并退出。
// 读与写共用 fd，因此都在 tx_mutex_ 下操作 —— 并发读写同一个 fd 会导致
// 帧错位（最坏情况永远凑不齐长度）。
// -----------------------------------------------------------------------------
void RawTransport::serialLoop()
{
  auto *ser = new serial::Serial();
  {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    serial_handle_ = ser;
  }

  try
  {
    ser->setPort(serial_port_);
    ser->setBaudrate(static_cast<uint32_t>(serial_baud_));
    // ★ 读超时：没数据时 read() 在 timeout 后返回 0，而不是永久阻塞
    ser->setTimeout(serial::Timeout::simpleTimeout(
        static_cast<uint32_t>(std::max(10, serial_read_timeout_ms_))));
    ser->open();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[Transport] open %s failed: %s",
                 serial_port_.c_str(), e.what());
    std::lock_guard<std::mutex> lk(tx_mutex_);
    if (serial_handle_ == ser)
    {
      serial_handle_ = nullptr;
    }
    delete ser;
    ready_.store(false);
    return;
  }

  if (!ser->isOpen())
  {
    RCLCPP_ERROR(node_->get_logger(), "[Transport] %s not open", serial_port_.c_str());
    std::lock_guard<std::mutex> lk(tx_mutex_);
    if (serial_handle_ == ser)
    {
      serial_handle_ = nullptr;
    }
    delete ser;
    ready_.store(false);
    return;
  }

  ready_.store(true);
  RCLCPP_INFO(node_->get_logger(), "[Transport] serial %s@%d opened (read timeout %dms)",
              serial_port_.c_str(), serial_baud_, serial_read_timeout_ms_);

  std::vector<uint8_t> chunk(kChunk);
  while (running_.load() && rclcpp::ok())
  {
    size_t got = 0;
    {
      std::lock_guard<std::mutex> lk(tx_mutex_);
      try
      {
        // read() 会一直等到至少 1 字节或超时
        got = ser->read(chunk.data(), chunk.size());
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(node_->get_logger(), "[Transport] serial read failed: %s", e.what());
        break;
      }
    }
    if (got > 0)
    {
      deliver(chunk.data(), got);
    }
  }

  {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    if (serial_handle_ == ser)
    {
      serial_handle_ = nullptr;
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
      delete ser;
    }
  }
  ready_.store(false);
  RCLCPP_INFO(node_->get_logger(), "[Transport] serial loop exit");
}

// -----------------------------------------------------------------------------
void RawTransport::udpLoop()
{
#ifdef _WIN32
  RCLCPP_ERROR(node_->get_logger(),
               "[Transport] udp transport unsupported on this platform");
  ready_.store(false);
  return;
#else
  udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_fd_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[Transport] socket() failed: %s",
                 std::strerror(errno));
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
    RCLCPP_ERROR(node_->get_logger(), "[Transport] bind %s:%d failed: %s",
                 udp_bind_host_.c_str(), udp_bind_port_, std::strerror(errno));
    ::close(udp_fd_);
    udp_fd_ = -1;
    ready_.store(false);
    return;
  }

  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = static_cast<suseconds_t>(
      std::max(10, udp_recv_timeout_ms_) * 1000);
  ::setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // 接收缓冲要够大，否则 MAP_FULL（330B 帧）会被截断
  int rcvbuf = 64 * 1024;
  ::setsockopt(udp_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  ready_.store(true);
  RCLCPP_INFO(node_->get_logger(),
              "[Transport] udp bound %s:%d -> peer %s:%d",
              udp_bind_host_.c_str(), udp_bind_port_,
              udp_peer_host_.c_str(), udp_peer_port_);

  std::vector<uint8_t> chunk(kRxCap);
  while (running_.load() && rclcpp::ok())
  {
    const ssize_t n = ::recv(udp_fd_, chunk.data(), chunk.size(), 0);
    if (n > 0)
    {
      deliver(chunk.data(), static_cast<size_t>(n));
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      RCLCPP_WARN(node_->get_logger(), "[Transport] recv failed: %s",
                  std::strerror(errno));
      break;
    }
  }

  ::close(udp_fd_);
  udp_fd_ = -1;
  ready_.store(false);
  RCLCPP_INFO(node_->get_logger(), "[Transport] udp loop exit");
#endif
}

// -----------------------------------------------------------------------------
// loopback：标记就绪，不产生任何合成数据。
// 合成帧由 AirLink 注入（它才知道 v2 协议的载荷），传输层不掺和协议。
// -----------------------------------------------------------------------------
void RawTransport::loopbackIdleLoop()
{
  ready_.store(true);
  RCLCPP_WARN(node_->get_logger(),
              "[Transport] LOOPBACK mode: 无外部链路。"
              "合成帧由 AirLink 注入（injectRx）。");
  while (running_.load() && rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ready_.store(false);
}

// -----------------------------------------------------------------------------
bool RawTransport::send(const uint8_t *data, size_t len)
{
  if (len == 0 || data == nullptr)
  {
    return true;
  }
  if (log_tx_)
  {
    logHex(node_->get_logger(), "TX", data, len);
  }

  if (transport_ == "serial")
  {
    std::lock_guard<std::mutex> lk(tx_mutex_);
    auto *ser = static_cast<serial::Serial *>(serial_handle_);
    if (ser == nullptr || !ser->isOpen())
    {
      return false;
    }
    try
    {
      const size_t w = ser->write(data, len);
      tx_bytes_.fetch_add(w);
      return w == len;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "[Transport] serial write failed: %s", e.what());
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
    const ssize_t n = ::sendto(udp_fd_, data, len, 0,
                               reinterpret_cast<sockaddr *>(&peer), sizeof(peer));
    if (n == static_cast<ssize_t>(len))
    {
      tx_bytes_.fetch_add(len);
      return true;
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[Transport] sendto failed: %s", std::strerror(errno));
    return false;
#endif
  }

  // loopback：只记账，不外发
  tx_bytes_.fetch_add(len);
  return true;
}

}  // namespace dzmission
