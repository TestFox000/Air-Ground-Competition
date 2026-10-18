// =============================================================================
// serial::Serial —— POSIX termios 实现
// =============================================================================
// 设计说明见 serial.h 顶部。本文件只做一件事：
//   把 termios 的原始接口包装成原库（wjwwood/serial）的语义。
//
// ★ 关键语义（与原库对齐，调用方依赖这些行为）：
//   · read() 在 read_timeout 内**至少等到 1 字节**；
//     若超时且一字节都没有 -> 返回空（不抛异常）—— 调用方的
//     `if (!buf.empty())` 判断依赖这一点。
//   · open() 失败 -> 抛 SerialException（调用方已 catch）
//   · isOpen() 必须真实反映 fd 状态
// =============================================================================

#include "serial/serial.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

// =============================================================================
// 非标准波特率支持（Linux termios2 / BOTHER）
// =============================================================================
// ★★ 这里踩过两个坑，都记录下来，避免以后重犯 ★★
//
// 【坑 1】不能同时 #include <termios.h> 与 <asm/termbits.h>
//   glibc 的 <termios.h>（经 bits/termios-struct.h）与内核的
//   <asm/termbits.h>（经 asm-generic/termbits.h）**都定义 struct termios**：
//       error: redefinition of 'struct termios'
//
// 【坑 2】也不能自己 #define TCGETS2
//   glibc 的 <bits/ioctls.h>（<sys/ioctl.h> 会带上）**已经定义过** TCGETS2，
//   而它引用的是 `struct termios2` —— 一个只被**前向声明**、并不完整的类型：
//       #define TCGETS2 _IOR('T', 0x2A, struct termios2)
//   于是 `#ifndef TCGETS2` 被跳过，我们的定义没生效，
//   宏展开出 sizeof(struct termios2) 就报：
//       error: invalid application of 'sizeof' to incomplete type 'struct termios2'
//
// ★ 最终做法（同时绕开两个坑）：
//   · 不包含任何内核头文件
//   · 用**我们自己命名的宏** DZ_TCGETS2 / DZ_TCSETS2（不与 glibc 撞名）
//   · 结构用我们自己的名字 serial_termios2
//   · 布局与 Linux 内核 struct termios2 完全一致（ABI 稳定，是通用做法）
//
// ★ 适用场景：波特率不在 termios.h 预定义集合里时走这里。
//   标准波特率（含 115200）走 standardBaud() 分支，**不执行本段**。
//   即便这里失败，也只是返回 false -> open() 抛含端口名的异常，
//   不会静默用错波特率。
// =============================================================================
#if defined(__linux__) && !defined(SERIAL_DISABLE_CUSTOM_BAUD)
#define SERIAL_HAVE_TERMIOS2 1

// Linux 内核 struct termios2 的完整布局（与 asm-generic/termbits.h 一致）
struct serial_termios2
{
  unsigned int c_iflag;
  unsigned int c_oflag;
  unsigned int c_cflag;
  unsigned int c_lflag;
  unsigned char c_line;
  unsigned char c_cc[19];
  unsigned int c_ispeed;
  unsigned int c_ospeed;
};

// 自定义波特率标志（内核值 0010000，ABI 稳定）
// ★ 同样用自有宏名，理由与 DZ_TCGETS2 相同（避免被系统的定义静默替换）
#define DZ_BOTHER 0010000

// ★ 用自己的宏名，避免与 glibc 的 TCGETS2/TCSETS2 冲突（见坑 2）
//   _IOR/_IOW 由 <sys/ioctl.h> 提供。
#define DZ_TCGETS2 _IOR('T', 0x2A, struct serial_termios2)
#define DZ_TCSETS2 _IOW('T', 0x2B, struct serial_termios2)

#endif  // __linux__ && !SERIAL_DISABLE_CUSTOM_BAUD

namespace serial
{

namespace
{

// 标准波特率 -> termios 常量
bool standardBaud(uint32_t baud, speed_t &out)
{
  switch (baud)
  {
    case 50: out = B50; return true;
    case 75: out = B75; return true;
    case 110: out = B110; return true;
    case 134: out = B134; return true;
    case 150: out = B150; return true;
    case 200: out = B200; return true;
    case 300: out = B300; return true;
    case 600: out = B600; return true;
    case 1200: out = B1200; return true;
    case 1800: out = B1800; return true;
    case 2400: out = B2400; return true;
    case 4800: out = B4800; return true;
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 500000: out = B500000; return true;
    case 576000: out = B576000; return true;
    case 921600: out = B921600; return true;
    case 1000000: out = B1000000; return true;
    case 1152000: out = B1152000; return true;
    case 1500000: out = B1500000; return true;
    case 2000000: out = B2000000; return true;
    case 2500000: out = B2500000; return true;
    case 3000000: out = B3000000; return true;
    case 3500000: out = B3500000; return true;
    case 4000000: out = B4000000; return true;
    default: return false;
  }
}

// 把波特率数值配置到 fd 上（先试标准，失败再试 termios2 自定义）
bool applyBaudrate(int fd, uint32_t baud)
{
  speed_t sp = B115200;
  if (standardBaud(baud, sp))
  {
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0)
    {
      return false;
    }
    cfsetospeed(&tio, sp);
    cfsetispeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
      return false;
    }
    return true;
  }

#ifdef SERIAL_HAVE_TERMIOS2
  // 非标准波特率：用 termios2 + BOTHER（Linux 上的通用做法）
  struct serial_termios2 tio2;
  memset(&tio2, 0, sizeof(tio2));
  if (ioctl(fd, DZ_TCGETS2, &tio2) == 0)
  {
    tio2.c_cflag &= ~static_cast<unsigned int>(CBAUD);
    tio2.c_cflag |= static_cast<unsigned int>(DZ_BOTHER);
    tio2.c_ispeed = baud;
    tio2.c_ospeed = baud;
    if (ioctl(fd, DZ_TCSETS2, &tio2) == 0)
    {
      return true;
    }
  }
#endif
  return false;
}

}  // namespace

// -----------------------------------------------------------------------------
Serial::Serial() = default;

Serial::Serial(const std::string &port, uint32_t baudrate, const Timeout &timeout)
    : port_(port), baud_(baudrate), timeout_(timeout)
{
  open();
}

Serial::~Serial()
{
  close();
}

void Serial::setPort(const std::string &port)
{
  port_ = port;
}

std::string Serial::getPort() const
{
  return port_;
}

uint32_t Serial::setBaudrate(uint32_t baudrate)
{
  baud_ = baudrate;
  // 若已打开，立即生效
  if (fd_ >= 0)
  {
    if (!applyBaudrate(fd_, baud_))
    {
      throw SerialException("setBaudrate failed for " + std::to_string(baudrate));
    }
  }
  return baud_;
}

uint32_t Serial::getBaudrate() const
{
  return baud_;
}

void Serial::setTimeout(const Timeout &timeout)
{
  timeout_ = timeout;
}

Timeout Serial::getTimeout() const
{
  return timeout_;
}

// -----------------------------------------------------------------------------
void Serial::open()
{
  if (fd_ >= 0)
  {
    return;   // 已打开（幂等）
  }
  if (port_.empty())
  {
    throw SerialException("port name is empty");
  }

  // O_NOCTTY：不让串口成为控制终端
  // O_NONBLOCK：先以非阻塞打开，避免某些设备 open 阻塞
  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    throw SerialException("cannot open " + port_ + ": " + strerror(errno));
  }

  struct termios tio;
  if (tcgetattr(fd, &tio) != 0)
  {
    ::close(fd);
    throw SerialException("tcgetattr failed on " + port_ + ": " + strerror(errno));
  }

  // ---- 8N1，无流控，raw 模式 ----
  tio.c_cflag |= (CLOCAL | CREAD);   // 忽略 modem 控制线，允许接收
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;                // 8 数据位
  tio.c_cflag &= ~PARENB;            // 无校验
  tio.c_cflag &= ~CSTOPB;            // 1 停止位
  tio.c_cflag &= ~CRTSCTS;           // 无硬件流控

  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   // raw
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);           // 无软件流控
  tio.c_iflag &= ~(INLCR | ICRNL | IGNCR);          // 不做换行转换
  tio.c_oflag &= ~OPOST;                            // 不做输出处理

  tio.c_cc[VMIN] = 0;    // 配合 select 使用：不阻塞等待固定字节数
  tio.c_cc[VTIME] = 0;   // 由我们自己用 select 控时

  if (tcsetattr(fd, TCSANOW, &tio) != 0)
  {
    ::close(fd);
    throw SerialException("tcsetattr failed on " + port_ + ": " + strerror(errno));
  }

  // ★ 波特率必须在 termios 基本设置**之后**配置：
  //   termios2 分支会整体覆盖 tio2，若先设波特率再设 8N1 会把波特率冲掉。
  if (!applyBaudrate(fd, baud_))
  {
    ::close(fd);
    throw SerialException("cannot set baudrate " + std::to_string(baud_) +
                          " on " + port_);
  }

  // 清空缓冲，避免上次残留数据被当成有效帧
  tcflush(fd, TCIOFLUSH);

  // 恢复阻塞模式（读写超时由 select 控制）
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0)
  {
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
  }

  fd_ = fd;
}

void Serial::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Serial::isOpen() const
{
  return fd_ >= 0;
}

// -----------------------------------------------------------------------------
size_t Serial::available()
{
  if (fd_ < 0)
  {
    return 0;
  }
  int n = 0;
  if (ioctl(fd_, FIONREAD, &n) != 0)
  {
    return 0;
  }
  return (n > 0) ? static_cast<size_t>(n) : 0;
}

size_t Serial::read(uint8_t *buffer, size_t size)
{
  if (fd_ < 0 || buffer == nullptr || size == 0)
  {
    return 0;
  }

  const uint32_t tmo_ms = timeout_.read_timeout_constant;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(tmo_ms);

  size_t got = 0;
  while (got < size)
  {
    // 先用 select 等可读（避免阻塞式 read 卡死线程）
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    struct timeval tv;
    if (tmo_ms == 0)
    {
      tv.tv_sec = 0;
      tv.tv_usec = 0;
    }
    else
    {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
      {
        break;   // 超时
      }
      const auto remain = std::chrono::duration_cast<std::chrono::microseconds>(
          deadline - now);
      tv.tv_sec = static_cast<long>(remain.count() / 1000000);
      tv.tv_usec = static_cast<long>(remain.count() % 1000000);
    }

    const int rc = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (rc < 0)
    {
      if (errno == EINTR)
      {
        continue;   // 被信号打断，重试
      }
      throw SerialException("select failed: " + std::string(strerror(errno)));
    }
    if (rc == 0)
    {
      break;   // 超时
    }

    const ssize_t n = ::read(fd_, buffer + got, size - got);
    if (n < 0)
    {
      if (errno == EINTR || errno == EAGAIN)
      {
        continue;
      }
      throw SerialException("read failed: " + std::string(strerror(errno)));
    }
    if (n == 0)
    {
      break;
    }
    got += static_cast<size_t>(n);
    // 已拿到至少 1 字节且缓冲里暂时没有更多 -> 立即返回
    // （这正是原库 simpleTimeout 的行为：不为了填满 size 而等满）
    if (available() == 0)
    {
      break;
    }
  }
  return got;
}

std::string Serial::read(size_t size)
{
  std::vector<uint8_t> buf(size);
  const size_t n = read(buf.data(), size);
  return std::string(reinterpret_cast<char *>(buf.data()), n);
}

size_t Serial::write(const uint8_t *data, size_t size)
{
  if (fd_ < 0 || data == nullptr || size == 0)
  {
    return 0;
  }
  size_t sent = 0;
  while (sent < size)
  {
    const ssize_t n = ::write(fd_, data + sent, size - sent);
    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw SerialException("write failed: " + std::string(strerror(errno)));
    }
    if (n == 0)
    {
      break;
    }
    sent += static_cast<size_t>(n);
  }
  return sent;
}

size_t Serial::write(const std::string &data)
{
  return write(reinterpret_cast<const uint8_t *>(data.data()), data.size());
}

void Serial::flushInput()
{
  if (fd_ >= 0)
  {
    tcflush(fd_, TCIFLUSH);
  }
}

void Serial::flushOutput()
{
  if (fd_ >= 0)
  {
    tcflush(fd_, TCOFLUSH);
  }
}

void Serial::flush()
{
  if (fd_ >= 0)
  {
    tcflush(fd_, TCIOFLUSH);
  }
}

}  // namespace serial
