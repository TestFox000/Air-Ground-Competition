// =============================================================================
// serial::Serial —— 极简串口封装（POSIX termios 实现）
// =============================================================================
// ★★ 为什么自己实现，而不用 wjwwood/serial ★★
//
// 1. 原库 `serial` 是 **ROS 1 时代**的包（catkin 构建），ROS Index 显示它
//    **只发布到 noetic**，ROS 2 Humble 上**没有对应的 apt 包**
//    （`ros-humble-serial` 不存在）。
// 2. 从源码编译的前提（GitHub 可达）在本环境**不成立** ——
//    `raw.githubusercontent.com` 已确认不通（rosdep update 也因它失败）。
// 3. 本代码实际只用 **9 个 API**（见下），实现量很小，而串口是与
//    STM32 底盘 / 云台 / 裁判板通信的**命脉**，自己掌控比依赖一个
//    2015 年后停更的库更稳妥。
//
// ★ 保持与原库**完全相同的接口**，这样 4 个使用者
//   （cmd_link / raw_transport / referee_board / air_link）
//   **一行都不用改**，只改 CMake 的 find_package 指向。
//
// 用到的 API（与原库签名一致）：
//     serial::Timeout::simpleTimeout(ms)
//     Serial::setPort / setBaudrate / setTimeout
//     Serial::open / close / isOpen / available
//     Serial::read / write
//
// ★★ 实机验证要点（本文件无法在无硬件环境验证的部分）★★
//   · 波特率是否真的生效（尤其非标准波特率走 termios2 分支）
//   · read 的超时语义是否与原库一致（原库是「至少 1 字节或超时」）
//   · 8N1 无流控的设置是否与底盘/裁判板匹配
//   → 这些都是**硬件相关**，必须上车实测。
// =============================================================================

#ifndef DZMISSION_SERIAL_SERIAL_H_
#define DZMISSION_SERIAL_SERIAL_H_

#include <cstddef>
#include <exception>
#include <cstdint>
#include <string>
#include <vector>

namespace serial
{

// -----------------------------------------------------------------------------
// 超时设置（与原库语义一致）
// -----------------------------------------------------------------------------
// 原库的 Timeout 有五段超时，但我们只用 simpleTimeout（读超时）。
// 这里保留结构体的完整字段，避免使用者访问不存在的成员。
struct Timeout
{
  uint32_t inter_byte_timeout{0};
  uint32_t read_timeout_constant{0};
  uint32_t read_timeout_multiplier{0};
  uint32_t write_timeout_constant{0};
  uint32_t write_timeout_multiplier{0};

  explicit Timeout(uint32_t inter_byte, uint32_t read_const,
                   uint32_t read_mult, uint32_t write_const,
                   uint32_t write_mult)
      : inter_byte_timeout(inter_byte),
        read_timeout_constant(read_const),
        read_timeout_multiplier(read_mult),
        write_timeout_constant(write_const),
        write_timeout_multiplier(write_mult)
  {
  }

  // ★ 本代码唯一用到的构造方式
  static Timeout simpleTimeout(uint32_t timeout_ms)
  {
    return Timeout(0, timeout_ms, 0, timeout_ms, 0);
  }
};

// -----------------------------------------------------------------------------
class Serial
{
public:
  Serial();
  // 便捷构造：立刻打开（失败时抛 SerialException —— 与原库一致）
  Serial(const std::string &port, uint32_t baudrate, const Timeout &timeout);
  ~Serial();

  Serial(const Serial &) = delete;
  Serial &operator=(const Serial &) = delete;

  // ---- 配置（须在 open 之前或之后都可调用，与原库一致）----
  void setPort(const std::string &port);
  std::string getPort() const;

  // 返回实际生效的波特率（原库语义）
  uint32_t setBaudrate(uint32_t baudrate);
  uint32_t getBaudrate() const;

  void setTimeout(const Timeout &timeout);
  Timeout getTimeout() const;

  // ---- 打开 / 关闭 ----
  // 失败时抛 SerialException（与原库一致，调用方已按此处理）
  void open();
  void close();
  bool isOpen() const;

  // ---- 读写 ----
  // 缓冲区已有的可读字节数
  size_t available();

  // 读：最多读 size 字节；在 read_timeout 内至少等到 1 字节；
  //     超时且无数据则返回空字符串（与原库语义一致）
  size_t read(uint8_t *buffer, size_t size);
  std::string read(size_t size = 1);

  // 写：返回实际写入字节数
  size_t write(const uint8_t *data, size_t size);
  size_t write(const std::string &data);

  // 丢弃收发缓冲（原库有，部分调用点可能用到）
  void flushInput();
  void flushOutput();
  void flush();

private:
  int fd_{-1};
  std::string port_;
  uint32_t baud_{115200};
  Timeout timeout_{Timeout::simpleTimeout(20)};
};

// -----------------------------------------------------------------------------
// 异常（与原库同名，调用方已有 catch 块依赖它）
// -----------------------------------------------------------------------------
class SerialException : public std::exception
{
public:
  explicit SerialException(const char *what) : what_(what ? what : "") {}
  explicit SerialException(const std::string &what) : what_(what) {}
  const char *what() const noexcept override { return what_.c_str(); }

private:
  std::string what_;
};

class IOException : public SerialException
{
public:
  using SerialException::SerialException;
};

}  // namespace serial

#endif  // DZMISSION_SERIAL_SERIAL_H_
