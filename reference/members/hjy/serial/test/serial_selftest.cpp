// =============================================================================
// serial 库自测
// =============================================================================
// 运行：  ros2 run serial serial_selftest
//     或  ./serial_selftest            （不依赖 ROS）
//
// ★★ 本程序能验证什么、不能验证什么 ★★
//
// 能验证（不需要真实串口）：
//   · 头文件与链接可用（能找到 serial/serial.h、能链上 libserial）
//   · Timeout::simpleTimeout 的语义
//   · 未打开时 isOpen() 为 false
//   · 未打开时 read/write/available 的边界行为（不崩溃）
//   · open() 对不存在的设备**抛异常**而不是静默失败
//   · 错误信息里包含端口名（便于现场定位）
//   · setPort/getPort、setBaudrate/getBaudrate、setTimeout/getTimeout 往返一致
//
// **不能**验证（必须上车接真实设备）：
//   · 波特率是否真的生效（尤其非标准波特率走 termios2 分支）
//   · read 的超时是否精确（本实现用 select，实际抖动需实测）
//   · 8N1/无流控是否与底盘、裁判板匹配
//   · 长时收发是否丢字节
//   → 这些是硬件相关，本程序**故意不假装能测**
//
// 若提供了真实串口，可加参数做回路测试：
//   ./serial_selftest /dev/ttyUSB0 115200
//   （需把该串口的 TX-RX 短接做自环）
// =============================================================================

#include "serial/serial.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <cstring>
#include <exception>
#include <string>

static int g_pass = 0;
static int g_fail = 0;

static void expect(bool cond, const std::string &what)
{
  if (cond)
  {
    ++g_pass;
    std::printf("  [ OK ] %s\n", what.c_str());
  }
  else
  {
    ++g_fail;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

static void sec(const char *t)
{
  std::printf("\n=== %s ===\n", t);
}

// -----------------------------------------------------------------------------
static void testTimeout()
{
  sec("1. Timeout 语义");

  serial::Timeout t = serial::Timeout::simpleTimeout(20);
  expect(t.read_timeout_constant == 20, "simpleTimeout(20) -> read_timeout=20");
  expect(t.write_timeout_constant == 20, "simpleTimeout(20) -> write_timeout=20");
  expect(t.inter_byte_timeout == 0, "simpleTimeout -> inter_byte=0");

  serial::Timeout t2 = serial::Timeout::simpleTimeout(0);
  expect(t2.read_timeout_constant == 0, "simpleTimeout(0) 可用（非阻塞语义）");

  serial::Timeout t3 = serial::Timeout::simpleTimeout(1500);
  expect(t3.read_timeout_constant == 1500, "大超时值不溢出");
}

// -----------------------------------------------------------------------------
static void testUnopenedBehaviour()
{
  sec("2. 未打开时的行为（必须安全，不能崩溃）");

  serial::Serial s;
  expect(!s.isOpen(), "默认构造后 isOpen() == false");
  expect(s.available() == 0, "未打开时 available() == 0");

  uint8_t buf[8];
  const size_t n = s.read(buf, sizeof(buf));
  expect(n == 0, "未打开时 read() 返回 0（不崩溃）");

  const std::string r = s.read(4);
  expect(r.empty(), "未打开时 read(size) 返回空串");

  const size_t w = s.write(buf, sizeof(buf));
  expect(w == 0, "未打开时 write() 返回 0（不崩溃）");

  const size_t w2 = s.write(std::string("abc"));
  expect(w2 == 0, "未打开时 write(string) 返回 0");

  // 调用 close 两次不应崩
  s.close();
  s.close();
  expect(true, "重复 close() 安全");
}

// -----------------------------------------------------------------------------
static void testAccessors()
{
  sec("3. 配置项往返（set/get）");

  serial::Serial s;

  s.setPort("/dev/ttyTEST0");
  expect(s.getPort() == "/dev/ttyTEST0", "setPort/getPort 往返一致");

  const uint32_t b = s.setBaudrate(921600);
  expect(b == 921600, "setBaudrate 返回生效值");
  expect(s.getBaudrate() == 921600, "getBaudrate 一致");

  serial::Timeout t = serial::Timeout::simpleTimeout(37);
  s.setTimeout(t);
  expect(s.getTimeout().read_timeout_constant == 37, "setTimeout/getTimeout 往返一致");

  // 未打开时 setBaudrate 不应抛异常
  bool threw = false;
  try
  {
    s.setBaudrate(115200);
  }
  catch (const std::exception &)
  {
    threw = true;
  }
  expect(!threw, "未打开时 setBaudrate 不抛异常");
}

// -----------------------------------------------------------------------------
static void testOpenFailure()
{
  sec("4. open() 失败必须**抛出异常**（不能静默）");

  serial::Serial s;
  s.setPort("/dev/definitely_not_exist_dz_test");
  s.setBaudrate(115200);
  s.setTimeout(serial::Timeout::simpleTimeout(20));

  bool threw = false;
  std::string msg;
  try
  {
    s.open();
  }
  catch (const serial::SerialException &e)
  {
    threw = true;
    msg = e.what();
  }
  catch (const std::exception &e)
  {
    threw = true;
    msg = e.what();
  }

  expect(threw, "open() 对不存在的设备抛异常");
  expect(!s.isOpen(), "失败后 isOpen() 仍为 false");
  // ★ 错误信息必须含端口名：现场有多个串口（底盘/云台/裁判板），
  //   没有端口名就无法定位是哪个没接上。
  expect(msg.find("definitely_not_exist") != std::string::npos,
         "异常信息包含端口名（现场定位必需）");
  if (!msg.empty())
  {
    std::printf("         异常信息: %s\n", msg.c_str());
  }

  // 抛异常后对象仍可复用（设置新端口再试）
  s.setPort("/dev/also_not_exist");
  bool threw2 = false;
  try
  {
    s.open();
  }
  catch (const std::exception &)
  {
    threw2 = true;
  }
  expect(threw2, "抛异常后对象仍可复用（再次 open 仍正确报错）");
}

// -----------------------------------------------------------------------------
static void testLoopback(const std::string &port, uint32_t baud)
{
  sec("5. 真实串口自环测试（需 TX-RX 短接）");

  std::printf("  端口: %s  波特率: %u\n", port.c_str(), baud);

  serial::Serial s;
  s.setPort(port);
  s.setBaudrate(baud);
  s.setTimeout(serial::Timeout::simpleTimeout(300));

  try
  {
    s.open();
  }
  catch (const std::exception &e)
  {
    std::printf("  [SKIP] 打不开 %s: %s\n", port.c_str(), e.what());
    std::printf("         （未接硬件时正常，不算失败）\n");
    return;
  }

  expect(s.isOpen(), "打开成功");

  // 发一段数据，看能否收回来（自环）
  const std::string tx = "DZLOOP123";
  s.flush();
  const size_t w = s.write(tx);
  expect(w == tx.size(), "write 写入了全部字节");

  // 等一小会儿让数据回环
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const std::string rx = s.read(tx.size());
  std::printf("         发送: %s\n", tx.c_str());
  std::printf("         接收: %s\n", rx.empty() ? "(空)" : rx.c_str());

  if (rx.empty())
  {
    std::printf("  [INFO] 未收到数据 —— 若 TX/RX 未短接，这是正常的\n");
  }
  else
  {
    expect(rx == tx, "自环数据一致（收发链路正常）");
  }

  s.close();
  expect(!s.isOpen(), "close() 后 isOpen() == false");
}

// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  std::printf("================================================================\n");
  std::printf(" serial 库自测（POSIX termios 实现）\n");
  std::printf(" 用途：dzmission 与 STM32 底盘/云台/裁判板 的串口通信\n");
  std::printf("================================================================\n");

  testTimeout();
  testUnopenedBehaviour();
  testAccessors();
  testOpenFailure();

  if (argc >= 3)
  {
    testLoopback(argv[1], static_cast<uint32_t>(std::atoi(argv[2])));
  }
  else
  {
    sec("5. 真实串口自环测试 —— 未提供端口，跳过");
    std::printf("  用法: %s /dev/ttyUSB0 115200   （需 TX-RX 短接）\n", argv[0]);
  }

  std::printf("\n================================================================\n");
  std::printf(" 结果：通过 %d，失败 %d\n", g_pass, g_fail);
  if (g_fail == 0)
  {
    std::printf(" ✓ 纯逻辑部分全部通过\n");
    std::printf("   ★ 注意：**波特率是否真生效、超时是否精确**，\n");
    std::printf("     必须接真实设备验证 —— 本程序测不了这两项。\n");
  }
  else
  {
    std::printf(" ✗ 有失败项\n");
  }
  std::printf("================================================================\n");
  return g_fail == 0 ? 0 : 1;
}
