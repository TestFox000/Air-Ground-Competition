#include "dzuav/UdpBroadcaster.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
namespace dzuav
{
namespace
{
// 协议按小端字节序原样 memcpy 数值（车端 Python 用 "<" 前缀 struct 解析），
// 编译期固化平台假设；Jetson aarch64 为小端。
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "udp protocol assumes a little-endian platform");

// CRC-16/Modbus 查表：与逐位算法（多项式 0xa001）数学等价，每字节 8 次迭代降为 1 次查表。
const std::array<uint16_t, 256>& crcTable()
{
    static const std::array<uint16_t, 256> table = []
    {
        std::array<uint16_t, 256> t{};
        for (int b = 0; b < 256; ++b)
        {
            uint16_t crc = static_cast<uint16_t>(b);
            for (int i = 0; i < 8; ++i)
                crc = crc & 1 ? static_cast<uint16_t>((crc >> 1) ^ 0xa001)
                              : static_cast<uint16_t>(crc >> 1);
            t[b] = crc;
        }
        return t;
    }();
    return table;
}
uint16_t crc(const uint8_t* d, size_t n)
{
    uint16_t c = 0xffff;
    const auto& table = crcTable();
    for (size_t i = 0; i < n; ++i)
        c = static_cast<uint16_t>((c >> 8) ^ table[(c ^ d[i]) & 0xff]);
    return c;
}
}  // namespace
template <class T>
static void put(std::vector<uint8_t>& b, T v)
{
    auto p = reinterpret_cast<uint8_t*>(&v);
    b.insert(b.end(), p, p + sizeof(T));
}
UdpBroadcaster::UdpBroadcaster(uint16_t p) : port_(p)
{
    // socket/setsockopt 失败必须显式抛错：否则 fd_=-1 时程序表面健康、
    // 每帧 sendto 静默失败，车端永远收不到包。
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
        throw std::runtime_error(std::string("cannot create udp socket: ") + std::strerror(errno));
    int yes = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) != 0)
    {
        const int e = errno;
        close(fd_);
        fd_ = -1;
        throw std::runtime_error(std::string("cannot enable SO_BROADCAST: ") + std::strerror(e));
    }
}
UdpBroadcaster::~UdpBroadcaster()
{
    if (fd_ >= 0)
        close(fd_);
}
void UdpBroadcaster::send(uint64_t ts, const std::vector<WorldObject>& o)
{
    buf_.clear();   // 复用容量，免每帧重新分配
    // count 字段仅 uint16：防御性截断（实际每帧检测数十个，正常不会触及），
    // 截断时长度字段与序列化目标数同步收窄，保持帧内自洽。
    const size_t count = o.size() < 0xffff ? o.size() : 0xffff;
    buf_.insert(buf_.end(), {0xef, 0xef, 0xef, 0xef, 0x10});
    put<uint64_t>(buf_, ts);
    put<uint16_t>(buf_, 0x4001);
    put<uint32_t>(buf_, uint32_t(2 + count * 25));
    put<uint16_t>(buf_, uint16_t(count));
    for (size_t i = 0; i < count; ++i)
    {
        const auto& x = o[i];
        put<uint8_t>(buf_, uint8_t(x.type));
        put<double>(buf_, x.position.x);
        put<double>(buf_, x.position.y);
        put<float>(buf_, x.length);
        put<float>(buf_, x.width);
    }
    put<uint16_t>(buf_, crc(buf_.data() + 4, buf_.size() - 4));
    buf_.insert(buf_.end(), 4, 0xfe);
    sendBuf();
}

void UdpBroadcaster::sendCarPose(uint64_t ts, double x, double y, float yaw)
{
    buf_.clear();
    buf_.insert(buf_.end(), {0xef, 0xef, 0xef, 0xef, 0x10});
    put<uint64_t>(buf_, ts);
    put<uint16_t>(buf_, 0x4002);
    put<uint32_t>(buf_, 20);   // body = x(8) + y(8) + yaw(4)
    put<double>(buf_, x);
    put<double>(buf_, y);
    put<float>(buf_, yaw);
    put<uint16_t>(buf_, crc(buf_.data() + 4, buf_.size() - 4));
    buf_.insert(buf_.end(), 4, 0xfe);
    sendBuf();
}

void UdpBroadcaster::sendBuf()
{
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port_);
    a.sin_addr.s_addr = INADDR_BROADCAST;
    if (sendto(fd_, buf_.data(), buf_.size(), 0, (sockaddr*)&a, sizeof(a)) < 0)
    {
        // 广播失败多为瞬时网络状态，不抛异常打断主循环；节流告警（每 300 次报一次）
        const int e = errno;
        static unsigned failures = 0;
        if (++failures % 300 == 1)
            std::fprintf(stderr, "udp broadcast failed: %s (cumulative failures: %u)\n",
                         std::strerror(e), failures);
    }
}
}  // namespace dzuav
