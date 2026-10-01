#include "dzuav/UdpBroadcaster.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
namespace dzuav
{
namespace
{
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
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    int yes = 1;
    setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
}
UdpBroadcaster::~UdpBroadcaster()
{
    if (fd_ >= 0)
        close(fd_);
}
void UdpBroadcaster::send(uint64_t ts, const std::vector<WorldObject>& o)
{
    buf_.clear();   // 复用容量，免每帧重新分配
    buf_.insert(buf_.end(), {0xef, 0xef, 0xef, 0xef, 0x10});
    put<uint64_t>(buf_, ts);
    put<uint16_t>(buf_, 0x4001);
    put<uint32_t>(buf_, uint32_t(2 + o.size() * 25));
    put<uint16_t>(buf_, uint16_t(o.size()));
    for (auto& x : o)
    {
        put<uint8_t>(buf_, uint8_t(x.type));
        put<double>(buf_, x.position.x);
        put<double>(buf_, x.position.y);
        put<float>(buf_, x.length);
        put<float>(buf_, x.width);
    }
    put<uint16_t>(buf_, crc(buf_.data() + 4, buf_.size() - 4));
    buf_.insert(buf_.end(), 4, 0xfe);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port_);
    a.sin_addr.s_addr = INADDR_BROADCAST;
    sendto(fd_, buf_.data(), buf_.size(), 0, (sockaddr*)&a, sizeof(a));
}
}  // namespace dzuav
