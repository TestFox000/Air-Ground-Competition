#include "dzuav/UdpBroadcaster.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
namespace dzuav
{
static uint16_t crc(const std::vector<uint8_t>& d)
{
    uint16_t c = 0xffff;
    for (auto x : d)
    {
        c ^= x;
        for (int i = 0; i < 8; i++)
            c = c & 1 ? (c >> 1) ^ 0xa001 : c >> 1;
    }
    return c;
}
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
    std::vector<uint8_t> b{0xef, 0xef, 0xef, 0xef, 0x10};
    put<uint64_t>(b, ts);
    put<uint16_t>(b, 0x4001);
    put<uint32_t>(b, uint32_t(2 + o.size() * 25));
    put<uint16_t>(b, uint16_t(o.size()));
    for (auto& x : o)
    {
        put<uint8_t>(b, uint8_t(x.type));
        put<double>(b, x.position.x);
        put<double>(b, x.position.y);
        put<float>(b, x.length);
        put<float>(b, x.width);
    }
    put<uint16_t>(b, crc(std::vector<uint8_t>(b.begin() + 4, b.end())));
    b.insert(b.end(), 4, 0xfe);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port_);
    a.sin_addr.s_addr = INADDR_BROADCAST;
    sendto(fd_, b.data(), b.size(), 0, (sockaddr*)&a, sizeof(a));
}
}  // namespace dzuav
