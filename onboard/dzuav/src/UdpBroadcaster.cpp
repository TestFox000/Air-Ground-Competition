#include "dzuav/UdpBroadcaster.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
// 目标四元有限性检查（0x4001 单目标 / 0x4002 车位姿共用口径）
bool finite(const WorldObject& x)
{
    return std::isfinite(x.position.x) && std::isfinite(x.position.y) &&
           std::isfinite(x.length) && std::isfinite(x.width);
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
    // 发送目标确定（语义变化注意）：
    //   1. 环境变量 DZUAV_TARGET_IP 设置时 -> 单播直发该 IP（现场应急：广播被
    //      交换机/AP 过滤或只想打一辆车时直连）；
    //   2. 否则枚举所有 UP 且支持 BROADCAST 的接口，逐接口发定向广播——多网卡下
    //      从"只走默认路由（INADDR_BROADCAST）"变为"全接口覆盖"。同一帧可能在
    //      多个网段各出现一次；车端按内容处理（整帧替换快照/差值计时），重复包无害。
    if (const char* target = std::getenv("DZUAV_TARGET_IP"))
    {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        if (inet_pton(AF_INET, target, &a.sin_addr) == 1)
        {
            targets_.push_back(a);
            std::fprintf(stderr, "udp unicast to %s:%u (DZUAV_TARGET_IP)\n", target,
                         static_cast<unsigned>(port_));
        }
        else
        {
            std::fprintf(stderr, "invalid DZUAV_TARGET_IP '%s' (ignored; using broadcast)\n",
                         target);
        }
    }
    if (targets_.empty())
    {
        ifaddrs* ifs = nullptr;
        if (getifaddrs(&ifs) == 0)
        {
            for (const ifaddrs* it = ifs; it != nullptr; it = it->ifa_next)
            {
                if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET)
                    continue;
                // 只收 UP 且支持 BROADCAST 的接口（lo 通常无 BROADCAST 标志，天然排除）
                if ((it->ifa_flags & IFF_UP) == 0 || (it->ifa_flags & IFF_BROADCAST) == 0)
                    continue;
                if (it->ifa_broadaddr == nullptr)
                    continue;
                sockaddr_in a = *reinterpret_cast<const sockaddr_in*>(it->ifa_broadaddr);
                a.sin_port = htons(port_);
                // 去重（bond/桥接下同一 v4 广播地址可能出现多次）
                bool dup = false;
                for (const auto& t : targets_)
                    if (t.sin_addr.s_addr == a.sin_addr.s_addr)
                    {
                        dup = true;
                        break;
                    }
                if (!dup)
                    targets_.push_back(a);
            }
            freeifaddrs(ifs);
        }
        else
        {
            std::fprintf(stderr, "getifaddrs failed: %s (falling back to 255.255.255.255)\n",
                         std::strerror(errno));
        }
        if (targets_.empty())
        {
            // 枚举不到任何广播接口（异常情形）：退回全网受限广播（旧行为）
            sockaddr_in a{};
            a.sin_family = AF_INET;
            a.sin_port = htons(port_);
            a.sin_addr.s_addr = INADDR_BROADCAST;
            targets_.push_back(a);
        }
        for (const auto& t : targets_)
        {
            char ip[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &t.sin_addr, ip, sizeof(ip));
            std::fprintf(stderr, "udp broadcast target: %s:%u\n", ip,
                         static_cast<unsigned>(port_));
        }
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
    // 有限性兜底：含 NaN/Inf 的目标不序列化。车端解析遇非有限值会整帧丢弃，
    // 这里过滤掉坏目标才能保住帧内其余目标（上游 project 已过滤，此处双保险，
    // 计数进健康行）。count 字段按过滤后的实际序列化数量填写，帧内自洽。
    size_t count = 0;
    for (const auto& x : o)
    {
        if (finite(x))
            ++count;
        else
            ++filtered_;
    }
    // count 字段仅 uint16：防御性截断（实际每帧检测数十个，正常不会触及），
    // 截断时长度字段与序列化目标数同步收窄，保持帧内自洽。
    if (count > 0xffff)
        count = 0xffff;
    buf_.insert(buf_.end(), {0xef, 0xef, 0xef, 0xef, 0x10});
    put<uint64_t>(buf_, ts);
    put<uint16_t>(buf_, 0x4001);
    put<uint32_t>(buf_, uint32_t(2 + count * 25));
    put<uint16_t>(buf_, uint16_t(count));
    size_t written = 0;
    for (const auto& x : o)
    {
        if (!finite(x) || written >= count)
            continue;
        put<uint8_t>(buf_, uint8_t(x.type));
        put<double>(buf_, x.position.x);
        put<double>(buf_, x.position.y);
        put<float>(buf_, x.length);
        put<float>(buf_, x.width);
        ++written;
    }
    put<uint16_t>(buf_, crc(buf_.data() + 4, buf_.size() - 4));
    buf_.insert(buf_.end(), 4, 0xfe);
    sendBuf();
}

void UdpBroadcaster::sendCarPose(uint64_t ts, double x, double y, float yaw)
{
    // 有限性兜底：车端对 NaN/Inf 整帧丢弃，本地直接不发（计数进健康行）；
    // 主防线在 main.cpp estimateCarYaw 的退化检查，这里是双保险。
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
    {
        ++filtered_;
        return;
    }
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
    // 逐目标发送（多接口定向广播 / 单播覆盖，见构造函数注释）
    for (const auto& a : targets_)
    {
        if (sendto(fd_, buf_.data(), buf_.size(), 0, reinterpret_cast<const sockaddr*>(&a),
                   sizeof(a)) < 0)
        {
            // 广播失败多为瞬时网络状态，不抛异常打断主循环；节流告警（每 300 次报一次）
            const int e = errno;
            if (++send_failures_ % 300 == 1)
                std::fprintf(stderr, "udp send failed: %s (cumulative failures: %lu)\n",
                             std::strerror(e), send_failures_);
        }
    }
}
}  // namespace dzuav
