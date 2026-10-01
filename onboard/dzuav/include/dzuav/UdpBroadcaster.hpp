#pragma once

#include "dzuav/types.hpp"

#include <cstdint>
#include <vector>

namespace dzuav
{

// 把世界坐标目标打成二进制帧，UDP 广播到局域网（默认端口 50000）。
// 帧格式：0xefefefef 魔数 + version + 时间戳 + msg_id + 长度 + 计数 +
//         每目标 25 字节 + CRC16/Modbus + 0xfefefefe。
class UdpBroadcaster
{
public:
    explicit UdpBroadcaster(uint16_t port = 50000);
    ~UdpBroadcaster();

    void send(uint64_t timestamp_ms, const std::vector<WorldObject>& objects);

private:
    int fd_ = -1;
    uint16_t port_ = 0;
    std::vector<uint8_t> buf_;   // 发送缓冲跨帧复用（clear 保留容量，免每帧分配）
};

}  // namespace dzuav
