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

    // 独占持有 socket fd，禁止拷贝（防 double-close）。
    UdpBroadcaster(const UdpBroadcaster&) = delete;
    UdpBroadcaster& operator=(const UdpBroadcaster&) = delete;

    void send(uint64_t timestamp_ms, const std::vector<WorldObject>& objects);

    // 车真实位姿帧（msg_id=0x4002，根除车端绕圈问题）：
    // magic+ver+ts+0x4002+len(=20)+x(double)+y(double)+yaw(float)+CRC+后缀，共 45 字节。
    // 与车端 ag_integrate/intel_protocol.py 的 parse_car_pose 严格对应。
    void sendCarPose(uint64_t timestamp_ms, double x, double y, float yaw);

private:
    void sendBuf();   // 把 buf_ 广播出去（send/sendCarPose 共用）

    int fd_ = -1;
    uint16_t port_ = 0;
    std::vector<uint8_t> buf_;   // 发送缓冲跨帧复用（clear 保留容量，免每帧分配）
};

}  // namespace dzuav
