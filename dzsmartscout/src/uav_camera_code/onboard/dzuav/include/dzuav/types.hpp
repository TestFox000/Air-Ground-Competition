#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace dzuav
{

// 检测类别。顺序必须与 main.cpp className 表及训练标签严格一致。
enum class ObjectClass : int
{
    Mine = 0,        // 地雷
    Car = 1,         // 车辆
    Arrow = 2,       // 桥顶可通行箭头
    TargetRed = 3,   // 红面靶（友方，禁火）
    CellLabel = 4,   // 地面格号
    TargetBlue = 5,  // 蓝面靶（敌方）
    Obstacle = 6,    // 障碍物
};

// 地面格号：列字母 A~N + 行号 1~11。
struct CellId
{
    char column = '\0';
    int row = 0;
    float confidence = 0.F;

    bool valid() const
    {
        return column >= 'A' && column <= 'N' && row >= 1 && row <= 11;
    }

    std::string text() const
    {
        return std::string(1, column) + std::to_string(row);
    }
};

// HbbDetector 的单条检测结果（像素框）。
struct Detection
{
    ObjectClass type = ObjectClass::Mine;
    float confidence = 0.F;
    cv::Rect box;
};

// 投影到场地世界坐标后的目标，交给 UdpBroadcaster。
struct WorldObject
{
    ObjectClass type = ObjectClass::Mine;
    cv::Point3f position;
    float length = 0.F;
    float width = 0.F;
    float confidence = 0.F;
};

}  // namespace dzuav
