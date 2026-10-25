#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace dzuav
{

// 检测类别。顺序必须与 main.cpp className 表及训练标签严格一致。
// 2026-10-16 起切换 D 方案 13 类统一模型（hbb_d13_best.pt，17 通道引擎）。
// Paper 语义由车端按落格判定：普通格纸片=雷，K 列桥洞区纸片=通行箭头。
enum class ObjectClass : int
{
    Paper = 0,       // 场地纸片（雷图卡/桥顶箭头标志）
    CellLabel = 1,   // 地面格号
    Car = 2,         // 行驶车辆（车位姿链）
    TargetRed = 3,   // 红面靶（友方，禁火）
    TargetBlue = 4,  // 蓝面靶（敌方）
    Obstacle = 5,    // 障碍物
    BucketRed = 6,   // 红桶（禁火）
    BucketGreen = 7, // 绿桶
    BucketCyan = 8,  // 青桶
    CarStart = 9,    // 智能车起点
    CarEnd = 10,     // 智能车终点
    UavStart = 11,   // 无人机起点
    UavEnd = 12,     // 无人机终点
};

constexpr int kObjectClassCount = 13;

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
    ObjectClass type = ObjectClass::Paper;
    float confidence = 0.F;
    cv::Rect box;
};

// 投影到场地世界坐标后的目标，交给 UdpBroadcaster。
struct WorldObject
{
    ObjectClass type = ObjectClass::Paper;
    cv::Point3f position;
    float length = 0.F;
    float width = 0.F;
    float confidence = 0.F;
};

}  // namespace dzuav
