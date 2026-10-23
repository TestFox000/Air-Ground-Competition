#pragma once

#include "dzuav/CellGrid.hpp"
#include "dzuav/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace dzuav
{

// 路线上的单个途经点（世界坐标，米）。
struct RouteWaypoint
{
    double x = 0;
    double y = 0;
    double z = 0;
    char column = '\0';
    int row = 0;  // 1-based
};

struct RoutePlan
{
    bool ok = false;
    std::string error;
    std::vector<std::string> warnings;
    std::vector<CellId> cells;  // 途经格序列（含起点/终点/每个雷）
    std::vector<RouteWaypoint> waypoints;
    double cost = 0;  // 加权步数（未知格按惩罚计）
    int minesTotal = 0;
    int minesVisited = 0;

    std::string cellsText() const;
    std::string toJson(int64_t timestampMs) const;
};

// 世界坐标 <-> FAST-LIO SLAM 帧对齐参考点：无人机悬停在已知世界点
// (worldX, worldY, worldZ) 时记录的 FAST-LIO 位姿 (slamX, slamY, slamZ)。
struct SlamRef
{
    double worldX = 0;
    double worldY = 0;
    double worldZ = 0;
    double slamX = 0;
    double slamY = 0;
    double slamZ = 0;
};

// 路线规划：起点/终点取矩阵中的通格，途经所有雷格（顺序用 Held-Karp 最优化），
// 障/靶格禁入，靶格尽量满足"经其侧面"（若主路线未贴近靶格，自动插入绕行格）。
// 未知格默认可通行但代价 ×unknownPenalty（优先走已确认安全的路）。
class RoutePlanner
{
public:
    struct Options
    {
        float cruiseHeightM = 1.2F;  // 巡航高度（米，含过桥）
        bool allowUnknown = true;    // 未知格是否可通行
        float unknownPenalty = 8.F;  // 未知格移动代价倍率
        // 官方固定起终点（形如 "N10"/"N5"）。非空时**只**用这一对作为
        // 路线起终点，不在矩阵里自动挑通格对（避免桥洞通格被误当起点）。
        std::string startCell;
        std::string endCell;
    };

    RoutePlanner() = default;
    explicit RoutePlanner(Options options);

    RoutePlan plan(const CellGrid& grid) const;

private:
    Options options_;
};

// 把规划结果写成 px4_path 兼容路径文件（FAST-LIO SLAM 帧）。
// 转换：x_slam = -(x_world - ref.worldX) + ref.slamX（y 同理，z 直接平移）。
// 四元数写恒等（qx=0 qy=0 qz=0 qw=1，对应 px4_path yaw_mode: fixed）。
bool writePathFile(const RoutePlan& plan, const std::string& path, const SlamRef& ref = {});

}  // namespace dzuav
