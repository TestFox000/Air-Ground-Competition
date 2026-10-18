#ifndef DZTRACK_ASTAR_H_
#define DZTRACK_ASTAR_H_

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <cstdint>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace dztrack {

class AStarPlanner
{
public:
    AStarPlanner() = default;

    // 设置地图
    void setMap(const nav_msgs::msg::OccupancyGrid& map);

    // 设置障碍膨胀半径（米）
    void setInflationRadius(double r) { inflation_radius_ = r; }

    // 世界坐标 -> 栅格
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    // A* 规划，返回世界坐标路径
    bool plan(const geometry_msgs::msg::PoseStamped& start,
              const geometry_msgs::msg::PoseStamped& goal,
              std::vector<geometry_msgs::msg::PoseStamped>& path);

    bool hasMap() const { return has_map_; }

private:
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int8_t> inflated_;
    bool has_map_ = false;
    double inflation_radius_ = 0.30;

    void inflate();

    bool isFree(int x, int y) const
    {
        int W = static_cast<int>(map_.info.width);
        int H = static_cast<int>(map_.info.height);
        if (x < 0 || x >= W || y < 0 || y >= H) return false;
        return inflated_[y * W + x] == 0;
    }

    static double heuristic(int x1, int y1, int x2, int y2)
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }
};

}  // namespace dztrack

#endif  // DZTRACK_ASTAR_H_