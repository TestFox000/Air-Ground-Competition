#include "dztrack/astar.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dztrack {

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid& map)
{
    map_ = map;
    has_map_ = true;
    inflate();
}

void AStarPlanner::inflate()
{
    int W = static_cast<int>(map_.info.width);
    int H = static_cast<int>(map_.info.height);
    double res = map_.info.resolution;

    inflated_.assign(W * H, 0);

    int r_cells = static_cast<int>(std::ceil(inflation_radius_ / res));
    int r2 = r_cells * r_cells;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int8_t v = map_.data[y * W + x];
            // 未知(-1) 或 占据(>=50) 都视为障碍
            bool occupied = (v < 0) || (v >= 50);
            if (!occupied) continue;

            for (int dy = -r_cells; dy <= r_cells; ++dy) {
                for (int dx = -r_cells; dx <= r_cells; ++dx) {
                    if (dx * dx + dy * dy > r2) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                    inflated_[ny * W + nx] = 1;
                }
            }
        }
    }
}

bool AStarPlanner::worldToGrid(double wx, double wy, int& gx, int& gy) const
{
    if (!has_map_) return false;
    gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
    gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    int W = static_cast<int>(map_.info.width);
    int H = static_cast<int>(map_.info.height);
    return gx >= 0 && gx < W && gy >= 0 && gy < H;
}

void AStarPlanner::gridToWorld(int gx, int gy, double& wx, double& wy) const
{
    wx = map_.info.origin.position.x + (gx + 0.5) * map_.info.resolution;
    wy = map_.info.origin.position.y + (gy + 0.5) * map_.info.resolution;
}

bool AStarPlanner::plan(const geometry_msgs::msg::PoseStamped& start,
                        const geometry_msgs::msg::PoseStamped& goal,
                        std::vector<geometry_msgs::msg::PoseStamped>& path)
{
    path.clear();
    if (!has_map_) return false;

    int sx, sy, gx, gy;
    if (!worldToGrid(start.pose.position.x, start.pose.position.y, sx, sy)) return false;
    if (!worldToGrid(goal.pose.position.x, goal.pose.position.y, gx, gy)) return false;

    if (!isFree(sx, sy) || !isFree(gx, gy)) return false;

    int W = static_cast<int>(map_.info.width);
    int H = static_cast<int>(map_.info.height);

    auto idx = [W](int x, int y) { return y * W + x; };

    struct Node {
        int x, y;
        double f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    std::vector<double> g_score(W * H, std::numeric_limits<double>::infinity());
    std::vector<int> parent(W * H, -1);
    std::vector<bool> closed(W * H, false);
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g_score[idx(sx, sy)] = 0.0;
    open.push({sx, sy, heuristic(sx, sy, gx, gy)});

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double step[8] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

    bool found = false;
    while (!open.empty()) {
        Node cur = open.top();
        open.pop();
        int ci = idx(cur.x, cur.y);
        if (closed[ci]) continue;
        closed[ci] = true;

        if (cur.x == gx && cur.y == gy) {
            found = true;
            break;
        }

        for (int k = 0; k < 8; ++k) {
            int nx = cur.x + dx[k];
            int ny = cur.y + dy[k];
            if (!isFree(nx, ny)) continue;

            // 对角移动防止穿角
            if (k >= 4) {
                if (!isFree(cur.x + dx[k], cur.y) || !isFree(cur.x, cur.y + dy[k]))
                    continue;
            }

            int ni = idx(nx, ny);
            double ng = g_score[ci] + step[k];
            if (ng < g_score[ni]) {
                g_score[ni] = ng;
                parent[ni] = ci;
                open.push({nx, ny, ng + heuristic(nx, ny, gx, gy)});
            }
        }
    }

    if (!found) return false;

    std::vector<int> rev;
    int cur = idx(gx, gy);
    while (cur != -1) {
        rev.push_back(cur);
        if (cur == idx(sx, sy)) break;
        cur = parent[cur];
    }
    std::reverse(rev.begin(), rev.end());

    auto now = builtin_interfaces::msg::Time();
    for (int i : rev) {
        int x = i % W;
        int y = i / W;
        double wx, wy;
        gridToWorld(x, y, wx, wy);

        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = wx;
        p.pose.position.y = wy;
        p.pose.position.z = 0.0;
        p.pose.orientation.w = 1.0;
        path.push_back(p);
    }

    // 用相邻点切线赋朝向
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        double ddx = path[i + 1].pose.position.x - path[i].pose.position.x;
        double ddy = path[i + 1].pose.position.y - path[i].pose.position.y;
        double yaw = std::atan2(ddy, ddx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        path[i].pose.orientation = tf2::toMsg(q);
    }
    if (path.size() >= 2) {
        path.back().pose.orientation = path[path.size() - 2].pose.orientation;
    }

    return !path.empty();
}

}  // namespace dztrack