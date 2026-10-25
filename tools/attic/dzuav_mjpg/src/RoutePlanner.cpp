#include "dzuav/RoutePlanner.hpp"

#include "dzuav/MapLocalizer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace dzuav
{

namespace
{
constexpr int kNodeCount = CellGrid::kRows * CellGrid::kCols;  // 154
constexpr double kInf = 1e18;

int indexOf(int col, int row) { return row * CellGrid::kCols + col; }

CellId idOf(int node)
{
    return {char('A' + node % CellGrid::kCols), node / CellGrid::kCols + 1, 0.F};
}

// Held-Karp：访问完 mask 中所有雷、最后停在 last 的最小代价与顺序。
// dist 为 154×154 全对最短路（Floyd-Warshall 后）。
struct HkResult
{
    double cost = kInf;
    std::vector<int> order;  // 雷的访问顺序（雷在 mines 中的下标）
};

HkResult heldKarp(const std::vector<std::vector<double>>& dist,
                  const std::vector<int>& mines, int start, int end)
{
    HkResult result;
    const int n = static_cast<int>(mines.size());
    const int full = (1 << n) - 1;
    std::vector<std::vector<double>> dp(1u << n, std::vector<double>(n, kInf));
    std::vector<std::vector<int>> parent(1u << n, std::vector<int>(n, -1));
    for (int i = 0; i < n; ++i)
    {
        const double d = dist[start][mines[i]];
        if (d < kInf)
            dp[1 << i][i] = d;
    }
    for (int mask = 1; mask <= full; ++mask)
        for (int last = 0; last < n; ++last)
        {
            if (dp[mask][last] >= kInf)
                continue;
            for (int next = 0; next < n; ++next)
            {
                if (mask & (1 << next))
                    continue;
                const double cand = dp[mask][last] + dist[mines[last]][mines[next]];
                if (cand < dp[mask | (1 << next)][next])
                {
                    dp[mask | (1 << next)][next] = cand;
                    parent[mask | (1 << next)][next] = last;
                }
            }
        }
    int bestLast = -1;
    double bestCost = kInf;
    for (int last = 0; last < n; ++last)
    {
        const double cand = dp[full][last] + dist[mines[last]][end];
        if (cand < bestCost)
        {
            bestCost = cand;
            bestLast = last;
        }
    }
    result.cost = bestCost;
    if (bestLast < 0)
        return result;
    result.order.push_back(bestLast);
    int mask = full, last = bestLast;
    while (parent[mask][last] >= 0)
    {
        const int prev = parent[mask][last];
        result.order.push_back(prev);
        mask ^= (1 << last);
        last = prev;
    }
    std::reverse(result.order.begin(), result.order.end());
    return result;
}

// 雷太多时的贪心兜底（最近邻链）。
std::vector<int> greedyOrder(const std::vector<std::vector<double>>& dist,
                             const std::vector<int>& mines, int start)
{
    std::vector<int> order;
    std::vector<bool> visited(mines.size(), false);
    int current = start;
    for (size_t k = 0; k < mines.size(); ++k)
    {
        int best = -1;
        double bestD = kInf;
        for (size_t i = 0; i < mines.size(); ++i)
            if (!visited[i] && dist[current][mines[i]] < bestD)
            {
                bestD = dist[current][mines[i]];
                best = static_cast<int>(i);
            }
        if (best < 0)
            return {};  // 有雷不可达
        order.push_back(best);
        visited[best] = true;
        current = mines[best];
    }
    return order;
}

std::vector<int> reconstructPath(const std::vector<std::vector<int>>& next, int from, int to)
{
    std::vector<int> path;
    if (from == to)
    {
        path.push_back(from);
        return path;
    }
    int node = from;
    path.push_back(node);
    while (node != to)
    {
        node = next[node][to];
        if (node < 0)
            return {};  // 不可达
        path.push_back(node);
    }
    return path;
}

void dedupeConsecutive(std::vector<int>& nodes)
{
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
}
}  // namespace

RoutePlanner::RoutePlanner(Options options) : options_(options) {}

RoutePlan RoutePlanner::plan(const CellGrid& grid) const
{
    RoutePlan plan;
    plan.minesTotal = 0;

    // ① 收集格子内容并建通行代价。
    std::vector<double> moveCost(kNodeCount, 1.0);
    std::vector<bool> blocked(kNodeCount, false);
    std::vector<int> passages, mines, targets;
    for (int row = 0; row < CellGrid::kRows; ++row)
        for (int col = 0; col < CellGrid::kCols; ++col)
        {
            const CellEntry entry = grid.entry(col, row);
            const int node = indexOf(col, row);
            if (entry.content == CellContent::Passage)
                passages.push_back(node);
            else if (entry.content == CellContent::Mine)
                mines.push_back(node);
            else if (entry.content == CellContent::Target)
            {
                targets.push_back(node);
                blocked[node] = true;
            }
            else if (entry.content == CellContent::Obstacle)
                blocked[node] = true;
            else if (entry.content == CellContent::Unknown)
            {
                if (entry.onBridge)
                    moveCost[node] = 1.0;  // 桥体格已知结构，安全
                else if (options_.allowUnknown)
                    moveCost[node] = options_.unknownPenalty;
                else
                    blocked[node] = true;
            }
        }
    plan.minesTotal = static_cast<int>(mines.size());

    // 配置了固定起终点时，矩阵里无需出现通格（视频可能没拍到车起终点）。
    if (passages.empty() && (options_.startCell.empty() || options_.endCell.empty()))
    {
        plan.error = "矩阵中没有通格（Passage），且未配置固定起终点";
        return plan;
    }
    if (grid.coverage() < 0.5F)
        plan.warnings.push_back("矩阵覆盖率不足 50%，规划可能不可靠");

    // ② Floyd-Warshall 全对最短路（4 连通）。
    std::vector<std::vector<double>> dist(kNodeCount, std::vector<double>(kNodeCount, kInf));
    std::vector<std::vector<int>> next(kNodeCount, std::vector<int>(kNodeCount, -1));
    for (int i = 0; i < kNodeCount; ++i)
    {
        dist[i][i] = 0;
        next[i][i] = i;
    }
    for (int row = 0; row < CellGrid::kRows; ++row)
        for (int col = 0; col < CellGrid::kCols; ++col)
        {
            const int from = indexOf(col, row);
            if (blocked[from])
                continue;
            const int neighbors[4][2] = {{col, row - 1}, {col, row + 1}, {col - 1, row}, {col + 1, row}};
            for (const auto& nbr : neighbors)
            {
                const int c = nbr[0], r = nbr[1];
                if (c < 0 || c >= CellGrid::kCols || r < 0 || r >= CellGrid::kRows)
                    continue;
                const int to = indexOf(c, r);
                if (blocked[to])
                    continue;
                dist[from][to] = moveCost[to];
                next[from][to] = to;
            }
        }
    for (int k = 0; k < kNodeCount; ++k)
        for (int i = 0; i < kNodeCount; ++i)
        {
            if (dist[i][k] >= kInf)
                continue;
            for (int j = 0; j < kNodeCount; ++j)
            {
                const double cand = dist[i][k] + dist[k][j];
                if (cand < dist[i][j])
                {
                    dist[i][j] = cand;
                    next[i][j] = next[i][k];
                }
            }
        }

    // ③ 起点/终点：官方固定起终点优先（若配置），否则用通格有序对。
    // 固定起终点 = 现场摆位常量（默认 N10/N5），避免视频未检出 car_start/
    // car_end 时无解，也避免桥洞通格被误选为起点。
    auto parseCell = [](const std::string& name, int& node) -> bool
    {
        if (name.size() < 2)
            return false;
        const char col = static_cast<char>(std::toupper(name[0]));
        if (col < 'A' || col >= 'A' + CellGrid::kCols)
            return false;
        int row = 0;
        for (std::size_t i = 1; i < name.size(); ++i)
        {
            if (!std::isdigit(static_cast<unsigned char>(name[i])))
                return false;
            row = row * 10 + (name[i] - '0');
        }
        if (row < 1 || row > CellGrid::kRows)
            return false;
        node = indexOf(col - 'A', row - 1);
        return true;
    };
    std::vector<std::pair<int, int>> passagePairs;
    int fixedStart = -1, fixedEnd = -1;
    if (!options_.startCell.empty() && !options_.endCell.empty() &&
        parseCell(options_.startCell, fixedStart) && parseCell(options_.endCell, fixedEnd))
    {
        passagePairs.emplace_back(fixedStart, fixedEnd);
    }
    else if (passages.size() == 1)
        passagePairs.emplace_back(passages[0], passages[0]);
    else if (passages.size() <= 8)
        for (const int start : passages)
            for (const int end : passages)
                if (start != end)
                    passagePairs.emplace_back(start, end);
    else
    {
        passagePairs.emplace_back(passages[0], passages[1]);
        passagePairs.emplace_back(passages[1], passages[0]);
        plan.warnings.push_back("通格多于 8 个，起点/终点取前两个");
    }
    const bool tooManyMines = static_cast<int>(mines.size()) > 20;
    if (tooManyMines)
        plan.warnings.push_back("雷数量 > 20，用贪心最近邻代替精确 DP");
    int bestStart = -1, bestEnd = -1;
    double bestCost = kInf;
    std::vector<int> bestOrder;  // 雷的下标访问顺序（对应 mines）
    for (const auto& [start, end] : passagePairs)
    {
        if (mines.empty())
        {
            if (dist[start][end] < bestCost)
            {
                bestCost = dist[start][end];
                bestStart = start;
                bestEnd = end;
            }
            continue;
        }
        std::vector<int> order;
        double cost = kInf;
        if (tooManyMines)
        {
            order = greedyOrder(dist, mines, start);
            cost = 0;
            int current = start;
            for (const int m : order)
            {
                cost += dist[current][mines[m]];
                current = mines[m];
            }
            cost += dist[current][end];
        }
        else
        {
            const HkResult hk = heldKarp(dist, mines, start, end);
            order = hk.order;
            cost = hk.cost;
        }
        if (cost < bestCost)
        {
            bestCost = cost;
            bestStart = start;
            bestEnd = end;
            bestOrder = order;
        }
    }
    if (bestStart < 0 || bestCost >= kInf)
    {
        // 找出第一个不可达的雷，报得更具体。
        for (const int m : mines)
            if (dist[bestStart < 0 ? passages.front() : bestStart][m] >= kInf)
            {
                plan.error = "雷格 " + idOf(m).text() + " 不可达（被障/靶围死）";
                return plan;
            }
        plan.error = "通格之间没有可行路线（被障/靶隔断）";
        return plan;
    }
    if (mines.empty())
        plan.warnings.push_back("矩阵中没有检测到雷");

    // ④ 拼接节点序列：起点 → 雷（最优序）→ 终点。
    std::vector<int> routeNodes;
    {
        std::vector<int> stops;
        stops.push_back(bestStart);
        for (const int m : bestOrder)
            stops.push_back(mines[m]);
        stops.push_back(bestEnd);
        for (size_t i = 0; i + 1 < stops.size(); ++i)
        {
            std::vector<int> segment = reconstructPath(next, stops[i], stops[i + 1]);
            if (segment.empty())
            {
                plan.error = "路径重建失败（" + idOf(stops[i]).text() + " -> " +
                             idOf(stops[i + 1]).text() + " 不可达）";
                return plan;
            }
            routeNodes.insert(routeNodes.end(), segment.begin(), segment.end());
        }
        dedupeConsecutive(routeNodes);
    }

    // ⑤ 靶侧面：主路线若无格与靶格相邻，插入代价最小的绕行格。
    for (const int target : targets)
    {
        const int tcol = target % CellGrid::kCols, trow = target / CellGrid::kCols;
        bool adjacent = false;
        for (const int node : routeNodes)
        {
            const int col = node % CellGrid::kCols, row = node / CellGrid::kCols;
            if (std::abs(col - tcol) + std::abs(row - trow) == 1)
            {
                adjacent = true;
                break;
            }
        }
        if (adjacent)
            continue;
        int bestInsertPos = -1, bestCandidate = -1;
        double bestAdded = kInf;
        for (const auto& nbr : std::vector<std::pair<int, int>>{{tcol, trow - 1},
                                                                {tcol, trow + 1},
                                                                {tcol - 1, trow},
                                                                {tcol + 1, trow}})
        {
            const int c = nbr.first, r = nbr.second;
            if (c < 0 || c >= CellGrid::kCols || r < 0 || r >= CellGrid::kRows)
                continue;
            const int candidate = indexOf(c, r);
            if (blocked[candidate])
                continue;
            for (size_t i = 0; i + 1 < routeNodes.size(); ++i)
            {
                const int a = routeNodes[i], b = routeNodes[i + 1];
                if (dist[a][candidate] >= kInf || dist[candidate][b] >= kInf)
                    continue;
                const double added = dist[a][candidate] + dist[candidate][b] - dist[a][b];
                if (added < bestAdded)
                {
                    bestAdded = added;
                    bestInsertPos = static_cast<int>(i);
                    bestCandidate = candidate;
                }
            }
        }
        if (bestCandidate < 0)
        {
            plan.warnings.push_back("靶格 " + idOf(target).text() +
                                    " 四周无可行邻格，无法经过其侧面");
            continue;
        }
        std::vector<int> spliced;
        spliced.insert(spliced.end(), routeNodes.begin(), routeNodes.begin() + bestInsertPos + 1);
        {
            std::vector<int> seg = reconstructPath(next, routeNodes[bestInsertPos], bestCandidate);
            spliced.insert(spliced.end(), seg.begin(), seg.end());
            seg = reconstructPath(next, bestCandidate, routeNodes[bestInsertPos + 1]);
            spliced.insert(spliced.end(), seg.begin() + 1, seg.end());
        }
        spliced.insert(spliced.end(), routeNodes.begin() + bestInsertPos + 1, routeNodes.end());
        dedupeConsecutive(spliced);
        routeNodes = std::move(spliced);
        plan.warnings.push_back("已绕行至 " + idOf(bestCandidate).text() +
                                " 以经过靶 " + idOf(target).text() + " 侧面");
    }

    // ⑥ 输出：格序列 + 世界坐标航点。
    plan.cost = bestCost;
    plan.minesVisited = 0;
    for (const int node : routeNodes)
    {
        const CellId id = idOf(node);
        plan.cells.push_back(id);
        RouteWaypoint waypoint;
        waypoint.x = MapLocalizer::kOriginX + (id.column - 'A') * MapLocalizer::kCellSize;
        waypoint.y = MapLocalizer::kOriginY + (CellGrid::kRows - id.row) * MapLocalizer::kCellSize;
        waypoint.z = options_.cruiseHeightM;
        waypoint.column = id.column;
        waypoint.row = id.row;
        plan.waypoints.push_back(waypoint);
        if (grid.entry(id.column - 'A', id.row - 1).content == CellContent::Mine)
            ++plan.minesVisited;
    }
    plan.ok = true;
    return plan;
}

std::string RoutePlan::cellsText() const
{
    std::ostringstream os;
    for (size_t i = 0; i < cells.size(); ++i)
    {
        if (i)
            os << ' ';
        os << cells[i].text();
    }
    return os.str();
}

std::string RoutePlan::toJson(int64_t timestampMs) const
{
    std::ostringstream os;
    os << "{\"type\":\"route_plan\",\"ts\":" << timestampMs << ",\"ok\":"
       << (ok ? "true" : "false") << ",\"cost\":" << cost
       << ",\"mines_total\":" << minesTotal << ",\"mines_visited\":" << minesVisited;
    if (!error.empty())
        os << ",\"error\":\"" << error << '"';
    os << ",\"warnings\":[";
    for (size_t i = 0; i < warnings.size(); ++i)
        os << (i ? "," : "") << '"' << warnings[i] << '"';
    os << "],\"cells\":[";
    for (size_t i = 0; i < cells.size(); ++i)
    {
        const auto& id = cells[i];
        if (i)
            os << ',';
        os << "{\"id\":\"" << id.text() << "\",\"col\":" << (id.column - 'A')
           << ",\"row\":" << (id.row - 1);
        if (i < waypoints.size())
            os << ",\"x\":" << waypoints[i].x << ",\"y\":" << waypoints[i].y
               << ",\"z\":" << waypoints[i].z;
        os << '}';
    }
    os << "]}";
    return os.str();
}

bool writePathFile(const RoutePlan& plan, const std::string& path, const SlamRef& ref)
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;
    file << "# dzuav planned route (FAST-LIO SLAM frame)\n"
         << "# format: x y z qx qy qz qw (path_loader: SLAM -> ENU -> PX4 NED)\n"
         << "# slam = rot180_z(world - ref.world) + ref.slam; yaw fixed (identity quat)\n";
    for (const auto& waypoint : plan.waypoints)
    {
        const double sx = -(waypoint.x - ref.worldX) + ref.slamX;
        const double sy = -(waypoint.y - ref.worldY) + ref.slamY;
        const double sz = (waypoint.z - ref.worldZ) + ref.slamZ;
        file << sx << ' ' << sy << ' ' << sz << " 0 0 0 1\n";
    }
    return static_cast<bool>(file);
}

}  // namespace dzuav
