#include "dzmission/mine_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace dzmission
{

const char *MinePlanner::dirName(uint8_t d)
{
  switch (d)
  {
    case kDirNorth: return "北(由南向北)";
    case kDirSouth: return "南(由北向南)";
    case kDirEast: return "东(由西向东)";
    case kDirWest: return "西(由东向西)";
    default: return "?";
  }
}

void MinePlanner::dirDelta(uint8_t d, int &dc, int &dr)
{
  dc = 0;
  dr = 0;
  switch (d)
  {
    case kDirNorth: dr = -1; break;   // 向北 → row 减小
    case kDirSouth: dr = +1; break;
    case kDirEast: dc = +1; break;
    case kDirWest: dc = -1; break;
    default: break;
  }
}

// -----------------------------------------------------------------------------
bool MinePlanner::makePass(const FieldGrid &grid, int col, int row, uint8_t d,
                           int approach_cells, Pass &out)
{
  out = Pass{};
  out.col = col;
  out.row = row;
  out.dir = d;

  int dc = 0;
  int dr = 0;
  dirDelta(d, dc, dr);

  // 入口在逆向 approach_cells 格，出口在正向 approach_cells 格
  out.in_col = col - dc * approach_cells;
  out.in_row = row - dr * approach_cells;
  out.out_col = col + dc * approach_cells;
  out.out_row = row + dr * approach_cells;

  // 入口、地雷格、出口都必须可通行
  if (!grid.inBounds(out.in_col, out.in_row) ||
      !grid.inBounds(out.out_col, out.out_row))
  {
    out.feasible = false;
    out.note = "入口/出口越界";
    return false;
  }
  if (!grid.cellPassable(out.in_col, out.in_row))
  {
    out.feasible = false;
    out.note = std::string("入口 ") +
               FieldGrid::cellName(out.in_col, out.in_row) + " 不可通行";
    return false;
  }
  if (!grid.cellPassable(out.out_col, out.out_row))
  {
    out.feasible = false;
    out.note = std::string("出口 ") +
               FieldGrid::cellName(out.out_col, out.out_row) + " 不可通行";
    return false;
  }
  if (!grid.cellPassable(col, row))
  {
    out.feasible = false;
    out.note = "地雷格本身不可通行";
    return false;
  }

  // ★★ 关键校验：入口 -> 地雷 -> 出口 必须是**合法的单步** ★★
  // 原实现只检查三格各自可通行，**从不检查它们之间的边**。后果：
  //   水平隔离线覆盖 D..N。若地雷在其中某列，则"北"/"南"方向的入口
  //   会落在隔离线的另一侧 —— 两格各自都可通行，但之间的**边被封死**。
  //   于是规划出一条跨越隔离线的通行段，实车根本走不了，
  //   而且不报错，只在最后拼路径时留下一条非法边（静默错误）。
  // ★ 这是 Python 对照实现里通过"路径合法性断言"发现的真实 bug，
  //   已在 mine_planner_check.py 修复，此处同步。
  if (!grid.canStep(out.in_col, out.in_row, col, row))
  {
    out.feasible = false;
    out.note = std::string("入口 ") + FieldGrid::cellName(out.in_col, out.in_row) +
               " 走不到地雷 " + FieldGrid::cellName(col, row) +
               "（边被封，可能跨隔离线）";
    return false;
  }
  if (!grid.canStep(col, row, out.out_col, out.out_row))
  {
    out.feasible = false;
    out.note = std::string("地雷 ") + FieldGrid::cellName(col, row) +
               " 走不到出口 " + FieldGrid::cellName(out.out_col, out.out_row) +
               "（边被封，可能跨隔离线）";
    return false;
  }

  // 评估沿该方向通过时，入口与出口两端的转向余量
  //   入口端：从 (in - d) 进到 in 是直行；真正要评估的是
  //   "接近 in 的那一段"与"从 out 离开的那一段"的转角。
  //   这里用与出入口相邻的垂直方向格做一次 checkTurn 探针。
  double worst_gap = 1e9;
  const int perp[2][2] = {{-dr, dc}, {dr, -dc}};   // 垂直方向
  for (int side = 0; side < 2; ++side)
  {
    // 入口端的转角：prev -> in -> mine
    const int pc = out.in_col + perp[side][0] * -1;
    const int pr = out.in_row + perp[side][1] * -1;
    if (grid.inBounds(pc, pr) && grid.cellPassable(pc, pr))
    {
      const auto t = grid.checkTurn(pc, pr, out.in_col, out.in_row, col, row);
      if (t.valid)
      {
        worst_gap = std::min(worst_gap, t.min_gap_cm);
      }
    }
    // 出口端的转角：mine -> out -> next
    const int nc = out.out_col + perp[side][0];
    const int nr = out.out_row + perp[side][1];
    if (grid.inBounds(nc, nr) && grid.cellPassable(nc, nr))
    {
      const auto t = grid.checkTurn(col, row, out.out_col, out.out_row, nc, nr);
      if (t.valid)
      {
        worst_gap = std::min(worst_gap, t.min_gap_cm);
      }
    }
  }
  if (worst_gap > 1e8)
  {
    worst_gap = 0.0;   // 没有可评估的转角（四周都是障碍）→ 保守记 0
  }
  out.turn_gap_cm = worst_gap;
  out.feasible = true;
  out.note = "可行";
  return true;
}

MinePlanner::Pass MinePlanner::choosePass(const FieldGrid &grid, int col,
                                          int row, const Config &cfg)
{
  Pass best;
  bool have = false;
  std::string best_why;

  // 依次试四个方向，优先级：可行 > 转角余量大 > 方向偏好（北/南优先，
  // 因为场地是纵向 11 行、横向 14 列，纵向通行通常更顺）
  const uint8_t order[4] = {kDirNorth, kDirSouth, kDirEast, kDirWest};
  double best_score = -1e18;

  for (int i = 0; i < 4; ++i)
  {
    const uint8_t d = order[i];

    // ★ approach 外延格数需要**按需回退到 0**（同步自 Python 对照实现）
    //
    // 为什么必须支持 approach=0：
    //   对**场地边缘行/列**的雷（例如第 11 行的 B11），强制"入口在雷之前一格"
    //   会把入口点放到 A11 —— 而 A11 只有在**穿过地雷格 B11 之后**才到得了。
    //   车被迫"先走到 A11，再折回 B11"，既产生 180° 折返又白跑一趟。
    //   更糟的是：若相邻格被挡，这种通行段会被判为不可行，
    //   于是一颗本来能压到的雷被**漏掉**。
    //   approach=0 的语义是"车直接开过地雷格"，对边缘格这是唯一合理走法。
    //
    // 实测效果：Python 侧覆盖率 20% -> 100%。
    Pass p;
    bool ok = false;
    int used_approach = cfg.approach_cells;
    for (int ap : {cfg.approach_cells, 0})
    {
      if (makePass(grid, col, row, d, ap, p))
      {
        ok = true;
        used_approach = ap;
        break;
      }
      // 保留一份失败原因用于诊断（只在还没成功过时覆盖）
    }
    if (!ok)
    {
      if (!have)
      {
        best = p;   // 记下失败原因，便于诊断
        best_why = p.note;
      }
      continue;
    }
    // 评分：转角余量为主，纵向略加分
    double score = p.turn_gap_cm;
    if (d == kDirNorth || d == kDirSouth)
    {
      score += 2.0;
    }
    if (!have || score > best_score)
    {
      have = true;
      best = p;
      best_score = score;
      best_why = "转角余量 " + std::to_string(static_cast<int>(p.turn_gap_cm)) +
                 " cm" + (used_approach == 0 ? "（外延 0，边缘格）" : "");
    }
  }

  if (have)
  {
    best.note = std::string("选 ") + dirName(best.dir) + "（" + best_why + "）";
  }
  else
  {
    best.feasible = false;
    if (best.note.empty())
    {
      best.note = "四个方向都不可行";
    }
  }
  return best;
}

// -----------------------------------------------------------------------------
std::vector<int> MinePlanner::orderPasses(
    const std::vector<std::vector<int>> &dist, int start_from, int rounds)
{
  const int n = static_cast<int>(dist.size());
  std::vector<int> order;
  if (n <= 0)
  {
    return order;
  }
  if (n == 1)
  {
    order.push_back(0);
    return order;
  }

  // ---- 最近邻构造 ----
  std::vector<uint8_t> used(static_cast<size_t>(n), 0);
  int cur = std::max(0, std::min(start_from, n - 1));
  order.push_back(cur);
  used[static_cast<size_t>(cur)] = 1;

  for (int k = 1; k < n; ++k)
  {
    int best = -1;
    int bestd = std::numeric_limits<int>::max();
    for (int j = 0; j < n; ++j)
    {
      if (used[static_cast<size_t>(j)] != 0)
      {
        continue;
      }
      const int d = dist[static_cast<size_t>(cur)][static_cast<size_t>(j)];
      if (d < 0)
      {
        continue;   // 不可达
      }
      if (d < bestd)
      {
        bestd = d;
        best = j;
      }
    }
    if (best < 0)
    {
      // 有不可达点：按数量级补一个很大的代价，仍加入序列（后续会报不可达）
      for (int j = 0; j < n; ++j)
      {
        if (used[static_cast<size_t>(j)] == 0)
        {
          best = j;
          break;
        }
      }
      if (best < 0)
      {
        break;
      }
    }
    order.push_back(best);
    used[static_cast<size_t>(best)] = 1;
    cur = best;
  }

  // ---- 2-opt 改进（开放路径，固定起点） ----
  auto total = [&](const std::vector<int> &o) -> long long {
    long long s = 0;
    for (size_t i = 1; i < o.size(); ++i)
    {
      const int d = dist[static_cast<size_t>(o[i - 1])][static_cast<size_t>(o[i])];
      if (d < 0)
      {
        return std::numeric_limits<long long>::max() / 4;
      }
      s += d;
    }
    return s;
  };

  long long best_cost = total(order);
  for (int round = 0; round < rounds; ++round)
  {
    bool improved = false;
    for (size_t i = 1; i + 1 < order.size(); ++i)
    {
      for (size_t j = i + 1; j < order.size(); ++j)
      {
        std::vector<int> cand = order;
        std::reverse(cand.begin() + static_cast<long>(i),
                     cand.begin() + static_cast<long>(j) + 1);
        const long long c = total(cand);
        if (c < best_cost)
        {
          order.swap(cand);
          best_cost = c;
          improved = true;
        }
      }
    }
    if (!improved)
    {
      break;
    }
  }
  return order;
}

// -----------------------------------------------------------------------------
MinePlanner::Plan MinePlanner::plan(
    const FieldGrid &grid,
    const std::vector<std::pair<int, int>> &mines,
    const std::pair<int, int> &start,
    const std::pair<int, int> &goal) const
{
  Plan out;
  out.stats.mines_total = static_cast<int>(mines.size());

  if (mines.empty())
  {
    out.ok = false;
    out.reason = "地雷列表为空（未收到 MINE_LIST）";
    return out;
  }
  if (!grid.cellPassable(start.first, start.second))
  {
    out.ok = false;
    out.reason = "起点格不可通行";
    return out;
  }

  // ---- 1. 每颗地雷选向生成 Pass ----
  std::vector<Pass> passes;
  passes.reserve(mines.size());
  for (const auto &m : mines)
  {
    if (!grid.inBounds(m.first, m.second))
    {
      out.stats.warnings.push_back("地雷格越界，已跳过");
      continue;
    }
    Pass p = choosePass(grid, m.first, m.second, cfg_);
    if (!p.feasible)
    {
      out.stats.warnings.push_back(
          FieldGrid::cellName(m.first, m.second) + " 无法规划通行段：" + p.note);
    }
    passes.push_back(p);
  }
  if (passes.empty())
  {
    out.ok = false;
    out.reason = "所有地雷格都无法生成通行段";
    return out;
  }

  // ---- 2. 距离矩阵（用入口做近似，出口影响在拼接时体现） ----
  // 简化：把每个 pass 视为"从起点到入口 -> 经过地雷 -> 从出口离开"，
  //       距离矩阵用 (出口 i -> 入口 j) 的最短路径长度。
  const int n = static_cast<int>(passes.size());
  std::vector<std::vector<int>> dist(
      static_cast<size_t>(n), std::vector<int>(static_cast<size_t>(n), -1));
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      if (i == j)
      {
        dist[static_cast<size_t>(i)][static_cast<size_t>(j)] = 0;
        continue;
      }
      dist[static_cast<size_t>(i)][static_cast<size_t>(j)] =
          grid.pathLength(passes[static_cast<size_t>(i)].out_col,
                          passes[static_cast<size_t>(i)].out_row,
                          passes[static_cast<size_t>(j)].in_col,
                          passes[static_cast<size_t>(j)].in_row);
    }
  }

  // ---- 3. 起点最近的 pass 作为起始 ----
  int start_idx = 0;
  int best_d = std::numeric_limits<int>::max();
  for (int i = 0; i < n; ++i)
  {
    const int d = grid.pathLength(start.first, start.second,
                                  passes[static_cast<size_t>(i)].in_col,
                                  passes[static_cast<size_t>(i)].in_row);
    if (d >= 0 && d < best_d)
    {
      best_d = d;
      start_idx = i;
    }
  }

  // ---- 4. 顺序优化 ----
  out.order = orderPasses(dist, start_idx, cfg_.two_opt_rounds);

  // ---- 5. 拼接完整路径 ----
  std::vector<std::pair<int, int>> full;
  auto appendPath = [&](const std::vector<std::pair<int, int>> &seg) {
    // 去掉与 full 末尾重复的首格
    size_t from = 0;
    if (!full.empty() && !seg.empty() && full.back() == seg.front())
    {
      from = 1;
    }
    for (size_t i = from; i < seg.size(); ++i)
    {
      full.push_back(seg[i]);
    }
  };

  std::pair<int, int> cursor = start;

  // ★ 上一段末尾的朝向（用于 findPathStaged 第 1 级的方向感知搜索）
  int prev_tail_dc = 0;
  int prev_tail_dr = 0;
  bool prev_tail_valid = false;

  // 如果上一个 pass 的出口方向与当前 pass 的入口方向相同，则无需转弯，
  // 这时优先选择"顺着来向"的 pass 方向，可显著减少紧转角。
  //
  // ★ 具体实现见下方 prev_tail_dc / prev_tail_dr -> hdc / hdr：
  //   每完成一个 pass 就记下它的出口方向（out_col-p.col, out_row-p.row），
  //   下一段搜索把它作为首选起始方向传给 findPathStaged。
  //   （这里原有一个 auto dirFrom = [](a,b)->int {...} 的 lambda，
  //     但**从未被调用**，是死代码 —— 上面这套机制才是真正生效的。
  //     留着它会触发 -Wunused-but-set-variable，也会误导读者。）

  for (size_t k = 0; k < out.order.size(); ++k)
  {
    Pass &p = passes[static_cast<size_t>(out.order[k])];
    if (!p.feasible)
    {
      out.stats.warnings.push_back(
          FieldGrid::cellName(p.col, p.row) + " 跳过（通行段不可行）");
      continue;
    }

    // (a) 从 cursor 走到 pass 入口
    //
    // ★ 用 findPathStaged（逐级放宽）而不是 findPath（纯格级最短）：
    //   - findPath 完全不管转弯可行性，会产出"格级最短但实车走不了"的路径
    //   - findPathStaged 优先给出可直接执行的路径；只有在严格约束下
    //     **确实不连通**时才逐级放宽，并在放宽到第 3 级以上时告警
    //   （放宽到第 3 级 = 路径含需原地转向的转角，这在 v5 拓扑的
    //     东侧窄条里是物理必然，由 PivotController 执行）
    std::vector<std::pair<int, int>> seg;
    int used_level = 0;
    // 上一段末尾的朝向：用于第一级的方向感知搜索
    int hdc = 0;
    int hdr = 0;
    if (prev_tail_valid)
    {
      hdc = prev_tail_dc;
      hdr = prev_tail_dr;
    }
    if (!grid.findPathStaged(cursor.first, cursor.second, p.in_col, p.in_row,
                             seg, hdc, hdr, &used_level))
    {
      out.stats.warnings.push_back(
          "无法从 " + FieldGrid::cellName(cursor.first, cursor.second) +
          " 到达 " + FieldGrid::cellName(p.in_col, p.in_row) +
          "（" + FieldGrid::cellName(p.col, p.row) + " 的地雷）");
      continue;
    }
    if (used_level >= 3)
    {
      out.stats.warnings.push_back(
          "接近 " + FieldGrid::cellName(p.col, p.row) + " 的路径放宽到第 " +
          std::to_string(used_level) + " 级（含需原地转向的转角）");
    }
    appendPath(seg);

    // (b) 入口 -> 地雷格 -> 出口（直线通行段）
    full.emplace_back(p.col, p.row);
    full.emplace_back(p.out_col, p.out_row);

    cursor = {p.out_col, p.out_row};
    // 记录本段末尾朝向，供下一段的第 1 级搜索使用
    prev_tail_dc = p.out_col - p.col;
    prev_tail_dr = p.out_row - p.row;
    prev_tail_valid = true;
    out.passes.push_back(p);
    out.stats.mines_covered += 1;
  }

  // (c) 最后走到 goal
  {
    std::vector<std::pair<int, int>> seg;
    int used_level = 0;
    const int hdc = prev_tail_valid ? prev_tail_dc : 0;
    const int hdr = prev_tail_valid ? prev_tail_dr : 0;
    if (grid.findPathStaged(cursor.first, cursor.second, goal.first, goal.second,
                            seg, hdc, hdr, &used_level))
    {
      if (used_level >= 4)
      {
        out.stats.warnings.push_back(
            "走向终点的路径放宽到第 " + std::to_string(used_level) + " 级");
      }
      appendPath(seg);
    }
    else
    {
      out.stats.warnings.push_back("最后无法到达终点，路径可能在半途结束");
    }
  }

  // ★ 路径合法性断言（同步自 Python 对照实现）
  //   拼接多段 BFS 结果时，若两段之间没有重叠、上一段末尾与下一段开头
  //   又恰好"看起来相邻"，就会凭空接出一条**跨越隔离线**的边。
  //   这种路径图上也过不了 canStep，实车更走不了，但不会报错 ——
  //   是典型的静默错误。这里显式检查并报警。
  {
    int illegal = 0;
    std::string first_bad;
    for (size_t i = 1; i < full.size(); ++i)
    {
      const auto &a = full[i - 1];
      const auto &b = full[i];
      if (a == b)
      {
        continue;
      }
      if (!grid.canStep(a.first, a.second, b.first, b.second))
      {
        ++illegal;
        if (first_bad.empty())
        {
          first_bad = FieldGrid::cellName(a.first, a.second) + " -> " +
                      FieldGrid::cellName(b.first, b.second);
        }
      }
    }
    if (illegal > 0)
    {
      out.stats.warnings.push_back(
          "★ 路径含 " + std::to_string(illegal) + " 处非法步骤（首处 " +
          first_bad + "）—— 拼接逻辑有缺陷");
      out.stats.illegal_steps = illegal;
    }
  }

  out.path = full;
  out.stats.waypoints = static_cast<int>(full.size());
  out.stats.segments = static_cast<int>(out.passes.size());

  // ---- 6. 统计行程与转角校验 ----
  double cells = 0.0;
  for (size_t i = 1; i < full.size(); ++i)
  {
    if (full[i] == full[i - 1])
    {
      continue;
    }
    cells += 1.0;
  }
  out.stats.total_cells = cells;
  out.stats.est_time_s =
      cells * grid.spec().cell_size / std::max(0.05, cfg_.cruise_speed);

  if (cfg_.enforce_turns)
  {
    std::vector<std::string> bad;
    if (!grid.pathTurnsFeasible(full, bad))
    {
      out.stats.infeasible_turns = static_cast<int>(bad.size());
      for (const auto &b : bad)
      {
        out.stats.warnings.push_back("转角：" + b);
      }
    }
    // 统计"紧转角"数量（可行但余量 < 3cm），供实车降速参考
    int tight = 0;
    for (size_t i = 2; i < full.size(); ++i)
    {
      const bool ab_vert = (full[i - 2].first == full[i - 1].first);
      const bool bc_vert = (full[i - 1].first == full[i].first);
      if (ab_vert == bc_vert)
      {
        continue;
      }
      const auto t = grid.checkTurn(full[i - 2].first, full[i - 2].second,
                                    full[i - 1].first, full[i - 1].second,
                                    full[i].first, full[i].second);
      if (t.feasible && t.tight)
      {
        ++tight;
      }
    }
    out.stats.tight_turns = tight;
  }

  out.ok = (out.stats.mines_covered == out.stats.mines_total) &&
           (out.stats.infeasible_turns == 0);
  if (out.ok)
  {
    out.reason = "规划成功";
    if (out.stats.tight_turns > 0)
    {
      out.reason += "（含 " + std::to_string(out.stats.tight_turns) +
                    " 处紧转角，实车应降速通过）";
    }
  }
  else if (out.stats.mines_covered < out.stats.mines_total)
  {
    out.reason = "有地雷未能纳入路径（" +
                 std::to_string(out.stats.mines_covered) + "/" +
                 std::to_string(out.stats.mines_total) + "）";
  }
  else
  {
    out.reason = "路径含不可行转角 " +
                 std::to_string(out.stats.infeasible_turns) + " 处";
  }
  return out;
}

}  // namespace dzmission
