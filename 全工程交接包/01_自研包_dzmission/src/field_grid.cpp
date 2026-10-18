#include "dzmission/field_grid.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <deque>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace dzmission
{

FieldGrid::FieldGrid() { reset(); }

FieldGrid::FieldGrid(const Spec &s)
{
  spec_ = s;
  reset();
}

void FieldGrid::setSpec(const Spec &s)
{
  spec_ = s;
  reset();
}

void FieldGrid::buildFixedEdges()
{
  const size_t n = static_cast<size_t>(spec_.cols) * static_cast<size_t>(spec_.rows);
  blocked_up_.assign(n, 0);
  blocked_right_.assign(n, 0);

  // 水平隔离线（黄色挡板 + 白色长条）：
  //   row(barrier_up_row) 与 row(barrier_up_row+1) 之间的**纵向边**，
  //   覆盖列 barrier_col_lo .. barrier_col_hi。
  //   格子本身可通行，只是这条边不可跨越。
  const int r = spec_.barrier_up_row;
  if (r >= 0 && r + 1 < spec_.rows)
  {
    for (int c = spec_.barrier_col_lo; c <= spec_.barrier_col_hi; ++c)
    {
      if (!inBounds(c, r))
      {
        continue;
      }
      blocked_up_[static_cast<size_t>(idx(c, r))] = 1;
    }
  }
}

void FieldGrid::reset()
{
  const size_t n = static_cast<size_t>(spec_.cols) * static_cast<size_t>(spec_.rows);
  cells_.assign(n, static_cast<uint8_t>(kFree));  // 默认格内全部可通行
  buildFixedEdges();

  // K 列是"桥洞排"：一排通道，通道之间由隔板隔离。
  // 初始状态下**没有任何通道被指定**，所以整列不可通行；
  // 由 openCrossing() 逐个开放（每局 2 个）。
  for (int r = 0; r < spec_.rows; ++r)
  {
    if (inBounds(spec_.bridge_col, r))
    {
      cells_[static_cast<size_t>(idx(spec_.bridge_col, r))] = kBlocked;
    }
  }

  // ---------------------------------------------------------------------------
  // ★ 无人机起降点：设为**禁入格**
  // ---------------------------------------------------------------------------
  // 无人机起飞前与降落后停在场上这两个格。地面车若规划路径经过就会撞机，
  // 而"撞固定场地持续超 10 s 裁判切手动"是硬规则（云台仍自动）。
  // 所以这两格必须是 kBlocked —— cellPassable() 因此会拒绝进入。
  //
  // ★ 为什么放在 reset()：这是**每局都存在的固定结构**。
  //   即使无人机端没下发 FIELD_FIXED（该消息是可选的），车端也要默认生效；
  //   若下发了，mission_manager 会调 setDroneCells() 用实际值覆盖。
  // ---------------------------------------------------------------------------
  applyDroneCellsAsBlocked();

  gate_a_row_ = -1;
  gate_b_row_ = -1;
}

void FieldGrid::applyDroneCellsAsBlocked()
{
  if (!spec_.drone_cells_valid)
  {
    return;
  }
  if (inBounds(spec_.drone_start_col, spec_.drone_start_row))
  {
    cells_[static_cast<size_t>(idx(spec_.drone_start_col, spec_.drone_start_row))] =
        kBlocked;
  }
  if (inBounds(spec_.drone_end_col, spec_.drone_end_row))
  {
    cells_[static_cast<size_t>(idx(spec_.drone_end_col, spec_.drone_end_row))] =
        kBlocked;
  }
}

bool FieldGrid::setDroneCells(int start_col, int start_row, int end_col, int end_row)
{
  if (!inBounds(start_col, start_row) || !inBounds(end_col, end_row))
  {
    return false;
  }
  // ★ 不允许与起终点重合 —— 那会让本局直接不可达，属配置错误，应拒绝。
  if ((start_col == spec_.start_col && start_row == spec_.start_row) ||
      (start_col == spec_.goal_col && start_row == spec_.goal_row) ||
      (end_col == spec_.start_col && end_row == spec_.start_row) ||
      (end_col == spec_.goal_col && end_row == spec_.goal_row))
  {
    return false;
  }
  // 旧值恢复为通行（仅当它确实是 kBlocked，避免把真正的障碍格误改）
  auto restore = [&](int c, int r) {
    if (inBounds(c, r) && cells_[static_cast<size_t>(idx(c, r))] == kBlocked)
    {
      cells_[static_cast<size_t>(idx(c, r))] = kFree;
    }
  };
  restore(spec_.drone_start_col, spec_.drone_start_row);
  restore(spec_.drone_end_col, spec_.drone_end_row);

  spec_.drone_start_col = start_col;
  spec_.drone_start_row = start_row;
  spec_.drone_end_col = end_col;
  spec_.drone_end_row = end_row;
  applyDroneCellsAsBlocked();
  return true;
}

bool FieldGrid::isDroneCell(int col, int row) const
{
  if (!spec_.drone_cells_valid)
  {
    return false;
  }
  return (col == spec_.drone_start_col && row == spec_.drone_start_row) ||
         (col == spec_.drone_end_col && row == spec_.drone_end_row);
}

// -----------------------------------------------------------------------------
// 名称
// -----------------------------------------------------------------------------
std::string FieldGrid::colName(int col)
{
  if (col < 0 || col > 25)
  {
    return "?";
  }
  return std::string(1, static_cast<char>('A' + col));
}

int FieldGrid::colIndex(char letter)
{
  if (letter >= 'a' && letter <= 'z')
  {
    letter = static_cast<char>(letter - 'a' + 'A');
  }
  if (letter < 'A' || letter > 'Z')
  {
    return -1;
  }
  return letter - 'A';
}

std::string FieldGrid::cellName(int col, int row)
{
  return colName(col) + std::to_string(row + 1);
}

bool FieldGrid::parseCellName(const std::string &name, int &col, int &row)
{
  if (name.size() < 2)
  {
    return false;
  }
  const int c = colIndex(name[0]);
  if (c < 0)
  {
    return false;
  }
  for (size_t i = 1; i < name.size(); ++i)
  {
    if (!std::isdigit(static_cast<unsigned char>(name[i])))
    {
      return false;
    }
  }
  try
  {
    const int r = std::stoi(name.substr(1)) - 1;
    if (r < 0)
    {
      return false;
    }
    col = c;
    row = r;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

// -----------------------------------------------------------------------------
// 边界 / 格内
// -----------------------------------------------------------------------------
bool FieldGrid::inBounds(int col, int row) const
{
  return col >= 0 && col < spec_.cols && row >= 0 && row < spec_.rows;
}

uint8_t FieldGrid::at(int col, int row) const
{
  if (!inBounds(col, row))
  {
    return kBlocked;
  }
  return cells_[static_cast<size_t>(idx(col, row))];
}

void FieldGrid::set(int col, int row, uint8_t v)
{
  if (!inBounds(col, row))
  {
    return;
  }
  cells_[static_cast<size_t>(idx(col, row))] = v;
}

bool FieldGrid::cellPassable(int col, int row) const
{
  if (!inBounds(col, row))
  {
    return false;
  }
  const uint8_t v = cells_[static_cast<size_t>(idx(col, row))];
  return v == kFree || v == kMine;
}

// -----------------------------------------------------------------------------
// 坐标变换
// -----------------------------------------------------------------------------
void FieldGrid::cellToWorld(int col, int row, double &x, double &y) const
{
  x = (static_cast<double>(col) + 0.5) * spec_.cell_size;
  y = (static_cast<double>(row) + 0.5) * spec_.cell_size;
}

bool FieldGrid::worldToCell(double x, double y, int &col, int &row) const
{
  col = static_cast<int>(std::floor(x / spec_.cell_size));
  row = static_cast<int>(std::floor(y / spec_.cell_size));
  return inBounds(col, row);
}

// -----------------------------------------------------------------------------
// 固定结构
// -----------------------------------------------------------------------------
bool FieldGrid::onBarrierLine(int col) const
{
  return col >= spec_.barrier_col_lo && col <= spec_.barrier_col_hi;
}

bool FieldGrid::isBarrierEdge(int col, int row) const
{
  return onBarrierLine(col) && row == spec_.barrier_up_row;
}

bool FieldGrid::isBridgeInterval(int row) const
{
  return (row >= spec_.bridge_a_lo && row <= spec_.bridge_a_hi) ||
         (row >= spec_.bridge_b_lo && row <= spec_.bridge_b_hi);
}

// -----------------------------------------------------------------------------
// 边通行性
// -----------------------------------------------------------------------------
bool FieldGrid::edgeOpen(int col, int row, Edge e) const
{
  if (!inBounds(col, row))
  {
    return false;
  }
  if (e == kEdgeUp)
  {
    if (row + 1 >= spec_.rows)
    {
      return false;
    }
    return blocked_up_[static_cast<size_t>(idx(col, row))] == 0;
  }
  if (col + 1 >= spec_.cols)
  {
    return false;
  }
  return blocked_right_[static_cast<size_t>(idx(col, row))] == 0;
}

void FieldGrid::setEdgeOpen(int col, int row, Edge e, bool open)
{
  if (!inBounds(col, row))
  {
    return;
  }
  if (e == kEdgeUp)
  {
    if (row + 1 >= spec_.rows)
    {
      return;
    }
    blocked_up_[static_cast<size_t>(idx(col, row))] = open ? 0 : 1;
    return;
  }
  if (col + 1 >= spec_.cols)
  {
    return;
  }
  blocked_right_[static_cast<size_t>(idx(col, row))] = open ? 0 : 1;
}

bool FieldGrid::canStep(int c0, int r0, int c1, int r1) const
{
  if (!cellPassable(c0, r0) || !cellPassable(c1, r1))
  {
    return false;
  }
  if (c0 == c1)
  {
    const int up = std::min(r0, r1);
    if (std::abs(r1 - r0) != 1)
    {
      return false;
    }
    return edgeOpen(c0, up, kEdgeUp);
  }
  if (r0 == r1)
  {
    const int left = std::min(c0, c1);
    if (std::abs(c1 - c0) != 1)
    {
      return false;
    }
    return edgeOpen(left, r0, kEdgeRight);
  }
  return false;
}

bool FieldGrid::isOpenGate(int col, int row) const
{
  // ★ 注意：这里**只判断"这个格是不是已指定的门"**，
  //   不要把它和"两个门是否都已指定"混在一起。
  //   早先我写成 `gate_a_row_ + gate_b_row_ >= 0 && ...`，
  //   导致只指定一个门时该格被判为不可通行 -> 单门无法跨线。
  //   但 K 列本身就是通道（可沿列走），水平隔离线的缺口只需要**一个**门：
  //     南侧进 K 列 -> 在门那一行跨过隔离线 -> 沿 K 列北上 -> 北侧出
  //   所以单门足够跨线；两个门都开只是让路程更短。
  //   （比赛每局固定指定 2 个门，但模型不应在只给 1 个时失效。）
  if (col != spec_.bridge_col)
  {
    return false;
  }
  return (gate_a_row_ >= 0 && row == gate_a_row_) ||
         (gate_b_row_ >= 0 && row == gate_b_row_);
}

// -----------------------------------------------------------------------------
// 桥洞通道
// -----------------------------------------------------------------------------
// 说明书：两处桥洞各随机指定 1 个可通行通道。
//   · 区段一 K1–K6   （6 个候选）
//   · 区段二 K7–K11  （5 个候选）
// 被指定的通道格变成可通行（其余 K 列格是隔板，不可通行）；
// 水平隔离线在 K 列的纵向边由门格本身"打通"——因为门格可通行，
// 车从东侧格进入门格、再出到西侧格，就是一次穿桥洞。
bool FieldGrid::openCrossing(int row)
{
  if (!isBridgeInterval(row) || !inBounds(spec_.bridge_col, row))
  {
    return false;
  }

  if (row >= spec_.bridge_a_lo && row <= spec_.bridge_a_hi)
  {
    if (gate_a_row_ >= 0 && gate_a_row_ != row && inBounds(spec_.bridge_col, gate_a_row_))
    {
      // 同区段改判：旧通道恢复为隔板
      cells_[static_cast<size_t>(idx(spec_.bridge_col, gate_a_row_))] = kBlocked;
    }
    gate_a_row_ = row;
  }
  else
  {
    if (gate_b_row_ >= 0 && gate_b_row_ != row && inBounds(spec_.bridge_col, gate_b_row_))
    {
      cells_[static_cast<size_t>(idx(spec_.bridge_col, gate_b_row_))] = kBlocked;
    }
    gate_b_row_ = row;
  }

  cells_[static_cast<size_t>(idx(spec_.bridge_col, row))] = kFree;
  // 门格所在的水平隔离线纵向边也要打开：桥洞是在挡板上开的缺口
  if (row == spec_.barrier_up_row || row == spec_.barrier_up_row + 1)
  {
    const int up = spec_.barrier_up_row;
    setEdgeOpen(spec_.bridge_col, up, kEdgeUp, true);
  }
  return true;
}

bool FieldGrid::closeCrossing(int row)
{
  if (!isBridgeInterval(row) || !inBounds(spec_.bridge_col, row))
  {
    return false;
  }
  if (row == gate_a_row_)
  {
    gate_a_row_ = -1;
  }
  else if (row == gate_b_row_)
  {
    gate_b_row_ = -1;
  }
  else
  {
    return false;
  }
  cells_[static_cast<size_t>(idx(spec_.bridge_col, row))] = kBlocked;

  // 两个门都不在隔离线行上时，把该纵向边恢复为封死
  const bool line_gate_open =
      (gate_a_row_ == spec_.barrier_up_row) || (gate_b_row_ == spec_.barrier_up_row) ||
      (gate_a_row_ == spec_.barrier_up_row + 1) || (gate_b_row_ == spec_.barrier_up_row + 1);
  if (!line_gate_open)
  {
    setEdgeOpen(spec_.bridge_col, spec_.barrier_up_row, kEdgeUp, false);
  }
  return true;
}

void FieldGrid::closeAllCrossings()
{
  for (int r : {gate_a_row_, gate_b_row_})
  {
    if (r >= 0 && inBounds(spec_.bridge_col, r))
    {
      cells_[static_cast<size_t>(idx(spec_.bridge_col, r))] = kBlocked;
    }
  }
  gate_a_row_ = -1;
  gate_b_row_ = -1;
  setEdgeOpen(spec_.bridge_col, spec_.barrier_up_row, kEdgeUp, false);
}

int FieldGrid::openCrossingCount() const
{
  return (gate_a_row_ >= 0 ? 1 : 0) + (gate_b_row_ >= 0 ? 1 : 0);
}

// -----------------------------------------------------------------------------
// 地形更新
// -----------------------------------------------------------------------------
void FieldGrid::markBlocked(int col, int row)
{
  if (!inBounds(col, row))
  {
    return;
  }
  cells_[static_cast<size_t>(idx(col, row))] = kBlocked;
}

void FieldGrid::markFree(int col, int row)
{
  if (!inBounds(col, row))
  {
    return;
  }
  cells_[static_cast<size_t>(idx(col, row))] = kFree;
}

void FieldGrid::markMine(int col, int row)
{
  if (!inBounds(col, row))
  {
    return;
  }
  cells_[static_cast<size_t>(idx(col, row))] = kMine;
}

void FieldGrid::markTarget(int col, int row)
{
  if (!inBounds(col, row))
  {
    return;
  }
  cells_[static_cast<size_t>(idx(col, row))] = kTarget;
}

// -----------------------------------------------------------------------------
// 连通性
// -----------------------------------------------------------------------------
bool FieldGrid::findPath(int c0, int r0, int c1, int r1,
                         std::vector<std::pair<int, int>> &path) const
{
  path.clear();
  if (!cellPassable(c0, r0) || !cellPassable(c1, r1))
  {
    return false;
  }

  const int W = spec_.cols;
  const int H = spec_.rows;
  std::vector<int> prev(static_cast<size_t>(W) * H, -2);
  const int start = r0 * W + c0;
  const int goal = r1 * W + c1;
  prev[static_cast<size_t>(start)] = -1;

  std::deque<std::pair<int, int>> q;
  q.emplace_back(c0, r0);
  bool found = false;

  const int dc[4] = {1, -1, 0, 0};
  const int dr[4] = {0, 0, 1, -1};

  while (!q.empty())
  {
    const int cc = q.front().first;
    const int cr = q.front().second;
    q.pop_front();
    if (cc == c1 && cr == r1)
    {
      found = true;
      break;
    }
    for (int k = 0; k < 4; ++k)
    {
      const int nc = cc + dc[k];
      const int nr = cr + dr[k];
      if (!canStep(cc, cr, nc, nr))
      {
        continue;
      }
      const int ni = nr * W + nc;
      if (prev[static_cast<size_t>(ni)] != -2)
      {
        continue;
      }
      prev[static_cast<size_t>(ni)] = cr * W + cc;
      q.emplace_back(nc, nr);
    }
  }

  if (!found)
  {
    return false;
  }

  std::vector<std::pair<int, int>> rev;
  int cur = goal;
  while (cur != -1)
  {
    rev.emplace_back(cur % W, cur / W);
    cur = prev[static_cast<size_t>(cur)];
  }
  path.assign(rev.rbegin(), rev.rend());
  return true;
}

int FieldGrid::pathLength(int c0, int r0, int c1, int r1) const
{
  std::vector<std::pair<int, int>> p;
  if (!findPath(c0, r0, c1, r1, p))
  {
    return -1;
  }
  return static_cast<int>(p.size());
}

// =============================================================================
// 原地转向（pivot turn）
// =============================================================================
bool FieldGrid::markMineByRecon(int col, int row)
{
  if (!inBounds(col, row))
  {
    return false;
  }
  // ★ 防御：K 列是桥洞隔板，只有 openCrossing() 能改它的可通行性。
  //   侦察数据不得覆盖 —— 否则会把整列打通，规划出穿过隔板的路径。
  if (col == spec_.bridge_col)
  {
    return false;
  }
  const size_t i = static_cast<size_t>(idx(col, row));
  if (cells_[i] == kBlocked)
  {
    return false;   // 已被标为障碍，不再标雷（障碍优先）
  }
  if (cells_[i] == kMine)
  {
    return false;   // 已标过，幂等
  }
  cells_[i] = kMine;
  return true;
}

bool FieldGrid::markBlockedByRecon(int col, int row)
{
  if (!inBounds(col, row))
  {
    return false;
  }
  // ★ 同上：K 列不参与侦察覆盖
  if (col == spec_.bridge_col)
  {
    return false;
  }
  // ★ 起终点格不允许被标为障碍（上层已过滤，这里再防一次）
  if ((col == spec_.start_col && row == spec_.start_row) ||
      (col == spec_.goal_col && row == spec_.goal_row))
  {
    return false;
  }
  const size_t i = static_cast<size_t>(idx(col, row));
  if (cells_[i] == kBlocked)
  {
    return false;   // 幂等
  }
  cells_[i] = kBlocked;
  return true;
}

bool FieldGrid::markTargetByRecon(int col, int row)
{
  if (!inBounds(col, row))
  {
    return false;
  }
  if (col == spec_.bridge_col)
  {
    return false;
  }
  const size_t i = static_cast<size_t>(idx(col, row));
  if (cells_[i] == kBlocked)
  {
    return false;
  }
  cells_[i] = kTarget;
  return true;
}

FieldGrid::PivotInfo FieldGrid::checkPivot(int col, int row) const
{
  PivotInfo info;
  info.col = col;
  info.row = row;

  if (!inBounds(col, row))
  {
    info.reason = "格坐标越界";
    return info;
  }
  info.in_bounds = true;
  if (!cellPassable(col, row))
  {
    info.reason = "本格不可通行";
    return info;
  }

  cellToWorld(col, row, info.cx, info.cy);
  info.cx *= 100.0;   // -> cm
  info.cy *= 100.0;

  // 车体绕后轴中心纯转动的扫掠半径 = 车体四角到后轴中心的最大距离
  const double hw = spec_.car_width_cm * 0.5;
  const double r_front = std::hypot(spec_.front_overhang_cm, hw);
  const double r_rear = std::hypot(spec_.rear_overhang_cm, hw);
  const double r_sweep = std::max(r_front, r_rear);
  info.sweep_radius_cm = r_sweep;

  const double cell = spec_.cell_size * 100.0;

  // ★★ 在格内取多个**候选位姿**，任一可行即认为该格可转向 ★★
  // 原实现只在**格心**判定。但车并不必须在格心转向 —— 它可以先在格内
  // 挪一点（例如贴边的格向内偏 5cm），再原地转。只在格心判定会把大量
  // **实际可行**的格误判为不可行，进而导致路径里出现"既不能单圆弧、
  // 也不能原地转向"的死角。
  // ★ 实测（Python 对照实现）：加入候选位姿后，"不可执行转角"从 55 处
  //   降到 5 处。剩下 5 处在 K4/K9 门格（贴水平挡板），需要更大内移量。
  const double offsets[9][2] = {
      {0.0, 0.0},
      {6.0, 0.0}, {-6.0, 0.0}, {0.0, 6.0}, {0.0, -6.0},
      {4.0, 4.0}, {4.0, -4.0}, {-4.0, 4.0}, {-4.0, -4.0}};

  const double half = cell * 0.5;
  const double half_diag = cell * 0.5 * std::sqrt(2.0);
  std::string last_reason = "在格内找不到可转向的位姿";

  for (int oi = 0; oi < 9; ++oi)
  {
    const double cx = info.cx + offsets[oi][0];
    const double cy = info.cy + offsets[oi][1];
    // 位姿必须仍在本格内
    if (std::fabs(cx - info.cx) > half || std::fabs(cy - info.cy) > half)
    {
      continue;
    }

    bool ok = true;

    // ---- 1. 场地边界（不得驶出 4m×5m）----
    const double field_w = spec_.cols * cell;
    const double field_h = spec_.rows * cell;
    if (cx - r_sweep < 0.0 || cx + r_sweep > field_w ||
        cy - r_sweep < 0.0 || cy + r_sweep > field_h)
    {
      ok = false;
      last_reason = "扫掠圆越出场地边界";
    }

    // ---- 2. 水平隔离线挡板 ----
    if (ok)
    {
      const double wall_y = (spec_.barrier_up_row + 1) * cell;
      const double wall_x0 = spec_.barrier_col_lo * cell;
      const double wall_x1 = (spec_.barrier_col_hi + 1) * cell;
      if (cx + r_sweep >= wall_x0 && cx - r_sweep <= wall_x1)
      {
        if (std::fabs(cy - wall_y) < r_sweep)
        {
          ok = false;
          last_reason = "扫掠圆压到水平隔离线挡板";
        }
      }
    }

    // ---- 3. 扫掠圆覆盖到的格必须全部可通行 ----
    // 注意：格与格之间的分界线**不是障碍物**（场地没有四周围挡），
    // 所以判据是"覆盖到的格都可通行"，而不是"圆心到格边界的距离"。
    if (ok)
    {
      const int c_lo = static_cast<int>(std::floor((cx - r_sweep) / cell));
      const int c_hi = static_cast<int>(std::floor((cx + r_sweep) / cell));
      const int r_lo = static_cast<int>(std::floor((cy - r_sweep) / cell));
      const int r_hi = static_cast<int>(std::floor((cy + r_sweep) / cell));

      int covered = 0;
      int passable_cnt = 0;
      for (int c = c_lo; c <= c_hi && ok; ++c)
      {
        for (int r = r_lo; r <= r_hi; ++r)
        {
          const double gx = (c + 0.5) * cell;
          const double gy = (r + 0.5) * cell;
          if (std::hypot(gx - cx, gy - cy) > r_sweep + half_diag)
          {
            continue;   // 该格与扫掠圆不相交
          }
          ++covered;
          if (!cellPassable(c, r))
          {
            ok = false;
            last_reason = std::string("扫掠圆覆盖到不可通行格 ") +
                          cellName(c, r);
            break;
          }
          ++passable_cnt;
        }
      }
      if (ok)
      {
        info.cells_covered = covered;
        info.cells_passable = passable_cnt;
      }
    }

    // ---- 4. 轮胎轨迹必须落在可通行格内 ----
    if (ok && spec_.turn_check_wheel_track)
    {
      const double htrack = spec_.track_cm * 0.5;
      const int steps = 72;
      for (int i = 0; i < steps && ok; ++i)
      {
        const double a = 2.0 * M_PI * i / steps;
        for (int sgn = -1; sgn <= 1 && ok; sgn += 2)
        {
          const double wx = cx + spec_.wheelbase_cm * std::cos(a) -
                            sgn * htrack * std::sin(a);
          const double wy = cy + spec_.wheelbase_cm * std::sin(a) +
                            sgn * htrack * std::cos(a);
          const int wc = static_cast<int>(std::floor(wx / cell));
          const int wr = static_cast<int>(std::floor(wy / cell));
          if (!cellPassable(wc, wr))
          {
            ok = false;
            info.wheels_ok = false;
            last_reason = std::string("轮胎轨迹进入不可通行格 ") +
                          cellName(wc, wr);
          }
        }
      }
    }

    if (ok)
    {
      // ---- 5. 扫掠圆到真实阻挡的最小间隙 ----
      double gap = std::min(std::min(cx - r_sweep, field_w - cx - r_sweep),
                            std::min(cy - r_sweep, field_h - cy - r_sweep));
      const double wall_y = (spec_.barrier_up_row + 1) * cell;
      const double wall_x0 = spec_.barrier_col_lo * cell;
      const double wall_x1 = (spec_.barrier_col_hi + 1) * cell;
      if (cx + r_sweep >= wall_x0 && cx - r_sweep <= wall_x1)
      {
        gap = std::min(gap, std::fabs(cy - wall_y) - r_sweep);
      }
      info.min_gap_cm = gap;
      if (gap < spec_.pivot_min_gap_cm)
      {
        last_reason = "扫掠圆到阻挡的间隙不足";
        continue;   // 试下一个候选位姿
      }

      info.feasible = true;
      info.reason = "可原地转向（扫掠半径 " +
                    std::to_string(static_cast<int>(r_sweep)) + " cm，覆盖 " +
                    std::to_string(info.cells_covered) + " 格全部可通行，" +
                    (oi == 0 ? std::string("格心位姿")
                             : "需内移 " +
                                   std::to_string(static_cast<int>(
                                       std::hypot(offsets[oi][0], offsets[oi][1]))) +
                                   " cm") +
                    "）";
      return info;
    }
  }

  info.feasible = false;
  info.reason = last_reason;
  return info;
}

bool FieldGrid::pathPivotsFeasible(const std::vector<std::pair<int, int>> &path,
                                   std::vector<std::string> &bad) const
{
  bad.clear();
  if (path.size() < 3)
  {
    return true;
  }
  // 只在方向发生变化的格检查（直行段不需要原地转向）
  for (size_t i = 2; i < path.size(); ++i)
  {
    const bool ab_vert = (path[i - 2].first == path[i - 1].first);
    const bool bc_vert = (path[i - 1].first == path[i].first);
    if (ab_vert == bc_vert)
    {
      continue;   // 直行
    }
    const auto p = checkPivot(path[i - 1].first, path[i - 1].second);
    if (!p.feasible)
    {
      bad.push_back("在 " + cellName(path[i - 1].first, path[i - 1].second) +
                    " 原地转向不可行：" + p.reason);
      if (bad.size() >= 8)
      {
        break;
      }
    }
  }
  return bad.empty();
}

// -----------------------------------------------------------------------------
// 转弯可行 + 方向感知的最短路径
// -----------------------------------------------------------------------------
namespace
{
// 方向编码：0=+x, 1=-x, 2=+y, 3=-y, 4=未定（起点无来向）
inline int dirCode(int dc, int dr)
{
  if (dc == 1) return 0;
  if (dc == -1) return 1;
  if (dr == 1) return 2;
  if (dr == -1) return 3;
  return 4;
}
inline void codeDelta(int code, int &dc, int &dr)
{
  dc = 0;
  dr = 0;
  switch (code)
  {
    case 0: dc = 1; break;
    case 1: dc = -1; break;
    case 2: dr = 1; break;
    case 3: dr = -1; break;
    default: break;
  }
}
}  // namespace

bool FieldGrid::findPathAware(int c0, int r0, int c1, int r1,
                              std::vector<std::pair<int, int>> &path,
                              const PathOptions &opt) const
{
  path.clear();
  if (!cellPassable(c0, r0) || !cellPassable(c1, r1))
  {
    return false;
  }

  const int W = spec_.cols;
  const int H = spec_.rows;
  const int kStates = 5;                     // 4 个方向 + 未定
  const size_t total = static_cast<size_t>(W) * H * kStates;

  std::vector<int> prev(total, -2);          // -2 未访问，-1 起点

  auto sidx = [&](int c, int r, int code) -> size_t {
    return (static_cast<size_t>(r) * W + static_cast<size_t>(c)) * kStates +
           static_cast<size_t>(code);
  };

  const bool have_heading = (opt.heading_dc != 0 || opt.heading_dr != 0);
  const int start_code = have_heading ? dirCode(opt.heading_dc, opt.heading_dr) : 4;
  const size_t start_id = sidx(c0, r0, start_code);
  prev[start_id] = -1;

  std::deque<std::array<int, 3>> q;          // {col, row, dir_code}
  q.push_back({c0, r0, start_code});

  const int dc4[4] = {1, -1, 0, 0};
  const int dr4[4] = {0, 0, 1, -1};

  size_t goal_id = total;
  bool found = false;

  while (!q.empty())
  {
    const auto cur = q.front();
    q.pop_front();
    const int cc = cur[0];
    const int cr = cur[1];
    const int hcode = cur[2];

    if (cc == c1 && cr == r1)
    {
      goal_id = sidx(cc, cr, hcode);
      found = true;
      break;
    }

    for (int k = 0; k < 4; ++k)
    {
      const int nc = cc + dc4[k];
      const int nr = cr + dr4[k];
      if (!canStep(cc, cr, nc, nr))
      {
        continue;
      }
      const int ncode = k;
      int hdc = 0;
      int hdr = 0;
      const bool have_dir = (hcode != 4);
      if (have_dir)
      {
        codeDelta(hcode, hdc, hdr);
      }
      // 禁止立即 180° 折返
      if (opt.forbid_reverse && have_dir &&
          dc4[k] == -hdc && dr4[k] == -hdr)
      {
        continue;
      }
      // 转弯可行性
      if (opt.turn_aware && have_dir && ncode != hcode)
      {
        const auto t = checkTurn(cc - hdc, cr - hdr, cc, cr, nc, nr);
        if (!t.feasible)
        {
          // ★ pivot_ok：单圆弧不行时，若该格原地转向可行则放行。
          //   这正是 v5 拓扑下窄条（K 以东）的实际情况。
          bool allow = false;
          if (opt.pivot_ok)
          {
            allow = checkPivot(cc, cr).feasible;
          }
          if (!allow)
          {
            continue;
          }
        }
      }
      const size_t nid = sidx(nc, nr, ncode);
      if (prev[nid] != -2)
      {
        continue;
      }
      prev[nid] = static_cast<int>(sidx(cc, cr, hcode));
      q.push_back({nc, nr, ncode});
    }
  }

  if (!found)
  {
    return false;
  }

  // 回溯
  std::vector<std::pair<int, int>> rev;
  size_t cur_id = goal_id;
  while (cur_id != static_cast<size_t>(-1))
  {
    const size_t cell = cur_id / static_cast<size_t>(kStates);
    rev.emplace_back(static_cast<int>(cell % static_cast<size_t>(W)),
                     static_cast<int>(cell / static_cast<size_t>(W)));
    const int p = prev[cur_id];
    if (p < 0)
    {
      break;
    }
    cur_id = static_cast<size_t>(p);
  }
  path.assign(rev.rbegin(), rev.rend());
  return true;
}

bool FieldGrid::findPathAware(int c0, int r0, int c1, int r1,
                              std::vector<std::pair<int, int>> &path,
                              bool turn_aware,
                              int heading_dc, int heading_dr,
                              bool forbid_reverse) const
{
  PathOptions opt;
  opt.turn_aware = turn_aware;
  opt.forbid_reverse = forbid_reverse;
  opt.heading_dc = heading_dc;
  opt.heading_dr = heading_dr;
  opt.pivot_ok = false;
  return findPathAware(c0, r0, c1, r1, path, opt);
}

// -----------------------------------------------------------------------------
// ★ 分层回退搜索
// -----------------------------------------------------------------------------
bool FieldGrid::findPathStaged(int c0, int r0, int c1, int r1,
                               std::vector<std::pair<int, int>> &path,
                               int heading_dc, int heading_dr,
                               int *used_level) const
{
  struct Level
  {
    bool turn_aware;
    bool forbid_reverse;
    bool use_heading;
    bool pivot_ok;
  };
  // ★ 顺序很关键：优先走严格约束（路径可直接执行），
  //   只有真的不连通才逐级放宽。一步放开会导致"覆盖率高但路径执行不了"。
  const Level ladder[5] = {
      {true,  true,  true,  false},   // 1 最严格
      {true,  true,  false, false},   // 2 放开 heading
      {true,  true,  false, true},    // 3 允许原地转向的转角
      {false, true,  true,  false},   // 4 放开 turn_aware，保留禁折返
      {false, false, false, false},   // 5 全放开
  };

  for (int i = 0; i < 5; ++i)
  {
    PathOptions opt;
    opt.turn_aware = ladder[i].turn_aware;
    opt.forbid_reverse = ladder[i].forbid_reverse;
    opt.pivot_ok = ladder[i].pivot_ok;
    opt.heading_dc = ladder[i].use_heading ? heading_dc : 0;
    opt.heading_dr = ladder[i].use_heading ? heading_dr : 0;
    if (findPathAware(c0, r0, c1, r1, path, opt))
    {
      if (used_level != nullptr)
      {
        *used_level = i + 1;
      }
      return true;
    }
  }
  path.clear();
  if (used_level != nullptr)
  {
    *used_level = 0;
  }
  return false;
}

// -----------------------------------------------------------------------------
// 轮心进圆校验
// -----------------------------------------------------------------------------
int FieldGrid::verifyWheelInCircle(const std::vector<std::pair<int, int>> &path,
                                   const std::vector<std::pair<int, int>> &mines,
                                   double mine_radius_cm,
                                   std::vector<double> *min_dist_out) const
{
  if (min_dist_out != nullptr)
  {
    min_dist_out->assign(mines.size(), 1e9);
  }
  if (path.size() < 2 || mines.empty())
  {
    return 0;
  }

  const double step = 4.0;   // cm，格宽 35cm → 每格约 9 个采样点

  int hits = 0;
  for (size_t mi = 0; mi < mines.size(); ++mi)
  {
    double mx = 0.0;
    double my = 0.0;
    cellToWorld(mines[mi].first, mines[mi].second, mx, my);
    mx *= 100.0;
    my *= 100.0;

    double best = 1e9;
    for (size_t i = 1; i < path.size(); ++i)
    {
      double x0 = 0.0;
      double y0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      cellToWorld(path[i - 1].first, path[i - 1].second, x0, y0);
      cellToWorld(path[i].first, path[i].second, x1, y1);
      x0 *= 100.0;
      y0 *= 100.0;
      x1 *= 100.0;
      y1 *= 100.0;

      const double seg = std::hypot(x1 - x0, y1 - y0);
      const int n = std::max(1, static_cast<int>(seg / step));
      const double yaw = std::atan2(y1 - y0, x1 - x0);

      for (int k = 0; k <= n; ++k)
      {
        const double t = static_cast<double>(k) / n;
        const double px = x0 + (x1 - x0) * t;
        const double py = y0 + (y1 - y0) * t;
        double wxs[4];
        double wys[4];
        wheelPoints(px, py, yaw, wxs, wys);
        for (int w = 0; w < 4; ++w)
        {
          const double d = std::hypot(wxs[w] - mx, wys[w] - my);
          best = std::min(best, d);
        }
      }
    }
    if (min_dist_out != nullptr)
    {
      (*min_dist_out)[mi] = best;
    }
    if (best < mine_radius_cm)
    {
      ++hits;
    }
  }
  return hits;
}

int FieldGrid::reachableCount(int c, int r) const
{
  if (!cellPassable(c, r))
  {
    return 0;
  }
  const int W = spec_.cols;
  const int H = spec_.rows;
  std::vector<uint8_t> seen(static_cast<size_t>(W) * H, 0);
  std::deque<std::pair<int, int>> q;
  q.emplace_back(c, r);
  seen[static_cast<size_t>(r * W + c)] = 1;
  int n = 0;
  const int dc[4] = {1, -1, 0, 0};
  const int dr[4] = {0, 0, 1, -1};
  while (!q.empty())
  {
    const int cc = q.front().first;
    const int cr = q.front().second;
    q.pop_front();
    ++n;
    for (int k = 0; k < 4; ++k)
    {
      const int nc = cc + dc[k];
      const int nr = cr + dr[k];
      if (!canStep(cc, cr, nc, nr))
      {
        continue;
      }
      const size_t ni = static_cast<size_t>(nr * W + nc);
      if (seen[ni] != 0)
      {
        continue;
      }
      seen[ni] = 1;
      q.emplace_back(nc, nr);
    }
  }
  return n;
}

// -----------------------------------------------------------------------------
// 净空与边界约束
// -----------------------------------------------------------------------------
double FieldGrid::clearanceCm(int col, int row) const
{
  double cx = 0.0;
  double cy = 0.0;
  cellToWorld(col, row, cx, cy);
  cx *= 100.0;   // -> cm
  cy *= 100.0;

  // 说明书明确"场地没有四周的围挡"，所以没有实体围栏可碰。
  // 但车不得驶出 4m×5m 场地范围，因此把"场地边缘"当作软边界：
  // 格心到场边的距离越小，可用余量越小。
  const double field_w = spec_.cols * spec_.cell_size * 100.0;
  const double field_h = spec_.rows * spec_.cell_size * 100.0;
  const double to_edge =
      std::min(std::min(cx, field_w - cx), std::min(cy, field_h - cy));

  // 水平隔离线挡板（黄色挡板 + 白色长条）：y = wall_y，覆盖 D..M 列
  const double wall_y = (spec_.barrier_up_row + 1) * spec_.cell_size * 100.0;
  double to_wall = 1e18;
  if (onBarrierLine(col))
  {
    to_wall = std::fabs(cy - wall_y);
  }

  // K 列隔板：车道两侧都是隔板，横向可用宽度受相邻通道影响
  const double to_bridge = (col == spec_.bridge_col) ? 1e18 : 0.0;
  (void)to_bridge;

  return std::min(to_edge, to_wall);
}

bool FieldGrid::clearanceOk(int col, int row) const
{
  if (spec_.min_clearance_cm <= 0.0)
  {
    return true;
  }
  return clearanceCm(col, row) >= spec_.min_clearance_cm;
}

bool FieldGrid::baselineRoute(std::vector<std::pair<int, int>> &path) const
{
  // 起终点固定通路，不依赖任何无人机信息：
  //   M7 → N7 → N6 → N5
  // 因 N 列不被不可跨越线覆盖，这条通路无需任何桥洞门。
  return findPath(spec_.start_col, spec_.start_row,
                  spec_.goal_col, spec_.goal_row, path);
}

bool FieldGrid::pathHasSharpCorner(const std::vector<std::pair<int, int>> &path) const
{
  if (path.size() < 3)
  {
    return false;
  }
  double dx0 = 0.0;
  double dy0 = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    const double dx = static_cast<double>(path[i].first - path[i - 1].first);
    const double dy = static_cast<double>(path[i].second - path[i - 1].second);
    if (i >= 2)
    {
      const bool turned = (dx != dx0) || (dy != dy0);
      if (turned)
      {
        // 四连通只有 90° 转向 —— 需要的最小转弯半径为半格
        const double half_cell = spec_.cell_size * 100.0 * 0.5;
        // 取最宽松的前轮转角（35°）都不够时，判定为"直角不可行"
        if (minTurnRadiusCm(35.0) > half_cell)
        {
          return true;
        }
      }
    }
    dx0 = dx;
    dy0 = dy;
  }
  return false;
}

double FieldGrid::minTurnRadiusCm(double max_steer_deg) const
{
  const double d = max_steer_deg * M_PI / 180.0;
  const double t = std::tan(d);
  if (t <= 1e-6)
  {
    return 1e9;
  }
  return spec_.wheelbase_cm / t;
}

// =============================================================================
// 转弯可行性
// =============================================================================
void FieldGrid::bodyCorners(double x, double y, double yaw,
                            double *xs, double *ys) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hw = spec_.car_width_cm * 0.5;
  const double f = spec_.front_overhang_cm;
  const double r = spec_.rear_overhang_cm;

  const double local[4][2] = {{f, hw}, {f, -hw}, {-r, -hw}, {-r, hw}};
  for (int i = 0; i < 4; ++i)
  {
    xs[i] = x + local[i][0] * c - local[i][1] * s;
    ys[i] = y + local[i][0] * s + local[i][1] * c;
  }
}

void FieldGrid::wheelPoints(double x, double y, double yaw,
                            double *xs, double *ys) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hw = spec_.track_cm * 0.5;

  // 0/1: 后轴左右（后轴中心即 base_link），2/3: 前轴左右
  const double local[4][2] = {{0.0, hw}, {0.0, -hw},
                              {spec_.wheelbase_cm, hw},
                              {spec_.wheelbase_cm, -hw}};
  for (int i = 0; i < 4; ++i)
  {
    xs[i] = x + local[i][0] * c - local[i][1] * s;
    ys[i] = y + local[i][0] * s + local[i][1] * c;
  }
}

// 世界点 (x,y)（cm）到"不可驶入区域"的间隙：
//   正 = 该点在可通行格内，值为到最近阻挡（含场地边界）的距离
//   负 = 该点落在阻挡格内或场外，值为侵入深度（取负）
double FieldGrid::obstacleGapCm(double x, double y) const
{
  const double cell = spec_.cell_size * 100.0;

  // 场地边界（没有四周围挡，但不得驶出 4m×5m）
  const double field_w = spec_.cols * cell;
  const double field_h = spec_.rows * cell;
  double to_edge = std::min(std::min(x, field_w - x), std::min(y, field_h - y));
  if (to_edge < 0.0)
  {
    return to_edge;   // 已出界
  }

  // 所在格
  const int c = static_cast<int>(std::floor(x / cell));
  const int r = static_cast<int>(std::floor(y / cell));

  // 水平隔离线：y = (barrier_up_row+1)*cell，覆盖 D..M 列
  // 它是一条线（不可跨越），车体压上去即为侵入
  double to_wall = 1e9;
  const double wall_y = (spec_.barrier_up_row + 1) * cell;
  if (onBarrierLine(c) ||
      // 若点在墙的 x 跨度内，也要算（用点的 x 判断而非格号）
      (x >= spec_.barrier_col_lo * cell && x <= (spec_.barrier_col_hi + 1) * cell))
  {
    to_wall = std::fabs(y - wall_y);
  }

  // 该格内通行性
  double to_blocked = 1e9;
  if (!inBounds(c, r) || !cellPassable(c, r))
  {
    // 落在阻挡格内：算作侵入。用格心到点的负距离近似深度
    const double ccx = (c + 0.5) * cell;
    const double ccy = (r + 0.5) * cell;
    const double d = std::hypot(x - ccx, y - ccy);
    to_blocked = -std::max(1.0, cell * 0.5 - d);   // 保守给负值
  }
  else
  {
    // 在可通行格内：到相邻阻挡格的边界距离
    const double dx = std::min(x - c * cell, (c + 1) * cell - x);
    const double dy = std::min(y - r * cell, (r + 1) * cell - y);
    // 只在与阻挡相邻的方向上限制
    const int nb[4][2] = {{c + 1, r}, {c - 1, r}, {c, r + 1}, {c, r - 1}};
    const double nd[4] = {dx,
                          std::min(x - c * cell, (c + 1) * cell - x),
                          dy,
                          std::min(y - r * cell, (r + 1) * cell - y)};
    for (int k = 0; k < 4; ++k)
    {
      if (!inBounds(nb[k][0], nb[k][1]) || !cellPassable(nb[k][0], nb[k][1]))
      {
        to_blocked = std::min(to_blocked, nd[k]);
      }
    }
  }

  return std::min(std::min(to_edge, to_wall), to_blocked);
}

double FieldGrid::sweepGap(double cx, double cy, double R, double yaw0,
                           double dpsi, bool *hit_body, bool *hit_wheel) const
{
  double best = 1e9;
  bool hb = false;
  bool hw = false;
  const int N = 60;   // 采样数（每 1.5°）

  for (int i = 0; i <= N; ++i)
  {
    const double f = static_cast<double>(i) / N;
    const double yaw = yaw0 + dpsi * f;
    // 后轴中心位于 yaw 方向前方 R（圆弧切向）——用圆心+半径反算
    const double px = cx + R * std::cos(yaw - M_PI / 2.0);
    const double py = cy + R * std::sin(yaw - M_PI / 2.0);

    if (spec_.turn_check_body_sweep)
    {
      double xs[4];
      double ys[4];
      bodyCorners(px, py, yaw, xs, ys);
      for (int k = 0; k < 4; ++k)
      {
        const double g = obstacleGapCm(xs[k], ys[k]);
        if (g < 0.0)
        {
          hb = true;
        }
        best = std::min(best, g);
      }
    }

    if (spec_.turn_check_wheel_track)
    {
      double xs[4];
      double ys[4];
      wheelPoints(px, py, yaw, xs, ys);
      for (int k = 0; k < 4; ++k)
      {
        const double g = obstacleGapCm(xs[k], ys[k]);
        if (g < 0.0)
        {
          hw = true;
        }
        best = std::min(best, g);
      }
    }
  }

  if (hit_body != nullptr)
  {
    *hit_body = hb;
  }
  if (hit_wheel != nullptr)
  {
    *hit_wheel = hw;
  }
  return best;
}

void FieldGrid::turnCenters(int ac, int ar, int bc, int br, int cc, int cr,
                            double &left_cx, double &left_cy,
                            double &right_cx, double &right_cy) const
{
  // ★ ar（入向格 a 的行号）**不参与**交点计算：
  //   交点由 b（转弯格）与 c（出向格）决定 ——
  //     先纵后横：x = b 列中心线，y = c 行中心线
  //     先横后纵：x = c 列中心线，y = b 行中心线
  //   两种情形都不需要 a 的行号。保留该参数是为了与其他转角函数的签名一致
  //   （调用方统一按 a,b,c 三格传入），故显式标注未使用。
  (void)ar;

  const double cell = spec_.cell_size * 100.0;
  // 入向车道中心线：a->b 方向的中心线
  // 出向车道中心线：b->c 方向的中心线
  // 转弯中心 = 两条中心线的交点
  //
  // 情形一：先纵向后横向（a->b 纵向，b->c 横向）
  //   入向中心线 x = b 列中心线；出向中心线 y = c 行中心线
  // 情形二：先横向后纵向
  //   入向中心线 y = b 行中心线；出向中心线 x = c 列中心线
  const bool in_vertical = (ac == bc);
  double ix_center = 0.0;
  double iy_center = 0.0;
  if (in_vertical)
  {
    ix_center = (bc + 0.5) * cell;
    iy_center = (cr + 0.5) * cell;
  }
  else
  {
    ix_center = (cc + 0.5) * cell;
    iy_center = (br + 0.5) * cell;
  }
  // 左转 / 右转在这里是同一个交点（只是行进方向相反），
  // 但扫掠方向不同，所以两个都返回同一位置，由上层的 yaw 递增方向区分。
  left_cx = ix_center;
  left_cy = iy_center;
  right_cx = ix_center;
  right_cy = iy_center;
}

FieldGrid::TurnInfo FieldGrid::checkTurn(int ac, int ar, int bc, int br,
                                         int cc, int cr) const
{
  TurnInfo info;
  info.in_col = bc;
  info.in_row = br;
  info.out_col = cc;
  info.out_row = cr;

  // ---- 1. 入参合法性：a->b->c 必须是四连通且构成 90° 转向 ----
  if (!inBounds(ac, ar) || !inBounds(bc, br) || !inBounds(cc, cr))
  {
    info.reason = "格坐标越界";
    return info;
  }
  const bool ab_vert = (ac == bc);
  const bool bc_vert = (bc == cc);
  if (std::abs(ar - br) + std::abs(ac - bc) != 1 ||
      std::abs(br - cr) + std::abs(bc - cc) != 1)
  {
    info.reason = "相邻性不成立（必须四连通）";
    return info;
  }
  if (ab_vert == bc_vert)
  {
    info.reason = "不是 90 度转向（前后方向共线）";
    return info;
  }
  if (!cellPassable(bc, br) || !cellPassable(cc, cr))
  {
    info.reason = "入向或出向格不可通行";
    return info;
  }
  info.valid = true;
  info.left_turn = ab_vert ? (cc > bc) : (cr < br);

  // ---- 2. 转弯中心：两条车道中心线的交点 ----
  double lcx = 0.0;
  double lcy = 0.0;
  double rcx = 0.0;
  double rcy = 0.0;
  turnCenters(ac, ar, bc, br, cc, cr, lcx, lcy, rcx, rcy);
  info.cx = lcx;
  info.cy = lcy;

  const double R = spec_.min_turn_radius_cm;

  // ---- 3. 起始偏航角：入向车道方向 ----
  double yaw_in = 0.0;
  if (ab_vert)
  {
    yaw_in = (br > ar) ? M_PI / 2.0 : -M_PI / 2.0;   // 沿 +y 或 -y
  }
  else
  {
    yaw_in = (bc > ac) ? 0.0 : M_PI;                 // 沿 +x 或 -x
  }
  const double dpsi = (info.left_turn ? 1.0 : -1.0) * M_PI / 2.0;

  // ---- 4. 扫掠 ----
  bool hit_body = false;
  bool hit_wheel = false;
  const double gap = sweepGap(info.cx, info.cy, R, yaw_in, dpsi,
                              &hit_body, &hit_wheel);
  info.min_gap_cm = gap;

  // 扫掠包络（用于日志与可视化）
  {
    double x_lo = 1e18;
    double x_hi = -1e18;
    double y_lo = 1e18;
    double y_hi = -1e18;
    const int N = 60;
    for (int i = 0; i <= N; ++i)
    {
      const double yaw = yaw_in + dpsi * i / N;
      const double px = info.cx + R * std::cos(yaw - M_PI / 2.0);
      const double py = info.cy + R * std::sin(yaw - M_PI / 2.0);
      double xs[4];
      double ys[4];
      bodyCorners(px, py, yaw, xs, ys);
      for (int k = 0; k < 4; ++k)
      {
        x_lo = std::min(x_lo, xs[k]);
        x_hi = std::max(x_hi, xs[k]);
        y_lo = std::min(y_lo, ys[k]);
        y_hi = std::max(y_hi, ys[k]);
      }
    }
    info.sweep_x_lo = x_lo;
    info.sweep_x_hi = x_hi;
    info.sweep_y_lo = y_lo;
    info.sweep_y_hi = y_hi;
  }

  // ---- 5. 判定 ----
  if (hit_body)
  {
    info.feasible = false;
    info.reason = "车身扫掠进入阻挡区（最小间隙 " +
                  std::to_string(static_cast<int>(gap)) + " cm）";
    return info;
  }
  if (hit_wheel)
  {
    info.feasible = false;
    info.reason = "车轮轨迹进入阻挡区（最小间隙 " +
                  std::to_string(static_cast<int>(gap)) + " cm）";
    return info;
  }
  if (gap < spec_.turn_min_clearance_cm)
  {
    info.feasible = false;
    info.reason = "转弯净空不足（间隙 " + std::to_string(static_cast<int>(gap)) +
                  " cm < 要求 " +
                  std::to_string(static_cast<int>(spec_.turn_min_clearance_cm)) +
                  " cm）";
    return info;
  }

  info.feasible = true;
  info.tight = (gap < 3.0);
  info.reason = "可转弯（最小间隙 " + std::to_string(static_cast<int>(gap)) +
                " cm" + (info.tight ? "，余量偏紧" : "") + "）";
  return info;
}

bool FieldGrid::suggestAlternativeTurn(int ac, int ar, int bc, int br,
                                       int cc, int cr,
                                       int &nc, int &nr) const
{
  // 在 b 周围尝试把转折点平移一格，保持 a->b->c 的形状
  const int cand[4][2] = {{bc + 1, br}, {bc - 1, br}, {bc, br + 1}, {bc, br - 1}};
  for (int k = 0; k < 4; ++k)
  {
    const int nbc = cand[k][0];
    const int nbr = cand[k][1];
    if (!inBounds(nbc, nbr) || !cellPassable(nbc, nbr))
    {
      continue;
    }
    // 新的 a'、c'：保持 a 相对 b 的方向与 c 相对 b 的方向
    const int dac = ac - bc;
    const int dar = ar - br;
    const int dcc = cc - bc;
    const int dcr = cr - br;
    const int nac = nbc + dac;
    const int nar = nbr + dar;
    const int ncc = nbc + dcc;
    const int ncr = nbr + dcr;
    if (!inBounds(nac, nar) || !inBounds(ncc, ncr))
    {
      continue;
    }
    const TurnInfo t = checkTurn(nac, nar, nbc, nbr, ncc, ncr);
    if (t.valid && t.feasible)
    {
      nc = nbc;
      nr = nbr;
      return true;
    }
  }
  return false;
}

bool FieldGrid::pathTurnsFeasible(const std::vector<std::pair<int, int>> &path,
                                  std::vector<std::string> &bad) const
{
  bad.clear();
  if (path.size() < 3)
  {
    return true;
  }
  for (size_t i = 2; i < path.size(); ++i)
  {
    const int ac = path[i - 2].first;
    const int ar = path[i - 2].second;
    const int bc = path[i - 1].first;
    const int br = path[i - 1].second;
    const int cc = path[i].first;
    const int cr = path[i].second;

    // 只检查 90° 转角
    const bool ab_vert = (ac == bc);
    const bool bc_vert = (bc == cc);
    if (ab_vert == bc_vert)
    {
      continue;   // 直行
    }

    const TurnInfo t = checkTurn(ac, ar, bc, br, cc, cr);
    if (!t.feasible)
    {
      int nc = 0;
      int nr = 0;
      std::string msg = "在 " + cellName(bc, br) + " 转弯不可行：" + t.reason;
      if (suggestAlternativeTurn(ac, ar, bc, br, cc, cr, nc, nr))
      {
        msg += "；建议改在 " + cellName(nc, nr) + " 转弯";
      }
      else
      {
        msg += "；无单点替代，需多段转向或在更开阔处转弯";
      }
      bad.push_back(msg);
      if (bad.size() >= 8)
      {
        break;
      }
    }
  }
  return bad.empty();
}

// -----------------------------------------------------------------------------
// 自检
// -----------------------------------------------------------------------------
std::vector<std::string> FieldGrid::selfCheck() const
{
  std::vector<std::string> problems;

  if (spec_.cols <= 0 || spec_.rows <= 0)
  {
    problems.push_back("cols/rows 非法");
    return problems;
  }
  if (spec_.barrier_up_row < 0 || spec_.barrier_up_row + 1 >= spec_.rows)
  {
    problems.push_back("barrier_up_row 越界（隔离线必须在两行之间）");
  }
  if (spec_.barrier_col_lo > spec_.barrier_col_hi)
  {
    problems.push_back("barrier_col_lo > barrier_col_hi");
  }
  // K 列是桥洞排，必须落在水平隔离线的覆盖范围内，否则门无法连通东西
  if (spec_.bridge_col < spec_.barrier_col_lo ||
      spec_.bridge_col > spec_.barrier_col_hi)
  {
    problems.push_back("bridge_col 不在水平隔离线的覆盖列内 —— 桥洞无法连通东西");
  }
  if (spec_.bridge_a_hi >= spec_.bridge_b_lo)
  {
    problems.push_back("两个桥洞区段重叠");
  }
  if (spec_.bridge_a_lo < 0 || spec_.bridge_b_hi >= spec_.rows)
  {
    problems.push_back("桥洞区段越界");
  }
  if (!problems.empty())
  {
    return problems;
  }

  // 1) 初始状态：K 列必须整列不可通行（未指定通道 = 隔板）
  {
    FieldGrid probe = *this;
    probe.closeAllCrossings();
    for (int r = 0; r < spec_.rows; ++r)
    {
      if (probe.cellPassable(spec_.bridge_col, r))
      {
        problems.push_back(cellName(spec_.bridge_col, r) +
                           " 在未指定通道时就可通行 —— 桥洞排应整列封闭");
      }
    }
  }

  // 2) 水平隔离线覆盖的列，其纵向边在门关闭时必须全部封死
  {
    FieldGrid probe = *this;
    probe.closeAllCrossings();
    for (int c = spec_.barrier_col_lo; c <= spec_.barrier_col_hi; ++c)
    {
      if (probe.canStep(c, spec_.barrier_up_row, c, spec_.barrier_up_row + 1))
      {
        problems.push_back(cellName(c, spec_.barrier_up_row) +
                           " 向下的边未被封死（隔离线有漏洞）");
      }
    }
  }

  // 3) 隔离线未覆盖的列必须仍可跨（A/B/C 与 N）
  {
    FieldGrid probe = *this;
    probe.closeAllCrossings();
    for (int c = 0; c < spec_.cols; ++c)
    {
      if (onBarrierLine(c))
      {
        continue;
      }
      if (!probe.canStep(c, spec_.barrier_up_row, c, spec_.barrier_up_row + 1))
      {
        problems.push_back(cellName(c, spec_.barrier_up_row) +
                           " 的边被误封（该列无隔离线）");
      }
    }
  }

  // 4) 通道的候选数（说明书：段一 6 个、段二 5 个）
  {
    const int n_a = spec_.bridge_a_hi - spec_.bridge_a_lo + 1;
    const int n_b = spec_.bridge_b_hi - spec_.bridge_b_lo + 1;
    if (n_a != 6)
    {
      problems.push_back("桥洞段一候选通道数应为 6（K1–K6），实际 " +
                         std::to_string(n_a));
    }
    if (n_b != 5)
    {
      problems.push_back("桥洞段二候选通道数应为 5（K7–K11），实际 " +
                         std::to_string(n_b));
    }
  }

  // 4b) ★ v5 拓扑的核心断言：两个门必须**同时**存在才可达
  //     起终点 M11(南,东条带) / N6(北,东条带) 分居水平隔离线两侧，
  //     而隔离线的唯一缺口在 K 列，K 列本身又是只有门格可通行的隔板。
  //     于是必须"南段门进西侧 + 从 A/B/C 列跨线 + 北段门回东条带"。
  //     只给一个门时**完全不可达** —— 这是必须固化的结论。
  {
    // 先确认起终点确实分居线两侧（若不符，说明起终点参数被改错了）
    const int s_side = (spec_.start_row <= spec_.barrier_up_row) ? 0 : 1;
    const int g_side = (spec_.goal_row <= spec_.barrier_up_row) ? 0 : 1;
    if (s_side != g_side)
    {
      // 起终点分居两侧 -> 检查"两门齐备才可达、单门不可达"
      FieldGrid probe = *this;
      probe.closeAllCrossings();

      // 单开南段任意一个门：不可达
      bool single_south_ok = false;
      for (int r = spec_.bridge_b_lo; r <= spec_.bridge_b_hi; ++r)
      {
        FieldGrid t = probe;
        t.openCrossing(r);
        if (t.pathLength(spec_.start_col, spec_.start_row,
                         spec_.goal_col, spec_.goal_row) >= 0)
        {
          single_south_ok = true;
          break;
        }
      }
      if (single_south_ok)
      {
        problems.push_back(
            "只开南段一个门就能从起点到终点 —— 与已确认的 v5 拓扑不符"
            "（应为必须两门齐备）。请复核水平隔离线的列范围是否确实到 N");
      }

      // 单开北段任意一个门：不可达
      bool single_north_ok = false;
      for (int r = spec_.bridge_a_lo; r <= spec_.bridge_a_hi; ++r)
      {
        FieldGrid t = probe;
        t.openCrossing(r);
        if (t.pathLength(spec_.start_col, spec_.start_row,
                         spec_.goal_col, spec_.goal_row) >= 0)
        {
          single_north_ok = true;
          break;
        }
      }
      if (single_north_ok)
      {
        problems.push_back(
            "只开北段一个门就能从起点到终点 —— 与已确认的 v5 拓扑不符");
      }

      // 南段一个 + 北段一个：必须可达
      {
        FieldGrid t = probe;
        t.openCrossing(spec_.bridge_b_lo);   // 南段最北的门，路径最短
        t.openCrossing(spec_.bridge_a_hi);   // 北段最南的门
        if (t.pathLength(spec_.start_col, spec_.start_row,
                         spec_.goal_col, spec_.goal_row) < 0)
        {
          problems.push_back(
              "两段各开一个门后仍不可达 —— 拓扑自相矛盾，请复核起终点与线列范围");
        }
      }

      // 全部 30 种门位组合都必须可达
      int combos = 0;
      int reachable = 0;
      for (int ra = spec_.bridge_a_lo; ra <= spec_.bridge_a_hi; ++ra)
      {
        for (int rb = spec_.bridge_b_lo; rb <= spec_.bridge_b_hi; ++rb)
        {
          ++combos;
          FieldGrid t = probe;
          t.openCrossing(ra);
          t.openCrossing(rb);
          if (t.pathLength(spec_.start_col, spec_.start_row,
                           spec_.goal_col, spec_.goal_row) >= 0)
          {
            ++reachable;
          }
        }
      }
      if (reachable != combos)
      {
        problems.push_back(
            "有门位组合不可达（" + std::to_string(reachable) + "/" +
            std::to_string(combos) + "）—— 每局的门位随机，必须 100% 可达");
      }
    }
  }

  // 5) 打开一个通道后，必须能实现一次"东西穿越"（东侧格 → 门格 → 西侧格）
  {
    for (int row : {spec_.bridge_a_lo, spec_.bridge_b_lo})
    {
      FieldGrid probe = *this;
      probe.closeAllCrossings();
      probe.set(spec_.start_col, spec_.start_row, kFree);
      probe.set(spec_.goal_col, spec_.goal_row, kFree);
      if (!probe.openCrossing(row))
      {
        problems.push_back("openCrossing(" + cellName(spec_.bridge_col, row) +
                           ") 失败");
        continue;
      }
      const int east = spec_.bridge_col + 1;
      const int west = spec_.bridge_col - 1;
      std::vector<std::pair<int, int>> p;
      if (!probe.findPath(east, row, west, row, p))
      {
        problems.push_back("打开 " + cellName(spec_.bridge_col, row) +
                           " 后仍无法从东侧穿到西侧 —— 门不连通东西");
      }
      else if (p.size() != 3)
      {
        problems.push_back("穿桥洞路径应为 3 格（东格→门格→西格），实际 " +
                           std::to_string(p.size()));
      }
    }
  }

  // 6) 起终点保底通路（不依赖任何无人机信息）
  {
    FieldGrid probe = *this;
    probe.closeAllCrossings();
    probe.set(spec_.start_col, spec_.start_row, kFree);
    probe.set(spec_.goal_col, spec_.goal_row, kFree);
    std::vector<std::pair<int, int>> baseline;
    if (!probe.baselineRoute(baseline))
    {
      problems.push_back("起终点在通道全关时不可达 —— 保底 10 分会丢");
    }
    else if (baseline.size() != 4)
    {
      problems.push_back("起终点保底路径应为 4 格（M7→N7→N6→N5），实际 " +
                         std::to_string(baseline.size()));
    }
    // 起终点都在 K 列以东，所以保底通路不应经过桥洞列
    for (const auto &q : baseline)
    {
      if (q.first == spec_.bridge_col)
      {
        problems.push_back("起终点保底通路经过了桥洞列 —— 与"
                           "『起终点无需穿桥洞』矛盾");
        break;
      }
    }
  }

  // 7) 起终点必须都在 K 列以东（这是 M7 / N5 的核心特征）
  {
    if (!isEastStrip(spec_.start_col) || !isEastStrip(spec_.goal_col))
    {
      problems.push_back("起点或终点不在 K 列以东 —— 与已确认的 M7 / N5 不符");
    }
  }

  // 8) 起终点分居水平隔离线两侧
  {
    const int s_side = (spec_.start_row <= spec_.barrier_up_row) ? 0 : 1;
    const int g_side = (spec_.goal_row <= spec_.barrier_up_row) ? 0 : 1;
    if (s_side == g_side)
    {
      problems.push_back("起终点在同一侧 —— 与已确认的 M7(南) / N5(北) 不符");
    }
  }

  // 9) 净空约束（若启用）
  if (spec_.min_clearance_cm > 0.0)
  {
    if (!clearanceOk(spec_.start_col, spec_.start_row) ||
        !clearanceOk(spec_.goal_col, spec_.goal_row))
    {
      problems.push_back("起终点不满足 min_clearance_cm");
    }
  }

  return problems;
}

// -----------------------------------------------------------------------------
// 可视化
// -----------------------------------------------------------------------------
std::string FieldGrid::dump() const
{
  std::string out;
  out += "     ";
  for (int c = 0; c < spec_.cols; ++c)
  {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%-3s", colName(c).c_str());
    out += buf;
  }
  out += "\n";

  for (int r = 0; r < spec_.rows; ++r)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%-4d ", r + 1);
    out += buf;
    for (int c = 0; c < spec_.cols; ++c)
    {
      char ch = ' ';
      if (c == spec_.bridge_col)
      {
        if (isOpenGate(c, r))
        {
          ch = 'G';                       // 已指定的可通行通道
        }
        else if (isBridgeInterval(r))
        {
          ch = '|';                       // 通道（未指定 = 隔板）
        }
        else
        {
          ch = '#';                       // 非通道
        }
      }
      else
      {
        switch (cells_[static_cast<size_t>(idx(c, r))])
        {
          case kBlocked: ch = '#'; break;
          case kMine: ch = 'm'; break;
          case kTarget: ch = 'T'; break;
          case kUnknown: ch = '.'; break;
          default: ch = ' '; break;
        }
      }
      if (c == spec_.start_col && r == spec_.start_row)
      {
        ch = 'S';
      }
      else if (c == spec_.goal_col && r == spec_.goal_row)
      {
        ch = 'E';
      }
      char cell[8];
      std::snprintf(cell, sizeof(cell), "%-3c", ch);
      out += cell;
    }
    out += "\n";

    if (r == spec_.barrier_up_row)
    {
      out += "     ";
      for (int c = 0; c < spec_.cols; ++c)
      {
        out += onBarrierLine(c) ? "---" : "   ";
      }
      out += "  <- 水平隔离线 D6–M6 / D7–M7\n";
    }
  }
  return out;
}

}  // namespace dzmission
