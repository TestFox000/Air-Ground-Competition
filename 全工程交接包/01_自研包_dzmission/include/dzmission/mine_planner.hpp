#ifndef DZMISSION_MINE_PLANNER_HPP
#define DZMISSION_MINE_PLANNER_HPP

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "dzmission/field_grid.hpp"

namespace dzmission
{

// =============================================================================
// MinePlanner —— 排雷覆盖规划
// -----------------------------------------------------------------------------
// 规则依据（《场地布置说明书 V1.2》+《规则》）：
//   · 地雷区域：**随机 8 个方格**，贴边长 **12cm 彩色**地雷图，
//     图片**置于格子中央**（说明书明确"粘在地雷所在区域"）
//   · 排雷判定：「当小车**任意一轮中心进入地雷标识圆形区域内部**，
//     即判定成功排雷」
//   · 共 8 颗，每颗 5 分 → **40 分，是车端单项最高分**
//
// 关键推论（决定算法形态）：
//   判据是"**轮**中心进入 12cm 圆"，而不是"车中心到达格心"。
//   圆心在格心（17.5, 17.5）cm 处，半径 6cm。
//   轮子位于车体横向 ±5.5cm（轮距 110mm），纵向 0（后轴）与 +15cm（前轴）。
//   → 车必须让**某个轮子**从半径 6cm 的圆内穿过。
//   → 最稳的做法：让后轴中心沿一条**穿过格心**的直线行驶，
//     此时有一个轮心距格心的横向偏差 = 5.5cm < 6cm → **判定成立**。
//     但余量只有 0.5cm，所以横向精度要求高；若让车中心线穿格心，
//     则两轮心横向各偏 5.5cm，都在圆内（另一种可行摆法）。
//
// 因此本规划器输出的是**"通行段"序列**（而非单点序列）：
//   每个地雷对应一段直线（pass），要求车辆沿该段行驶，
//   从而保证轮心扫过 12cm 圆。
//
// 算法：
//   1. 地雷格 → 每颗地雷生成一个 Pass{方向 d, 入口 in, 出口 out}
//      优先选**可通行**且**转向约束宽松**的方向（用 FieldGrid::checkTurn 评估）
//   2. 距离矩阵：任意两个 Pass 的入口/出口之间的 A* 最短路径长度
//   3. 顺序优化：最近邻构造 + 2-opt 改进（8 个点，规模很小）
//   4. 拼接：按顺序把 Pass 串成一条完整路径
//   5. 全程校验：逐段 A* + 逐转角 pathTurnsFeasible
// =============================================================================
class MinePlanner
{
public:
  // 一颗地雷上的通行段
  struct Pass
  {
    int col{0};
    int row{0};
    uint8_t dir{0};             // 指向 kDir* 常量
    // 该段的入口/出口格（dir 决定）
    int in_col{0}, in_row{0};
    int out_col{0}, out_row{0};
    // 沿该方向通过时的最小转弯间隙（越大越宽松）
    double turn_gap_cm{0.0};
    bool feasible{true};
    std::string note;           // 选向理由 / 不可行原因
  };

  enum Dir : uint8_t
  {
    kDirNorth = 0,   // 从南向北走（-row 方向）
    kDirSouth = 1,
    kDirEast = 2,
    kDirWest = 3,
  };

  struct PlanStats
  {
    int mines_total{0};
    int mines_covered{0};
    int waypoints{0};
    int segments{0};
    double total_cells{0.0};        // 总行程（格数）
    double est_time_s{0.0};         // 以 cruise_speed 估算
    int infeasible_turns{0};
    int tight_turns{0};             // 可行但余量 < 3cm 的转角数
    // ★ 非法步骤数：拼接多段路径时若接缝两侧并不相邻，就是静默错误。
    //   正常必须为 0；>0 说明路径拼接逻辑有缺陷。
    int illegal_steps{0};
    std::vector<std::string> warnings;
  };

  struct Plan
  {
    bool ok{false};
    std::string reason;
    std::vector<std::pair<int, int>> path;      // 完整格路径（含起终点）
    std::vector<Pass> passes;                   // 按访问顺序
    std::vector<int> order;                     // passes 的原始下标顺序
    PlanStats stats;
  };

  struct Config
  {
    // 巡航速度（m/s），用于估算时间；官方最高 1.0
    double cruise_speed{0.5};
    // 排雷通行段的"外延"格数：每个 pass 从地雷格往回退一格进入，
    // 再往前伸一格退出，保证整段直线穿过格心
    int approach_cells{1};
    // 是否要求每个转角都通过 FieldGrid::checkTurn
    bool enforce_turns{true};
    // 2-opt 迭代上限
    int two_opt_rounds{64};
  };

  MinePlanner() = default;

  void setConfig(const Config &c) { cfg_ = c; }
  const Config &config() const { return cfg_; }

  // 主入口：从 start 出发，依次排掉 mines 中的地雷，最后到 goal。
  //   grid 需已完成地形更新（地雷格用 markMine 标记、障碍用 markBlocked、
  //   桥洞门用 openCrossing 打开）。
  //   mines 为地雷格列表（一般直接来自 AIR_GROUND_PROTOCOL 的 MINE_LIST）。
  Plan plan(const FieldGrid &grid,
            const std::vector<std::pair<int, int>> &mines,
            const std::pair<int, int> &start,
            const std::pair<int, int> &goal) const;

  // 单颗地雷选向：返回最优方向与理由
  static Pass choosePass(const FieldGrid &grid, int col, int row,
                         const Config &cfg);

  // 最近邻 + 2-opt 排序。返回顺序下标。
  static std::vector<int> orderPasses(
      const std::vector<std::vector<int>> &dist, int start_from,
      int rounds);

  // 方向工具
  static const char *dirName(uint8_t d);
  static void dirDelta(uint8_t d, int &dc, int &dr);

private:
  // 生成某地雷在指定方向上的 Pass
  static bool makePass(const FieldGrid &grid, int col, int row, uint8_t d,
                       int approach_cells, Pass &out);

  Config cfg_;
};

}  // namespace dzmission

#endif  // DZMISSION_MINE_PLANNER_HPP
