#ifndef DZMISSION_FIELD_GRID_HPP
#define DZMISSION_FIELD_GRID_HPP

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace dzmission
{

// =============================================================================
// FieldGrid —— 比赛场地栅格模型（边阻挡 + 桥洞排）
// -----------------------------------------------------------------------------
// 规格来源：《场地布置说明书 V1.2》（已从 PDF 提取，见 _extract/field.txt）
//
//   场地      : 4 m × 5 m，栅格 35 cm
//   字母轴 A..N: 14 格   → x，14 × 0.35 = 4.90 m
//   数字轴 1..11: 11 格  → y，11 × 0.35 = 3.85 m
//   ★ 场地没有四周围挡（说明书原文）—— 所以"不碰边界"指的是
//     不碰下面两条隔离结构，以及不驶出 4m×5m 范围。
//
// 坐标系：x 沿 A→N，y 沿 1→11；格 (col,row) 中心 = ((col+.5)*.35, (row+.5)*.35)
//
// ★ 两级通行性（说明书强调"格子本身可通行，隔离在格与格之间"）：
//     1) 格内通行性  cells_[r][c]
//     2) 边通行性    blocked_up_ / blocked_right_
//
// 两条"隔离"（都必须建模，缺一会算错路径 —— 见 topology_v4_check.py）：
//
//   A) 水平隔离线 D6–M6 / D7–M7 之间
//      构成：黄色挡板 + 可自粘白色长条（说明书："用于阻挡小车穿行"）
//      位置：row(6) 与 row(7) 之间的**纵向边**，覆盖列 D..M
//      语义：格子可通行，但这条边不可跨越
//
//   B) K 列桥洞排（K1…K11 是一排通道，通道间由隔板隔离）
//      每局随机指定 2 个可通行通道：
//        区段一 K1–K6   （6 个候选）
//        区段二 K7–K11  （5 个候选）
//      说明书的示例门位：K4（段一）与 K9（段二）
//      语义：K 列**只有门格可通行**，其余格不可通行
//            → K 列是唯一的东西向通路
//
// 起终点（已确认）：
//   小车起点 M7 = (12, 6)，格心 (437.5, 227.5) cm
//   小车终点 N5 = (13, 4)，格心 (472.5, 157.5) cm
//   二者 col 均 > bridge_col，都在 K 列**以东**的窄条区，
//   所以起终点任务只需跨水平隔离线，**不需要穿桥洞**：
//     M7 → N7 → N6 → N5   （4 格，断链也能独立完成）
//   而西半场（地雷 / 障碍所在）**必须穿桥洞**才能到达。
//
// 几何约束（控制层必须知道的数字）：
//   最紧净空 17.5 cm（M7 距水平挡板、N5 距场地边缘）
//   车宽 250mm 时单侧余量仅 5.0 cm，横向允许偏移带 ±5.0 cm
//   轴距 15cm 时最小转弯半径 R = L/tan(δ) ≥ 21.4cm（δ=35°）
//     > 半格 17.5cm → 路径中不允许单格内直角转向，必须多段转向
// =============================================================================
class FieldGrid
{
public:
  // ---- 格内语义 ----
  enum Cell : uint8_t
  {
    kUnknown = 0,   // 尚未观测
    kFree = 1,      // 可通行
    kBlocked = 2,   // 不可驶入（如水桶障碍占格）
    kMine = 3,      // 地雷图（可通行，但需要排雷）
    kTarget = 4,    // 电子靶标所在
  };

  // ---- 边方向 ----
  enum Edge : uint8_t
  {
    kEdgeUp = 0,     // (c,r) -> (c,r+1)
    kEdgeRight = 1,  // (c,r) -> (c+1,r)
  };

  struct Spec
  {
    int cols{14};
    int rows{11};
    double cell_size{0.35};

    // ---- 水平隔离线（★ 已现场确认）----
    // 位于 barrier_up_row 与 barrier_up_row+1 之间。现场图上的黄线在
    // **row6 与 row7 之间**（即 D6–N6 / D7–N7 的公共边），
    // 所以 barrier_up_row = 5（"6" 的 0-based 下标）。
    // **覆盖列 D..N（col 3..13），全宽** —— 依据：现场地图上那条黄线
    // 一直画到最右列 N。
    //
    // ★ 这一条的后果很严重：起点 M11 与终点 N6 分居线两侧，
    //   而 M 列与 N 列都在 D..N 覆盖范围内，所以**东侧条带被完全切断**。
    //   M11 无法直接北上到 N6，**必须穿桥洞绕行**。
    //   => "起终点 10 分不依赖无人机"的保底结论**不成立**，
    //      桥洞门位成为起终点得分的硬依赖（详见 test/topology_v5_check.py）。
    int barrier_up_row{5};      // "6" 的 0-based 下标：线在 row6 与 row7 之间
    int barrier_col_lo{3};      // D
    int barrier_col_hi{13};     // N（全宽）

    // ---- 桥洞列（竖直黄线 = 桥洞通道那一排，整列 K）----
    int bridge_col{10};         // K

    // 桥洞区间（0-based row，闭区间）
    // 说明书：区段一 K1–K6（6 个候选）、区段二 K7–K11（5 个候选）
    int bridge_a_lo{0};         // K1
    int bridge_a_hi{5};         // K6
    int bridge_b_lo{6};         // K7
    int bridge_b_hi{10};        // K11

    // ---- 起终点（★ 已现场确认，以规则与现场图为准）----
    int start_col{12};          // M
    int start_row{10};          // 11
    int goal_col{13};           // N
    int goal_row{5};            // 6
    // 起点朝向：M11 车头朝 M10 方向 = 行号减小 = -y
    //   若需要按朝向初始化定位，用本字段（度，0=+x，-90=-y）
    double start_heading_deg{-90.0};

    // ---- ★ 无人机起降点（现场图固有标注，v2.1 起随 FIELD_FIXED 下发）----
    //   无人机**起飞前与降落后**停在场上这两个格。
    //   地面车必须把它们视为**禁入格**：撞机等价于"撞固定场地"，
    //   而规则 R 明确"持续超 10 s 裁判切手动（云台仍自动）"。
    //     无人机起点 M9 = (col 12, row 8)
    //     无人机终点 N2 = (col 13, row 1)
    //   ★ 可通过 setDroneCells() 由 FIELD_FIXED 覆盖；
    //     (-1,-1) 表示"未知/不启用"。
    int drone_start_col{12};    // M
    int drone_start_row{8};     // 9
    int drone_end_col{13};      // N
    int drone_end_row{1};       // 2
    bool drone_cells_valid{true};   // false = 不把这两格当禁入（兼容旧配置）

    // ---- 车体参数（官方实测，用于转弯与净空判定）----
    // 《SmartMaster 硬件操作指南 V1.0》表 1-1
    double car_width_cm{14.0};    // 宽度 140 mm（检录上限 250）
    double car_length_cm{27.8};   // 长度 278 mm（检录上限 300）
    // 轴距 150 mm —— 取官方手册与 dzactuator.h 的 CARL 值。
    //   ★ 提醒：学员工程用的是 0.17 m（差 13%）。若现场实测轴距确实是 170mm，
    //     则最小转弯半径 = 170/tan(30°) = 29.4 cm（而非 26.0 cm），
    //     扫掠半径会从 44.6 cm 增到 48.0 cm，原地转向仍不受影响（只与车体对角
    //     半径有关），但单圆弧判定会更严。改这里时 min_turn_radius_cm 要同步改。
    double wheelbase_cm{15.0};    // 轴距
    double track_cm{11.0};        // 轮距 110 mm
    // 最小转弯半径（后轴中心到转弯中心），官方实测 24~26 cm
    double min_turn_radius_cm{26.0};
    // 后轴中心到车体前/后端的距离（官方未给，按长度比例保守分配）
    double front_overhang_cm{17.2};  // 0.62 × 长度
    double rear_overhang_cm{10.6};   // 0.38 × 长度
    // 轮胎半径（用于轮胎轨迹检查），官方未给，按 1/2 离地+结构估
    double wheel_radius_cm{3.0};
    // ★ 横向稳态偏差补偿（度）。学员代码实测：底盘有"恒定的机械左偏"，
    //   需要用一个小角度右偏抵消。dzmission 的转向指令在发出前统一叠加该值。
    //   负 = 向右偏。调整：左漂更多 -> -1.5；右漂 -> -0.5。
    double steer_trim_deg{-1.0};
    // 原地转向判据的安全余量（cm）：扫掠圆到真实阻挡至少要留这么多
    double pivot_min_gap_cm{0.0};
    // 要求的最小净空（格心到最近挡板/场边）。0 = 不检查
    double min_clearance_cm{0.0};
    // 转弯可行性判据
    //   转向时车体与阻挡的最小允许间隙（cm）。
    //
    //   ★ 取值很关键，容易设错：
    //     在**格内转弯**（转弯中心落在格角上）时，车体后角到最近格边界的
    //     几何余量**天然只有 0.1~3.5 cm** —— 这是正常几何，不是"擦边"。
    //     而沿**宽走廊转弯**时余量可以到 50~85 cm。
    //     所以：
    //       · 取 0    → 只要不穿插就通过（正确的"能否转"判据）
    //       · 取 7    → 等价于"必须在宽走廊转弯"，格内转弯全部被拒
    //                   （对排雷这类格内机动过于苛刻）
    //       · 取 0.5  → 在不穿插之上留一点余量，推荐默认
    //     planner 应读取 TurnInfo::min_gap_cm 自行权衡，而不是只信 feasible。
    double turn_min_clearance_cm{0.5};
    //   轮胎轨迹必须落在可通行格内？（比车身扫掠更严，推荐 true）
    bool turn_check_wheel_track{true};
    //   车体扫掠必须落在可通行格内？（车身可能越过格心线但不压障碍）
    bool turn_check_body_sweep{true};
  };

  FieldGrid();
  explicit FieldGrid(const Spec &s);

  void setSpec(const Spec &s);
  const Spec &spec() const { return spec_; }
  void reset();   // 清空为"全可通行 + 固定线"，门全部关闭

  // ---- 下标 <-> 名称 ----
  static std::string colName(int col);
  static int colIndex(char letter);
  static std::string cellName(int col, int row);
  static bool parseCellName(const std::string &name, int &col, int &row);

  // ---- 边界 / 格内访问 ----
  bool inBounds(int col, int row) const;
  uint8_t at(int col, int row) const;
  void set(int col, int row, uint8_t v);
  bool cellPassable(int col, int row) const;

  // ---- 坐标变换 ----
  void cellToWorld(int col, int row, double &x, double &y) const;
  bool worldToCell(double x, double y, int &col, int &row) const;

  // ---- 固定结构查询 ----
  bool onBarrierLine(int col) const;          // 该列是否被线覆盖
  bool isBarrierEdge(int col, int row) const; // (col,row)->(col,row+1) 是否是线上的边
  bool isBridgeInterval(int row) const;
  bool isBridgeColumn(int col) const { return col == spec_.bridge_col; }

  // ---- 边通行性（核心） ----
  // 两个相邻格之间能否跨越。非相邻或越界返回 false。
  bool canStep(int c0, int r0, int c1, int r1) const;
  bool edgeOpen(int col, int row, Edge e) const;
  void setEdgeOpen(int col, int row, Edge e, bool open);

  // ---- 桥洞门 ----
  // 打开某个桥洞通道（无人机识别顶标后下发）。
  // K 列只有被打开的格可通行，其余为隔板 / 未指定，不可通行。
  // row 必须落在 bridge_a 或 bridge_b 区间内，否则返回 false。
  bool openCrossing(int row);
  bool closeCrossing(int row);
  void closeAllCrossings();
  int openCrossingCount() const;
  int crossingRowA() const { return gate_a_row_; }
  int crossingRowB() const { return gate_b_row_; }
  // 该格是否是"已打开的通道"
  bool isOpenGate(int col, int row) const;

  // 把 spec_ 里的无人机起降点落成 kBlocked（reset() 与 setDroneCells() 都调）
  void applyDroneCellsAsBlocked();

  // ---- 地形更新 ----
  void markBlocked(int col, int row);   // 障碍桶占格
  void markFree(int col, int row);
  void markMine(int col, int row);
  void markTarget(int col, int row);
  // ★ 无人机侦察数据的应用（校验已在 MissionManager 做过，这里只落盘并
  //   做最后一道防御：不改变 K 列隔板状态 —— 门位只能由 openCrossing 改）
  bool markMineByRecon(int col, int row);
  bool markBlockedByRecon(int col, int row);
  bool markTargetByRecon(int col, int row);

  // ---- ★ 无人机起降点（禁入格）----
  // 无人机起飞前/降落后停在场上这两格，地面车经过会撞机。
  //   setDroneCells()：由 FIELD_FIXED（v2.1）覆盖实际坐标；
  //                    拒绝越界或与起终点重合（会让本局不可达）。
  //   isDroneCell()  ：供规划/日志判定"这格是不是无人机位"。
  //   applyDroneCellsAsBlocked()：把两格落成 kBlocked（reset 内也会调）。
  bool setDroneCells(int start_col, int start_row, int end_col, int end_row);
  bool isDroneCell(int col, int row) const;

  // 小端 16 位读取（公开版，供上层解析 MAP_FULL 载荷）
  static uint16_t rdU16Pub(const uint8_t *p)
  {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
  }

  // ---- 连通性 ----
  // 四连通 BFS（考虑边阻挡与 K 列隔板）。返回是否连通，path 为最短路径
  bool findPath(int c0, int r0, int c1, int r1,
                std::vector<std::pair<int, int>> &path) const;

  // ★ 转弯可行 + 方向感知的最短路径
  //   状态 = (格, 进入该格的方向)。把方向放进状态空间，才能把
  //   「在该格拐弯是否可行」作为状态转移约束 —— 只在事后检查转角，
  //   会得到"格级最短但实车走不了"的路径。
  struct PathOptions
  {
    bool turn_aware{true};       // 拐弯必须通过 checkTurn
    bool forbid_reverse{true};   // 禁止立即 180° 折返
    int heading_dc{0};           // 起始朝向；两个都为 0 表示不约束
    int heading_dr{0};
    // ★ pivot_ok：单圆弧不可行时，若该格**原地转向可行**则放行。
    //   依据 v5 拓扑：窄条（K 以东 L/M/N）内无法单圆弧 90° 转弯，
    //   但 PivotController 的原地转向可行（见 test/pivot_turn_check.py）。
    //   设 true 后路径仍保持"实车可执行"的语义，比直接关掉 turn_aware 精确得多。
    bool pivot_ok{false};
  };

  bool findPathAware(int c0, int r0, int c1, int r1,
                     std::vector<std::pair<int, int>> &path,
                     const PathOptions &opt) const;

  // 兼容旧签名（等价于 turn_aware + 可选 heading + 禁折返）
  bool findPathAware(int c0, int r0, int c1, int r1,
                     std::vector<std::pair<int, int>> &path,
                     bool turn_aware,
                     int heading_dc = 0, int heading_dr = 0,
                     bool forbid_reverse = true) const;

  // ★ 分层回退搜索（同步自 Python 对照实现）
  //   逐级放宽约束，返回**第一级可行**的路径与所用级别。
  //   为什么要逐级而不是一步放开：
  //     turn_aware 会把"拐弯不可行"的状态转移从状态空间剪掉，
  //     某些"起点朝向 + 目标"组合下整张图会被剪成不连通（返回 false），
  //     但物理上其实有路 —— 只是更长（绕开不可行转角）或需要原地转向。
  //     一步放开会导致覆盖率上升但路径含**车执行不了**的转角（假成功）。
  //   级别：
  //     1 turn_aware + heading + 禁折返
  //     2 turn_aware，放开 heading
  //     3 turn_aware + pivot_ok（允许原地转向的转角）
  //     4 放开 turn_aware，保留禁折返
  //     5 全部放开
  bool findPathStaged(int c0, int r0, int c1, int r1,
                      std::vector<std::pair<int, int>> &path,
                      int heading_dc = 0, int heading_dr = 0,
                      int *used_level = nullptr) const;

  // 路径长度（格数），不连通返回 -1
  int pathLength(int c0, int r0, int c1, int r1) const;

  // 从 (c,r) 出发能到达的格数
  int reachableCount(int c, int r) const;

  // ---- 净空（几何约束） ----
  // 格心到最近"水平隔离线挡板 / 场地边缘"的距离（cm）
  double clearanceCm(int col, int row) const;
  bool clearanceOk(int col, int row) const;
  // 起点 → 终点的固定保底路径（不依赖任何无人机信息）
  bool baselineRoute(std::vector<std::pair<int, int>> &path) const;
  // 给定最小转弯半径，判断某路径是否含"单格直角转向"（违反转弯约束）
  bool pathHasSharpCorner(const std::vector<std::pair<int, int>> &path) const;
  // 由前轮转角反推最小转弯半径（cm）
  double minTurnRadiusCm(double max_steer_deg) const;
  // 该格是否位于 K 列以东（起终点所在的窄条区）
  bool isEastStrip(int col) const { return col > spec_.bridge_col; }
  // 该格是否位于 K 列以西（地雷/障碍所在的西半场）
  bool isWestField(int col) const { return col < spec_.bridge_col; }

  // ===========================================================================
  // 转弯可行性（★ 核心约束）
  // ---------------------------------------------------------------------------
  // 原理：走标准 90° 弯时，转弯中心 C 被几何锁定在**两条车道中心线的交点**上
  //   —— 因为终点时后轴中心必须落在目标车道中心线上，而它距 C 恰好为 R
  //   且方向垂直于目标车道。所以 C 不能自由选择。
  //
  //   于是「能否在此转弯」= 车体绕 C 扫过 90° 的过程中，
  //   扫掠包络是否始终落在可通行区域内。
  //
  //   本场地的关键事实（由 test/geometry_check.py 核算）：
  //     · 在 row7 中心线上转弯时，车体最低点距水平挡板只剩 3 mm
  //     · N 列中心线距场地边界 175 mm，而车体转弯要扫出 446 mm
  //     ⇒ K 以东（L/M/N 三列）**无法**完成单圆弧 90° 转向
  // ===========================================================================
  struct TurnInfo
  {
    bool valid{false};            // 入参是否构成合法的 90° 转向
    bool feasible{false};         // 转弯是否可行
    int in_col{0}, in_row{0};     // 入向格（a->b->c 中的 b）
    int out_col{0}, out_row{0};   // 出向格（c）
    bool left_turn{true};         // 左转（+90°）还是右转（-90°）
    // 转弯中心（世界系，cm）
    double cx{0.0}, cy{0.0};
    // 扫掠包络（世界系，cm）
    double sweep_x_lo{0.0}, sweep_x_hi{0.0};
    double sweep_y_lo{0.0}, sweep_y_hi{0.0};
    // 最近阻挡的间隙（cm）；>= 0 表示未碰撞
    double min_gap_cm{0.0};
    // 余量是否偏紧（< 3 cm）。planner 应优先选不紧的转角
    bool tight{false};
    // 不可行原因（中文，可直接打日志）
    std::string reason;
  };

  // 在 b 处由方向 (a->b) 转向 (b->c) 是否可行。a/b/c 需为四连通相邻格。
  TurnInfo checkTurn(int ac, int ar, int bc, int br, int cc, int cr) const;

  // 覆盖整个路径：逐段检查每个 90° 转角。不可行的转角追加到 bad（最多 8 条）。
  bool pathTurnsFeasible(const std::vector<std::pair<int, int>> &path,
                         std::vector<std::string> &bad) const;

  // 若在 b 处转弯不可行，给出一个候选替代转折点（尝试上下左右偏移一格）。
  // 返回空表示找不到单点替代，需要多段转向。
  bool suggestAlternativeTurn(int ac, int ar, int bc, int br, int cc, int cr,
                              int &nc, int &nr) const;

  // ===========================================================================
  // ★ 原地转向（pivot turn）可行性 —— 窄条内 90° 转向的唯一可行方案
  // ---------------------------------------------------------------------------
  // 为什么要单独做这个：
  //   单圆弧 90° 转向的扫掠半径 = R + 车体对角半径 ≈ 44.6 cm，
  //   而 K 列以东窄条（L/M/N）的格心距场边只有 17.5 cm，
  //   所以**窄条内单圆弧几何上不可行**（已由 checkTurn 的普查证实）。
  //
  //   原地转向绕后轴中心**纯转动**，没有平动扫掠，扫掠半径 = 车体对角半径
  //   = hypot(前伸, 半宽) ≈ 18.6 cm，只有单圆弧的 1/2.4。
  //   因此窄条内可转，且规划约束极宽松。
  //
  // 判据（与实车对齐）：
  //   扫掠圆必须完全落在**可通行格**内。注意「格与格之间的分界线不是障碍物」
  //   —— 场地没有四周围挡，只有场边、水平挡板、桥洞隔板、水桶是真实阻挡。
  //   所以判据 = 扫掠圆覆盖到的每个格都可通行。
  // ===========================================================================
  struct PivotInfo
  {
    bool in_bounds{false};
    bool feasible{false};         // 能否原地转向
    int col{0}, row{0};
    double cx{0.0}, cy{0.0};      // 后轴中心（世界系 cm）
    double sweep_radius_cm{0.0};  // 车体对角半径
    // 扫掠圆覆盖的格：总数 / 可通行数
    int cells_covered{0};
    int cells_passable{0};
    // 扫掠圆到"真实阻挡"的最小间隙（cm）：
    //   场边 / 水平隔离线挡板 / 轮胎轨迹与不可通行格的重叠
    double min_gap_cm{0.0};
    // 旋转过程中轮胎轨迹是否始终落在可通行格内
    bool wheels_ok{true};
    std::string reason;
  };

  PivotInfo checkPivot(int col, int row) const;

  // 路径是否可以用"逐格原地转向"走通：每个 90° 转角处都必须 checkPivot 可行。
  // 这是窄条路径的正确判据（比 pathTurnsFeasible 宽松得多）。
  bool pathPivotsFeasible(const std::vector<std::pair<int, int>> &path,
                          std::vector<std::string> &bad) const;

  // 车体绕给定中心扫掠时，与所有阻挡的最小间隙（cm）。负值表示穿插。
  //   cx,cy   转弯中心（世界系 cm）
  //   R       转弯半径（后轴中心到中心，cm）
  //   yaw0    起始车头朝向（rad）
  //   dpsi    转过的角度（rad，正=逆时针）
  //   hit_body / hit_wheel 分别返回车身扫掠 / 车轮轨迹是否穿入阻挡
  double sweepGap(double cx, double cy, double R, double yaw0, double dpsi,
                  bool *hit_body, bool *hit_wheel) const;

  // ★ 轮心进圆校验（排雷判据的直接仿真）
  //   规则：「小车任意一轮中心进入地雷标识圆形区域内部即判定成功排雷」。
  //   地雷图直径 12cm、圆心在格心 → mine_radius_cm 默认 6.0。
  //   本函数沿 path 推进，采样四个轮子的中心轨迹，统计有多少颗雷被
  //   "某个轮心"进入圆内。返回值 = 命中颗数。
  //   min_dist_out 可选，输出每颗雷的"最近轮心距圆心"距离（cm）。
  int verifyWheelInCircle(const std::vector<std::pair<int, int>> &path,
                          const std::vector<std::pair<int, int>> &mines,
                          double mine_radius_cm = 6.0,
                          std::vector<double> *min_dist_out = nullptr) const;

  // 转弯中心的 4 个候选（左转/右转 × 车道中心线交点）
  void turnCenters(int ac, int ar, int bc, int br, int cc, int cr,
                   double &left_cx, double &left_cy,
                   double &right_cx, double &right_cy) const;

  // ---- 自检 ----
  std::vector<std::string> selfCheck() const;

  // 文本可视化（调试）
  std::string dump() const;

private:
  int idx(int col, int row) const { return row * spec_.cols + col; }
  void buildFixedEdges();

  // ---- 转弯扫掠的实现细节 ----
  // 该世界点（cm）是否落在"可通行区域"内；返回与最近阻挡的间隙（cm，正=在内）
  double obstacleGapCm(double x, double y) const;
  // 车辆后轴中心位于 (x,y)、朝向 yaw 时的四个车角（世界系 cm）
  void bodyCorners(double x, double y, double yaw,
                   double *xs, double *ys) const;
  // 轮胎接地点（后轴两轮 + 前轴两轮）在给定位姿下的位置
  void wheelPoints(double x, double y, double yaw,
                   double *xs, double *ys) const;

  Spec spec_;
  std::vector<uint8_t> cells_;
  // blocked_up_[r][c]    : (c,r)->(c,r+1) 被禁（水平隔离线）
  // blocked_right_[r][c] : (c,r)->(c+1,r) 被禁
  std::vector<uint8_t> blocked_up_;
  std::vector<uint8_t> blocked_right_;
  // K 列两个已打开的通道行号，-1 = 未指定（= 隔板）
  int gate_a_row_{-1};
  int gate_b_row_{-1};
};

}  // namespace dzmission

#endif  // DZMISSION_FIELD_GRID_HPP
