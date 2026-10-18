// =============================================================================
// 桥洞穿梭（BridgeTracker）—— 头文件
// =============================================================================
// 详细设计说明见 bridge_tracker.cpp 顶部注释。
//
// 职责：
//   1. 接收无人机下发的桥洞门位（每区段 1 格），做**严格校验**
//   2. 判定车是否真的"穿过"了门（进入 + 穿出，而不是仅到达）
//   3. 记录两个门各自是否已通过，供任务层上报 EVENT(BRIDGE_PASSED)
//
// 为什么单独成类而不是塞进 MissionManager：
//   · 校验规则和判定条件是**纯逻辑**，可以脱离 ROS 单测
//     （见 test/bridge_check.py 的对照实现）
//   · selfCheck() 可在启动时验证 30 种合法门位组合都被接受、
//     非法组合都被拒绝 —— 这类"静默错误"靠现场调试很难发现
// =============================================================================

#ifndef DZMISSION__BRIDGE_TRACKER_HPP_
#define DZMISSION__BRIDGE_TRACKER_HPP_

#include <string>
#include <utility>
#include <vector>

namespace dzmission
{

enum class BridgeState
{
  kNoInfo = 0,    // 一个门都没收到
  kPartial = 1,   // 只收到一个门（★ 不可用 —— 单个门时起终点不可达）
  kReady = 2,     // 两个门都就绪，可用于规划
  kInvalid = 3,   // 收到的门位非法（越界 / 同段）
};

const char *bridgeStateName(BridgeState s);

class BridgeTracker
{
public:
  struct PassResult
  {
    BridgeState state{BridgeState::kNoInfo};
    bool just_passed_a{false};   // 本次更新**刚刚**穿过区段一的门
    bool just_passed_b{false};   // 本次更新**刚刚**穿过区段二的门
    bool all_passed{false};      // 两个门都穿过了
    int gate_row{-1};            // 本次穿过的门所在行（-1 表示本次无过门）
  };

  BridgeTracker() = default;

  // 用场地规格初始化（与 FieldGrid::Spec 一致）
  void configure(int cols, int rows, int bridge_col,
                 int a_lo, int a_hi, int b_lo, int b_hi)
  {
    cols_ = cols;
    rows_ = rows;
    bridge_col_ = bridge_col;
    a_lo_ = a_lo;
    a_hi_ = a_hi;
    b_lo_ = b_lo;
    b_hi_ = b_hi;
    reset();
  }

  void reset();

  // ---------------------------------------------------------------------------
  // 设置门位（row_a / row_b 用 0-based 行号；-1 表示"该区段未提供"）
  //
  // 返回 true 表示**两个门都有效且分属两个区段**（可以用于规划）。
  //
  // ★ 设计取舍：允许分多次调用（无人机可能分两帧下发），
  //   内部保留已接受的那一个。但**只有两个都到齐才算 kReady**。
  //
  // ★ 拒绝的情形（记入 rejected()）：
  //     · 行号越界（不在 K1..K11）
  //     · 两个门落在同一区段（规则说上下各 1 格）
  // ---------------------------------------------------------------------------
  bool setGates(int row_a, int row_b);

  // ---------------------------------------------------------------------------
  // 每周期用**当前所在格**更新一次，检测是否发生了"穿门"动作
  //
  // ★ 只传"当前格"而不是整条路径：因为车是边走边判，
  //   而且路径可能被重规划 —— 用当前格序列判定与路径无关，更稳。
  // ---------------------------------------------------------------------------
  PassResult update(int cur_col, int cur_row);

  // ---- 查询 ----
  BridgeState state() const { return state_; }
  bool ready() const { return state_ == BridgeState::kReady; }
  bool usable() const { return state_ == BridgeState::kReady; }
  int gateRowA() const { return gate_a_row_; }
  int gateRowB() const { return gate_b_row_; }
  bool passedA() const { return passed_a_; }
  bool passedB() const { return passed_b_; }
  bool allPassed() const { return passed_a_ && passed_b_; }
  int bridgeCol() const { return bridge_col_; }
  int invalidCount() const { return invalid_count_; }
  const std::string &note() const { return note_; }
  const std::vector<std::string> &rejected() const { return rejected_; }

  // 启动自检：返回空串表示通过，否则返回错误描述
  //   ★ 会验证全部 30 种合法门位组合都被接受、非法组合都被拒绝
  std::string selfCheck();

private:
  bool inSegmentA(int row) const;
  bool inSegmentB(int row) const;
  void markPassed(int gate_row);

  // 场地规格
  int cols_{14};
  int rows_{11};
  int bridge_col_{10};
  int a_lo_{0};
  int a_hi_{5};
  int b_lo_{6};
  int b_hi_{10};

  // 门位（-1 = 未提供）
  int gate_a_row_{-1};
  int gate_b_row_{-1};

  // 过门记录
  bool passed_a_{false};
  bool passed_b_{false};

  // 最近 3 个格（判"进入+穿出"用）
  std::vector<std::pair<int, int>> prev_;

  BridgeState state_{BridgeState::kNoInfo};
  bool last_had_gate_{false};
  int invalid_count_{0};
  std::string note_;
  std::vector<std::string> rejected_;
};

}  // namespace dzmission

#endif  // DZMISSION__BRIDGE_TRACKER_HPP_
