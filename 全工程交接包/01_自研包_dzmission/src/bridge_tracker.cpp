// =============================================================================
// 桥洞穿梭（BridgeTracker）
// =============================================================================
// 规则（规则.docx image7）：
//   · 下方桥洞占 5 格，随机选 1 格可通行；上方桥洞同样随机选 1 格
//   · **从可通行通道通过即完成穿棱**
//   · 每通过一个桥洞通道得 10 分，每个只计一次；上下两个共 20 分
// 规则（image6）：桥洞标识设在**桥梁顶部**，地面小车受视角限制无法直接
//   识别，**必须由无人机高空侦察后把结论传给小车**。
//
// v5 拓扑：
//   · 桥洞在 K 列（col 10），整列隔板
//   · 区段一 K1..K6（row 0..5）；区段二 K7..K11（row 6..10）
//
// ★★ 两个核心设计决策（详见 dzmission/test/bridge_check.py）★★
//
// 【决策 1】穿越判定不能只看"到达门格"
//   车可以走到 K 列门前却不穿过去（例如只是路过），这不构成"通过通道"。
//   正确判据：必须观察到"从门格的一侧跨到另一侧"这一**动作**，
//   即 (K-1 -> K -> K+1) 或 (K+1 -> K -> K-1) 这样的三格序列。
//   只看"到达 K 列"会把路过也算通过 -> 误报得分。
//
// 【决策 2】门位缺失时必须拒绝，不能硬走
//   桥顶标识只能靠无人机下发，所以必须处理"没收到/只收到一个/收到错的"。
//   v5 拓扑下**单个门时起点到终点不可达**（水平隔离线只有 K 列一个缺口，
//   进西侧和回东侧需要两个缺口）。
//   若硬走：撞固定场地**持续超 10 秒** -> 裁判切手动
//   （image9：切手动后底盘不再动，只保留云台自动）
//   => 20 分拿不到，还赔上剩余所有任务。
//
//   校验规则：
//     · row 必须落在 K1..K11（越界拒绝）
//     · 两个门必须**分属两个区段**（同段拒绝 —— 规则说上下各 1 格）
//     · 两个门到齐之前，不认为"桥洞信息可用"
// =============================================================================

#include "dzmission/bridge_tracker.hpp"

#include <algorithm>
#include <string>

namespace dzmission
{

const char *bridgeStateName(BridgeState s)
{
  switch (s)
  {
    case BridgeState::kNoInfo: return "无门位信息";
    case BridgeState::kPartial: return "只有一个门";
    case BridgeState::kReady: return "两个门就绪";
    case BridgeState::kInvalid: return "门位非法";
    default: return "?";
  }
}

void BridgeTracker::reset()
{
  gate_a_row_ = -1;
  gate_b_row_ = -1;
  passed_a_ = false;
  passed_b_ = false;
  last_had_gate_ = false;
  prev_.clear();
  state_ = BridgeState::kNoInfo;
  note_.clear();
  rejected_.clear();
}

bool BridgeTracker::inSegmentA(int row) const
{
  return row >= a_lo_ && row <= a_hi_;
}

bool BridgeTracker::inSegmentB(int row) const
{
  return row >= b_lo_ && row <= b_hi_;
}

bool BridgeTracker::setGates(int row_a, int row_b)
{
  rejected_.clear();
  int ga = -1;
  int gb = -1;

  // ---- 逐个校验 ----
  auto accept = [&](int row, const char *tag) -> int {
    if (row < 0)
    {
      return -1;   // 未提供，不算错误
    }
    if (!inSegmentA(row) && !inSegmentB(row))
    {
      rejected_.push_back(std::string(tag) + " 越界（K" +
                          std::to_string(row + 1) + " 不在 K1..K11）");
      invalid_count_++;
      return -1;
    }
    return row;
  };

  const int ra = accept(row_a, "区段一");
  const int rb = accept(row_b, "区段二");

  // ---- 分段归位：不假设调用者传的顺序 ----
  if (ra >= 0)
  {
    if (inSegmentA(ra)) { ga = ra; }
    else { gb = ra; }
  }
  if (rb >= 0)
  {
    if (inSegmentA(rb))
    {
      if (ga >= 0 && ga != rb)
      {
        // 两个都在区段一 -> 非法
        rejected_.push_back("两个门都在区段一（K" + std::to_string(ga + 1) +
                            " / K" + std::to_string(rb + 1) + "）");
        invalid_count_++;
        return false;
      }
      ga = rb;
    }
    else
    {
      if (gb >= 0 && gb != rb)
      {
        rejected_.push_back("两个门都在区段二（K" + std::to_string(gb + 1) +
                            " / K" + std::to_string(rb + 1) + "）");
        invalid_count_++;
        return false;
      }
      gb = rb;
    }
  }

  // ---- 更新状态 ----
  // ★ 关键：先判断**门位是否真的变了**，变了就重置过门记录。
  //
  // 原实现只在"从未就绪变为就绪"时重置，漏掉了"改判"这一情形：
  //   门位从 K4 改成 K5 时，状态**仍然是 kReady**（前后都就绪），
  //   于是旧门位（K4）的过门记录被保留 -> 会**误报/漏报 10 分**。
  // 这是实机自测（bridge_selftest）抓到的真实 bug，不是测试写错。
  const bool changed =
      ((ga >= 0) && (ga != gate_a_row_)) ||
      ((gb >= 0) && (gb != gate_b_row_));

  if (ga >= 0)
  {
    gate_a_row_ = ga;
  }
  if (gb >= 0)
  {
    gate_b_row_ = gb;
  }

  if (changed)
  {
    // 门位改判：此前基于旧门位的过门记录全部作废
    passed_a_ = false;
    passed_b_ = false;
    prev_.clear();
  }

  if (gate_a_row_ >= 0 && gate_b_row_ >= 0)
  {
    state_ = BridgeState::kReady;
    note_ = "K" + std::to_string(gate_a_row_ + 1) + " + K" +
            std::to_string(gate_b_row_ + 1);
    return true;
  }
  else if (gate_a_row_ >= 0 || gate_b_row_ >= 0)
  {
    state_ = BridgeState::kPartial;
    note_ = (gate_a_row_ >= 0 ? "K" + std::to_string(gate_a_row_ + 1) : "?") +
            " + " +
            (gate_b_row_ >= 0 ? "K" + std::to_string(gate_b_row_ + 1) : "?");
    return false;
  }

  state_ = rejected_.empty() ? BridgeState::kNoInfo : BridgeState::kInvalid;
  note_ = rejected_.empty() ? "" : rejected_.front();
  return false;
}

void BridgeTracker::markPassed(int gate_row)
{
  if (gate_row < 0)
  {
    return;
  }
  if (gate_row == gate_a_row_)
  {
    passed_a_ = true;
  }
  else if (gate_row == gate_b_row_)
  {
    passed_b_ = true;
  }
}

BridgeTracker::PassResult BridgeTracker::update(int cur_col, int cur_row)
{
  PassResult r;
  r.state = state_;
  r.just_passed_a = false;
  r.just_passed_b = false;
  r.all_passed = allPassed();

  // 状态不可用时不上报过门
  if (state_ != BridgeState::kReady)
  {
    prev_.clear();
    return r;
  }

  // 记录格序列（只保留最近 3 格，够判"进入+穿出"）
  prev_.push_back({cur_col, cur_row});
  while (prev_.size() > 3)
  {
    prev_.erase(prev_.begin());
  }

  // ★ 核心判据：三格序列 (west, K, east) 或 (east, K, west)
  //   只看"到达 K 列"会把路过也算通过 -> 误报得分。
  if (prev_.size() >= 3)
  {
    const auto &g0 = prev_[prev_.size() - 3];
    const auto &g1 = prev_[prev_.size() - 2];
    const auto &g2 = prev_[prev_.size() - 1];

    const bool entered_from_west = (g0.first == bridge_col_ - 1 &&
                                    g1.first == bridge_col_);
    const bool entered_from_east = (g0.first == bridge_col_ + 1 &&
                                    g1.first == bridge_col_);
    const bool in_gate = (g1.first == bridge_col_);

    if (in_gate && (entered_from_west || entered_from_east))
    {
      const bool exited =
          (g2.first == bridge_col_ + 1) || (g2.first == bridge_col_ - 1);
      // 必须真的从门格走到另一侧（不是原地不动、也不是折返）
      const bool not_backtrack =
          !((entered_from_west && g2.first == bridge_col_ - 1) ||
            (entered_from_east && g2.first == bridge_col_ + 1));
      if (exited && not_backtrack)
      {
        const int gate_row = g1.second;
        const bool was_a = passed_a_;
        const bool was_b = passed_b_;
        markPassed(gate_row);
        if (!was_a && passed_a_ && gate_row == gate_a_row_)
        {
          r.just_passed_a = true;
        }
        if (!was_b && passed_b_ && gate_row == gate_b_row_)
        {
          r.just_passed_b = true;
        }
        r.gate_row = gate_row;
        prev_.clear();   // 过门后重新起算，避免重复计入
      }
    }
  }

  r.all_passed = allPassed();
  r.state = state_;
  return r;
}

std::string BridgeTracker::selfCheck()
{
  // 用拓扑 v5 的规格自检：30 种合法门位组合都必须在两个区段内
  std::string err;
  int a_cnt = a_hi_ - a_lo_ + 1;
  int b_cnt = b_hi_ - b_lo_ + 1;
  if (a_cnt <= 0 || b_cnt <= 0)
  {
    return "桥洞区段范围非法";
  }
  if (a_lo_ < 0 || b_hi_ >= rows_)
  {
    return "桥洞区段超出场地行数";
  }
  if (bridge_col_ < 0 || bridge_col_ >= cols_)
  {
    return "桥洞列超出场地列数";
  }
  if (a_hi_ + 1 != b_lo_)
  {
    return "两个桥洞区段不连续（应为上下相邻）";
  }

  // 自检：合法的行必须被接受，非法的必须被拒绝
  BridgeTracker t;
  t.configure(cols_, rows_, bridge_col_, a_lo_, a_hi_, b_lo_, b_hi_);
  for (int a = a_lo_; a <= a_hi_; ++a)
  {
    for (int b = b_lo_; b <= b_hi_; ++b)
    {
      if (!t.setGates(a, b))
      {
        return "合法的门位组合被拒绝：K" + std::to_string(a + 1) +
               " + K" + std::to_string(b + 1);
      }
      if (t.state() != BridgeState::kReady)
      {
        return "合法门位未进入 kReady";
      }
    }
  }
  // 非法：两个都在区段一
  t.reset();
  if (t.setGates(a_lo_, a_hi_))
  {
    return "同段门位（都在区段一）未被拒绝";
  }
  // 非法：越界
  t.reset();
  if (t.setGates(b_hi_ + 1, b_lo_))
  {
    return "越界门位未被拒绝";
  }
  // 单门：必须不是 kReady
  t.reset();
  t.setGates(a_lo_, -1);
  if (t.state() == BridgeState::kReady)
  {
    return "单个门被误判为就绪";
  }
  return "";   // 空串 = 通过
}

}  // namespace dzmission
