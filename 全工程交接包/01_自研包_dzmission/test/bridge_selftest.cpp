// =============================================================================
// BridgeTracker 自测（可在车端直接运行，不需要 ROS）
// =============================================================================
// 为什么要有这个程序：
//   桥洞"过门判定"是本赛题里最容易**静默出错**的逻辑 ——
//   判松了（只看"到达门格"）会误报得分，判紧了会漏报。
//   而这两者在赛场上都极难现场发现（只有 3 分钟、无人干预）。
//   所以把判定逻辑做成可离线穷举验证的纯函数 + 自测程序。
//
// 运行： ros2 run dzmission bridge_selftest
//   或   ./bridge_selftest
// =============================================================================

#include "dzmission/bridge_tracker.hpp"

#include <cstdio>
#include <string>
#include <vector>

using dzmission::BridgeTracker;
using dzmission::BridgeState;

static int g_fail = 0;
static int g_pass = 0;

static void expect(bool cond, const std::string &what)
{
  if (cond)
  {
    ++g_pass;
    std::printf("  [ OK ] %s\n", what.c_str());
  }
  else
  {
    ++g_fail;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

static BridgeTracker makeTracker()
{
  BridgeTracker t;
  // 与 FieldGrid::Spec 的 v5 规格一致
  t.configure(14, 11, /*bridge_col=*/10,
              /*a_lo=*/0, /*a_hi=*/5, /*b_lo=*/6, /*b_hi=*/10);
  return t;
}

// -----------------------------------------------------------------------------
static void testGateValidation()
{
  std::printf("\n=== 1. 门位校验（合法接受 / 非法拒绝）===\n");

  {
    BridgeTracker t = makeTracker();
    expect(t.setGates(3, 8), "K4 + K9 被接受");
    expect(t.state() == BridgeState::kReady, "K4 + K9 -> kReady");
    expect(t.gateRowA() == 3 && t.gateRowB() == 8, "门位记录正确");
  }
  {
    // 顺序颠倒也应正确归位（无人机不保证先发哪个区段）
    BridgeTracker t = makeTracker();
    expect(t.setGates(8, 3), "K9 + K4（顺序颠倒）被接受");
    expect(t.gateRowA() == 3 && t.gateRowB() == 8,
           "顺序颠倒仍正确归位到区段一/二");
  }
  {
    BridgeTracker t = makeTracker();
    expect(!t.setGates(3, -1), "只给区段一 -> 不是 kReady");
    expect(t.state() == BridgeState::kPartial, "只给区段一 -> kPartial");
    expect(!t.usable(), "kPartial 不可用于规划");
  }
  {
    BridgeTracker t = makeTracker();
    expect(!t.setGates(-1, 8), "只给区段二 -> 不是 kReady");
    expect(t.state() == BridgeState::kPartial, "只给区段二 -> kPartial");
  }
  {
    BridgeTracker t = makeTracker();
    expect(!t.setGates(-1, -1), "都不给 -> 不是 kReady");
    expect(t.state() == BridgeState::kNoInfo, "都不给 -> kNoInfo");
  }
  {
    // 越界
    BridgeTracker t = makeTracker();
    expect(!t.setGates(11, 8), "越界行 K12 被拒绝");
    expect(t.state() != BridgeState::kReady, "越界后不是 kReady");
    expect(t.invalidCount() > 0, "越界计入 invalidCount");
  }
  {
    // 两个都在区段一
    BridgeTracker t = makeTracker();
    expect(!t.setGates(2, 4), "两个门都在区段一 -> 被拒绝");
    expect(!t.usable(), "同段门位不可用");
  }
  {
    // 两个都在区段二
    BridgeTracker t = makeTracker();
    expect(!t.setGates(7, 9), "两个门都在区段二 -> 被拒绝");
    expect(!t.usable(), "同段门位不可用");
  }
  {
    // 分两次下发（无人机可能分帧）
    BridgeTracker t = makeTracker();
    t.setGates(3, -1);
    expect(!t.usable(), "分两次：第一次后不可用");
    t.setGates(-1, 8);
    expect(t.usable(), "分两次：第二次后可用");
  }
}

// -----------------------------------------------------------------------------
static void testCrossingDetection()
{
  std::printf("\n=== 2. ★ 穿越判定（核心：进入+穿出，不能只看到达）===\n");

  // --- 情形 A：只是路过门前 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    bool got = false;
    // J3 -> J4 -> J5（都在 K 以西，col 9）
    for (auto &p : std::vector<std::pair<int, int>>{{9, 2}, {9, 3}, {9, 4}})
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a || r.just_passed_b) { got = true; }
    }
    expect(!got, "只是路过门前（不进门）-> 不上报过门");
  }

  // --- 情形 B：进入门又退回 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    bool got = false;
    // J4(col9,row3) -> K4(col10,row3) -> J4(col9,row3)
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {9, 3}})
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a || r.just_passed_b) { got = true; }
    }
    expect(!got, "进入门又退回 -> 不上报过门（不算穿过）");
  }

  // --- 情形 C：完整穿过（西->东）---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    bool got_a = false;
    // J4 -> K4 -> L4
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {11, 3}})
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a) { got_a = true; }
    }
    expect(got_a, "完整穿过 K4（西->东）-> 上报过门");
    expect(t.passedA(), "passedA 已置位");
    expect(!t.allPassed(), "只过一个门 -> allPassed 为假");
  }

  // --- 情形 D：完整穿过（东->西）---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    bool got_a = false;
    // L4 -> K4 -> J4
    for (auto &p : std::vector<std::pair<int, int>>{{11, 3}, {10, 3}, {9, 3}})
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a) { got_a = true; }
    }
    expect(got_a, "完整穿过 K4（东->西）-> 上报过门");
  }

  // --- 情形 E：两个门都过 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    // J4 -> K4 -> L4 ... L9 -> K9 -> J9
    std::vector<std::pair<int, int>> path{
        {9, 3}, {10, 3}, {11, 3},
        {11, 8}, {10, 8}, {9, 8}};
    for (auto &p : path) { t.update(p.first, p.second); }
    expect(t.passedA(), "第一个门（K4）已过");
    expect(t.passedB(), "第二个门（K9）已过");
    expect(t.allPassed(), "两个门都过 -> allPassed");
  }

  // --- 情形 F：同一个门过两次（不应重复计数）---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    int count_a = 0;
    std::vector<std::pair<int, int>> path{
        {9, 3}, {10, 3}, {11, 3},     // 过 K4（西->东）
        {10, 3}, {9, 3},              // 过 K4（东->西）
        {10, 3}, {11, 3}};            // 再过 K4（西->东）
    for (auto &p : path)
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a) { ++count_a; }
    }
    expect(count_a == 1, "同一个门重复穿过 -> 只上报一次（不重复计分）");
  }

  // --- 情形 G：门位变更后重置过门记录 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {11, 3}})
    {
      t.update(p.first, p.second);
    }
    expect(t.passedA(), "变更前已过 K4");
    t.setGates(4, 8);   // 区段一改判为 K5
    expect(!t.passedA(), "门位变更后过门记录被重置");
    expect(!t.allPassed(), "变更后 allPassed 为假");
  }

  // --- 情形 G2：改判到另一个门后，新门仍可正常判定 ---
  //   （G 只验证了"记录被重置"，这里验证"重置后仍能正确工作"）
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {11, 3}})
    {
      t.update(p.first, p.second);
    }
    expect(t.passedA(), "G2 改判前已过 K4");
    t.setGates(4, 8);              // 区段一改判为 K5
    expect(!t.passedA(), "G2 改判后过门记录被重置");
    // 此时若还去穿旧门 K4（row=3），不应再被计入
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {11, 3}})
    {
      t.update(p.first, p.second);
    }
    expect(!t.passedA(), "G2 穿旧门 K4（已改判掉）-> 不再计入");
    // 穿新门 K5（row=4）应正常计入
    for (auto &p : std::vector<std::pair<int, int>>{{9, 4}, {10, 4}, {11, 4}})
    {
      t.update(p.first, p.second);
    }
    expect(t.passedA(), "G2 穿新门 K5 -> 正确计入");
    expect(!t.allPassed(), "G2 只过区段一 -> allPassed 为假");
  }

  // --- 情形 G3：分两次下发导致门位变化，也应重置 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    for (auto &p : std::vector<std::pair<int, int>>{{9, 3}, {10, 3}, {11, 3}})
    {
      t.update(p.first, p.second);
    }
    expect(t.passedA(), "G3 已过 K4");
    t.setGates(3, 9);              // 区段二从 K9 改判为 K10
    expect(!t.passedA(), "G3 区段二改判 -> 过门记录也重置（保守）");
  }

  // --- 情形 H：非门格不算过门 ---
  {
    BridgeTracker t = makeTracker();
    t.setGates(3, 8);
    bool got = false;
    // K 列第 5 行（row4）不是门（门是 row3/row8）—— 实际上不可通行，
    // 但即便传入也不应被判为过门
    for (auto &p : std::vector<std::pair<int, int>>{{9, 4}, {10, 4}, {11, 4}})
    {
      auto r = t.update(p.first, p.second);
      if (r.just_passed_a || r.just_passed_b) { got = true; }
    }
    expect(!got, "穿过非门格 -> 不上报过门");
  }
}

// -----------------------------------------------------------------------------
static void testExhaustiveGates()
{
  std::printf("\n=== 3. 穷举 30 种合法门位组合 ===\n");

  int ok = 0;
  int total = 0;
  for (int a = 0; a <= 5; ++a)
  {
    for (int b = 6; b <= 10; ++b)
    {
      ++total;
      BridgeTracker t = makeTracker();
      if (!t.setGates(a, b) || t.state() != BridgeState::kReady)
      {
        std::printf("  [FAIL] K%d + K%d 未被接受\n", a + 1, b + 1);
        ++g_fail;
        continue;
      }
      // 模拟实际穿过这两个门
      std::vector<std::pair<int, int>> path{
          {9, a}, {10, a}, {11, a},
          {11, b}, {10, b}, {9, b}};
      for (auto &p : path) { t.update(p.first, p.second); }
      if (t.allPassed())
      {
        ++ok;
      }
      else
      {
        std::printf("  [FAIL] K%d + K%d 穿过后 allPassed 仍为假\n",
                    a + 1, b + 1);
        ++g_fail;
      }
    }
  }
  expect(ok == total, "全部 " + std::to_string(total) +
                      " 种组合都被接受且穿过后 allPassed");
}

// -----------------------------------------------------------------------------
static void testSelfCheck()
{
  std::printf("\n=== 4. 内建 selfCheck ===\n");
  BridgeTracker t = makeTracker();
  const std::string err = t.selfCheck();
  expect(err.empty(), "selfCheck 通过" + (err.empty() ? "" : ("：" + err)));
}

// -----------------------------------------------------------------------------
int main()
{
  std::printf("================================================================\n");
  std::printf(" BridgeTracker 自测（桥洞过门判定）\n");
  std::printf(" 规则：上下各 5 格随机 1 格可通行；从通道通过得 10 分，共 20 分\n");
  std::printf("================================================================\n");

  testGateValidation();
  testCrossingDetection();
  testExhaustiveGates();
  testSelfCheck();

  std::printf("\n================================================================\n");
  std::printf(" 结果：通过 %d，失败 %d\n", g_pass, g_fail);
  if (g_fail == 0)
  {
    std::printf(" ✓ 全部通过 —— 过门判定可上线\n");
  }
  else
  {
    std::printf(" ✗ 有失败项，**不要上线**\n");
  }
  std::printf("================================================================\n");
  return g_fail == 0 ? 0 : 1;
}
