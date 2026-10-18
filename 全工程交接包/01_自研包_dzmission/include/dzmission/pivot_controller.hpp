#ifndef DZMISSION_PIVOT_CONTROLLER_HPP
#define DZMISSION_PIVOT_CONTROLLER_HPP

#include <string>

#include "dzmission/field_grid.hpp"

namespace dzmission
{

// =============================================================================
// PivotController —— 原地转向（pivot turn）三段式状态机
// -----------------------------------------------------------------------------
// 为什么需要它：
//   K 列以东窄条（L/M/N）内，单圆弧 90° 转向的扫掠半径 ≈ 44.6 cm，
//   而格心距场边仅 17.5 cm，几何上不可行（FieldGrid::checkTurn 已证实）。
//   原地转向绕后轴中心纯转动，扫掠半径 = 车体对角半径 ≈ 18.6 cm，
//   只有单圆弧的 1/2.4，所以窄条内可转。
//
// 三段式（借鉴学员代码并在两点上改进）：
//   phase 0 停车     : v=0，先把舵机打到目标角度
//   phase 1 等舵机   : v=0，保持 wait_servo_s 让舵机物理到位
//   phase 2 蠕行对中 : v=creep_v，保持满舵，直到 |yaw_err| <= tol_deg
//
// 学员代码用"行程 / 偏航 / 超时"三者之一结束圆弧，但实测发现
// **舵机左右行程严重不对称**（同一满舵：右转 0.38m 仅 50°，左转 0.37m 达 140°），
// 所以基于距离的角度永远不可能两边都对。本实现因此：
//   · 主判据 = 偏航角闭环（phase 2 的 yaw_err）
//   · 副判据 = 蠕行行程上限 creep_max_cm（防止偏航数据失效时无限转）
//   · 兜底   = 总超时 total_timeout_s
//   三者任一满足即结束，且行程只作为**安全上界**（乘 safety_scale 留裕量）。
//
// 与 MotionArbiter 的接口：
//   tick() 返回的 steer_deg 直接送 MotionArbiter::setServoSource(kPivot, v, steer)，
//   量纲是**度**（舵机角度），不是角速度。
// =============================================================================
class PivotController
{
public:
  enum class Phase
  {
    kIdle = 0,     // 未在转向
    kStop = 1,     // 停车 + 打舵机
    kWaitServo = 2,// 等舵机到位
    kCreep = 3,    // 蠕行对中
    kDone = 4,     // 转向完成（主循环应收起并清除 pivot 源）
  };

  struct Cmd
  {
    bool active{false};      // 是否占用底盘（true 时主循环必须注入 kPivot 源）
    double v{0.0};           // m/s
    double steer_deg{0.0};   // 舵机角度（度）
    Phase phase{Phase::kIdle};
  };

  struct Result
  {
    bool started{false};
    bool finished{false};
    std::string reason;
    // 完成时的实际表现，供日志与在线标定
    double yaw_traveled_deg{0.0};
    double distance_cm{0.0};
    double elapsed_s{0.0};
    const char *stop_by{""};   // "yaw" / "travel" / "timeout"
  };

  PivotController() = default;

  // 请求一次原地转向
  //   current_yaw_deg : 当前偏航角（度）
  //   turn_sign       : +1 = 左转（逆时针），-1 = 右转（顺时针）
  //   target_angle_deg: 需要转过的绝对角度（度），一般 90
  //   grid            : 用于可行性复核（可选，nullptr 则跳过）
  //   col,row         : 当前所在格（用于复核）
  bool start(double current_yaw_deg, int turn_sign, double target_angle_deg,
             const FieldGrid *grid = nullptr, int col = 0, int row = 0);

  // 周期性调用（建议与主控制循环同频，如 50 Hz）
  //   now_s           : 当前时间（秒，单调递增）
  //   current_yaw_deg : 当前偏航角（度）
  //   distance_cm     : 本次转向开始以来的累计行驶距离（cm）
  Result tick(double now_s, double current_yaw_deg, double distance_cm);

  // 当前应发布的指令
  const Cmd &cmd() const { return cmd_; }

  // 取消（异常/急停）
  void cancel();

  bool active() const { return phase_ != Phase::kIdle && phase_ != Phase::kDone; }
  Phase phase() const { return phase_; }
  const char *phaseName() const;
  double targetAngleDeg() const { return target_angle_deg_; }
  int turnSign() const { return turn_sign_; }

  // ---- 参数（可由 params/dzmission.yaml 覆盖） ----
  // 舵机角度（度）。硬件允许 [-35, +35]，控制层默认留 5° 余量
  double lock_deg{30.0};
  // phase 1 等待舵机物理到位的时间
  double wait_servo_s{0.5};
  // phase 2 蠕行速度（m/s）—— 必须慢，否则前轴会甩出格
  double creep_v{0.08};
  // phase 2 的对中容差（度）。学员用 15°，本实现收紧到 6° 以提高精度
  double tol_deg{6.0};
  // 提前停止量（度）：舵机回中会有滞后，提前一点停，剩余靠下一段纯跟踪闭环
  double lead_deg{3.0};
  // 蠕行行程上限（cm）。按 90° 圆弧名义长度 × safety_scale 估算
  double creep_max_cm{0.0};     // 0 = 用公式自动算
  double safety_scale{1.6};     // 行程安全上界倍率
  // 总超时（s），兜底
  double total_timeout_s{6.0};
  // 横向稳态偏差补偿（度），负 = 向右偏。dzactuator 有恒定机械左偏
  double steer_trim_deg{-1.0};
  // 原地转向的最小转弯半径假设（cm），仅用于估算行程上限
  double nominal_radius_cm{26.0};

private:
  Phase phase_{Phase::kIdle};
  Cmd cmd_;

  double start_yaw_deg_{0.0};
  double target_angle_deg_{0.0};
  int turn_sign_{1};
  double start_time_s_{0.0};
  double phase_start_s_{0.0};
  double creep_limit_cm_{0.0};
  Result last_;
};

}  // namespace dzmission

#endif  // DZMISSION_PIVOT_CONTROLLER_HPP
