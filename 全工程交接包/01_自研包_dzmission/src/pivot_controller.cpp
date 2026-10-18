#include "dzmission/pivot_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dzmission
{

namespace
{
// 归一化到 [-180, 180]（度）
double normDeg(double a)
{
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
}  // namespace

const char *PivotController::phaseName() const
{
  switch (phase_)
  {
    case Phase::kIdle: return "IDLE";
    case Phase::kStop: return "STOP";
    case Phase::kWaitServo: return "WAIT_SERVO";
    case Phase::kCreep: return "CREEP";
    case Phase::kDone: return "DONE";
    default: return "?";
  }
}

bool PivotController::start(double current_yaw_deg, int turn_sign,
                            double target_angle_deg, const FieldGrid *grid,
                            int col, int row)
{
  last_ = Result{};
  cmd_ = Cmd{};

  if (turn_sign != 1 && turn_sign != -1)
  {
    last_.reason = "turn_sign 必须为 +1（左）或 -1（右）";
    return false;
  }
  if (target_angle_deg <= 0.0 || target_angle_deg > 180.0)
  {
    last_.reason = "target_angle_deg 必须在 (0, 180] 内";
    return false;
  }

  // 可行性复核：如果给了场地模型，就现场再确认一次
  if (grid != nullptr)
  {
    const auto p = grid->checkPivot(col, row);
    if (!p.feasible)
    {
      last_.reason = "原地转向可行性复核失败：" + p.reason;
      return false;
    }
  }

  // 自动估算蠕行行程上限：
  //   原地转向时后轴中心位移≈0，但为了对中需要少量蠕行。
  //   名义行程 = R * 角度（弧度），再乘安全倍率。
  if (creep_max_cm <= 0.0)
  {
    const double rad = target_angle_deg * M_PI / 180.0;
    creep_limit_cm_ = nominal_radius_cm * rad * safety_scale;
    // 给一个下限，避免参数异常导致上限太小而立刻超时
    creep_limit_cm_ = std::max(creep_limit_cm_, 15.0);
  }
  else
  {
    creep_limit_cm_ = creep_max_cm;
  }

  start_yaw_deg_ = current_yaw_deg;
  target_angle_deg_ = target_angle_deg;
  turn_sign_ = turn_sign;
  start_time_s_ = -1.0;      // 首次 tick 时填
  phase_start_s_ = -1.0;
  phase_ = Phase::kStop;

  cmd_.active = true;
  cmd_.phase = phase_;
  cmd_.v = 0.0;
  // phase 0 就把舵机打到目标角度（学员做法），让 phase 1 纯粹用来等待
  cmd_.steer_deg = std::clamp(turn_sign * lock_deg + steer_trim_deg,
                              -35.0, 35.0);

  last_.started = true;
  last_.reason = "开始原地转向";
  return true;
}

PivotController::Result PivotController::tick(double now_s, double current_yaw_deg,
                                              double distance_cm)
{
  if (phase_ == Phase::kIdle || phase_ == Phase::kDone)
  {
    last_.finished = (phase_ == Phase::kDone);
    return last_;
  }

  if (start_time_s_ < 0.0)
  {
    start_time_s_ = now_s;
    phase_start_s_ = now_s;
  }

  const double elapsed = now_s - start_time_s_;
  const double in_phase = now_s - phase_start_s_;

  // 带符号的实际已转角度（沿转向方向为正）
  const double rotated =
      turn_sign_ * normDeg(current_yaw_deg - start_yaw_deg_);
  // 目标角度扣掉提前停止量
  const double target_rot = std::max(0.0, target_angle_deg_ - lead_deg);
  const double yaw_err = target_rot - rotated;

  auto finish = [&](const char *by) {
    phase_ = Phase::kDone;
    cmd_.active = false;
    cmd_.v = 0.0;
    cmd_.steer_deg = 0.0;   // 交还舵机，由下一段纯跟踪接管回正
    cmd_.phase = phase_;
    last_.finished = true;
    last_.yaw_traveled_deg = rotated;
    last_.distance_cm = distance_cm;
    last_.elapsed_s = elapsed;
    last_.stop_by = by;
    last_.reason = std::string("原地转向完成（") + by + "）";
  };

  // ---- 兜底 1：总超时 ----
  if (elapsed >= total_timeout_s)
  {
    finish("timeout");
    return last_;
  }

  switch (phase_)
  {
    case Phase::kStop:
    {
      // 停车 + 打舵机（舵机指令已由 start() 设好）
      cmd_.active = true;
      cmd_.v = 0.0;
      cmd_.steer_deg = std::clamp(turn_sign_ * lock_deg + steer_trim_deg,
                                  -35.0, 35.0);
      cmd_.phase = phase_;
      // 立刻进入等待阶段
      phase_ = Phase::kWaitServo;
      phase_start_s_ = now_s;
      break;
    }

    case Phase::kWaitServo:
    {
      // 保持零速与满舵，让舵机物理到位
      cmd_.active = true;
      cmd_.v = 0.0;
      cmd_.steer_deg = std::clamp(turn_sign_ * lock_deg + steer_trim_deg,
                                  -35.0, 35.0);
      cmd_.phase = phase_;
      if (in_phase >= wait_servo_s)
      {
        phase_ = Phase::kCreep;
        phase_start_s_ = now_s;
      }
      break;
    }

    case Phase::kCreep:
    {
      cmd_.active = true;
      cmd_.v = creep_v;
      cmd_.steer_deg = std::clamp(turn_sign_ * lock_deg + steer_trim_deg,
                                  -35.0, 35.0);
      cmd_.phase = phase_;

      // ---- 主判据：偏航角已对准 ----
      if (yaw_err <= 0.0)
      {
        finish("yaw");
        return last_;
      }
      // ---- 兜底 2：蠕行行程超上限（偏航数据失效时防止无限转） ----
      if (distance_cm >= creep_limit_cm_)
      {
        finish("travel");
        return last_;
      }
      break;
    }

    default:
      break;
  }

  last_.finished = false;
  last_.yaw_traveled_deg = rotated;
  last_.distance_cm = distance_cm;
  last_.elapsed_s = elapsed;
  return last_;
}

void PivotController::cancel()
{
  phase_ = Phase::kIdle;
  cmd_ = Cmd{};
  last_ = Result{};
  last_.reason = "已取消";
}

}  // namespace dzmission
