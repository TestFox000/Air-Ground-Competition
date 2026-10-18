#ifndef DZMISSION_MISSION_MANAGER_HPP
#define DZMISSION_MISSION_MANAGER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "dzmission/cmd_link.hpp"
#include "dzmission/mission_command.hpp"
#include "dzmission/motion_arbiter.hpp"
#include "dzmission/referee_board.hpp"
#include "dzmission/field_grid.hpp"
#include "dzmission/safety_monitor.hpp"
#include "dzmission/target_aimer.hpp"
#include "dzmission/bridge_tracker.hpp"

namespace dzmission
{

// =============================================================================
// MissionManager —— 地面车任务状态机
// -----------------------------------------------------------------------------
// 状态：
//   DISABLED    —— 未启动任务（等待第一条指令）
//   IDLE        —— 待命，接受指令
//   NAVIGATING  —— 前往无人机指定的坐标点
//   ENGAGING    —— 交战：云台找靶 + 激光打击（可同时 NAVIGATING 就位）
//   RETURNING   —— 返航到 home
//   HOLDING     —— 原地待命（保持航向）
//   ESTOP       —— 安全急停（雷达近距 / 定位失效），障碍清除后自动恢复
//
// 指令 -> 动作：
//   NAV_TO      进入 NAVIGATING，下发 NavigateToPose
//   STOP        清空所有意图，零速
//   ENGAGE      进入 ENGAGING；若有相对方位/距离则先解算目标点并前往
//   CEASE_FIRE  退出开火，保留交战姿态
//   RETURN_BASE 进入 RETURNING
//   HOLD        进入 HOLDING
//   TARGET_REPORT 只更新目标缓存，不改变状态（用于视觉引导）
//   SET_HOME    设置 home 点
//
// 安全联锁（任一触发即降级）：
//   - 定位不健康        -> ESTOP + FAULT_NO_LOCALIZATION
//   - 雷达近距          -> ESTOP + FAULT_OBSTACLE_BLOCKED（自动恢复）
//   - 链路超时          -> 自动 RETURNING + FAULT_CMD_LINK_LOST
//   - 电量低            -> 禁止交战 + RETURNING + FAULT_LOW_BATTERY
//   - 弹量为 0          -> 禁止开火 + FAULT_NO_AMMO
// =============================================================================
class MissionManager : public rclcpp::Node
{
public:
  enum class State : uint8_t
  {
    kDisabled = 0,
    kIdle = 1,
    kNavigating = 2,
    kEngaging = 3,
    kReturning = 4,
    kHolding = 5,
    kEstop = 6,
    kArrived = 7,
  };

  static const char *stateName(State s);

  explicit MissionManager(const rclcpp::NodeOptions &options);
  ~MissionManager() override;

private:
  // ---- 初始化 ----
  void loadParams();
  void setupRosInterfaces();
  void startTimer();

  // ---- 主循环 ----
  void tick();

  // ---- 指令处理 ----
  void onMissionCommand(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void handleNavTo(const MissionCommand &m);
  void handleStop(const MissionCommand &m);
  void handleEngage(const MissionCommand &m);
  void handleCeaseFire(const MissionCommand &m);
  void handleReturnBase(const MissionCommand &m);
  void handleHold(const MissionCommand &m);
  void handleTargetReport(const MissionCommand &m);
  void handleSetHome(const MissionCommand &m);

  // ==========================================================================
  // ★★ 无人机侦察结果处理（规则 image5/image6）★★
  // --------------------------------------------------------------------------
  // 无人机负责四类识别：地雷位置 · 打击区位置 · 障碍物 · 桥洞通道。
  // 协议层（mission_command.cpp）只做长度检查，**领域校验在这里做**，
  // 因为只有 MissionManager 持有 FieldGrid 的场地规格。
  //
  // 为什么校验必须严格：无人机端尚未实现，字段若有不一致，最可能的
  // 后果是"静默错误"（例如门位越界被当成合法行 -> 规划出撞墙的路径）。
  // 这里一律"校验不过就当没收"，并请求重发，绝不带病规划。
  // ==========================================================================
  void handleMineList(const MissionCommand &m);
  void handleObstacleList(const MissionCommand &m);
  void handleTargetList(const MissionCommand &m);
  void handleBridgeGate(const MissionCommand &m);
  void handleMapFull(const MissionCommand &m);

  // 把校验过的地雷/障碍喂给格子图
  void applyReconToGrid();
  // 用当前位置驱动桥洞过门判定，并在过门时上报 EVENT
  void tickBridgeCrossing();

  // ---- 状态迁移 ----
  void setState(State s, const std::string &reason);

  // ---- 导航 ----
  void sendNavGoal(double x, double y, double yaw, bool has_yaw);
  void cancelNavGoal();
  void onNavGoalResponse(
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle);
  static void onNavGoalResult(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &r);
  void checkArrival();
  double distanceToGoal() const;

  // ---- 反馈 ----
  void publishStatus();
  void sendReport(uint8_t cmd, const VehicleStatus &st);
  void sendTargetAck(uint8_t result, const MissionCommand &req);
  void sendFault(uint8_t fault, uint8_t detail);

  // ---- 回调 ----
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void dztrackCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void locHealthyCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  // ★ FIELD_FIXED（场地固定结构）—— 取无人机起降点并设为禁入格
  void fieldFixedCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // 话题回调与指令分派共用的唯一实现
  void applyFieldFixed(const uint8_t *d, size_t n);
  void handleFieldFixed(const MissionCommand &m);

  // ---- 工具 ----
  double normalizeAngle(double a) const;

  // =========================== 成员 ===========================
  std::unique_ptr<CmdLink> cmd_link_;
  std::unique_ptr<MotionArbiter> arbiter_;
  std::unique_ptr<SafetyMonitor> safety_;
  std::unique_ptr<TargetAimer> aimer_;
  std::unique_ptr<RefereeBoard> referee_;

  // ★ 桥洞过门判定（规则 image7：从通道通过即完成穿棱，每个区段 10 分）
  std::unique_ptr<BridgeTracker> bridge_;

  // ★ 格子图（场地拓扑 v5）。用于排雷规划、桥洞穿行、目标射击位计算。
  std::unique_ptr<FieldGrid> grid_;

  // ---- 无人机侦察结果缓存（校验通过后才应用） ----
  struct ReconPoint
  {
    int col{-1};
    int row{-1};
    int face{0};        // 仅目标列表用：0=未知 1=红(友) 2=蓝(敌)
    int confidence{0};
  };
  std::vector<ReconPoint> recon_mines_;
  std::vector<ReconPoint> recon_obstacles_;
  std::vector<ReconPoint> recon_targets_;
  uint32_t recon_mine_ts_{0};
  uint32_t recon_obstacle_ts_{0};
  uint32_t recon_target_ts_{0};
  uint32_t recon_mine_hash_{0};      // 内容去重：同一份数据不重复打日志/重规划
  uint32_t recon_obstacle_hash_{0};
  bool bridge_all_reported_{false};
  rclcpp::Time bridge_last_request_time_;
  int bridge_request_count_{0};

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_;
  std::atomic<bool> nav_goal_active_{false};
  std::atomic<bool> nav_goal_rejected_{false};

  // ---- 参数 ----
  bool enable_autonomy_{true};
  bool require_motion_enable_{false};
  bool motion_enabled_{true};
  double arrive_dist_{0.25};
  double arrive_yaw_tol_deg_{20.0};
  double goal_timeout_s_{60.0};
  double goal_retry_period_s_{3.0};
  double low_battery_pct_{20.0};
  double link_lost_return_delay_s_{5.0};
  bool auto_return_on_link_loss_{true};
  bool engage_requires_localization_{true};
  double engage_approach_dist_{1.2};   // 交战时保持的相对距离
  bool gimbal_lock_chassis_{true};     // 交战中锁定底盘
  double clear_recovery_delay_s_{1.0};
  double dztrack_active_timeout_s_{0.5};
  bool use_dztrack_when_active_{true};
  int default_ammo_when_link_lost_{0};

  // ---- 状态 ----
  mutable std::mutex state_mutex_;
  State state_{State::kDisabled};
  State resume_state_{State::kIdle};
  uint8_t fault_{kFaultNone};
  std::string last_reason_;

  // 故障位图：每一位对应一个活跃条件，按 tick 全量重算，
  // 避免"单一 fault_ 变量被不同条件互相覆盖/无法清除"
  enum FaultBit : uint32_t
  {
    kBitNone = 0u,
    kBitNoLocalization = 1u << 0,
    kBitObstacle = 1u << 1,
    kBitCmdLink = 1u << 2,
    kBitRefereeLink = 1u << 3,
    kBitNoAmmo = 1u << 4,
    kBitLowBattery = 1u << 5,
    kBitVisionLost = 1u << 6,
    // ★ 侦察信息不完整（无人机下发的地雷/障碍/桥洞/打击区有缺失或非法）
    //   典型：只收到 1 个桥洞门位 —— v5 拓扑下单门时起终点不可达
    kBitReconIncomplete = 1u << 7,
  };
  uint32_t fault_bits_{kBitNone};
  uint8_t last_fault_code_{kFaultNone};
  void setFaultBit(uint32_t bit, bool on);

  // 发送 EVENT 帧（协议 air::kEvent = 0x31）
  //   ev  见 air::EventCode（kEvBridgePassed 等）
  //   a0/a1/a2 事件参数，含义随 ev 而定
  bool sendEvent(uint8_t ev, uint8_t a0 = 0, uint8_t a1 = 0, uint8_t a2 = 0);
  void reportFaults();

  // 当前任务
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  bool goal_has_yaw_{false};
  bool goal_valid_{false};
  rclcpp::Time goal_sent_time_;
  rclcpp::Time goal_last_retry_time_;
  int goal_retry_count_{0};

  double home_x_{0.0};
  double home_y_{0.0};
  double home_yaw_{0.0};
  bool home_valid_{false};

  // 目标缓存（无人机 / 本地视觉）
  uint8_t target_id_{kTargetNone};
  uint8_t target_confidence_{0};
  double target_bearing_deg_{0.0};
  double target_range_m_{0.0};
  rclcpp::Time target_stamp_;
  bool target_valid_{false};

  // 本车状态
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double vel_v_{0.0};
  double vel_w_{0.0};
  bool pose_valid_{false};
  rclcpp::Time pose_stamp_;

  bool loc_healthy_{false};
  double battery_pct_{100.0};
  double dztrack_v_{0.0};
  double dztrack_w_{0.0};
  double nav2_v_{0.0};
  double nav2_w_{0.0};
  rclcpp::Time dztrack_stamp_;
  rclcpp::Time nav2_stamp_;
  bool nav2_seen_{false};
  rclcpp::Time teleop_stamp_;
  bool teleop_prev_active_{false};

  uint8_t tx_seq_{0};
  rclcpp::Time last_status_time_;
  rclcpp::Time last_fault_tx_time_;
  rclcpp::Time link_lost_since_;
  bool link_lost_reported_{false};
  bool link_was_alive_{false};
  rclcpp::Time estop_since_;
  int shots_at_engage_start_{0};
  int target_hp_at_engage_start_{0};

  // ---- 订阅 ----
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_dztrack_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav2_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_loc_healthy_;
  // ★ FIELD_FIXED：无人机起降点（禁入格）
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_field_;
  bool field_fixed_applied_{false};   // 是否已收到过 FIELD_FIXED（只提示一次）

  // ---- 发布 ----
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_fault_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_debug_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dzmission

#endif  // DZMISSION_MISSION_MANAGER_HPP
