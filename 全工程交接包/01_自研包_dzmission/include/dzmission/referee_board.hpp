#ifndef DZMISSION_REFEREE_BOARD_HPP
#define DZMISSION_REFEREE_BOARD_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace dzmission
{

// =============================================================================
// RefereeBoard —— 车载裁判板 / 裁判系统交互
// -----------------------------------------------------------------------------
// 说明：
//   本节点【不直接打开串口】，只订阅 dzjudgment 已经解析好的话题，避免两个
//   进程争抢 /dev/dzjudgment。这样做的前提是 dzjudgment 已在运行；
//   若裁判板需要由本包独占（例如不启动 dzjudgment），把参数
//     referee.direct_serial 设为 true，并确认 dzjudgment 没有同时运行。
//
// 订阅（来自 dzjudgment）：
//   /HpAndHitmsg             [targethp, self_hp, Bing_hit, Shooting_Count, ammo]
//   /all_Material_Number     [物资点编号...]
//   /self_Material_Number    [己方已获得物资...]
//   /enemy_Material_Number   [敌方已获得物资...]
//
// 发布：
//   /referee/self_hp         UInt8
//   /referee/target_hp       UInt8
//   /referee/ammo            UInt8
//   /referee/hit_dir         UInt8   0x01左 0x02前 0x03右 0x04后 0x10未中弹
//   /referee/shoot_count     UInt8
//   /referee/being_hit       UInt8   1=最近 hit_hold_ms 内被击打
//   /referee/summary         UInt8MultiArray [self_hp, target_hp, ammo, hit_dir, shoot_count, flags]
//   /LaserShot_Command       UInt8   （仅用于"自研裁判板/仿真"模式下的直连回放）
//
// 直连模式（direct_serial=true）：
//   与 dzjudgment 相同的 0xB5 0x5B 帧协议：收 0x02/0x03/0x04/0x07，
//   发 0x21 开火。这样即使没有 dzjudgment 也能与裁判板闭环。
// =============================================================================
class RefereeBoard
{
public:
  struct Status
  {
    uint8_t self_hp{0};
    uint8_t target_hp{0};
    uint8_t hit_dir{0x10};
    uint8_t shoot_count{0};
    uint8_t ammo{0};
    bool being_hit{false};
    bool link_ok{false};
    double last_rx_age_s{1e9};
    std::array<uint8_t, 64> self_material{};
    std::array<uint8_t, 64> enemy_material{};
    std::array<uint8_t, 64> all_material{};
    size_t self_material_len{0};
    size_t enemy_material_len{0};
    size_t all_material_len{0};
  };

  RefereeBoard(rclcpp::Node *node, const std::string &ns);
  ~RefereeBoard();

  RefereeBoard(const RefereeBoard &) = delete;
  RefereeBoard &operator=(const RefereeBoard &) = delete;

  void loadParams();
  void start();
  void stop();

  Status status() const;

  int ammo() const { return ammo_.load(); }
  int selfHp() const { return self_hp_.load(); }
  bool beingHit() const;

  // 直连模式下：转义层开火请求（由 LaserShot_Command 订阅触发）
  bool sendLaserShot();

private:
  void loadHpAndHit(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void loadMaterial(const std_msgs::msg::UInt8MultiArray::SharedPtr msg, int which);
  void laserCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  // 必须在持有 mtx_ 的情况下调用，避免 std::mutex 非递归导致的死锁
  bool beingHitUnlocked() const;

  void serialLoop();
  void recvESP32Info();
  void processData(const uint8_t *frame, uint8_t dataLength);
  bool sendFrame(const uint8_t *buf, size_t len);
  bool reconnectSerial();

  void publishSummary();

  rclcpp::Node *node_{nullptr};

  // ---- 参数 ----
  bool direct_serial_{false};
  std::string serial_port_{"/dev/dzjudgment"};
  int serial_baud_{115200};
  double link_timeout_ms_{1000.0};
  double hit_hold_ms_{1500.0};
  bool publish_laser_command_{false};

  // ---- 订阅 ----
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_hp_hit_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_all_material_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_self_material_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_enemy_material_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_laser_cmd_;

  // ---- 发布 ----
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_self_hp_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_target_hp_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_ammo_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hit_dir_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_shoot_count_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_being_hit_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_summary_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_laser_shot_;

  // ---- 状态 ----
  mutable std::mutex mtx_;
  std::atomic<int> self_hp_{0};
  std::atomic<int> target_hp_{0};
  std::atomic<int> ammo_{0};
  std::atomic<int> shoot_count_{0};
  std::atomic<int> hit_dir_{0x10};
  std::atomic<bool> being_hit_{false};
  std::atomic<bool> link_ok_{false};

  rclcpp::Time last_rx_time_;
  rclcpp::Time last_hit_time_;
  bool hit_latched_{false};

  std::array<uint8_t, 64> self_material_{};
  std::array<uint8_t, 64> enemy_material_{};
  std::array<uint8_t, 64> all_material_{};
  size_t self_material_len_{0};
  size_t enemy_material_len_{0};
  size_t all_material_len_{0};

  // ---- 串口线程 ----
  std::atomic<bool> running_{false};
  std::atomic<bool> reconnect_requested_{false};
  std::thread worker_;
  void *serial_handle_{nullptr};
  std::mutex tx_mutex_;
  uint8_t previous_header2_{0};
};

}  // namespace dzmission

#endif  // DZMISSION_REFEREE_BOARD_HPP
