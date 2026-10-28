// ROS-free adapter only. All controller decisions come from production files.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct FixtureSteadyClock {
  using duration = std::chrono::steady_clock::duration;
  using time_point = std::chrono::time_point<FixtureSteadyClock>;
  static time_point value;
  static time_point now() { return value; }
};
FixtureSteadyClock::time_point FixtureSteadyClock::value{};
double ros_seconds = 10.0;
constexpr int RCL_ROS_TIME = 1;
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
namespace rclcpp {
struct Duration {
  double value;
  static Duration from_seconds(double s) { return {s}; }
  double seconds() const { return value; }
};
struct Time {
  double value;
  Time(int sec = 0, unsigned nsec = 0, int = 0) : value(sec + nsec / 1e9) {}
  static Time from_seconds(double s) { Time t; t.value = s; return t; }
};
Duration operator-(Time a, Time b) { return {a.value - b.value}; }
Time operator+(Time a, Duration b) { return Time::from_seconds(a.value + b.value); }
bool operator>=(Time a, Time b) { return a.value >= b.value; }
struct Node {
  explicit Node(std::string) {}
  Time now() const { return Time::from_seconds(ros_seconds); }
};
template<class T> struct Subscription { using SharedPtr = std::shared_ptr<Subscription<T>>; };
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher<T>>;
  std::vector<T> messages;
  void publish(const T& msg) { messages.push_back(msg); }
};
}
namespace std_msgs { namespace msg {
struct UInt8 { using SharedPtr = std::shared_ptr<UInt8>; uint8_t data = 0; };
struct Int32 { using SharedPtr = std::shared_ptr<Int32>; int32_t data = 0; };
struct Float32 { using SharedPtr = std::shared_ptr<Float32>; float data = 0; };
struct Int32MultiArray { using SharedPtr = std::shared_ptr<Int32MultiArray>; std::vector<int32_t> data; };
}}
namespace sensor_msgs { namespace msg {
struct Imu { using SharedPtr = std::shared_ptr<Imu>; struct { double z = 0; } angular_velocity; };
}}
#include "production_header.h"
#include "production_callbacks.cpp"

// Construction wires fake publishers and starts deterministic clocks; it does
// not substitute any decision, callback, scan step, or controller parameter.
turn_on_robot::turn_on_robot(std::string name) : Node(name) {
  moveBaseControl = {};
  curYuntai_feedback_data = {};
  pub_LaserShot_Command = std::make_shared<rclcpp::Publisher<std_msgs::msg::UInt8>>();
  pub_shot_count = std::make_shared<rclcpp::Publisher<std_msgs::msg::Int32>>();
  pub_aim_status = std::make_shared<rclcpp::Publisher<std_msgs::msg::Int32>>();
  pub_voice_switch = std::make_shared<rclcpp::Publisher<std_msgs::msg::UInt8>>();
}
turn_on_robot::~turn_on_robot() = default;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
auto byte_msg(int value) {
  auto msg = std::make_shared<std_msgs::msg::UInt8>(); msg->data = value; return msg;
}
void vision(turn_on_robot& node, int x = 0, int cls = 1) {
  auto msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
  msg->data = {x, 0, cls, 80, 80}; node.callback_offset_center(msg);
}
void advance(int ms) { FixtureSteadyClock::value += std::chrono::milliseconds(ms); ros_seconds += ms / 1000.0; }
void reset_clock() { ros_seconds = 10.0; FixtureSteadyClock::value = FixtureSteadyClock::time_point(std::chrono::seconds(10)); }
void prepared(turn_on_robot& node, bool external = true) {
  node.aim_state_ = node.AIM_TRACK;
  node.last_vision_time_ = node.now();
  node.locked_cls_ = 1;
  node.target_seen_count_ = node.target_confirm_frames_;
  node.fire_hold_frames_ = 1;
  node.shot_cooldown_ms_ = 0;
  node.use_external_fire_gate_ = external;
}
void authorize(turn_on_robot& node) { node.callback_fire_gate(byte_msg(1)); node.callback_fire_ready(byte_msg(1)); }
void require_home(turn_on_robot& node) {
  node.CaremaMontorControl();
  require(node.aim_state_ == node.AIM_IDLE, "stop must remain IDLE");
  require(!node.scan_continuous_ && !node.scan_homing_, "stop must cancel continuous and pending scanning");
  require(node.moveBaseControl.Position_1 == node.gimbal_center_tick_, "real control must command forward yaw");
  require(node.moveBaseControl.Position_0 == node.gimbal_center_tick_ + std::lround(node.scanPitchDeg() * node.pitch_ticks_per_deg_ * node.pitch_deg_sign_), "real control must command configured pitch");
  require(node.moveBaseControl.Speed_1 > 0 && node.moveBaseControl.Speed_0 > 0, "home must issue nonzero motion speed");
}
int passed = 0, failed = 0;
void test(const char* name, const std::function<void()>& fn) {
  reset_clock();
  try { fn(); ++passed; std::cerr << "PASS " << name << '\n'; }
  catch (const std::exception& error) { ++failed; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
}
void home_tests() {
  test("HOME stop TRACK commands home", [] {
    turn_on_robot n("test"); prepared(n); n.scan_continuous_ = true;
    n.callback_scan_request(byte_msg(0)); require_home(n);
  });
  test("HOME stop SCAN commands home", [] {
    turn_on_robot n("test"); n.aim_state_ = n.AIM_SCAN;
    n.callback_scan_request(byte_msg(0)); require_home(n);
  });
  test("HOME stop pending homing before arrival", [] {
    turn_on_robot n("test"); n.callback_scan_request(byte_msg(2));
    n.callback_scan_request(byte_msg(0));
    n.curYuntai_feedback_data.Position_0 = n.curYuntai_feedback_data.Position_1 = n.gimbal_center_tick_;
    require_home(n);
  });
  test("HOME stop pending homing before timeout", [] {
    turn_on_robot n("test"); n.callback_scan_request(byte_msg(2));
    n.callback_scan_request(byte_msg(0)); advance(2000); require_home(n);
  });
  test("HOME repeated stop stays idle despite detections and silence", [] {
    turn_on_robot n("test"); prepared(n); n.callback_scan_request(byte_msg(0));
    for (int i = 0; i < 5; ++i) { vision(n); advance(2000); require_home(n); n.callback_scan_request(byte_msg(0)); }
    require(n.shots_fired_ == 0, "stopped vision must never fire or reenter TRACK");
  });
  test("HOME new explicit request may scan again", [] {
    turn_on_robot n("test"); prepared(n); n.callback_scan_request(byte_msg(0));
    n.callback_scan_request(byte_msg(2)); require(n.scan_homing_, "new request must enter homing");
    advance(2000); n.CaremaMontorControl(); require(n.aim_state_ == n.AIM_SCAN, "new request must enter SCAN after homing");
  });
  test("HOME nonzero request does not interrupt TRACK", [] {
    turn_on_robot n("test"); prepared(n); n.scan_continuous_ = false;
    n.callback_scan_request(byte_msg(2)); require(n.aim_state_ == n.AIM_TRACK && !n.scan_continuous_, "nonzero TRACK behavior must stay unchanged");
  });
  test("HOME repeated scan request retains progress", [] {
    turn_on_robot n("test"); n.aim_state_ = n.AIM_SCAN; n.scan_yaw_idx_ = 4; n.scan_yaw_dir_ = -1;
    n.callback_scan_request(byte_msg(2)); n.callback_scan_request(byte_msg(1));
    require(n.scan_yaw_idx_ == 4 && n.scan_yaw_dir_ == -1 && !n.scan_continuous_, "repeat scan must only update mode");
  });
  test("HOME continuous loss still resumes without homing", [] {
    turn_on_robot n("test"); prepared(n); n.scan_continuous_ = true; n.scan_yaw_dir_ = -1;
    advance(1500); n.CaremaMontorControl();
    require(n.aim_state_ == n.AIM_SCAN && !n.scan_homing_ && n.scan_yaw_dir_ == -1, "continuous loss must preserve existing resume behavior");
  });
}
void gate_tests() {
  test("GATE missing either permission blocks real fire", [] {
    for (int key = 0; key < 3; ++key) { turn_on_robot n("test"); prepared(n);
      if (key == 1) n.callback_fire_gate(byte_msg(1));
      if (key == 2) n.callback_fire_ready(byte_msg(1));
      vision(n); require(n.shots_fired_ == 0, "missing key must block shot"); }
  });
  test("GATE fresh pair emits one pulse and count", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); vision(n);
    require(n.shots_fired_ == 1 && n.shots_on_target_ == 1, "fresh pair must fire once");
    const auto& pulses = n.pub_LaserShot_Command->messages;
    require(pulses.size() == 2 && pulses[0].data == 1 && pulses[1].data == 0, "real publisher must emit 1 then 0");
    require(n.pub_shot_count->messages.size() == 1 && n.pub_shot_count->messages[0].data == 1, "real count publisher must account once");
  });
  test("GATE both streams stale fail closed", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); advance(501); vision(n);
    require(n.shots_fired_ == 0, "two old true values must expire");
  });
  test("GATE stale gate despite live ready fails closed", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); advance(501); n.callback_fire_ready(byte_msg(1)); vision(n);
    require(n.shots_fired_ == 0, "ready heartbeat must not refresh gate");
  });
  test("GATE stale ready despite live gate fails closed", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); advance(501); n.callback_fire_gate(byte_msg(1)); vision(n);
    require(n.shots_fired_ == 0, "gate heartbeat must not refresh ready");
  });
  test("GATE 499ms fresh and 500ms expired boundary", [] {
    turn_on_robot fresh("test"); prepared(fresh); authorize(fresh); advance(499); vision(fresh);
    require(fresh.shots_fired_ == 1, "499ms permissions must remain fresh");
    turn_on_robot expired("test"); prepared(expired); authorize(expired); advance(500); vision(expired);
    require(expired.shots_fired_ == 0, "500ms permissions must expire");
  });
  test("GATE heartbeats maintain authorization", [] {
    turn_on_robot n("test"); prepared(n);
    for (int i = 0; i < 20; ++i) { authorize(n); advance(100); }
    vision(n); require(n.shots_fired_ == 1, "fresh 10Hz heartbeat must allow firing");
  });
  test("GATE explicit false immediately revokes either key", [] {
    for (int key = 0; key < 2; ++key) { turn_on_robot n("test"); prepared(n); authorize(n);
      if (key == 0) n.callback_fire_gate(byte_msg(0)); else n.callback_fire_ready(byte_msg(0));
      vision(n); require(n.shots_fired_ == 0, "false must immediately block fire"); }
  });
  test("GATE false then true cannot inherit hold frames", [] {
    for (int key = 0; key < 2; ++key) { turn_on_robot n("test"); prepared(n); authorize(n);
      n.fire_hold_frames_ = 5; for (int i = 0; i < 4; ++i) vision(n);
      if (key == 0) n.callback_fire_gate(byte_msg(0)); else n.callback_fire_ready(byte_msg(0));
      authorize(n); vision(n); require(n.shots_fired_ == 0 && n.fire_hold_count_ == 1, "regrant must start a new hold streak"); }
  });
  test("GATE expiration observed by vision clears hold", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); n.fire_hold_frames_ = 5;
    for (int i = 0; i < 4; ++i) vision(n);
    advance(501); vision(n); require(n.shots_fired_ == 0 && n.fire_hold_count_ == 0, "expired frame must reset hold");
  });
  test("GATE stale refresh without vision cannot inherit hold", [] {
    for (int order = 0; order < 2; ++order) { turn_on_robot n("test"); prepared(n); authorize(n); n.fire_hold_frames_ = 5;
      for (int i = 0; i < 4; ++i) vision(n);
      advance(501);
      if (order == 0) authorize(n); else { n.callback_fire_ready(byte_msg(1)); n.callback_fire_gate(byte_msg(1)); }
      vision(n); require(n.shots_fired_ == 0 && n.fire_hold_count_ == 1, "refresh after silent expiration must start a new streak"); }
  });
  test("GATE ROS clock jumps cannot renew expired permissions", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); advance(501); ros_seconds = 100000;
    vision(n); require(n.shots_fired_ == 0, "ROS forward jump must not renew permission");
    ros_seconds = -100; n.shot_cooldown_ms_ = -200000; vision(n);
    require(n.shots_fired_ == 0, "ROS backward jump must not renew permission");
  });
  test("GATE external switch off preserves internal stability", [] {
    turn_on_robot stable("test"); prepared(stable, false); advance(1000); vision(stable);
    require(stable.shots_fired_ == 1, "disabled external mode must not require permission");
    turn_on_robot moving("test"); prepared(moving, false); moving.filt_inited_ = true; moving.dx_prev_filt_ = 30;
    vision(moving); require(moving.shots_fired_ == 0, "internal image stability must still block moving image");
  });
  test("GATE external mode retains its stability override", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); n.filt_inited_ = true; n.dx_prev_filt_ = 30;
    vision(n); require(n.shots_fired_ == 1, "fresh external pair substitutes only aim-ready as before");
  });
  test("GATE external switch off ignores permission revocations for hold", [] {
    turn_on_robot n("test"); prepared(n, false); n.fire_hold_frames_ = 5;
    for (int i = 0; i < 4; ++i) vision(n);
    n.callback_fire_gate(byte_msg(0)); n.callback_fire_ready(byte_msg(0));
    require(n.fire_hold_count_ == 4, "disabled external mode must retain internal hold on false callbacks");
    advance(501); authorize(n); vision(n);
    require(n.shots_fired_ == 1, "disabled external mode must ignore stale permission history");
  });
  test("GATE original fire gates still block under fresh permission", [] {
    for (int barrier = 0; barrier < 7; ++barrier) { turn_on_robot n("test"); prepared(n); authorize(n); int cls = 1, x = 0;
      switch (barrier) {
      case 0: x = 100; break;
      case 1: n.max_shots_ = 0; break;
      case 2: n.shots_fired_ = n.max_shots_; break;
      case 3: n.shots_on_target_ = n.max_shots_per_target_; break;
      case 4: n.locked_cls_ = 2; cls = 2; break;
      case 5: n.last_shot_time_ = n.now(); n.shot_cooldown_ms_ = 500; break;
      case 6: n.target_seen_count_ = 0; n.target_confirm_frames_ = 8; break;
      }
      const int before = n.shots_fired_; vision(n, x, cls);
      require(n.shots_fired_ == before && n.pub_LaserShot_Command->messages.empty(), "fresh permission must not bypass original fire barriers");
    }
  });
  test("GATE fire hold threshold remains enforced", [] {
    turn_on_robot n("test"); prepared(n); authorize(n); n.fire_hold_frames_ = 5;
    for (int i = 0; i < 4; ++i) vision(n);
    require(n.shots_fired_ == 0, "first four frames must not fire"); vision(n);
    require(n.shots_fired_ == 1 && n.fire_hold_count_ == 0, "fifth frame fires once and resets hold");
  });
}
int main(int argc, char** argv) {
  const std::string suite = argc > 1 ? argv[1] : "all";
  std::ostringstream logs;
  auto* original = std::cout.rdbuf(logs.rdbuf());
  if (suite == "home" || suite == "all") home_tests();
  if (suite == "gate" || suite == "all") gate_tests();
  std::cout.rdbuf(original);
  std::cout << "control contract: " << passed << " passed, " << failed << " failed\n";
  return failed ? 1 : 0;
}
