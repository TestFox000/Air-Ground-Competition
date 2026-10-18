#include "dzmission/referee_board.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "serial/serial.h"

namespace dzmission
{

namespace
{
constexpr uint8_t kHdr1 = 0xB5;
constexpr uint8_t kHdr2 = 0x5B;
constexpr uint8_t kCmdAllMaterial = 0x02;
constexpr uint8_t kCmdEnemyMaterial = 0x03;
constexpr uint8_t kCmdSelfMaterial = 0x04;
constexpr uint8_t kCmdHpAndHit = 0x07;
constexpr uint8_t kCmdLaserShot = 0x21;
constexpr size_t kMaxValidDataLength = 100;
}  // namespace

RefereeBoard::RefereeBoard(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  (void)ns;
  last_rx_time_ = node_->now();
  last_hit_time_ = node_->now();
}

RefereeBoard::~RefereeBoard() { stop(); }

void RefereeBoard::loadParams()
{
  const std::string p = "referee.";

  node_->declare_parameter(p + "direct_serial", direct_serial_);
  node_->declare_parameter(p + "serial_port", serial_port_);
  node_->declare_parameter(p + "serial_baud", serial_baud_);
  node_->declare_parameter(p + "link_timeout_ms", link_timeout_ms_);
  node_->declare_parameter(p + "hit_hold_ms", hit_hold_ms_);
  node_->declare_parameter(p + "publish_laser_command", publish_laser_command_);

  node_->get_parameter(p + "direct_serial", direct_serial_);
  node_->get_parameter(p + "serial_port", serial_port_);
  node_->get_parameter(p + "serial_baud", serial_baud_);
  node_->get_parameter(p + "link_timeout_ms", link_timeout_ms_);
  node_->get_parameter(p + "hit_hold_ms", hit_hold_ms_);
  node_->get_parameter(p + "publish_laser_command", publish_laser_command_);

  sub_hp_hit_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "HpAndHitmsg", rclcpp::QoS(10),
      std::bind(&RefereeBoard::loadHpAndHit, this, std::placeholders::_1));

  sub_all_material_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "all_Material_Number", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr m) { loadMaterial(m, 0); });

  sub_self_material_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "self_Material_Number", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr m) { loadMaterial(m, 1); });

  sub_enemy_material_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "enemy_Material_Number", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr m) { loadMaterial(m, 2); });

  sub_laser_cmd_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "LaserShot_Command", rclcpp::QoS(10),
      std::bind(&RefereeBoard::laserCommandCallback, this, std::placeholders::_1));

  pub_self_hp_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/self_hp", 10);
  pub_target_hp_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/target_hp", 10);
  pub_ammo_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/ammo", 10);
  pub_hit_dir_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/hit_dir", 10);
  pub_shoot_count_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/shoot_count", 10);
  pub_being_hit_ = node_->create_publisher<std_msgs::msg::UInt8>("referee/being_hit", 10);
  pub_summary_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("referee/summary", 10);

  if (publish_laser_command_)
  {
    pub_laser_shot_ = node_->create_publisher<std_msgs::msg::UInt8>("LaserShot_Command", 10);
  }

  RCLCPP_INFO(node_->get_logger(),
              "[Referee] direct_serial=%d port=%s baud=%d link_timeout=%.0fms hit_hold=%.0fms",
              static_cast<int>(direct_serial_), serial_port_.c_str(), serial_baud_,
              link_timeout_ms_, hit_hold_ms_);
  if (direct_serial_)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[Referee] direct_serial=true —— 必须确保 dzjudgment 没有同时运行，"
                "否则串口会被争抢。");
  }
}

void RefereeBoard::start()
{
  if (direct_serial_)
  {
    if (running_.exchange(true))
    {
      return;
    }
    worker_ = std::thread(&RefereeBoard::serialLoop, this);
  }
}

void RefereeBoard::stop()
{
  if (!running_.exchange(false))
  {
    return;
  }
  if (worker_.joinable())
  {
    worker_.join();
  }
  std::lock_guard<std::mutex> lk(tx_mutex_);
  if (serial_handle_ != nullptr)
  {
    auto *ser = static_cast<serial::Serial *>(serial_handle_);
    try
    {
      if (ser->isOpen())
      {
        ser->close();
      }
    }
    catch (...)
    {
    }
    delete ser;
    serial_handle_ = nullptr;
  }
  link_ok_.store(false);
}

// -----------------------------------------------------------------------------
// 话题输入
// -----------------------------------------------------------------------------
void RefereeBoard::loadHpAndHit(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  // dzjudgment 的 0x07 帧顺序: [targethp, self_hp, Bing_hit, Shooting_Count, ammo]
  if (msg->data.size() < 5)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "[Referee] HpAndHitmsg size=%zu < 5, ignore", msg->data.size());
    return;
  }

  const uint8_t new_target_hp = msg->data[0];
  const uint8_t new_self_hp = msg->data[1];
  const uint8_t new_hit_dir = msg->data[2];
  const uint8_t new_shoot = msg->data[3];
  const uint8_t new_ammo = msg->data[4];

  const int prev_self_hp = self_hp_.load();

  target_hp_.store(new_target_hp);
  self_hp_.store(new_self_hp);
  shoot_count_.store(new_shoot);
  ammo_.store(new_ammo);
  hit_dir_.store(new_hit_dir);

  // 被击判定：hit_dir 落在 0x01..0x04 即表示被击，锁存 hit_hold_ms
  if (new_hit_dir >= 0x01 && new_hit_dir <= 0x04)
  {
    if (!hit_latched_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[Referee] BEING HIT dir=0x%02x self_hp %d -> %d",
                  new_hit_dir, prev_self_hp, new_self_hp);
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      hit_latched_ = true;
      last_hit_time_ = node_->now();
    }
    being_hit_.store(true);
  }
  else if (new_self_hp < prev_self_hp && prev_self_hp > 0)
  {
    // 有些裁判板固件只在 hp 变化时给出 hit_dir，这里做兜底
    {
      std::lock_guard<std::mutex> lk(mtx_);
      hit_latched_ = true;
      last_hit_time_ = node_->now();
    }
    being_hit_.store(true);
    RCLCPP_WARN(node_->get_logger(), "[Referee] hp drop detected %d -> %d",
                prev_self_hp, new_self_hp);
  }
  else
  {
    // 不要清 hit_latched_/being_hit_：被击锁存靠 hit_hold_ms 窗口过期，
    // 否则下一帧就会把锁存清掉（裁判板通常只在被击瞬间置位一次）。
    if (being_hit_.load() &&
        (node_->now() - last_hit_time_).seconds() * 1000.0 >= hit_hold_ms_)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      hit_latched_ = false;
    }
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_rx_time_ = node_->now();
  }
  link_ok_.store(true);

  publishSummary();
}

void RefereeBoard::loadMaterial(const std_msgs::msg::UInt8MultiArray::SharedPtr msg, int which)
{
  std::lock_guard<std::mutex> lk(mtx_);
  const size_t n = std::min(msg->data.size(), static_cast<size_t>(64));
  if (which == 0)
  {
    std::copy(msg->data.begin(), msg->data.begin() + static_cast<long>(n),
              all_material_.begin());
    all_material_len_ = n;
  }
  else if (which == 1)
  {
    std::copy(msg->data.begin(), msg->data.begin() + static_cast<long>(n),
              self_material_.begin());
    self_material_len_ = n;
  }
  else
  {
    std::copy(msg->data.begin(), msg->data.begin() + static_cast<long>(n),
              enemy_material_.begin());
    enemy_material_len_ = n;
  }
  last_rx_time_ = node_->now();
}

void RefereeBoard::laserCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 1 && direct_serial_)
  {
    sendLaserShot();
  }
}

// -----------------------------------------------------------------------------
// 被击判定（不加锁版本）：调用者必须已持有 mtx_
// -----------------------------------------------------------------------------
bool RefereeBoard::beingHitUnlocked() const
{
  if (!being_hit_.load())
  {
    return false;
  }
  // 注意：不清除 hit_latched_/being_hit_，由 hit_hold_ms 窗口自然过期。
  // 否则下一个 10ms/50Hz 帧（hit_dir 回到 0x10）就会把锁存清掉，
  // 导致被击状态只维持一帧，交战联锁形同虚设。
  return (node_->now() - last_hit_time_).seconds() * 1000.0 < hit_hold_ms_;
}

bool RefereeBoard::beingHit() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return beingHitUnlocked();
}

RefereeBoard::Status RefereeBoard::status() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  Status s;
  s.self_hp = static_cast<uint8_t>(self_hp_.load());
  s.target_hp = static_cast<uint8_t>(target_hp_.load());
  s.hit_dir = static_cast<uint8_t>(hit_dir_.load());
  s.shoot_count = static_cast<uint8_t>(shoot_count_.load());
  s.ammo = static_cast<uint8_t>(ammo_.load());
  s.being_hit = beingHitUnlocked();
  s.self_material = self_material_;
  s.enemy_material = enemy_material_;
  s.all_material = all_material_;
  s.self_material_len = self_material_len_;
  s.enemy_material_len = enemy_material_len_;
  s.all_material_len = all_material_len_;
  s.last_rx_age_s = (node_->now() - last_rx_time_).seconds();
  s.link_ok = direct_serial_ ? link_ok_.load()
                             : (s.last_rx_age_s * 1000.0 < link_timeout_ms_);
  return s;
}

void RefereeBoard::publishSummary()
{
  // 在本函数内只加锁一次：先快照全部状态，再在锁外发布。
  // （publishSummary 会被 loadHpAndHit / processData 在持锁状态下调用，
  //   若此处再次 lock 会死锁 —— std::mutex 不是递归锁。）
  uint8_t self_hp = 0;
  uint8_t target_hp = 0;
  uint8_t ammo = 0;
  uint8_t hit_dir = 0;
  uint8_t shoot_count = 0;
  uint8_t being_hit = 0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    self_hp = static_cast<uint8_t>(self_hp_.load());
    target_hp = static_cast<uint8_t>(target_hp_.load());
    ammo = static_cast<uint8_t>(ammo_.load());
    hit_dir = static_cast<uint8_t>(hit_dir_.load());
    shoot_count = static_cast<uint8_t>(shoot_count_.load());
    being_hit = beingHitUnlocked() ? 1 : 0;
  }

  std_msgs::msg::UInt8 m;
  m.data = self_hp;
  pub_self_hp_->publish(m);
  m.data = target_hp;
  pub_target_hp_->publish(m);
  m.data = ammo;
  pub_ammo_->publish(m);
  m.data = hit_dir;
  pub_hit_dir_->publish(m);
  m.data = shoot_count;
  pub_shoot_count_->publish(m);

  std_msgs::msg::UInt8 hb;
  hb.data = being_hit;
  pub_being_hit_->publish(hb);

  std_msgs::msg::UInt8MultiArray s;
  s.data.resize(6);
  s.data[0] = self_hp;
  s.data[1] = static_cast<uint8_t>(target_hp_.load());
  s.data[2] = static_cast<uint8_t>(ammo_.load());
  s.data[3] = static_cast<uint8_t>(hit_dir_.load());
  s.data[4] = static_cast<uint8_t>(shoot_count_.load());
  s.data[5] = hb.data;
  pub_summary_->publish(s);
}

// -----------------------------------------------------------------------------
// 直连串口：与 dzjudgment 相同的 0xB5 0x5B 帧协议
// -----------------------------------------------------------------------------
void RefereeBoard::serialLoop()
{
  auto *ser = new serial::Serial();
  serial_handle_ = ser;

  if (!reconnectSerial())
  {
    RCLCPP_ERROR(node_->get_logger(), "[Referee] open %s failed", serial_port_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[Referee] serial %s@%d opened",
              serial_port_.c_str(), serial_baud_);

  while (running_.load() && rclcpp::ok())
  {
    // 由 sendFrame 在写失败时置位的重连请求：在锁外执行 close/open，
    // 避免在持有 tx_mutex_ 的执行器线程里阻塞 30ms 串口超时。
    if (reconnect_requested_.exchange(false))
    {
      RCLCPP_WARN(node_->get_logger(), "[Referee] reconnect requested by writer");
      if (!reconnectSerial())
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }

    try
    {
      recvESP32Info();
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node_->get_logger(), "[Referee] serial error: %s", e.what());
      if (!reconnectSerial())
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(node_->get_logger(), "[Referee] serial loop exit");
}

bool RefereeBoard::reconnectSerial()
{
  auto *ser = static_cast<serial::Serial *>(serial_handle_);
  if (ser == nullptr)
  {
    return false;
  }
  try
  {
    if (ser->isOpen())
    {
      ser->close();
    }
    ser->setPort(serial_port_.c_str());
    ser->setBaudrate(static_cast<uint32_t>(serial_baud_));
    ser->setTimeout(serial::Timeout::simpleTimeout(30));
    ser->open();
    link_ok_.store(true);
    return true;
  }
  catch (const std::exception &e)
  {
    link_ok_.store(false);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "[Referee] reconnect failed: %s", e.what());
    return false;
  }
}

void RefereeBoard::recvESP32Info()
{
  auto *ser = static_cast<serial::Serial *>(serial_handle_);
  if (ser == nullptr)
  {
    return;
  }

  uint8_t header1 = 0;
  uint8_t header2 = 0;

  // 读操作与 sendFrame 的写操作共用同一把锁，避免并发读写同一个 fd
  // 导致 {0xB5,0x5B} 帧错位（最坏情况会死等凑不齐长度）。
  // 注意：本函数内部不得再调用会加锁的 sendFrame。
  std::lock_guard<std::mutex> lk(tx_mutex_);

  // 最多前移 64 字节做同步，避免无限循环
  int sync_limit = 64;

  if (previous_header2_ == kHdr2)
  {
    header1 = previous_header2_;
  }
  else
  {
    while (ser->available() > 0 && sync_limit-- > 0)
    {
      ser->read(&header1, 1);
      if (header1 == kHdr1)
      {
        break;
      }
    }
  }

  if (ser->available() > 0)
  {
    ser->read(&header2, 1);
    previous_header2_ = header2;
  }

  if (header1 != kHdr1 || header2 != kHdr2)
  {
    return;
  }

  uint8_t dataLength = 0;
  if (ser->available() < 1)
  {
    return;
  }
  ser->read(&dataLength, 1);
  if (dataLength == 0 || dataLength > kMaxValidDataLength)
  {
    previous_header2_ = 0;
    return;
  }
  if (ser->available() < static_cast<size_t>(dataLength))
  {
    return;
  }

  uint8_t frame[kMaxValidDataLength + 3] = {0};
  frame[0] = header1;
  frame[1] = header2;
  frame[2] = dataLength;
  ser->read(&frame[3], dataLength);

  processData(frame, dataLength);
}

void RefereeBoard::processData(const uint8_t *frame, uint8_t dataLength)
{
  if (frame[0] != kHdr1 || frame[1] != kHdr2)
  {
    return;
  }
  if (dataLength < 2)
  {
    return;
  }

  const uint8_t valid_len = static_cast<uint8_t>(dataLength - 2);
  const uint8_t type = frame[3];

  switch (type)
  {
    case kCmdAllMaterial:
    {
      std::lock_guard<std::mutex> lk(mtx_);
      all_material_len_ = std::min<size_t>(valid_len, 64);
      std::memcpy(all_material_.data(), frame + 4, all_material_len_);
      last_rx_time_ = node_->now();
      break;
    }
    case kCmdEnemyMaterial:
    {
      std::lock_guard<std::mutex> lk(mtx_);
      enemy_material_len_ = std::min<size_t>(valid_len, 64);
      std::memcpy(enemy_material_.data(), frame + 4, enemy_material_len_);
      last_rx_time_ = node_->now();
      break;
    }
    case kCmdSelfMaterial:
    {
      std::lock_guard<std::mutex> lk(mtx_);
      self_material_len_ = std::min<size_t>(valid_len, 64);
      std::memcpy(self_material_.data(), frame + 4, self_material_len_);
      last_rx_time_ = node_->now();
      break;
    }
    case kCmdHpAndHit:
    {
      if (valid_len < 5)
      {
        return;
      }
      const int prev_self_hp = self_hp_.load();
      target_hp_.store(frame[4]);
      self_hp_.store(frame[5]);
      hit_dir_.store(frame[6]);
      shoot_count_.store(frame[7]);
      ammo_.store(frame[8]);

      if (frame[6] >= 0x01 && frame[6] <= 0x04)
      {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          hit_latched_ = true;
          last_hit_time_ = node_->now();
        }
        being_hit_.store(true);
        if (prev_self_hp != frame[5])
        {
          RCLCPP_WARN(node_->get_logger(),
                      "[Referee] BEING HIT dir=0x%02x self_hp %d -> %d",
                      frame[6], prev_self_hp, frame[5]);
        }
      }
      else
      {
        // 同 loadHpAndHit：锁存交由 hit_hold_ms 窗口过期
        if (being_hit_.load() &&
            (node_->now() - last_hit_time_).seconds() * 1000.0 >= hit_hold_ms_)
        {
          std::lock_guard<std::mutex> lk(mtx_);
          hit_latched_ = false;
        }
      }

      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_rx_time_ = node_->now();
      }
      link_ok_.store(true);
      publishSummary();
      break;
    }
    default:
      break;
  }
}

bool RefereeBoard::sendLaserShot()
{
  // 与 dzjudgment::sendLaserShot() 完全一致的 6 字节帧
  const uint8_t buf[6] = {kHdr1, kHdr2, 0x03, kCmdLaserShot, 0x01, 0x25};
  return sendFrame(buf, sizeof(buf));
}

bool RefereeBoard::sendFrame(const uint8_t *buf, size_t len)
{
  std::lock_guard<std::mutex> lk(tx_mutex_);
  auto *ser = static_cast<serial::Serial *>(serial_handle_);
  if (ser == nullptr)
  {
    return false;
  }
  try
  {
    const size_t n = ser->write(buf, len);
    if (n != len)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "[Referee] laser shot write %zu/%zu bytes", n, len);
      return false;
    }
    // 本地回显，便于任务层确认开火动作已下发
    if (publish_laser_command_ && pub_laser_shot_)
    {
      std_msgs::msg::UInt8 m;
      m.data = 1;
      pub_laser_shot_->publish(m);
    }
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "[Referee] laser shot failed: %s", e.what());
    link_ok_.store(false);
    // 不在持锁状态下重连：交给串口线程执行
    reconnect_requested_.store(true);
    return false;
  }
}

}  // namespace dzmission
