#include "dzmission/air_link.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace dzmission
{

namespace
{
constexpr const char *kNodeVersion = "v2-airlink";

double wallNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void put32(std::vector<uint8_t> &v, uint32_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}
uint32_t get32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
uint16_t get16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

AirLink::AirLink(rclcpp::Node *node, const std::string &ns) : node_(node)
{
  (void)ns;
  transport_ = std::make_unique<RawTransport>(node, ns);
  transport_->setRxCallback(
      [this](const uint8_t *d, size_t n) { onBytes(d, n); });
}

AirLink::~AirLink() { stop(); }

// -----------------------------------------------------------------------------
void AirLink::loadParams()
{
  const std::string p = "air.";

  node_->declare_parameter(p + "transport", transport_->transport_);
  node_->declare_parameter(p + "serial_port", transport_->serial_port_);
  node_->declare_parameter(p + "serial_baud", transport_->serial_baud_);
  node_->declare_parameter(p + "serial_read_timeout_ms",
                           transport_->serial_read_timeout_ms_);
  node_->declare_parameter(p + "udp_bind_host", transport_->udp_bind_host_);
  node_->declare_parameter(p + "udp_bind_port", transport_->udp_bind_port_);
  node_->declare_parameter(p + "udp_peer_host", transport_->udp_peer_host_);
  node_->declare_parameter(p + "udp_peer_port", transport_->udp_peer_port_);

  node_->declare_parameter(p + "heartbeat_period_s", heartbeat_period_s);
  node_->declare_parameter(p + "link_timeout_s", link_timeout_s);
  node_->declare_parameter(p + "ack_timeout_s", ack_timeout_s);
  node_->declare_parameter(p + "max_retries", max_retries);
  node_->declare_parameter(p + "log_tx", log_tx);
  node_->declare_parameter(p + "loopback_synth", loopback_synth);
  node_->declare_parameter(p + "loopback_synth_period_s",
                           loopback_synth_period_s);

  node_->declare_parameter(p + "field_up_row", static_cast<int>(field_up_row));
  node_->declare_parameter(p + "field_col_lo", static_cast<int>(field_col_lo));
  node_->declare_parameter(p + "field_col_hi", static_cast<int>(field_col_hi));
  node_->declare_parameter(p + "field_bridge_col",
                           static_cast<int>(field_bridge_col));

  node_->get_parameter(p + "transport", transport_->transport_);
  node_->get_parameter(p + "serial_port", transport_->serial_port_);
  node_->get_parameter(p + "serial_baud", transport_->serial_baud_);
  node_->get_parameter(p + "serial_read_timeout_ms",
                       transport_->serial_read_timeout_ms_);
  node_->get_parameter(p + "udp_bind_host", transport_->udp_bind_host_);
  node_->get_parameter(p + "udp_bind_port", transport_->udp_bind_port_);
  node_->get_parameter(p + "udp_peer_host", transport_->udp_peer_host_);
  node_->get_parameter(p + "udp_peer_port", transport_->udp_peer_port_);

  node_->get_parameter(p + "heartbeat_period_s", heartbeat_period_s);
  node_->get_parameter(p + "link_timeout_s", link_timeout_s);
  node_->get_parameter(p + "ack_timeout_s", ack_timeout_s);
  node_->get_parameter(p + "max_retries", max_retries);
  node_->get_parameter(p + "log_tx", log_tx);
  node_->get_parameter(p + "loopback_synth", loopback_synth);
  node_->get_parameter(p + "loopback_synth_period_s", loopback_synth_period_s);

  int tmp = 0;
  node_->get_parameter(p + "field_up_row", tmp);
  field_up_row = static_cast<uint8_t>(tmp);
  node_->get_parameter(p + "field_col_lo", tmp);
  field_col_lo = static_cast<uint8_t>(tmp);
  node_->get_parameter(p + "field_col_hi", tmp);
  field_col_hi = static_cast<uint8_t>(tmp);
  node_->get_parameter(p + "field_bridge_col", tmp);
  field_bridge_col = static_cast<uint8_t>(tmp);

  transport_->setLogTx(log_tx);
}

// -----------------------------------------------------------------------------
void AirLink::start()
{
  // 发布器（QoS 与车端其他节点一致：BEST_EFFORT，避免与发布端不匹配时静默丢消息）
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  pub_hb_ = node_->create_publisher<std_msgs::msg::String>("air/heartbeat", qos);
  pub_err_ = node_->create_publisher<std_msgs::msg::String>("air/last_error", 5);
  pub_alive_ = node_->create_publisher<std_msgs::msg::Bool>("air/link_alive", 5);
  pub_gate_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/bridge_gate", 20);
  pub_mine_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/mine_list", 10);
  pub_obs_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "air/obstacle_list", 10);
  pub_tgt_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/target_list", 10);
  pub_map_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/map_full", 5);
  pub_field_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/field_fixed", 5);
  pub_task_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/task_cmd", 20);
  pub_ev_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/event_rx", 20);
  pub_ack_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/ack_rx", 20);
  pub_raw_ =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>("air/raw", 20);

  transport_->start();

  const int period_ms =
      std::max(10, static_cast<int>(heartbeat_period_s * 1000.0 / 5.0));
  tick_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AirLink::tickLoop, this));

  RCLCPP_INFO(node_->get_logger(),
              "[AirLink %s] transport=%s 心跳%.2fs 链路超时%.2fs ACK超时%.2fs 重试%d",
              kNodeVersion, transport_->transport().c_str(), heartbeat_period_s,
              link_timeout_s, ack_timeout_s, max_retries);
  RCLCPP_INFO(node_->get_logger(),
              "[AirLink] 帧格式 A5C3 VER=0x%02X LEN<=%zu 最大帧%zu B；"
              "CRC-16/CCITT-FALSE",
              air::kVer, air::kMaxPayload, air::kMaxFrameLen);
}

void AirLink::stop()
{
  if (transport_)
  {
    transport_->stop();
  }
  tick_timer_.reset();
}

// -----------------------------------------------------------------------------
// 接收：字节流 -> 帧同步 -> 解码分发
// -----------------------------------------------------------------------------
void AirLink::onBytes(const uint8_t *data, size_t len)
{
  sync_.feed(data, len, [this](const air::Decoded &d) { onFrame(d); });
}

void AirLink::publishMulti(const std::string &topic, const std::vector<uint8_t> &v)
{
  std_msgs::msg::UInt8MultiArray m;
  m.data = v;
  if (topic == "gate" && pub_gate_) pub_gate_->publish(m);
  else if (topic == "mine" && pub_mine_) pub_mine_->publish(m);
  else if (topic == "obs" && pub_obs_) pub_obs_->publish(m);
  else if (topic == "tgt" && pub_tgt_) pub_tgt_->publish(m);
  else if (topic == "map" && pub_map_) pub_map_->publish(m);
  else if (topic == "field" && pub_field_) pub_field_->publish(m);
  else if (topic == "task" && pub_task_) pub_task_->publish(m);
  else if (topic == "ev" && pub_ev_) pub_ev_->publish(m);
  else if (topic == "ack" && pub_ack_) pub_ack_->publish(m);
}

void AirLink::publishRaw(const std::vector<uint8_t> &frame)
{
  if (pub_raw_)
  {
    std_msgs::msg::UInt8MultiArray m;
    m.data = frame;
    pub_raw_->publish(m);
  }
}

void AirLink::onFrame(const air::Decoded &d)
{
  rx_frames_.fetch_add(1);
  last_rx_wall_.store(wallNow());

  // 原始帧（调试用）：重新组帧仅供观察
  {
    std::vector<uint8_t> raw = air::encodeFrame(d.typ, d.seq, d.payload);
    publishRaw(raw);
  }

  const uint8_t *p = d.payload.data();
  const size_t n = d.payload.size();

  switch (d.typ)
  {
    case air::kHeartbeat:
    {
      if (n < 8) break;
      const uint32_t ts = get32(p);
      const uint16_t mv = get16(p + 4);
      std_msgs::msg::String s;
      s.data = "ts=" + std::to_string(ts) + " map_version=" + std::to_string(mv) +
               " quality=" + std::to_string(p[6]) +
               " flags=0x" + std::to_string(p[7]);
      if (pub_hb_) pub_hb_->publish(s);
      break;
    }

    case air::kBridgeGate:
    {
      if (n < 4) break;
      BridgeGate g;
      g.bridge_id = p[0];
      g.gate_row = p[1];
      g.confidence = p[2];
      g.flags = p[3];
      // 越界校验：bridge_id 必须 1/2；gate_row 必须落在对应区段
      const bool id_ok = (g.bridge_id == 1 || g.bridge_id == 2);
      const bool row_ok = (g.bridge_id == 1) ? (g.gate_row <= 5)
                                             : (g.gate_row >= 6 && g.gate_row <= 10);
      g.valid = id_ok && row_ok;
      {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if (g.bridge_id == 1) gate_a_ = g;
        else if (g.bridge_id == 2) gate_b_ = g;
      }
      if (!g.valid)
      {
        std::lock_guard<std::mutex> lk(err_mutex_);
        last_error_ = "BRIDGE_GATE 越界: bridge_id=" + std::to_string(g.bridge_id) +
                      " gate_row=" + std::to_string(g.gate_row);
        if (pub_err_) { std_msgs::msg::String s; s.data = last_error_; pub_err_->publish(s); }
        RCLCPP_ERROR(node_->get_logger(), "[AirLink] %s", last_error_.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(),
                    "[AirLink] 收到门位 bridge_id=%u -> K%u (置信度 %u)",
                    g.bridge_id, static_cast<unsigned>(g.gate_row + 1),
                    g.confidence);
      }
      publishMulti("gate", d.payload);
      break;
    }

    case air::kMineList:
    {
      publishMulti("mine", d.payload);
      if (n >= 5)
      {
        RCLCPP_INFO(node_->get_logger(), "[AirLink] 地雷清单 %u 颗 (ts=%u)", p[4],
                    get32(p));
      }
      break;
    }

    case air::kObstacleList:
    {
      // ★ 本条没有 ts_ms，首字节就是 count
      publishMulti("obs", d.payload);
      if (n >= 1)
      {
        RCLCPP_INFO(node_->get_logger(), "[AirLink] 障碍清单 %u 个", p[0]);
      }
      break;
    }

    case air::kTargetList:
    {
      publishMulti("tgt", d.payload);
      if (n >= 5)
      {
        RCLCPP_INFO(node_->get_logger(), "[AirLink] 标靶清单 %u 格", p[4]);
      }
      break;
    }

    case air::kMapFull:
    {
      publishMulti("map", d.payload);
      if (n >= 10)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "[AirLink] 全场地图 ver=%u %ux%u %zu 格", get32(p),
                    static_cast<unsigned>(p[8]), static_cast<unsigned>(p[9]),
                    (n - 10) / 2);
      }
      break;
    }

    case air::kFieldFixed:
    {
      publishMulti("field", d.payload);
      break;
    }

    case air::kTaskCmd:
    {
      if (n >= 2)
      {
        TaskCommand tc;
        tc.cmd = p[0];
        tc.cmd_seq = p[1];
        tc.extra.assign(p + 2, p + n);
        tc.valid = true;
        static const char *kNames[] = {
            "NOP", "NAV_TO_CELL", "NAV_TO_POSE", "MINE_CLEAR_ALL",
            "MINE_CLEAR_ONE", "BRIDGE_CROSS", "ENGAGE", "HOLD",
            "STOP", "RETURN_BASE", "SAFE_RETREAT"};
        tc.name = (tc.cmd <= 10) ? kNames[tc.cmd] : "?";
        {
          std::lock_guard<std::mutex> lk(data_mutex_);
          last_task_ = tc;
        }
        last_task_seq_.store(tc.cmd_seq);
        RCLCPP_INFO(node_->get_logger(), "[AirLink] 任务指令 %s(0x%02X) seq=%u extra=%zu B",
                    tc.name.c_str(), tc.cmd, tc.cmd_seq, tc.extra.size());
      }
      publishMulti("task", d.payload);
      break;
    }

    case air::kEvent:
    {
      publishMulti("ev", d.payload);
      if (n >= 8)
      {
        RCLCPP_INFO(node_->get_logger(), "[AirLink] 事件 0x%02X args=(%u,%u,%u)",
                    p[4], p[5], p[6], p[7]);
      }
      break;
    }

    case air::kAck:
    {
      publishMulti("ack", d.payload);
      if (n >= 4)
      {
        const uint32_t ack_seq = get32(p);
        // 从待确认队列移除
        std::lock_guard<std::mutex> lk(pend_mutex_);
        pending_.erase(
            std::remove_if(pending_.begin(), pending_.end(),
                           [ack_seq](const Pending &q) { return q.seq == ack_seq; }),
            pending_.end());
      }
      break;
    }

    case air::kTeamInfo:
    case air::kMapDelta:
    case air::kRoute:
    default:
    {
      // 未知 TYP 必须**丢弃而不是报错/断链**（协议向前兼容要求）
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// 发送
// -----------------------------------------------------------------------------
bool AirLink::sendFrameTyp(uint8_t typ, const std::vector<uint8_t> &payload,
                           bool need_ack)
{
  if (!transport_)
  {
    return false;
  }
  const uint32_t seq = nextSeq();
  const std::vector<uint8_t> frame = air::encodeFrame(typ, seq, payload);
  if (frame.empty())
  {
    std::lock_guard<std::mutex> lk(err_mutex_);
    last_error_ = "组帧失败（载荷过长 " + std::to_string(payload.size()) + " B）";
    return false;
  }
  const bool ok = transport_->send(frame.data(), frame.size());
  if (ok)
  {
    tx_frames_.fetch_add(1);
    if (need_ack)
    {
      std::lock_guard<std::mutex> lk(pend_mutex_);
      Pending q;
      q.frame = frame;
      q.seq = seq;
      q.typ = typ;
      q.sent_at = wallNow();
      pending_.push_back(q);
    }
  }
  return ok;
}

bool AirLink::sendHeartbeat()
{
  std::vector<uint8_t> pl;
  put32(pl, static_cast<uint32_t>(node_->now().nanoseconds() / 1000000));
  pl.push_back(static_cast<uint8_t>(0));   // map_version 低
  pl.push_back(static_cast<uint8_t>(0));   // map_version 高
  pl.push_back(100);                       // quality
  pl.push_back(0);                         // flags
  return sendFrameTyp(air::kHeartbeat, pl, false);
}

bool AirLink::sendVehicleStatus(const std::vector<uint8_t> &payload32)
{
  if (payload32.size() != 32)
  {
    std::lock_guard<std::mutex> lk(err_mutex_);
    last_error_ = "VEH_STATUS 载荷必须是 32 字节，收到 " +
                  std::to_string(payload32.size());
    return false;
  }
  return sendFrameTyp(air::kVehStatus, payload32, false);
}

bool AirLink::sendEvent(uint8_t code, uint8_t a0, uint8_t a1, uint8_t a2)
{
  std::vector<uint8_t> pl;
  put32(pl, static_cast<uint32_t>(node_->now().nanoseconds() / 1000000));
  pl.push_back(code);
  pl.push_back(a0);
  pl.push_back(a1);
  pl.push_back(a2);
  return sendFrameTyp(air::kEvent, pl, false);
}

bool AirLink::sendAck(uint32_t ack_seq, uint8_t status, uint8_t detail)
{
  std::vector<uint8_t> pl;
  put32(pl, ack_seq);
  pl.push_back(status);
  pl.push_back(detail);
  pl.push_back(0);
  pl.push_back(0);
  return sendFrameTyp(air::kAck, pl, false);
}

void AirLink::injectFrame(const std::vector<uint8_t> &frame)
{
  if (transport_)
  {
    transport_->injectRx(frame.data(), frame.size());
  }
}

// -----------------------------------------------------------------------------
void AirLink::tickLoop()
{
  const double now = wallNow();

  // 1) 周期心跳
  static double last_hb = 0.0;
  if (now - last_hb >= heartbeat_period_s)
  {
    last_hb = now;
    sendHeartbeat();
  }

  // 2) ACK 超时重发
  {
    std::vector<Pending> resend;
    {
      std::lock_guard<std::mutex> lk(pend_mutex_);
      for (auto it = pending_.begin(); it != pending_.end();)
      {
        if (now - it->sent_at >= ack_timeout_s)
        {
          if (it->retries >= max_retries)
          {
            std::lock_guard<std::mutex> lk2(err_mutex_);
            last_error_ = "消息 TYP=0x" + std::to_string(it->typ) +
                          " 重试 " + std::to_string(it->retries) + " 次未收到 ACK";
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[AirLink] %s", last_error_.c_str());
            it = pending_.erase(it);
          }
          else
          {
            ++it->retries;
            it->sent_at = now;
            resend.push_back(*it);
            ++it;
          }
        }
        else
        {
          ++it;
        }
      }
    }
    for (const auto &q : resend)
    {
      if (transport_->send(q.frame.data(), q.frame.size()))
      {
        tx_frames_.fetch_add(1);
        retransmits_.fetch_add(1);
      }
    }
  }

  // 3) 链路存活判定 + 变化时发布
  const bool alive = linkAlive();
  if (alive != alive_reported_)
  {
    alive_reported_ = alive;
    std_msgs::msg::Bool b;
    b.data = alive;
    if (pub_alive_) pub_alive_->publish(b);
    if (alive)
    {
      RCLCPP_INFO(node_->get_logger(), "[AirLink] 链路恢复");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[AirLink] 链路断开（%.2fs 无合法帧）", lastRxAge());
    }
  }

  // 4) loopback 合成帧（离线自测）
  if (loopback_synth)
  {
    loopback_acc_ += heartbeat_period_s;
    if (loopback_acc_ >= loopback_synth_period_s)
    {
      loopback_acc_ = 0.0;
      synthesizeLoopbackFrame();
    }
  }
}

// -----------------------------------------------------------------------------
// 离线自测：注入一个**完整且合法**的场景
//   两个门位（K4 + K9）+ 8 颗地雷 + 8 个障碍 + 1 个标靶 + 一条 NAV_TO_CELL
//   —— 与 test/topology_v5_check.py 的参考门位一致，便于离线验证发车闸门。
// -----------------------------------------------------------------------------
void AirLink::synthesizeLoopbackFrame()
{
  // 用固定序列号，避免每次刷新
  static uint32_t synth_seq = 1000;

  auto inject = [&](uint8_t typ, const std::vector<uint8_t> &pl) {
    const auto f = air::encodeFrame(typ, synth_seq++, pl);
    injectFrame(f);
  };

  // 门位：区段一 K4（row 3）、区段二 K9（row 8）
  inject(air::kBridgeGate, air::enc::bridgeGate(1, 3, 100, 0x01));
  inject(air::kBridgeGate, air::enc::bridgeGate(2, 8, 100, 0x01));

  // 地雷 8 颗（西侧为主）
  inject(air::kMineList,
         air::enc::mineList(1000, {{3, 2, 90}, {6, 4, 88}, {11, 1, 85},
                                   {4, 7, 92}, {9, 8, 80}, {0, 3, 95},
                                   {8, 9, 70}, {5, 2, 88}}));

  // 障碍 8 个
  inject(air::kObstacleList,
         air::enc::obstacleList({{1, 1, 1}, {7, 5, 2}, {13, 10, 1}, {2, 6, 2},
                                 {10, 3, 1}, {6, 8, 2}, {0, 9, 1}, {9, 4, 2}}));

  // 标靶（打击区两格，face 未知交给车端）
  inject(air::kTargetList,
         air::enc::targetList(1000, {{6, 4, 0, 0}, {6, 5, 0, 0}}));

  // 固定结构（车端已固化，这里下发用于交叉校验）
  inject(air::kFieldFixed,
         air::enc::fieldFixed(field_up_row, field_col_lo, field_col_hi,
                              field_bridge_col, 0, 5, 6, 10));

  // 心跳
  inject(air::kHeartbeat, air::enc::heartbeat(1000, 1, 100, 0x04));

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "[AirLink] loopback 合成场景已注入"
                       "（门位 K4+K9 / 8 雷 / 8 障碍 / 标靶 / 固定结构）");
}

// -----------------------------------------------------------------------------
bool AirLink::linkAlive() const
{
  const double t = last_rx_wall_.load();
  if (t < 0.0)
  {
    return false;
  }
  return (wallNow() - t) <= link_timeout_s;
}

double AirLink::lastRxAge() const
{
  const double t = last_rx_wall_.load();
  if (t < 0.0)
  {
    return 1e9;
  }
  return wallNow() - t;
}

AirLink::Stats AirLink::stats() const
{
  Stats s;
  s.rx_frames = rx_frames_.load();
  s.rx_errors = rx_errors_.load();
  s.tx_frames = tx_frames_.load();
  s.retransmits = retransmits_.load();
  s.last_rx_age_s = lastRxAge();
  s.link_alive = linkAlive();
  {
    std::lock_guard<std::mutex> lk(err_mutex_);
    s.last_error = last_error_;
  }
  if (transport_)
  {
    s.rx_bytes = transport_->rxBytes();
    s.transport = transport_->transport();
  }
  return s;
}

bool AirLink::gate(uint8_t bridge_id, BridgeGate &out) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  if (bridge_id == 1 && gate_a_.valid)
  {
    out = gate_a_;
    return true;
  }
  if (bridge_id == 2 && gate_b_.valid)
  {
    out = gate_b_;
    return true;
  }
  return false;
}

bool AirLink::hasBothGates() const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  return gate_a_.valid && gate_b_.valid;
}

AirLink::TaskCommand AirLink::lastTaskCommand() const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  return last_task_;
}

}  // namespace dzmission
