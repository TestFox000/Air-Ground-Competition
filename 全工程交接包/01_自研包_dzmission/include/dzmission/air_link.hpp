#ifndef DZMISSION_AIR_LINK_HPP
#define DZMISSION_AIR_LINK_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "dzmission/air_protocol.hpp"
#include "dzmission/raw_transport.hpp"

namespace dzmission
{

// =============================================================================
// AirLink —— 空地协同协议 v2 的链路层（车端接收端）
// -----------------------------------------------------------------------------
// 分层：
//   RawTransport  —— 纯字节收发（串口 / UDP / loopback）
//   AirLink       —— 帧同步 + 载荷解码 + 心跳/ACK/重发 + 高层话题分发
//   MissionManager—— 消费高层话题，驱动任务
//
// 为什么不让 CmdLink 兼任：
//   旧 CmdLink 把「传输 + v1 帧校验 + 解析 + 发话题」揉在一起，帧格式一换
//   整类都要动。拆开后传输可复用，协议可独立测试（air_protocol_selftest）。
//
// ★ 本实现必须与 test/air_protocol.py（无人机端已对撞通过）逐字节一致。
//
// 上行（无人机 → 车）话题：
//   /air/heartbeat      String   文本摘要，便于 ros2 topic echo 观察
//   /air/bridge_gate    UInt8MultiArray  [bridge_id, gate_row, confidence, flags]
//   /air/mine_list      UInt8MultiArray  [ts_ms(4) | count | (col,row,conf)*]
//   /air/obstacle_list  UInt8MultiArray  [count | (col,row,color)*]
//   /air/target_list    UInt8MultiArray  [ts_ms(4) | count | (col,row,face,conf)*]
//   /air/map_full       UInt8MultiArray  [map_ver(4) ts(4) cols rows | cell*2]
//   /air/field_fixed    UInt8MultiArray  固定结构参数
//   /air/task_cmd       UInt8MultiArray  [cmd, cmd_seq | extra]
//   /air/event_rx       UInt8MultiArray  [ts(4) ev a0 a1 a2]
//   /air/ack_rx         UInt8MultiArray  [ack_seq(4) status detail]
//   /air/raw            UInt8MultiArray  原始整帧（调试）
//
// 链路状态：
//   /air/link_alive     Bool
//   /air/rx_count       UInt8（取低 8 位，仅示意）
//   /air/last_error     String
// =============================================================================
class AirLink
{
public:
  // ---- 解码后的事件（供 MissionManager 直接消费，避免再解一次）----
  struct BridgeGate
  {
    uint8_t bridge_id{0};      // 1 = 区段一 K1-K6，2 = 区段二 K7-K11
    uint8_t gate_row{0};
    uint8_t confidence{0};
    uint8_t flags{0};
    bool valid{false};
  };

  struct MineEntry
  {
    uint8_t col{0};
    uint8_t row{0};
    uint8_t confidence{0};
  };

  struct ObstacleEntry
  {
    uint8_t col{0};
    uint8_t row{0};
    uint8_t color{0};
  };

  struct TargetEntry
  {
    uint8_t col{0};
    uint8_t row{0};
    uint8_t face{0};        // 0=未知 1=红/友 2=蓝/敌
    uint8_t confidence{0};
  };

  struct TaskCommand
  {
    uint8_t cmd{0};
    uint8_t cmd_seq{0};
    std::vector<uint8_t> extra;
    bool valid{false};
    std::string name;
  };

  struct Received
  {
    uint32_t ts_ms{0};
    uint8_t code{0};
    uint8_t a0{0};
    uint8_t a1{0};
    uint8_t a2{0};
  };

  struct Stats
  {
    uint64_t rx_frames{0};
    uint64_t rx_errors{0};
    uint64_t rx_bytes{0};
    uint64_t tx_frames{0};
    uint64_t retransmits{0};
    double last_rx_age_s{1e9};
    bool link_alive{false};
    std::string last_error;
    std::string transport;
  };

  AirLink(rclcpp::Node *node, const std::string &ns);
  ~AirLink();

  AirLink(const AirLink &) = delete;
  AirLink &operator=(const AirLink &) = delete;

  void loadParams();
  void start();
  void stop();

  // 链路是否存活（最近 link_timeout_ms 内有合法帧）
  bool linkAlive() const;
  // 链路是否就绪（串口/UDP 已打开）
  bool linkReady() const { return transport_ && transport_->ready(); }
  double lastRxAge() const;

  // ---- 发送 API ----
  bool sendHeartbeat();
  bool sendVehicleStatus(const std::vector<uint8_t> &payload32);
  bool sendEvent(uint8_t code, uint8_t a0 = 0, uint8_t a1 = 0, uint8_t a2 = 0);
  bool sendAck(uint32_t ack_seq, uint8_t status = 0, uint8_t detail = 0);

  // ---- 离线自测：注入一帧（走完整解码路径）----
  void injectFrame(const std::vector<uint8_t> &frame);

  // ---- 观测 ----
  Stats stats() const;
  // 最近一次收到的门位（两个 bridge_id 分别记录）
  bool gate(uint8_t bridge_id, BridgeGate &out) const;
  bool hasBothGates() const;
  // 最近一次收到的任务指令
  TaskCommand lastTaskCommand() const;
  uint32_t lastTaskSeq() const { return last_task_seq_.load(); }

  // ---- 参数 ----
  // 链路参数（由 loadParams 从节点读入；也可外部直接设置）
  double heartbeat_period_s{0.2};
  double link_timeout_s{1.0};
  double ack_timeout_s{0.5};
  int max_retries{5};
  bool log_tx{false};
  // loopback 模式下自动注入合成帧（离线自测用）
  bool loopback_synth{false};
  double loopback_synth_period_s{1.0};

  // 固定结构参数（用于 loopback 合成与 FIELD_FIXED 校验）
  uint8_t field_up_row{5};
  uint8_t field_col_lo{3};
  uint8_t field_col_hi{13};
  uint8_t field_bridge_col{10};

private:
  void onBytes(const uint8_t *data, size_t len);
  void onFrame(const air::Decoded &d);
  void publishMulti(const std::string &topic, const std::vector<uint8_t> &v);
  void publishRaw(const std::vector<uint8_t> &frame);
  void tickLoop();
  void synthesizeLoopbackFrame();

  // 待确认消息（需要 ACK 的类型）
  struct Pending
  {
    std::vector<uint8_t> frame;
    uint32_t seq{0};
    uint8_t typ{0};
    int retries{0};
    double sent_at{0.0};
  };

  bool sendFrameTyp(uint8_t typ, const std::vector<uint8_t> &payload,
                    bool need_ack);
  uint32_t nextSeq() { return seq_.fetch_add(1); }

  rclcpp::Node *node_{nullptr};
  std::unique_ptr<RawTransport> transport_;

  // 帧同步（用协议层提供的状态机，含 A5 A5 C3 容错）
  air::FrameSync sync_;

  // 发送序号
  std::atomic<uint32_t> seq_{1};

  // 待确认队列
  mutable std::mutex pend_mutex_;
  std::vector<Pending> pending_;
  std::atomic<uint64_t> retransmits_{0};

  // 统计
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> rx_errors_{0};
  std::atomic<uint64_t> tx_frames_{0};
  mutable std::mutex err_mutex_;
  std::string last_error_;

  // 最近一次收到的内容
  mutable std::mutex data_mutex_;
  BridgeGate gate_a_;
  BridgeGate gate_b_;
  TaskCommand last_task_;
  std::atomic<uint32_t> last_task_seq_{0};
  std::atomic<double> last_rx_wall_{-1.0};

  // 定时器
  rclcpp::TimerBase::SharedPtr tick_timer_;
  double loopback_acc_{0.0};

  // 发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_err_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_alive_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_gate_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_mine_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_obs_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_tgt_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_map_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_field_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_task_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_ev_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_ack_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_raw_;

  bool alive_reported_{false};
};

}  // namespace dzmission

#endif  // DZMISSION_AIR_LINK_HPP
