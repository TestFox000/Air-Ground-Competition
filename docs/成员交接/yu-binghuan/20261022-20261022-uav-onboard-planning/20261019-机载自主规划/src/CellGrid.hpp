#pragma once

#include <cstdint>
#include <string>

namespace dzuav
{

// 单个格子的占地物类别（矩阵语义，与 YOLO 类别解耦）。
enum class CellContent : int
{
    Unknown = 0,   // 未观测或证据不足
    Empty = 1,     // 观测过且为空
    Passage = 2,   // 通：绿色通过格，路线起点/终点
    Mine = 3,      // 雷
    Obstacle = 4,  // 障
    Target = 5,    // 靶（红/蓝面合并；占相邻两格，路线走其侧面）
};

struct CellEntry
{
    CellContent content = CellContent::Unknown;
    float confidence = 0.F;  // 获胜证据的累计权重
    bool idRead = false;     // 编号曾被直接识别
    float idConfidence = 0.F;
    bool onBridge = false;   // 位于固定桥体上（预设，不参与投票）
    int observations = 0;
};

// 场地格子矩阵：14 列（A~N）× 11 行（1~11）。
// 跨帧证据按指数滑动平均累积：observe() 时 votes = votes*decay + weight；
// 未观测格子的证据保持不变，飞离视野后矩阵不会遗忘。
class CellGrid
{
public:
    static constexpr int kCols = 14;
    static constexpr int kRows = 11;
    static constexpr float kContentThreshold = 1.0F;  // 物类证据达标线
    static constexpr float kEmptyThreshold = 0.5F;    // 空证据达标线

    explicit CellGrid(float decayPerObservation = 0.6F);

    void reset();
    void setBridge(int col, int row);
    void observe(int col, int row, CellContent content, float weight);
    void confirmId(int col, int row, float confidence);
    CellEntry entry(int col, int row) const;
    float coverage() const;   // 非 unknown 格子占比
    uint64_t version() const; // 矩阵内容每次变化 +1，供上层节流输出

    std::string render() const;                // 控制台字符矩阵 + 摘要
    std::string toJson(int64_t timestampMs) const;

private:
    static const char* contentName(CellContent content);
    static int slot(CellContent content);      // Passage~Target → 0~3，其余 -1
    CellContent resolve(int col, int row) const;

    float votes_[kRows][kCols][4] = {};  // passage/mine/obstacle/target
    float emptyVotes_[kRows][kCols] = {};
    float idConf_[kRows][kCols] = {};
    int obs_[kRows][kCols] = {};
    bool bridge_[kRows][kCols] = {};
    float decay_;
    uint64_t version_ = 0;
};

}  // namespace dzuav
