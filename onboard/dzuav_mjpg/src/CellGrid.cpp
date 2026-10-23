#include "dzuav/CellGrid.hpp"

#include <algorithm>
#include <sstream>

namespace dzuav
{
namespace
{
const char* contentGlyph(CellContent content)
{
    switch (content)
    {
    case CellContent::Empty:
        return "·";
    case CellContent::Passage:
        return "通";
    case CellContent::Mine:
        return "雷";
    case CellContent::Obstacle:
        return "障";
    case CellContent::Target:
        return "靶";
    default:
        return "?";
    }
}
}  // namespace

CellGrid::CellGrid(float decayPerObservation) : decay_(decayPerObservation)
{
}

void CellGrid::reset()
{
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
        {
            for (auto& vote : votes_[r][c])
                vote = 0;
            emptyVotes_[r][c] = 0;
            idConf_[r][c] = 0;
            obs_[r][c] = 0;
            bridge_[r][c] = false;
        }
    version_ = 0;
}

void CellGrid::setBridge(int col, int row)
{
    if (col < 0 || col >= kCols || row < 0 || row >= kRows || bridge_[row][col])
        return;
    bridge_[row][col] = true;
    ++version_;
}

const char* CellGrid::contentName(CellContent content)
{
    switch (content)
    {
    case CellContent::Unknown:
        return "unknown";
    case CellContent::Empty:
        return "empty";
    case CellContent::Passage:
        return "passage";
    case CellContent::Mine:
        return "mine";
    case CellContent::Obstacle:
        return "obstacle";
    case CellContent::Target:
        return "target";
    }
    return "unknown";
}

int CellGrid::slot(CellContent content)
{
    return content >= CellContent::Passage && content <= CellContent::Target
               ? static_cast<int>(content) - static_cast<int>(CellContent::Passage)
               : -1;
}

CellContent CellGrid::resolve(int col, int row) const
{
    float best = 0;
    CellContent bestContent = CellContent::Unknown;
    for (int i = 0; i < 4; ++i)
        if (votes_[row][col][i] > best)
        {
            best = votes_[row][col][i];
            bestContent =
                static_cast<CellContent>(static_cast<int>(CellContent::Passage) + i);
        }
    if (best >= kContentThreshold)
        return bestContent;
    if (emptyVotes_[row][col] >= kEmptyThreshold)
        return CellContent::Empty;
    return CellContent::Unknown;
}

void CellGrid::observe(int col, int row, CellContent content, float weight)
{
    if (col < 0 || col >= kCols || row < 0 || row >= kRows)
        return;
    const CellContent before = resolve(col, row);
    const int index = slot(content);
    if (index >= 0)
        votes_[row][col][index] = votes_[row][col][index] * decay_ + weight;
    else if (content == CellContent::Empty)
        emptyVotes_[row][col] = emptyVotes_[row][col] * decay_ + weight;
    else
        return;
    ++obs_[row][col];
    if (resolve(col, row) != before)
        ++version_;
}

void CellGrid::confirmId(int col, int row, float confidence)
{
    if (col < 0 || col >= kCols || row < 0 || row >= kRows)
        return;
    const bool before = idConf_[row][col] > 0;
    idConf_[row][col] = std::max(idConf_[row][col], confidence);
    if (!before)
        ++version_;
}

CellEntry CellGrid::entry(int col, int row) const
{
    CellEntry out;
    if (col < 0 || col >= kCols || row < 0 || row >= kRows)
        return out;
    out.content = resolve(col, row);
    const int index = slot(out.content);
    out.confidence = index >= 0 ? votes_[row][col][index] : emptyVotes_[row][col];
    out.idRead = idConf_[row][col] > 0;
    out.idConfidence = idConf_[row][col];
    out.onBridge = bridge_[row][col];
    out.observations = obs_[row][col];
    return out;
}

float CellGrid::coverage() const
{
    int known = 0;
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
            if (resolve(c, r) != CellContent::Unknown)
                ++known;
    return float(known) / (kCols * kRows);
}

uint64_t CellGrid::version() const
{
    return version_;
}

std::string CellGrid::render() const
{
    std::ostringstream os;
    os << "    ";
    for (int c = 0; c < kCols; ++c)
        os << char('A' + c) << ' ';
    os << '\n';
    for (int r = 0; r < kRows; ++r)
    {
        os << (r + 1 < 10 ? "  " : " ") << r + 1 << ' ';
        for (int c = 0; c < kCols; ++c)
            os << contentGlyph(resolve(c, r)) << ' ';
        os << '\n';
    }
    os << "coverage=" << int(coverage() * 100) << "%  通:";
    bool first = true;
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
        {
            const CellEntry e = entry(c, r);
            if (e.content == CellContent::Passage)
            {
                os << (first ? "" : " ") << char('A' + c) << r + 1;
                first = false;
            }
        }
    const CellContent summary[] = {CellContent::Mine, CellContent::Obstacle,
                                   CellContent::Target};
    const char* names[] = {"雷", "障", "靶"};
    for (int i = 0; i < 3; ++i)
    {
        os << "  " << names[i] << ':';
        first = true;
        for (int r = 0; r < kRows; ++r)
            for (int c = 0; c < kCols; ++c)
            {
                const CellEntry e = entry(c, r);
                if (e.content == summary[i])
                {
                    os << (first ? "" : " ") << char('A' + c) << r + 1;
                    first = false;
                }
            }
    }
    return os.str();
}

std::string CellGrid::toJson(int64_t timestampMs) const
{
    std::ostringstream os;
    os << "{\"type\":\"cell_matrix\",\"ts\":" << timestampMs << ",\"cols\":" << kCols
       << ",\"rows\":" << kRows << ",\"cell_size_m\":0.35,\"coverage\":" << coverage()
       << ",\"cells\":[";
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
        {
            const CellEntry e = entry(c, r);
            if (r || c)
                os << ',';
            os << "{\"id\":\"" << char('A' + c) << r + 1 << "\",\"col\":" << c
               << ",\"row\":" << r << ",\"content\":\"" << contentName(e.content)
               << "\",\"conf\":" << e.confidence << ",\"id_read\":"
               << (e.idRead ? "true" : "false") << ",\"id_conf\":" << e.idConfidence
               << ",\"bridge\":" << (e.onBridge ? "true" : "false")
               << ",\"obs\":" << e.observations << '}';
        }
    os << "]}";
    return os.str();
}
}  // namespace dzuav
