#include "postprocess.h"

#include <algorithm>
#include <cmath>
#include <vector>

// 类别名：与训练 data.yaml 的 names 一一对应
static const char *labels[OBJ_CLASS_NUM] = {"blue_target", "red_target"};

static const int cnum = OBJ_CLASS_NUM;
static cv::Scalar_<int> randColor[cnum];
static bool init_colors = false;

inline static int clamp(float val, int min, int max)
{
  return val > min ? (val < max ? val : max) : min;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0,
                              float xmin1, float ymin1, float xmax1, float ymax1)
{
  const float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1));
  const float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1));
  const float i = w * h;
  const float u = (xmax0 - xmin0) * (ymax0 - ymin0) + (xmax1 - xmin1) * (ymax1 - ymin1) - i;
  return u <= 0.f ? 0.f : (i / u);
}

// 内部候选框（坐标为模型输入尺度）
typedef struct
{
  float x1, y1, x2, y2;
  float score;
  int cls;
  float kx, ky, kconf;
} RawBox;

// 贪心 NMS：按分数从高到低，同类之间做 IoU 抑制
static void nms(std::vector<RawBox> &boxes, float threshold)
{
  std::sort(boxes.begin(), boxes.end(),
            [](const RawBox &a, const RawBox &b) { return a.score > b.score; });

  std::vector<char> dead(boxes.size(), 0);
  for (size_t i = 0; i < boxes.size(); ++i)
  {
    if (dead[i]) continue;
    for (size_t j = i + 1; j < boxes.size(); ++j)
    {
      if (dead[j] || boxes[i].cls != boxes[j].cls) continue;
      if (CalculateOverlap(boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2,
                           boxes[j].x1, boxes[j].y1, boxes[j].x2, boxes[j].y2) > threshold)
      {
        dead[j] = 1;
      }
    }
  }

  std::vector<RawBox> keep;
  keep.reserve(boxes.size());
  for (size_t i = 0; i < boxes.size(); ++i)
  {
    if (!dead[i]) keep.push_back(boxes[i]);
  }
  boxes.swap(keep);
}

// YOLOv8-pose 后处理
//   output 布局：[1, num_cols, num_preds]，取值方式 output[channel * num_preds + index]
//     channel 0..3        : cx, cy, w, h（导出时已完成 DFL 解码，单位为模型输入尺度）
//     channel 4..4+nc-1   : 各类别分数
//     channel 之后         : 关键点 x, y, conf（kpt_shape [1,3]）
int post_process(float *output, int num_preds, int num_cols,
                 int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold,
                 BOX_RECT pads, float scale_w, float scale_h,
                 DetectResultsGroup *group)
{
  (void)pads;

  if (output == nullptr || group == nullptr || num_preds <= 0 || num_cols < 4 + OBJ_CLASS_NUM)
  {
    return -1;
  }

  const int cls_offset = 4;                 // bbox 之后是类别分数
  const int kpt_offset = 4 + OBJ_CLASS_NUM; // 类别分数之后是关键点
  const bool has_kpt = (num_cols >= kpt_offset + KPT_NUM * KPT_DIM);

  std::vector<RawBox> boxes;
  boxes.reserve(64);

  for (int n = 0; n < num_preds; ++n)
  {
    // 1) 找置信度最高的类别
    float maxScore = 0.f;
    int maxId = 0;
    for (int c = 0; c < OBJ_CLASS_NUM; ++c)
    {
      const float s = output[(cls_offset + c) * num_preds + n];
      if (s > maxScore)
      {
        maxScore = s;
        maxId = c;
      }
    }
    if (maxScore < conf_threshold)
    {
      continue;
    }

    // 2) bbox：cx, cy, w, h -> 左上角 + 宽高
    const float cx = output[0 * num_preds + n];
    const float cy = output[1 * num_preds + n];
    const float bw = output[2 * num_preds + n];
    const float bh = output[3 * num_preds + n];

    if (bw <= 1.f || bh <= 1.f)
    {
      continue;
    }

    RawBox b;
    b.x1 = cx - bw * 0.5f;
    b.y1 = cy - bh * 0.5f;
    b.x2 = cx + bw * 0.5f;
    b.y2 = cy + bh * 0.5f;
    b.score = maxScore;
    b.cls = maxId;

    // 3) 关键点（x, y 为模型输入尺度，conf 为概率）
    if (has_kpt)
    {
      b.kx = output[(kpt_offset + 0) * num_preds + n];
      b.ky = output[(kpt_offset + 1) * num_preds + n];
      b.kconf = output[(kpt_offset + 2) * num_preds + n];
    }
    else
    {
      b.kx = 0.f;
      b.ky = 0.f;
      b.kconf = 0.f;
    }

    boxes.push_back(b);
  }

  nms(boxes, nms_threshold);

  // 4) 映射回原图坐标
  for (const auto &b : boxes)
  {
    DetectionBox new_box;
    const int _x1 = (int)(clamp(b.x1, 0, model_in_w) / scale_w);
    const int _y1 = (int)(clamp(b.y1, 0, model_in_h) / scale_h);
    const int _x2 = (int)(clamp(b.x2, 0, model_in_w) / scale_w);
    const int _y2 = (int)(clamp(b.y2, 0, model_in_h) / scale_h);
    new_box.box = cv::Rect_<int>(_x1, _y1, _x2 - _x1, _y2 - _y1);
    new_box.score = b.score;
    new_box.cls = b.cls;
    new_box.det_name = (b.cls >= 0 && b.cls < OBJ_CLASS_NUM) ? labels[b.cls] : "unknown";
    new_box.kpt_x = b.kx / scale_w;
    new_box.kpt_y = b.ky / scale_h;
    new_box.kpt_conf = b.kconf;
    group->dets.push_back(new_box);
  }

  return 0;
}

// 初始化随机颜色
static void initializeRandColors()
{
  cv::RNG rng(0xFFFFFFFF); // 生成随机颜色
  for (int i = 0; i < cnum; i++)
  {
    rng.fill(randColor[i], cv::RNG::UNIFORM, 0, 256);
  }
  init_colors = true;
}


void show_draw_results(DetectResultsGroup &results_group)
{
  if (!init_colors)
  {
    initializeRandColors();
  }
  char text[256];
  for (const auto &res : results_group.dets)
  {
    sprintf(text, "%s", res.det_name.c_str());
    cv::rectangle(results_group.cur_img, res.box, randColor[1], 2, 8, 0);
    cv::putText(results_group.cur_img, text, cv::Point(res.box.x, res.box.y + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));

    // 框中心（红点）
    int cv_center_x = res.box.x + res.box.width / 2;
    int cv_center_y = res.box.y + res.box.height / 2;
    cv::circle(results_group.cur_img, cv::Point(cv_center_x, cv_center_y), 10, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

    // YOLOv8-pose 关键点（绿点），便于确认靶心位置
    if (res.kpt_conf > 0.f)
    {
      cv::circle(results_group.cur_img,
                 cv::Point((int)res.kpt_x, (int)res.kpt_y),
                 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
  }
}
