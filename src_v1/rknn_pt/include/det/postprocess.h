#ifndef POSTPROCESS_H_
#define POSTPROCESS_H_

#include <stdint.h>
#include <vector>

#include <iomanip>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common.h"

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 64

// 类别数：与训练 data.yaml 的 names 对应（0=blue target, 1=red target）
#define OBJ_CLASS_NUM 2
// 关键点：data.yaml 的 kpt_shape: [1, 3] -> 1 个点，每个点 (x, y, conf)
#define KPT_NUM 1
#define KPT_DIM 3

#define NMS_THRESH 0.45
#define BOX_THRESH 0.4

// YOLOv8-pose 后处理（单输出、float）
//   output    : 模型输出张量，layout [1, num_cols, num_preds]
//   num_preds : 候选点数量（YOLOv8 通常为 8400 = 80*80 + 40*40 + 20*20）
//   num_cols  : 每个候选的通道数 = 4(bbox) + OBJ_CLASS_NUM + KPT_NUM*KPT_DIM
//   数据访问方式：output[channel * num_preds + index]
int post_process(float *output, int num_preds, int num_cols,
                 int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold,
                 BOX_RECT pads, float scale_w, float scale_h,
                 DetectResultsGroup *group);

void show_draw_results(DetectResultsGroup &results_group);


#endif //POSTPROCESS_H_
