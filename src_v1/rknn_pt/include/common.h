#ifndef COMMON_H
#define COMMON_H

#include <opencv2/core.hpp>

#define OBJ_NUMB_MAX_SIZE 64

typedef struct _BOX_RECT
{
    int left;
    int right;
    int top;
    int bottom;
} BOX_RECT;

typedef struct _DetectionBox
{
    float score;
    std::string det_name;
    cv::Rect_<int> box;
    int cls = 0; // 类别 ID：0=blue target, 1=red target
    // YOLOv8-pose 关键点（已缩放到原图像素坐标；模型无关键点时保持 0）
    float kpt_x = 0.f;
    float kpt_y = 0.f;
    float kpt_conf = 0.f;
}DetectionBox;



// typedef struct _DetectionBox
// {
//     float score;
//     std::string det_name;
//     BOX_RECT box;
// }DetectionBox;



typedef struct _DetectResultsGroup
{
    cv::Mat cur_img;
    int cur_frame_id;
    std::vector<DetectionBox> dets; // 修改为vector
} DetectResultsGroup;

// typedef struct _DetectResultsGroup
// {
//     cv::Mat cur_img;
//     int cur_frame_id;
//     int count;
//     DetectionBox results[OBJ_NUMB_MAX_SIZE];
// } DetectResultsGroup;


#endif // COMMON_H
