#ifndef RKPT_H
#define RKPT_H

#include "rknn_api.h"

#include "opencv2/core/core.hpp"
#include "postprocess.h"

static void dump_tensor_attr(rknn_tensor_attr *attr);
static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz);
static unsigned char *load_model(const char *filename, int *model_size);
static int saveFloat(const char *file_name, float *output, int element_size);

class RkPt
{
private:
    int ret;
    std::mutex mtx;
    std::string model_path;
    unsigned char *model_data;

    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
    rknn_input inputs[1];

    int channel, width, height;
    int img_width, img_height;

    float nms_threshold, box_conf_threshold;

public:
    RkPt(const std::string &model_path);
    int init(rknn_context *ctx_in, bool isChild);
    rknn_context *get_pctx();
    // cv::Mat infer(cv::Mat &ori_img);
    // std::vector<detect_result_t> infer(cv::Mat &ori_img);
    //
    // ★ want_debug_img：是否把原图放进结果里（供可视化使用）。
    //   默认 false —— cur_img 只被 show_draw_results / imshow 消费，
    //   而 orig_img.clone() 是一次**全画幅拷贝**
    //   （640×480 BGR = 0.92MB/帧，@30fps = 28MB/s 纯内存带宽）。
    //   关掉可视化时这次拷贝纯属浪费。
    DetectResultsGroup infer(cv::Mat &ori_img, int cur_frame_id,
                             bool want_debug_img = false);
    ~RkPt();
};

#endif
