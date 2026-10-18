#ifndef CORENUM_H
#define CORENUM_H

#include <stdio.h>
#include <mutex>

#include "rknn_api.h"

// =============================================================================
// NPU 核心分配
// -----------------------------------------------------------------------------
// RK3588S 的 NPU 有 **3 个核**。把不同的推理上下文绑到不同核才能真正并行；
// 否则 rknn 默认 RKNN_NPU_CORE_AUTO 会把负载随机丢给某个核，多线程互相争抢。
//
// ★ 本次修正原实现的两个问题：
//   1. `rknn_core_mask core_mask;` 声明后**未初始化**，而 switch 没有 default
//      分支 —— 若 get_core_num() 返回 >= 3 就会使用未初始化变量（UB）。
//      现在提供 core_num_to_mask() 并在 default 分支返回 CORE_AUTO。
//   2. `#include <mutex>` 缺失（原文件用了 std::mutex 却没包含）。
//
// 轮询序列：第 0 个上下文绑核 0、第 1 个绑核 1、第 2 个绑核 2、第 3 个回核 0。
// 因此 **推理线程数取 3 最理想**（与 NPU 核数一致，无争抢）。
// det_node 的 infer_threads 默认值已设为 3。
// =============================================================================

const int RK3588_NPU_CORES = 3;

// 取下一个要绑定的 NPU 核号（0 .. RK3588_NPU_CORES-1），线程安全
inline int get_core_num()
{
    static int core_num = 0;
    static std::mutex mtx;

    std::lock_guard<std::mutex> lock(mtx);

    int temp = core_num % RK3588_NPU_CORES;
    core_num++;
    return temp;
}

// 把核号映射为 rknn 的 core_mask 枚举（default 兜底，杜绝未初始化）
inline rknn_core_mask core_num_to_mask(int n)
{
    switch (n)
    {
        case 0:  return RKNN_NPU_CORE_0;
        case 1:  return RKNN_NPU_CORE_1;
        case 2:  return RKNN_NPU_CORE_2;
        default: return RKNN_NPU_CORE_AUTO;
    }
}

#endif  // CORENUM_H
