#pragma once

#include <cuda_runtime_api.h>

#include <stdexcept>
#include <string>

// CUDA 运行时调用的统一检查宏：失败抛 std::runtime_error，带调用表达式与
// cudaGetErrorString 描述。CUDA 错误不检查就会拿 nullptr/垃圾缓冲继续跑。
// 注意：析构函数里不要用——析构不可抛异常（进程退出期 CUDA 上下文可能已拆，
// 此时 cudaFree 失败无害，保持原有裸调用即可）。
#define cudaCheck(call)                                                          \
    do                                                                           \
    {                                                                            \
        const cudaError_t cuda_check_err = (call);                               \
        if (cuda_check_err != cudaSuccess)                                       \
            throw std::runtime_error(std::string(#call " failed: ") +            \
                                     cudaGetErrorString(cuda_check_err));        \
    } while (0)
