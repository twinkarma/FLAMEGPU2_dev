#ifndef INCLUDE_FLAMEGPU_GPU_CUDAERRORCHECKING_H_
#define INCLUDE_FLAMEGPU_GPU_CUDAERRORCHECKING_H_
/**
 * @file CUDAErrorChecking.h
 * @authors Paul
 * @date 5 Mar 2014
 * @brief
 *
 * @see
 * @warning
 */



#include <device_launch_parameters.h>
#include <cuda_runtime.h>

#include <string>
#include <stdexcept>
#include "flamegpu/exception/FGPUDeviceException.h"

/* Error check function for safe CUDA API calling */
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line) {
    if (code != cudaSuccess) {
        if (code == cudaErrorIllegalAddress) {
            // asm('trap') was called
            // check for a DeviceException
            if(DeviceException::check()) {
                printf(DeviceException::getLastErrorMsg());
            }
            // If error not thrown by now, it must not have been a DeviceException
            // Report the error DeviceException handler ate
            throw std::runtime_error("CUDA Error: " + std::string(cudaGetErrorString(cudaErrorLaunchFailure)) + " " + file + " " + std::to_string(line));
        }
        throw std::runtime_error("CUDA Error: " + std::string(cudaGetErrorString(code)) + " " + file + " " + std::to_string(line));
    }
}

/* Error check function for post CUDA Kernel calling */
#define gpuErrchkLaunch() { gpuLaunchAssert(__FILE__, __LINE__); }
inline void gpuLaunchAssert(const char *file, int line) {
#ifdef _DEBUG
    gpuAssert(cudaDeviceSynchronize(), file, line);
#endif
    gpuAssert(cudaPeekAtLastError(), file, line);
}

#endif  // INCLUDE_FLAMEGPU_GPU_CUDAERRORCHECKING_H_
