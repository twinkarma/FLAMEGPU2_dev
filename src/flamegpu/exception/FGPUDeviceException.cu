#include "flamegpu/exception/FGPUDeviceException.h"

#include <device_launch_parameters.h>

#include <cstdio>
#include <limits>

/**
 * stb_sprintf is used to provide vsnprintf in device code.
 */
// #define STB_SPRINTF_MIN 1024 // how many characters per callback
#define STB_SPRINTF_NOUNALIGNED
#define STB_SPRINTF_IMPLEMENTATION
#include <stb/stb_sprintf.h>

namespace flamegpu_internal {
    namespace DeviceException {
        __device__ unsigned int error_flag;
        __device__ char location_buffer[STB_SPRINTF_MIN];
        __device__ char error_buffer[STB_SPRINTF_MIN];
        __device__ const char *file_location;
        __device__ unsigned int line_location;
        __device__ void setLocation(const char *file, const unsigned int line) {
            file_location = file;
            line_location = line;
        }
        // Test texture err
        __device__ __constant__ cudaTextureObject_t d_tex_err;
    }  // namespace DeviceException
}  // namespace flamegpu_internal

#define gpuErrchk(ans) { gpuAssert_2((ans), __FILE__, __LINE__); }
inline void gpuAssert_2(cudaError_t code, const char *file, int line) {
    if (code != cudaSuccess) {
        printf("Woops: %s\n", cudaGetErrorString(code));
    }
}

std::string DeviceException::last_error = "";

__device__ DeviceException::DeviceException(const char *format, ...) {
    // Are we the first exception
    const unsigned int result = atomicInc(&flamegpu_internal::DeviceException::error_flag, UINT_MAX);
    if (!result) {
        //We are not the first
        delete this;
        return;
    }
    // Process our location data into location_buffer
    if (flamegpu_internal::DeviceException::file_location) {
        int ct = stbsp_snprintf(flamegpu_internal::DeviceException::location_buffer, STB_SPRINTF_MIN,
            "%s(%u)[%d, %d, %d][%d, %d, %d]",
            flamegpu_internal::DeviceException::file_location,
            flamegpu_internal::DeviceException::line_location,
            blockIdx.x, blockIdx.y, blockIdx.z,
            threadIdx.x, threadIdx.y, threadIdx.z);
        if (ct >= 0) {
            // Success!
        }
    }
    // Process VA args into error_buffer
    va_list argp;
    va_start(argp, format);
    const int buffLen = 1024;// stbsp_vsnprintf(nullptr, 0, format, argp) + 1;
    char *buffer = reinterpret_cast<char *>(malloc(buffLen * sizeof(char)));
    int ct = stbsp_vsnprintf(flamegpu_internal::DeviceException::error_buffer, STB_SPRINTF_MIN, format, argp);
    va_end(argp);
    if (ct >= 0) {
        // Success!
    }
    printf("location buffer: %s\n", flamegpu_internal::DeviceException::location_buffer);
    printf("error buffer: %s\n", flamegpu_internal::DeviceException::error_buffer);
    free(buffer);
    // Forcibly crash CUDA kernel currently executing
    unsigned int i = tex1D<unsigned int>(flamegpu_internal::DeviceException::d_tex_err, 12.5f);
    printf("%u\n", i);
    //char *t = reinterpret_cast<char*>(malloc(ULONG_MAX));
    //for(uint64_t i = 0;i<ULONG_MAX;++i) {
    //    t[i] = 0;
    //}
    // asm("trap;");
}

void DeviceException::reset() {
    const unsigned int ZERO = 0;
    const char *NULLPTR = nullptr;
    const unsigned int UNSIGNED_INT_MAX = std::numeric_limits<unsigned int>::max();
    gpuErrchk(cudaMemcpyToSymbol(flamegpu_internal::DeviceException::error_flag, &ZERO, sizeof(unsigned int)));
    gpuErrchk(cudaMemcpyToSymbol(flamegpu_internal::DeviceException::file_location, &NULLPTR, sizeof(const char *)));
    gpuErrchk(cudaMemcpyToSymbol(flamegpu_internal::DeviceException::file_location, &UNSIGNED_INT_MAX, sizeof(unsigned int)));
    // Attempt tex error
    // Allocate CUDA array in device memory
    cudaChannelFormatDesc channelDesc =
        cudaCreateChannelDesc(32, 0, 0, 0,
            cudaChannelFormatKindUnsigned);
    cudaArray* cuArray;
    cudaMallocArray(&cuArray, &channelDesc, 13, 13);

    // Copy to device memory some data located at address h_data
    // in host memory 
    struct cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = cuArray;

    // Specify texture object parameters
    struct cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.addressMode[0] = cudaAddressModeWrap;
    texDesc.addressMode[1] = cudaAddressModeWrap;
    texDesc.filterMode = cudaFilterModePoint; // cudaFilterModeLinear
    texDesc.readMode = cudaReadModeNormalizedFloat; // cudaReadModeElementType;
    texDesc.normalizedCoords = 1;

    // Create texture object
    cudaTextureObject_t texObj = 0;
    cudaCreateTextureObject(&texObj, &resDesc, &texDesc, NULL);
    gpuErrchk(cudaMemcpyToSymbol(flamegpu_internal::DeviceException::d_tex_err, &texObj, sizeof(cudaTextureObject_t)));
}

bool DeviceException::check() {
    unsigned int errorFlag = 0;
    gpuErrchk(cudaMemcpyFromSymbol(&errorFlag, flamegpu_internal::DeviceException::error_flag, sizeof(unsigned int)));
    if (!errorFlag)
        return false;
    // Process device data to local
    const size_t OUTBUFFER_SIZE = STB_SPRINTF_MIN * 2;
    char locationBuffer[STB_SPRINTF_MIN];
    char errorBuffer[STB_SPRINTF_MIN];
    gpuErrchk(cudaMemcpyFromSymbol(locationBuffer, flamegpu_internal::DeviceException::location_buffer, sizeof(unsigned int)));
    gpuErrchk(cudaMemcpyFromSymbol(errorBuffer, flamegpu_internal::DeviceException::error_buffer, sizeof(unsigned int)));
    char outBuffer[OUTBUFFER_SIZE];
    int ct = snprintf(outBuffer, OUTBUFFER_SIZE - 1, "%s: %s\n", locationBuffer, errorBuffer);
    if (ct >= 0 && ct<OUTBUFFER_SIZE) {
        outBuffer[ct + 1] = '\0';
        last_error = outBuffer;
    }
    return true;
}
const char *DeviceException::getLastErrorMsg() {
    return last_error.c_str();
}