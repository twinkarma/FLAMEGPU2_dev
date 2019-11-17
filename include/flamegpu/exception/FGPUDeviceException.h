#ifndef INCLUDE_FLAMEGPU_EXCEPTION_FGPUDEVICEEXCEPTION_H_
#define INCLUDE_FLAMEGPU_EXCEPTION_FGPUDEVICEEXCEPTION_H_

#include <cuda_runtime.h>
#include <string>

/**
* stb_sprintf is used to provide vsnprintf in device code.
*/
#define STB_SPRINTF_MIN 1024 // how many characters per callback

namespace flamegpu_internal {
    namespace DeviceException {
        extern __device__ unsigned int error_flag;
        extern __device__ char location_buffer[STB_SPRINTF_MIN];
        extern __device__ char error_buffer[STB_SPRINTF_MIN];
        extern __device__ const char *file_location;
        extern __device__ unsigned int line_location;
        extern __device__ void setLocation(const char *file, const unsigned int line);
    }  // namespace DeviceException
}  // namespace flamegpu_internal


#ifdef __CUDA_ARCH__
/**
 * If this macro is used instead of 'throw', FGPUException will
 * prepend '__FILE__ (__LINE__): ' to err_message
 */
#define THROW flamegpu_internal::DeviceException::setLocation(__FILE__, __LINE__); new
/**
 * Can't use 'throw' in device code, so might aswell override it too
 */
//#define throw THROW
#endif

/**
 * Header conflict so redefine here
 */
#define gpuErrchk(ans) ans

class DeviceException {
 public:
     __device__ DeviceException(const char *format, ...);
    /**
     * r
     */
    static void reset();
    static bool check();
    static const char *getLastErrorMsg();
    static std::string last_error;
};

#undef gpuErrchk

#endif  // INCLUDE_FLAMEGPU_EXCEPTION_FGPUDEVICEEXCEPTION_H_
