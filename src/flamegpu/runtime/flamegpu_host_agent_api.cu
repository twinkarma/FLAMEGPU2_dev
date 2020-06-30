#include "flamegpu/runtime/flamegpu_host_agent_api.h"

__global__ void initToThreadIndex(unsigned int *output, unsigned int threadCount) {
    const unsigned int TID = blockIdx.x * blockDim.x + threadIdx.x;
    if (TID < threadCount) {
        output[TID] = TID;
    }
}

void HostAgentInstance::fillTIDArray(unsigned int *buffer, const unsigned int &threadCount) {
    initToThreadIndex<<<(threadCount/512)+1, 512 >>>(buffer, threadCount);
    gpuErrchkLaunch();
}
