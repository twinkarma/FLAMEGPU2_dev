#ifndef __GOALS_KERNELS__
#define __GOALS_KERNELS__

#include "modelspec.cuh"

__device__ void getAgentGoal(int goalIndex,
                        int &goalType,
                        float2 &targetLocation,
                        float & desiredSpeed,
                        float& timeDuration);

__device__ int nextAgentGoalIndex(int goalIndex);

void indexAgentGoals(ModelEnvSpecPtr envSpec);
void uploadAgentGoals();

#endif //__GOALS_KERNELS__
