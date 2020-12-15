#ifndef __GOALS_KERNELS__
#define __GOALS_KERNELS__

#include "simulation_spec.cuh"

__device__ void getAgentGoal(int goalIndex,
                        int &goalType,
                        float2 &targetLocation,
                        float & desiredSpeed,
                        float& timeDuration);

__device__ int nextAgentGoalIndex(int goalIndex);

void uploadAgentGoals(SimulationSpecPtr envSpec);

#endif //__GOALS_KERNELS__
