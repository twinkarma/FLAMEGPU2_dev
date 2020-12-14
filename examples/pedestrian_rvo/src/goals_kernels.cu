#include "goals_kernels.cuh"

#define FG_CPU_ALLOC(allocVar, type, length) allocVar = (type*) malloc(sizeof(type)*length)
#define FG_GPU_ALLOC(allocVar, type, length) cudaMallocManaged(&allocVar, sizeof(type)*length);


struct AgentGoalGSOA{
    int size;
    int* goalType;
    float3* targetLocation;
    float* desiredSpeed;
    float* timeDuration;
    int* nextIndex;
};

__constant__ AgentGoalGSOA goal_d;

__device__ void getAgentGoal(int goalIndex, int &goalType, float2 &targetLocation, float & desiredSpeed, float& timeDuration){
    goalType = goal_d.goalType[goalIndex];
    targetLocation = make_float2(goal_d.targetLocation[goalIndex].x, goal_d.targetLocation[goalIndex].z);
    desiredSpeed = goal_d.desiredSpeed[goalIndex];
    timeDuration = goal_d.timeDuration[goalIndex];
}
__device__ int nextAgentGoalIndex(int goalIndex){
    return goal_d.nextIndex[goalIndex];
}

void uploadAgentGoals(ModelEnvSpecPtr envSpec){
    AgentGoalGSOA goal_temp;
    goal_temp.size = envSpec->agentGoals.size();

    //Allocate
    FG_GPU_ALLOC(goal_temp.goalType, int, goal_temp.size);
    FG_GPU_ALLOC(goal_temp.targetLocation, float3, goal_temp.size);
    FG_GPU_ALLOC(goal_temp.desiredSpeed, float, goal_temp.size);
    FG_GPU_ALLOC(goal_temp.timeDuration, float, goal_temp.size);
    FG_GPU_ALLOC(goal_temp.nextIndex, int, goal_temp.size);

    int goalIndex = 0;
    for(auto& agentGoal: envSpec->agentGoals){
        goal_temp.goalType[goalIndex] = agentGoal.goalType;
        goal_temp.targetLocation[goalIndex] = agentGoal.targetLocation;
        goal_temp.desiredSpeed[goalIndex] = agentGoal.desiredSpeed;
        goal_temp.timeDuration[goalIndex] = agentGoal.timeDuration;
        goal_temp.nextIndex[goalIndex] = agentGoal.nextIndex;
        goalIndex ++;
    }

    //Upload pointer
    cudaMemcpyToSymbol(goal_d, &goal_temp, sizeof(AgentGoalGSOA));

}

