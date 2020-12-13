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
AgentGoalGSOA goal;

__device__ void getAgentGoal(int goalIndex, int &goalType, float2 &targetLocation, float & desiredSpeed, float& timeDuration){
    goalType = goal_d.goalType[goalIndex];
    targetLocation = make_float2(goal_d.targetLocation[goalIndex].x, goal_d.targetLocation[goalIndex].z);
    desiredSpeed = goal_d.desiredSpeed[goalIndex];
    timeDuration = goal_d.timeDuration[goalIndex];
}
__device__ int nextAgentGoalIndex(int goalIndex){
    return goal_d.nextIndex[goalIndex];
}



void indexAgentGoals(ModelEnvSpecPtr envSpec){
    int goalIndex = 0;

    for(auto& agent: envSpec->agents){
        if(agent.goals.size() > 0)
            agent.goalIndex = goalIndex;
        else
            agent.goalIndex = -1;
        goalIndex += agent.goals.size();
    }

    for(auto& agent: envSpec->agentRegions){
        if(agent.goals.size() > 0)
            agent.goalIndex = goalIndex;
        else
            agent.goalIndex = -1;
        goalIndex += agent.goals.size();
    }

    goal.size = goalIndex;
    FG_CPU_ALLOC(goal.goalType, int, goal.size);
    FG_CPU_ALLOC(goal.targetLocation, float3, goal.size);
    FG_CPU_ALLOC(goal.desiredSpeed, float, goal.size);
    FG_CPU_ALLOC(goal.timeDuration, float, goal.size);
    FG_CPU_ALLOC(goal.nextIndex, int, goal.size);

    goalIndex = 0;
    for(auto& agent: envSpec->agents){
        unsigned int i = 0;
        for(auto& agentGoal: agent.goals){
            goal.goalType[goalIndex] = agentGoal.goalType;
            goal.targetLocation[goalIndex] = agentGoal.targetLocation;
            goal.desiredSpeed[goalIndex] = agentGoal.desiredSpeed;
            goal.timeDuration[goalIndex] = agentGoal.timeDuration;
            if(i + 1 < agent.goals.size() )
                goal.nextIndex[goalIndex] = goalIndex + 1;
            else
                goal.nextIndex[goalIndex] = -1;

            goalIndex ++;
            i++;
        }
    }

    for(auto& agent: envSpec->agentRegions){
        unsigned int i = 0;
        for(auto& agentGoal: agent.goals){
            goal.goalType[goalIndex] = agentGoal.goalType;
            goal.targetLocation[goalIndex] = agentGoal.targetLocation;
            goal.desiredSpeed[goalIndex] = agentGoal.desiredSpeed;
            goal.timeDuration[goalIndex] = agentGoal.timeDuration;
            if(i + 1 < agent.goals.size() )
                goal.nextIndex[goalIndex] = goalIndex + 1;
            else
                goal.nextIndex[goalIndex] = -1;

            goalIndex ++;
            i++;
        }
    }
}

void uploadAgentGoals(){
    AgentGoalGSOA goal_temp;

    //Allocate
    FG_GPU_ALLOC(goal_temp.goalType, int, goal.size);
    FG_GPU_ALLOC(goal_temp.targetLocation, float3, goal.size);
    FG_GPU_ALLOC(goal_temp.desiredSpeed, float, goal.size);
    FG_GPU_ALLOC(goal_temp.timeDuration, float, goal.size);
    FG_GPU_ALLOC(goal_temp.nextIndex, int, goal.size);

    //Copy
    goal_temp.size = goal.size;
    memcpy(goal_temp.goalType, goal.goalType, sizeof(int)* goal.size);
    memcpy(goal_temp.targetLocation, goal.targetLocation, sizeof(float3)* goal.size);
    memcpy(goal_temp.desiredSpeed, goal.desiredSpeed, sizeof(float)* goal.size);
    memcpy(goal_temp.timeDuration, goal.timeDuration, sizeof(float)* goal.size);
    memcpy(goal_temp.nextIndex, goal.nextIndex, sizeof(int)* goal.size);

    //Upload pointer
    cudaMemcpyToSymbol(goal_d, &goal_temp, sizeof(AgentGoalGSOA));



}

