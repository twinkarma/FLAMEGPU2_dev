#ifndef __PEDRVO_MODELSPEC__
#define __PEDRVO_MODELSPEC__


#include <string>
#include <vector>
#include <memory>

struct Bounds {
    float3 min;
    float3 max;
};

struct ObstacleRegion{
    int numObstacles;
    float obstacleSize;
    float obstacleHeight;
    Bounds regionBounds;
};

#define AGENT_GOAL_SEEK_TARGET 0
#define AGENT_GOAL_FLEE_TARGET 1
#define AGENT_GOAL_IDLE 2


struct AgentGoal{
    std::string targetName;
    int goalType;
    float3 targetLocation;
    float desiredSpeed;
    float timeDuration;
};


struct Agent{
    std::string name; //Agent only

    int numAgents; //Agent region only
    Bounds regionBounds; //Agent region only

    //Initial conditions
    float radius;
    float3 position;
    float speed;
    float3 direction;
    bool isDirectionRandom;

    //Goals
    std::vector<AgentGoal> goals;

    //Goal index obtained after agent goals have been uploaded
    int goalIndex;
};

struct ModelEnvSpec {
    Bounds envBounds;
    std::vector<Bounds> obstacles;
    std::vector<ObstacleRegion> obstacleRegions;
    std::vector<Agent> agents;
    std::vector<Agent> agentRegions;
};

typedef std::shared_ptr<ModelEnvSpec> ModelEnvSpecPtr;

#endif //__PEDRVO_MODELSPEC__