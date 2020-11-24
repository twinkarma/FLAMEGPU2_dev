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

struct AgentGoal{
    std::string targetName;
    bool seekTarget; //If true navigate to target, false then flees target
    bool fleeTarget;
    bool idle;
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