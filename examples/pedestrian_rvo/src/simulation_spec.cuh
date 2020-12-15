#ifndef __PEDRVO_SIMSPEC__
#define __PEDRVO_SIMSPEC__


#include <string>
#include <vector>
#include <memory>

/**
 * Represents an axis-aligned boundary
 */
struct Bounds {
    float3 min;
    float3 max;
};

/**
 * Can represent multiple obstacles to be created within regionBounds
 */
struct ObstacleRegion{
    int numObstacles;
    float obstacleSize;
    float obstacleHeight;
    Bounds regionBounds;
};

#define AGENT_GOAL_SEEK_TARGET 0
#define AGENT_GOAL_FLEE_TARGET 1
#define AGENT_GOAL_IDLE 2

/**
 * Represent a single pedestrian agent goal. Each pedestrian can have a sequence of these goals.
 * Goals are completed when the pedestrian reaches the targetLocation or when timeDuration expires.
 *
 * targetName is not currently used
 */
struct AgentGoal{

    std::string targetName;
    int goalType;
    float3 targetLocation;
    float desiredSpeed;
    float timeDuration;
    int nextIndex;
};

/**
 * A pedestrian agent.
 *
 */
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

    int goalIndex;
};

/**
 * Defines a complete test case for running the pedestrian simulation
 */
struct SimulationSpec {
    Bounds envBounds;
    std::vector<Bounds> obstacles;
    std::vector<ObstacleRegion> obstacleRegions;
    std::vector<Agent> agents;
    std::vector<Agent> agentRegions;
    std::vector<AgentGoal> agentGoals;
};

typedef std::shared_ptr<SimulationSpec> SimulationSpecPtr;

#endif //__PEDRVO_SIMSPEC__