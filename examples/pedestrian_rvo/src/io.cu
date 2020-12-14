#include "io.cuh"
#include "tinyxml2/tinyxml2.h"
#include "flamegpu/exception/FGPUException.h"
#include <stdlib.h>
#include <random>

using namespace tinyxml2;
using namespace std;

std::random_device rd;
std::mt19937 e2(rd());
std::uniform_real_distribution<> dist(0, 1);

#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult) if (a_eResult != tinyxml2::XML_SUCCESS) { FGPUException::setLocation(__FILE__, __LINE__);\
    switch (a_eResult) { \
    case tinyxml2::XML_ERROR_FILE_NOT_FOUND : \
    case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED : \
        throw InvalidInputFile("TinyXML error: File could not be opened.\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ERROR_FILE_READ_ERROR : \
        throw InvalidInputFile("TinyXML error: File could not be read.\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ERROR_PARSING_ELEMENT : \
    case tinyxml2::XML_ERROR_PARSING_ATTRIBUTE : \
    case tinyxml2::XML_ERROR_PARSING_TEXT : \
    case tinyxml2::XML_ERROR_PARSING_CDATA : \
    case tinyxml2::XML_ERROR_PARSING_COMMENT : \
    case tinyxml2::XML_ERROR_PARSING_DECLARATION : \
    case tinyxml2::XML_ERROR_PARSING_UNKNOWN : \
    case tinyxml2::XML_ERROR_PARSING : \
        throw TinyXMLError("TinyXML error: Error parsing file.\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ERROR_EMPTY_DOCUMENT : \
        throw TinyXMLError("TinyXML error: XML_ERROR_EMPTY_DOCUMENT\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ERROR_MISMATCHED_ELEMENT : \
        throw TinyXMLError("TinyXML error: XML_ERROR_MISMATCHED_ELEMENT\n Error code: %d", a_eResult); \
    case tinyxml2::XML_CAN_NOT_CONVERT_TEXT : \
        throw TinyXMLError("TinyXML error: XML_CAN_NOT_CONVERT_TEXT\n Error code: %d", a_eResult); \
    case tinyxml2::XML_NO_TEXT_NODE : \
        throw TinyXMLError("TinyXML error: XML_NO_TEXT_NODE\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ELEMENT_DEPTH_EXCEEDED : \
        throw TinyXMLError("TinyXML error: XML_ELEMENT_DEPTH_EXCEEDED\n Error code: %d", a_eResult); \
    case tinyxml2::XML_ERROR_COUNT : \
        throw TinyXMLError("TinyXML error: XML_ERROR_COUNT\n Error code: %d", a_eResult); \
    case tinyxml2::XML_NO_ATTRIBUTE: \
        throw TinyXMLError("TinyXML error: XML_NO_ATTRIBUTE\n Error code: %d", a_eResult); \
    case tinyxml2::XML_WRONG_ATTRIBUTE_TYPE : \
        throw TinyXMLError("TinyXML error: XML_WRONG_ATTRIBUTE_TYPE\n Error code: %d", a_eResult); \
    default: \
        throw TinyXMLError("TinyXML error: Unrecognised error code\n Error code: %d", a_eResult); \
    } \
}
#endif

std::string getString(XMLElement* node)
{
    if(node)
    {
        return std::string(node->GetText());
    }
    else{
        return std::string();
    }
}

float getInt(XMLElement* node)
{
    if(node){
        return node->IntText(0);
    }else{
        return 0;
    }
}

float getFloat(XMLElement* node)
{
    if(node){
        return node->FloatText(0);
    }
    else{
        return 0;
    }
}

float3 getFloat3(XMLElement* node, bool* isRandom = nullptr)
{
    if(node){
        if(node->FirstChildElement("random"))
        {
            if(isRandom) *isRandom = true;
            //Make random
            return make_float3(dist(e2), dist(e2), dist(e2));
        }
        else{
            if(isRandom) *isRandom = false;
            float x = getFloat(node->FirstChildElement("x"));
            float y = getFloat(node->FirstChildElement("y"));
            float z = getFloat(node->FirstChildElement("z"));
            return make_float3(x,y,z);
        }
    }
    else{
        if(isRandom) *isRandom = false;
        return make_float3(0,0,0);
    }

}


Bounds getBounds(XMLElement * node)
{
    auto xmin = node->FirstChildElement("xmin")->FloatText(0);
    auto xmax = node->FirstChildElement("xmax")->FloatText(0);
    auto ymin = node->FirstChildElement("ymin")->FloatText(0);
    auto ymax = node->FirstChildElement("ymax")->FloatText(0);
    auto zmin = node->FirstChildElement("zmin")->FloatText(0);
    auto zmax = node->FirstChildElement("zmax")->FloatText(0);
    Bounds bounds;
    bounds.min = make_float3(xmin, ymin, zmin);
    bounds.max = make_float3(xmax, ymax, zmax);

    return bounds;
}

int getAgentGoals(XMLElement* goalSequenceNode, std::vector<AgentGoal> &goalsVector)
{
    int goalIndex = goalsVector.size();

    auto goalNode = goalSequenceNode->FirstChild();
    while(goalNode){

        int currentIndex = goalsVector.size();
        AgentGoal goal;

        //Get goal type
        std::string goalName = std::string(goalNode->Value());
        if(goalName.compare("seekDynamicTarget") == 0){
            goal.goalType = AGENT_GOAL_SEEK_TARGET;

        }
        else if(goalName.compare("fleeDynamicTarget") == 0){
            goal.goalType = AGENT_GOAL_FLEE_TARGET;

        }
        else if(goalName.compare("seekStaticTarget") == 0){
            goal.goalType = AGENT_GOAL_SEEK_TARGET;

        }
        else if(goalName.compare("fleeStaticTarget") == 0){
            goal.goalType = AGENT_GOAL_FLEE_TARGET;

        }
        else if(goalName.compare("idle") == 0){
            goal.goalType = AGENT_GOAL_IDLE;
        }


        goal.targetName = getString(goalNode->FirstChildElement("targetName"));
        goal.desiredSpeed = getFloat(goalNode->FirstChildElement("desiredSpeed"));
        goal.timeDuration = getFloat(goalNode->FirstChildElement("timeDuration"));
        goal.targetLocation = getFloat3(goalNode->FirstChildElement("targetLocation"));

        goalNode = goalNode->NextSibling();
        if(goalNode){
            goal.nextIndex = currentIndex + 1; //Has more goals
        }
        else{
            goal.nextIndex = -1; //End of goals list
        }

        goalsVector.push_back(goal);

    }

    return goalIndex;
}

ModelEnvSpecPtr importSteerBenchXML(string filePath)
{
    ModelEnvSpecPtr env(new ModelEnvSpec());

    XMLDocument doc;
    XMLError errorId = doc.LoadFile(filePath.c_str());
    XMLCheckResult(errorId);

    XMLNode* pRoot = doc.FirstChildElement("SteerBenchTestCase");
    if(pRoot == nullptr){
        THROW TinyXMLError("TinyXML error: Error parsing doc %s.", filePath.c_str());
    }

    auto headerElem = pRoot->FirstChildElement("header");
    if(!headerElem){
        THROW TinyXMLError("TinyXML error: No header element");
    }

    auto worldBounds = headerElem->FirstChildElement("worldBounds");
    if(!worldBounds){
        THROW TinyXMLError("TinyXML error: No worldBounds element");
    }
    env->envBounds = getBounds(worldBounds);
    //Gets the environment boundaries



    //Obstacles
    int obstacleCount = 0;
    auto obstacleNode = pRoot->FirstChildElement("obstacle");
    while(obstacleNode)
    {
        auto bounds = getBounds(obstacleNode);
        env->obstacles.push_back(bounds);
        obstacleCount ++;
        obstacleNode = obstacleNode->NextSiblingElement("obstacle");
    }

    //Obstacle regions
    auto obtacleRegionNode = pRoot->FirstChildElement("obstacleRegion");
    while(obtacleRegionNode)
    {
        ObstacleRegion obstacleRegion;
        obstacleRegion.numObstacles = getInt(obtacleRegionNode->FirstChildElement("numObstacles"));
        obstacleRegion.obstacleSize = getFloat(obtacleRegionNode->FirstChildElement("obstacleSize"));
        obstacleRegion.obstacleHeight = getFloat(obtacleRegionNode->FirstChildElement("numObstacles"));
        obstacleRegion.regionBounds = getBounds(obtacleRegionNode->FirstChildElement("obstacleHeight"));

        env->obstacleRegions.push_back(obstacleRegion);
        obtacleRegionNode = obtacleRegionNode->NextSiblingElement("obstacleRegion");
    }

    //Agents
    auto agentNode = pRoot->FirstChildElement("agent");
    while(agentNode)
    {
        Agent agent;

        auto agentNameNode = agentNode->FirstChildElement("name");
        if(agentNameNode)
            agent.name = std::string(agentNameNode->GetText());

        auto initialConditionsNode = agentNode->FirstChildElement("initialConditions");
        agent.radius = getFloat(initialConditionsNode->FirstChildElement("radius"));
        agent.position = getFloat3(initialConditionsNode->FirstChildElement("position"));
        agent.direction = getFloat3(initialConditionsNode->FirstChildElement("direction"));
        agent.speed = getFloat(initialConditionsNode->FirstChildElement("speed"));

        agent.goalIndex = getAgentGoals(agentNode->FirstChildElement("goalSequence"), env->agentGoals);

        env->agents.push_back(agent);

        agentNode = agentNode->NextSiblingElement("agent");

    }

    //Agent regions
    auto agentRegionNode = pRoot->FirstChildElement("agentRegion");
    while(agentRegionNode != nullptr)
    {
        Agent agent;

        agent.numAgents = getFloat(agentRegionNode->FirstChildElement("numAgents"));
        agent.regionBounds = getBounds(agentRegionNode->FirstChildElement("regionBounds"));

        auto initialConditionsNode = agentRegionNode->FirstChildElement("initialConditions");
        agent.radius = getFloat(initialConditionsNode->FirstChildElement("radius"));
        agent.position = getFloat3(initialConditionsNode->FirstChildElement("position"));
        agent.direction = getFloat3(initialConditionsNode->FirstChildElement("direction"), &agent.isDirectionRandom);
        agent.speed = getFloat(initialConditionsNode->FirstChildElement("speed"));

        agent.goalIndex = getAgentGoals(agentRegionNode->FirstChildElement("goalSequence"), env->agentGoals);

        env->agentRegions.push_back(agent);

        agentRegionNode = agentRegionNode->NextSiblingElement("agentRegion");

    }


    return env;
}


void expandSpecRegions(ModelEnvSpecPtr env)
{
    std::random_device rd;
    std::mt19937 e2(rd());

    // Expand obstacle regions
    for( auto& obstacleRegion: env->obstacleRegions)
    {
        //Always square obstacle, height not used
        auto obsWidth = obstacleRegion.obstacleSize;
        auto regionBounds = obstacleRegion.regionBounds;
        std::uniform_real_distribution<> distx(regionBounds.min.x, regionBounds.max.x - obsWidth);
//        std::uniform_real_distribution<> disty(regionBounds.min.y, regionBounds.max.y - obsWidth);
        std::uniform_real_distribution<> distz(regionBounds.min.z, regionBounds.max.z - obsWidth);
        for(int i = 0 ; i < obstacleRegion.numObstacles; i++){
            Bounds b;
            b.min = make_float3(distx(e2), 0, distz(e2));
            b.max = make_float3(b.min.x + obsWidth, 0, b.min.z + obsWidth);
            env->obstacles.push_back(b);
        }

    }

    //Expand agent regions
    for( auto& agentRegion: env->agentRegions)
    {
        auto regionBounds = agentRegion.regionBounds;
        std::uniform_real_distribution<> distx(regionBounds.min.x, regionBounds.max.x);
//        std::uniform_real_distribution<> disty(regionBounds.min.y, regionBounds.max.y - obsWidth);
        std::uniform_real_distribution<> distz(regionBounds.min.z, regionBounds.max.z);

        for( int i = 0; i < agentRegion.numAgents; i++)
        {
            Agent agent;
            agent.position = make_float3(distx(e2), 0, distz(e2));
            agent.radius = agentRegion.radius;
            agent.speed = agentRegion.speed;
            if(agentRegion.isDirectionRandom){
                agent.direction = make_float3(dist(e2), 0, dist(e2));
            }
            else{
                agent.direction = agentRegion.direction;
            }

            agent.goalIndex = agentRegion.goalIndex;

            env->agents.push_back(agent);
        }
    }


}


std::vector<float2> getLineFromBounds(Bounds& bounds){
    std::vector<float2> line;
    line.push_back(make_float2(bounds.min.x, bounds.min.z));
    line.push_back(make_float2(bounds.max.x, bounds.min.z));
    line.push_back(make_float2(bounds.max.x, bounds.max.z));
    line.push_back(make_float2(bounds.min.x, bounds.max.z));
    return line;
}

ModelEnvSpecPtr createTestSimSpec(){
    ModelEnvSpecPtr envSpec(new ModelEnvSpec());
    //Create agents by default
    envSpec->envBounds.min = make_float3(-50, -5, -50);
    envSpec->envBounds.max = make_float3(50, 5, 50);

    //Create obstacles
    float subDiv = 2;
    for(int i = 0 ; i < subDiv; i++){
        for( int j = 0; j < subDiv; j++){
            float envWidth = envSpec->envBounds.max.x - envSpec->envBounds.min.x;
            float envHeight = envSpec->envBounds.max.z - envSpec->envBounds.min.z;
            float xSpace = envWidth/4.0f;
            float ySpace = envHeight/4.0f;
            float offx = i*envWidth*0.5f + xSpace + envSpec->envBounds.min.x;
            float offy = j*envHeight*0.5f + ySpace + envSpec->envBounds.min.z;
            float length = 10;
            Bounds obs;
            obs.min = make_float3(offx, 0, offy);
            obs.max = make_float3(offx + length, 0, offy + length);
            envSpec->obstacles.push_back(obs);
        }
    }

    //Merseyrail env
//            Bounds obs0, obs1, obs2, obs3, obs4, obs5;
//            obs0.min = make_float3(0, 0, 0);
//            obs0.max = make_float3(0.1, 1.0, 12.0);
//            obs1.min = make_float3(0, 0, 12);
//            obs1.max = make_float3( 30, 1, 12.1);
//            obs2.min = make_float3( 30, 0, 0);
//            obs2.max = make_float3(30.1, 1.0, 12.0);
//            obs3.min = make_float3( 0, 0, 0);
//            obs3.max = make_float3(2, 1, 0.1);
//            obs4.min = make_float3(4.0, 0.0, 0.0);
//            obs4.max = make_float3(7.0, 1.0, 0.1);
//            obs5.min = make_float3(9.0, 0.0, 0.0);
//            obs5.max = make_float3(30.0, 1.0, 0.1);
//            envSpec->obstacles.push_back(obs0);
//            envSpec->obstacles.push_back(obs1);
//            envSpec->obstacles.push_back(obs2);
//            envSpec->obstacles.push_back(obs3);
//            envSpec->obstacles.push_back(obs4);
//            envSpec->obstacles.push_back(obs5);


    //Create agents
    int w = 10;
    int h = 10;
    float envWidth = envSpec->envBounds.max.x - envSpec->envBounds.min.x;
    float envHeight = envSpec->envBounds.max.z - envSpec->envBounds.min.z;
    float x_space = (envWidth*0.1f)/(float)w ;
    float y_space = (envHeight*0.1f)/(float)h;
    int populationSize = w*h;


    auto ag = AgentGoal();
    ag.goalType = AGENT_GOAL_FLEE_TARGET;
    ag.desiredSpeed = 2.0;
    ag.timeDuration = 3;
    ag.targetLocation = make_float3(25, 0, 0);
    ag.nextIndex = 1;

    auto ag1 = AgentGoal();
    ag1.goalType = AGENT_GOAL_IDLE;
    ag1.desiredSpeed = 2.0;
    ag1.timeDuration = 3;
    ag1.targetLocation = make_float3(25, 0, 0);
    ag1.nextIndex = 2;

    auto ag2 = AgentGoal();
    ag2.goalType = AGENT_GOAL_SEEK_TARGET;
    ag2.desiredSpeed = 2.0;
    ag2.timeDuration = 1000;
    ag2.targetLocation = make_float3(25, 0, 0);
    ag2.nextIndex = -1;

    envSpec->agentGoals.push_back(ag);
    envSpec->agentGoals.push_back(ag1);
    envSpec->agentGoals.push_back(ag2);



    std::default_random_engine rng;
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
//            for(int i = 0; i < w; i++){
//                for(int j = 0; j < h; j++){
//                    Agent agent;
//                    float x = (float)i*x_space;// - (envWidth*0.5f);
//                    float y = (float)j*y_space;// - (envHeight*0.5f);
//                    agent.position = make_float3(x, 0, y);
//                    agent.radius = 0.3;
//                    agent.speed = 2.0f;
//                    agent.direction = make_float3(dist(rng), 0, dist(rng));
//                    agent.goals.push_back(ag);
//                    agent.goals.push_back(ag1);
//                    agent.goals.push_back(ag2);
//                    envSpec->agents.push_back(agent);
//
//
//                }
//            }

    Agent agentRegion;
    agentRegion.numAgents = 10;
    agentRegion.regionBounds.min = make_float3(0,0,0);
    agentRegion.regionBounds.max = make_float3(10,0,10);
    agentRegion.radius = 0.3;
    agentRegion.speed = 2.0f;
    agentRegion.direction = make_float3(dist(rng), 0, dist(rng));
    agentRegion.goalIndex = 0;

    envSpec->agentRegions.push_back(agentRegion);

    return envSpec;
}