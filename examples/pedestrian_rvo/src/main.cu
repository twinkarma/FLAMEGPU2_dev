#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <array>


#include "flamegpu/flame_api.h"

#include "io.cuh"
#include "rvo_kernels.cuh"
#include "goals_kernels.cuh"
#include "RVOGraph.cuh"


#define env_get(x, type) FLAMEGPU->environment.get<type>(#x)
#define agent_get(x, type) FLAMEGPU->getVariable<type>(#x)
#define agent_getFloat(x) agent_get(x, float)
#define agent_getInt(x) agent_get(x, int)
#define agent_set(x, value, type) FLAMEGPU->setVariable<type>(#x, value)

FLAMEGPU_AGENT_FUNCTION(output_pedestrian_location, MsgNone, MsgSpatial3D) {
    // Output each agents publicly visible properties.
    FLAMEGPU->message_out.setVariable<int>("id", FLAMEGPU->getVariable<int>("id"));
     FLAMEGPU->message_out.setVariable<float>("x", FLAMEGPU->getVariable<float>("x"));
     FLAMEGPU->message_out.setVariable<float>("y", FLAMEGPU->getVariable<float>("y"));
     FLAMEGPU->message_out.setVariable<float>("z", FLAMEGPU->getVariable<float>("z"));
    FLAMEGPU->message_out.setVariable<float>("velx", FLAMEGPU->getVariable<float>("velx"));
    FLAMEGPU->message_out.setVariable<float>("vely", FLAMEGPU->getVariable<float>("vely"));
    FLAMEGPU->message_out.setVariable<float>("radius", FLAMEGPU->getVariable<float>("radius"));

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(move, MsgSpatial3D, MsgNone) {

    //Input
    RVOLine orcaLines[ORCA_ARRAY_SIZE];
    Ped neighbours[ORCA_ARRAY_SIZE];
    int orcaLineCount = 0;
    int neighbourCount = 0;
    float simTimeStep = FLAMEGPU->environment.get<float>("TIME_SCALER");
    float timeHorizon = FLAMEGPU->environment.get<float>("TIME_HORIZON");;
    float timeHorizonObst = FLAMEGPU->environment.get<float>("TIME_HORIZON_OBSTACLE");;
    float agentMaxSpeed = FLAMEGPU->environment.get<float>("MAXIMUM_SPEED");
    float interactionRange  = FLAMEGPU->environment.get<float>("INTERACTION_RANGE");
    float goalReachedDistance = 1.0f;

    int id = FLAMEGPU->getVariable<int>("id");
    float2 agentPos = make_float2(agent_getFloat(x), agent_getFloat(z));
    float2 agentVel = make_float2(agent_getFloat(velx), agent_getFloat(vely));
    float agentRadius = agent_getFloat(radius);

    int goalIndex = agent_getInt(goalIndex);
    int goalType = agent_getInt(goalType);
    float2 agentDest = make_float2(agent_getFloat(destx), agent_getFloat(desty));
    float desiredSpeed = agent_getFloat(desiredSpeed);
    float timeDuration = agent_getFloat(timeDuration);
    float2 targetVector = make_float2(0,0);

    //Output
    float2 agentObsVector = make_float2(0, 0);
    int obsVectorCount = 0;
    float2 agent_newvel = make_float2(0, 0);

    //Goals
    timeDuration -= simTimeStep;

    if(goalIndex < 0 )
        return DEAD; //Kill agent if no goal index

    if(timeDuration < 0 ||
            (goalType == AGENT_GOAL_SEEK_TARGET && length(agentDest - agentPos) <= goalReachedDistance)
            )
    {
        //Reached current goal objective, move on to next goal

        goalIndex = nextAgentGoalIndex(goalIndex);
        if(goalIndex < 0)
            //Reached goal, kill agent
            return DEAD;
        else{
            //Has another goal, get parameters
            getAgentGoal(goalIndex, goalType, agentDest, desiredSpeed, timeDuration);
        }
    }
    else{
        //Navigate to goal,
        switch (goalType) {
            case AGENT_GOAL_SEEK_TARGET:
                targetVector = normalize(agentDest - agentPos) * desiredSpeed;
                break;
            case AGENT_GOAL_FLEE_TARGET:
                targetVector = normalize(agentPos - agentDest) * desiredSpeed;
                break;
            case AGENT_GOAL_IDLE:
                //Target vector defaults at 0,0
                break;
        }

    }


    //Obstacles
     getObstacles(orcaLines, orcaLineCount, timeHorizonObst,
                  agentPos, agentVel, agentRadius, agentMaxSpeed, agentObsVector, obsVectorCount);
     int numObstLines = orcaLineCount;

    if (obsVectorCount > 0)
        agentObsVector = agentObsVector / ((float) obsVectorCount * 2.0f);

    //Other pedestrian avoidance
    for (const auto &message : FLAMEGPU->message_in(agentPos.x, agentPos.y, 0)){
        // Ignore self messages.
        if (message.getVariable<int>("id") != id) {
            const float message_x = message.getVariable<float>("x");
            const float message_y = message.getVariable<float>("y");
            const float message_z = message.getVariable<float>("z");
            const float message_velx = message.getVariable<float>("velx");
            const float message_vely = message.getVariable<float>("vely");
            const float message_radius = message.getVariable<float>("radius");
            float2 other_pos = make_float2(message_x, message_z); //Get position
            float2 other_vel = make_float2(message_velx, message_vely);
            float2 replusion_vec = agentPos - other_pos;
            float separationSq = absSq(replusion_vec);

            if (separationSq > 0.001f && separationSq < interactionRange * interactionRange) {
                ///Need to sort agent by distance
                Ped p;
                p.pos = other_pos;
                p.vel = other_vel;
                p.radius = message_radius;
                p.separationSq = separationSq;
                addNeighbourSorted(neighbours, neighbourCount, p);

            }
        }

    }

    for (int i = 0;i < neighbourCount;i++) {
        addOrcaAgent(orcaLines, orcaLineCount, timeHorizon, simTimeStep,
                     agentPos, agentVel, agentRadius,
                     neighbours[i].pos, neighbours[i].vel, neighbours[i].radius);
    }

    //Perform collision avoidance
    performCollisionAvoidance(orcaLines, orcaLineCount, numObstLines, agentMaxSpeed, targetVector, agent_newvel);

    //Update
    agentPos = agent_newvel * simTimeStep + agentPos;

    agent_set(x, agentPos.x, float);
    agent_set(z, agentPos.y, float);
    agent_set(velx, agent_newvel.x, float);
    agent_set(vely, agent_newvel.y, float);

    agent_set(goalIndex, goalIndex, int);
    agent_set(goalType, goalType, int);
    agent_set(destx, agentDest.x, float);
    agent_set(desty, agentDest.y, float);
    agent_set(desiredSpeed, desiredSpeed, float);
    agent_set(timeDuration, timeDuration, float);

    return ALIVE;

}

void drawBounds(LineVis& pen, Bounds& bounds)
{
    pen.addVertex(bounds.max.x, 0, bounds.max.z);
    pen.addVertex(bounds.min.x, 0, bounds.max.z);
    pen.addVertex(bounds.max.x, 0, bounds.min.z);
    pen.addVertex(bounds.min.x, 0, bounds.min.z);

    pen.addVertex(bounds.max.x, 0, bounds.max.z);
    pen.addVertex(bounds.max.x, 0, bounds.min.z);
    pen.addVertex(bounds.min.x, 0, bounds.max.z);
    pen.addVertex(bounds.min.x, 0, bounds.min.z);

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

    auto ag1 = AgentGoal();
    ag1.goalType = AGENT_GOAL_IDLE;
    ag1.desiredSpeed = 2.0;
    ag1.timeDuration = 3;
    ag1.targetLocation = make_float3(25, 0, 0);

    auto ag2 = AgentGoal();
    ag2.goalType = AGENT_GOAL_SEEK_TARGET;
    ag2.desiredSpeed = 2.0;
    ag2.timeDuration = 1000;
    ag2.targetLocation = make_float3(25, 0, 0);



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
    agentRegion.goals.push_back(ag);
    agentRegion.goals.push_back(ag1);
    agentRegion.goals.push_back(ag2);
    envSpec->agentRegions.push_back(agentRegion);

    return envSpec;
}

int main(int argc, const char ** argv) {



    ModelEnvSpecPtr envSpec = nullptr;

    for(int i = 0; i < argc; i++){
        std::string arg(argv[i]);

        if(arg.compare("-m") == 0 || arg.compare("--m") == 0){
            if(i + 1 < argc){
                auto modelFilePath = argv[i+1];
                printf("Loading xml spec %s", modelFilePath);
                try{
                    envSpec = importSteerBenchXML(modelFilePath);
                }catch(std::exception* e){
                    printf("Could not load xml spec file %s", modelFilePath);
                }
            }
        }
    }

    try{
        /**
        * Environment boundaries
        *
        */
        float envCommRadius = 5.0f;
        float separationRadius = 0.5f;


        if(!envSpec)
        {
            printf("No initial environment settings, running test case");
            envSpec = createTestSimSpec();
        }

        if(envSpec){

            //Check that environment has height
            auto envSize = envSpec->envBounds.max - envSpec->envBounds.min;
            if(envSize.y <= 0)
                envSpec->envBounds.min.y = -envCommRadius;
            envSpec->envBounds.max.y = envCommRadius;

            indexAgentGoals(envSpec);
            expandSpecRegions(envSpec);
        }
        else{
            printf("No model environment spec");
            return 1;
        }





        /**
         * Create pedestrian model
         *
         */
        ModelDescription model("pedestrian_rvo");

        /**
         * GLOBALS
         */
        {
            EnvironmentDescription &env = model.Environment();
            env.add("TIME_SCALER", 0.02f);
            env.add("TIME_HORIZON", 0.5f);
            env.add("TIME_HORIZON_OBSTACLE", 2.0f);
            env.add("MAXIMUM_SPEED", 3.0f);
            env.add("INTERACTION_RANGE", 5.0f);
        }

        /**
         * Messages
         *
         */
        {
            EnvironmentDescription &env = model.Environment();
            MsgSpatial3D::Description &message = model.newMessage<MsgSpatial3D>("pedestrian_location");
            // Set the range and bounds.
            message.setRadius(envCommRadius);
            message.setMin(envSpec->envBounds.min.x, envSpec->envBounds.min.y, envSpec->envBounds.min.z);
            message.setMax(envSpec->envBounds.max.x, envSpec->envBounds.max.z, envSpec->envBounds.max.z);

            // A message to hold the location of an agent.
            message.newVariable<int>("id");
            // X Y Z are implicit.
            // message.newVariable<float>("x");
            // message.newVariable<float>("y");
            // message.newVariable<float>("z");
            message.newVariable<float>("velx");
            message.newVariable<float>("vely");
            message.newVariable<float>("radius");
        }

        /**
         * Agents
         *
         */
        {
            AgentDescription &agent = model.newAgent("Pedestrian");
            agent.newVariable<int>("id");
            agent.newVariable<float>("x", 0);
            agent.newVariable<float>("y", 0);
            agent.newVariable<float>("z", 0);
            agent.newVariable<float>("velx", 0);
            agent.newVariable<float>("vely", 0);
            agent.newVariable<float>("radius", 0.25);



            //Goal-based behaviour variables
            agent.newVariable<int>("goalIndex", -1);
            agent.newVariable<int>("goalType", 0);
            agent.newVariable<float>("destx", 0);
            agent.newVariable<float>("desty", 0);
            agent.newVariable<float>("desiredSpeed", 0);
            agent.newVariable<float>("timeDuration", 0);

            //Agent functions
            agent.newFunction("output_pedestrian_location", output_pedestrian_location).setMessageOutput("pedestrian_location");
            agent.newFunction("move", move).setMessageInput("pedestrian_location");
        }

        /**
        * Control flow
        *
        */
        {
            LayerDescription &layer = model.newLayer();
            layer.addAgentFunction(output_pedestrian_location);
        }
        {
            LayerDescription &layer = model.newLayer();
            layer.addAgentFunction(move);
        }

        /**
        * Model runner
        *
        */
        CUDASimulation cuda_model(model);


        /**
        * Visualisation
        *
        */
        ModelVis &visualisation = cuda_model.getVisualisation();
        {
            EnvironmentDescription &env = model.Environment();
            float envWidth = envSpec->envBounds.max.x - envSpec->envBounds.min.x;
            const float INIT_CAM = envSpec->envBounds.max.x * 1.25f;
            visualisation.setInitialCameraLocation(INIT_CAM, INIT_CAM, INIT_CAM);
            visualisation.setCameraSpeed(0.002f * envWidth);
            auto &ped_agt = visualisation.addAgent("Pedestrian");
            // Position vars are named x, y, z; so they are used by default
            ped_agt.setModel(Stock::Models::ICOSPHERE);
            ped_agt.setModelScale(separationRadius);

            //Env bounds
            {
                auto pen = visualisation.newLineSketch(1, 1, 1, 0.2f);  // white
                drawBounds(pen, envSpec->envBounds);

                //Visualise obstacles
                for( auto& obs : envSpec->obstacles){
                    drawBounds(pen, obs);
                }
            }

        }
        visualisation.activate();

        // Initialisation
        cuda_model.initialise(1, argv);

        // Generate a population from spec
        {

            int populationSize = envSpec->agents.size();
            int id = 0;
            AgentPopulation population(model.Agent("Pedestrian"), populationSize);
            for(auto& agentSpec: envSpec->agents)
            {
                auto agentVel = agentSpec.direction*agentSpec.speed;

                AgentInstance instance = population.getNextInstance();
                instance.setVariable<int>("id", id);
                instance.setVariable<float>("x", agentSpec.position.x);
                instance.setVariable<float>("y", agentSpec.position.y);
                instance.setVariable<float>("z", agentSpec.position.z);
                instance.setVariable<float>("velx", agentVel.x);
                instance.setVariable<float>("vely", agentVel.z);
                instance.setVariable<float>("radius", agentSpec.radius);

                //Load first goal into agent
                instance.setVariable<int>("goalIndex", agentSpec.goalIndex);
                if(agentSpec.goals.size() > 0){

                    instance.setVariable<int>("goalType", agentSpec.goals[0].goalType);
                    instance.setVariable<float>("destx", agentSpec.goals[0].targetLocation.x);
                    instance.setVariable<float>("desty", agentSpec.goals[0].targetLocation.z);
                    instance.setVariable<float>("desiredSpeed", agentSpec.goals[0].desiredSpeed);
                    instance.setVariable<float>("timeDuration", agentSpec.goals[0].timeDuration);
                }


                id += 1;
            }
            cuda_model.setPopulationData(population);
        }

        /**
         * Build RVO data structures
         *
         */
        auto rvoGraph = new RVOGraph();
        std::vector<std::vector<float2>> obstacles;
        for( auto& obs: envSpec->obstacles)
            obstacles.push_back(getLineFromBounds(obs));
        rvoGraph->buildRVO(obstacles, getRVOObstaclePointer(), getRVOKDNodePointer());


        /**
         * Upload agent goals
         */
        uploadAgentGoals();

        /**
         * Execution
         */
        cuda_model.simulate();

        visualisation.join();

    }
    catch(std::exception* e){
        printf("Error: %s", e->what());
    }



    return 0;

}