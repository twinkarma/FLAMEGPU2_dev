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


#define fg_getfloat(x) FLAMEGPU->getVariable<float>(#x)


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

    int id = FLAMEGPU->getVariable<int>("id");
    float2 agentPos = make_float2(fg_getfloat(x), fg_getfloat(z));
    float2 agentDest = make_float2(fg_getfloat(destx), fg_getfloat(desty));
    float2 agentVel = make_float2(fg_getfloat(velx), fg_getfloat(vely));
    float agentRadius = fg_getfloat("radius");

    //Output
    float2 agentObsVector = make_float2(0, 0);
    int obsVectorCount = 0;
    float2 agent_newvel = make_float2(0, 0);

    ///Gets obstacles first
     getObstacles(orcaLines, orcaLineCount, timeHorizonObst,
                  agentPos, agentVel, agentRadius, agentMaxSpeed, agentObsVector, obsVectorCount);
     int numObstLines = orcaLineCount;

    if (obsVectorCount > 0)
        agentObsVector = agentObsVector / ((float) obsVectorCount * 2.0f);

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

    float2 prefVel = agentDest * agentMaxSpeed * 0.5f;

    performCollisionAvoidance(orcaLines, orcaLineCount, numObstLines, agentMaxSpeed, prefVel, agent_newvel);

    agentPos = agent_newvel * simTimeStep + agentPos;

    FLAMEGPU->setVariable<float>("x", agentPos.x);
    FLAMEGPU->setVariable<float>("z", agentPos.y);
    FLAMEGPU->setVariable<float>("velx", agent_newvel.x);
    FLAMEGPU->setVariable<float>("vely", agent_newvel.y);

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



int main(int argc, const char ** argv) {

    ModelEnvSpecPtr envSpec = nullptr;
    try{
        envSpec = importSteerBenchXML("merseyrail.xml");
        indexAgentGoals(envSpec);
        expandSpecRegions(envSpec);
    }catch(std::exception* e){
        printf("Could not load xml spec file ");
    }

    if(!envSpec)
    {
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

        //Create agents
        int w = 10;
        int h = 10;
        float envWidth = envSpec->envBounds.max.x - envSpec->envBounds.min.x;
        float envHeight = envSpec->envBounds.max.z - envSpec->envBounds.min.z;
        float x_space = (envWidth*0.1f)/(float)w ;
        float y_space = (envHeight*0.1f)/(float)h;
        int populationSize = w*h;
        int id = 0;

        std::default_random_engine rng;
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        for(int i = 0; i < w; i++){
            for(int j = 0; j < h; j++){
                Agent agent;
                float x = (float)i*x_space;// - (envWidth*0.5f);
                float y = (float)j*y_space;// - (envHeight*0.5f);
                agent.position = make_float3(x, 0, y);
                agent.direction = make_float3(dist(rng), 0, dist(rng));
                id += 1;

            }
        }
    }


    /**
     * Environment boundaries
     * 
     */
     float envCommRadius = 5.0f;
     float3 envMin = envSpec->envBounds.min;
     float3 envMax = envSpec->envBounds.max;
     float separationRadius = 0.5f;

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
        message.setMin(envMin.x, envMin.y, envMin.z);
        message.setMax(envMax.x, envMax.y, envMax.z);

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
        float envWidth = envMax.x - envMin.x;
        const float INIT_CAM = envMax.x * 1.25f;
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
    cuda_model.initialise(argc, argv);

    // Generate a population from spec
    {

        int populationSize = envSpec->agents.size();
        int id = 0;
        AgentPopulation population(model.Agent("Pedestrian"), populationSize);
        for(auto& agentSpec: envSpec->agents)
        {
            auto agentDes = make_float3(0,0,0);
            if(agentSpec.direction.x != 0 || agentSpec.direction.z != 0)
                agentDes = normalize(agentSpec.direction);
            auto agentVel = agentDes*agentSpec.speed;

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

    return 0;

}