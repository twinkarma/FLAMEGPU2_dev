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
#include "steersuite/RecFileIO.h"


#define env_get(x, type) FLAMEGPU->environment.get<type>(#x)
#define agent_get(x, type) FLAMEGPU->getVariable<type>(#x)
#define agent_getFloat(x) agent_get(x, float)
#define agent_getInt(x) agent_get(x, int)
#define agent_set(x, value, type) FLAMEGPU->setVariable<type>(#x, value)

/**
 *
 *
 * GPU Kernels
 *
 */

/**
 * Output the pedestrian's location
 */
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

/**
 * Move pedestrian, performs goal lookup, collision avoidance then movement
 */
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

/**
 *
 * HOST Code
 *
 */

/**
 *
 */
std::string recordOutputPath;
SimulationSpecPtr simSpec = nullptr;
ModelDescription* model = nullptr;
CUDASimulation* cudaSim = nullptr;

SteerLib::RecFileWriter* fileWriter = nullptr;

FLAMEGPU_INIT_FUNCTION(init_func){
    printf("Initialising simulation\n");
    fileWriter = new SteerLib::RecFileWriter();
    if(!recordOutputPath.empty()){
        auto pedAgent = FLAMEGPU->agent("Pedestrian");
        auto numAgent = pedAgent.count();
        fileWriter->startRecording(numAgent, recordOutputPath);
        printf("Started recording simulation at file %s\n", recordOutputPath.c_str());
    }
}


FLAMEGPU_STEP_FUNCTION(step_func){

    if(!fileWriter->isRecording())
        return;

    auto stepCount = FLAMEGPU->getStepCounter();
    auto timeScaler = FLAMEGPU->environment.get<float>("TIME_SCALER");
    fileWriter->startFrame((float)stepCount*timeScaler, timeScaler);
    auto pedAgent = FLAMEGPU->agent("Pedestrian");
    AgentPopulation pop(model->getAgent("Pedestrian"));
    cudaSim->getPopulationData(pop);
    auto currentPedNum = pop.getCurrentListSize();
    for(unsigned int i = 0; i < currentPedNum; i++){
        auto agent = pop.getInstanceAt(i);
        int id = agent.getVariable<int>("id");
        float x = agent.getVariable<float>("x");
        float y = agent.getVariable<float>("y");
        float z = agent.getVariable<float>("z");
        float dirx = agent.getVariable<float>("velx");
        float diry = 0.0f;
        float dirz = agent.getVariable<float>("vely");
        float goalx = agent.getVariable<float>("destx");
        float goaly = 0.0f;
        float goalz = agent.getVariable<float>("desty");
        float radius = agent.getVariable<float>("radius");
        fileWriter->setAgentInfoForCurrentFrame(i, x, y, z, dirx, diry, dirz, goalx, goaly, goalz, radius, true);
    }
    fileWriter->finishFrame();


}



FLAMEGPU_EXIT_CONDITION(exit_condition){
    //Leaving this blank for now
    return CONTINUE;
}


FLAMEGPU_EXIT_FUNCTION(exit_func){
    if(fileWriter->isRecording()){
        fileWriter->finishRecording();
        printf("Stopped recording simulation\n");
    }



    printf("Exiting simulation\n");
}

/**
 * Draw bounds using LineVis pen for visualisation
 * @param pen
 * @param bounds
 */
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

/**
 * Pedestrian RVO model
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, const char ** argv) {

    std::vector<const char*> fgpuArgs;

    /**
     * Get initialisation parameters
     */
    for(int i = 0; i < argc; i++){
        std::string arg(argv[i]);

        if(arg.compare("-m") == 0 || arg.compare("--model") == 0){
            if(i + 1 < argc){
                auto modelFilePath = argv[++i];
                printf("Loading xml spec %s\n", modelFilePath);
                try{
                    simSpec = importSteerBenchXML(modelFilePath);
                }catch(std::exception* e){
                    printf("Could not load xml spec file %s", modelFilePath);
                    exit(1);
                }
            }
        }
        else if(arg.compare("-o") == 0 || arg.compare("--output") == 0){
            if(i + 1 < argc){
                auto outputFilePath = argv[++i];
                recordOutputPath = std::string(outputFilePath);
            }
        }
        else{
            fgpuArgs.push_back(argv[i]);
        }
    }

    try{
        /**
        * Environment boundaries
        *
        */
        float envCommRadius = 5.0f;
        float separationRadius = 0.5f;


        if(!simSpec)
        {
            printf("No initial environment settings, running test case\n");
            simSpec = createTestSimSpec();
        }

        if(simSpec){

            //Check that environment has height
            auto envSize = simSpec->envBounds.max - simSpec->envBounds.min;
            if(envSize.y <= 0)
                simSpec->envBounds.min.y = -envCommRadius;
            simSpec->envBounds.max.y = envCommRadius;
            expandSpecRegions(simSpec);
        }
        else{
            printf("No model environment spec");
            return 1;
        }

        /**
         * Create pedestrian model
         *
         */
        model = new ModelDescription("pedestrian_rvo");


        /**
         * GLOBALS
         */
        {
            EnvironmentDescription &env = model->Environment();
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
            EnvironmentDescription &env = model->Environment();
            MsgSpatial3D::Description &message = model->newMessage<MsgSpatial3D>("pedestrian_location");
            // Set the range and bounds.
            message.setRadius(envCommRadius);
            message.setMin(simSpec->envBounds.min.x, simSpec->envBounds.min.y, simSpec->envBounds.min.z);
            message.setMax(simSpec->envBounds.max.x, simSpec->envBounds.max.z, simSpec->envBounds.max.z);

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
            AgentDescription &agent = model->newAgent("Pedestrian");
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
         * Model event handlers
         */
        {
            model->addInitFunction(init_func);
            model->addStepFunction(step_func);
            model->addExitFunction(exit_func);
            model->addExitCondition(exit_condition);
        }

        /**
        * Control flow
        *
        */
        {
            LayerDescription &layer = model->newLayer();
            layer.addAgentFunction(output_pedestrian_location);
        }
        {
            LayerDescription &layer = model->newLayer();
            layer.addAgentFunction(move);
        }


        /**
        * Model runner
        *
        */
        cudaSim = new CUDASimulation(*model);


        /**
        * Visualisation
        *
        */
        ModelVis &visualisation = cudaSim->getVisualisation();
        {
            EnvironmentDescription &env = model->Environment();
            float envWidth = simSpec->envBounds.max.x - simSpec->envBounds.min.x;
            const float INIT_CAM = simSpec->envBounds.max.x * 1.25f;
            visualisation.setInitialCameraLocation(INIT_CAM, INIT_CAM, INIT_CAM);
            visualisation.setCameraSpeed(0.002f * envWidth);
            visualisation.setClearColor(0.8, 0.8, 0.8);
            auto &ped_agt = visualisation.addAgent("Pedestrian");
            // Position vars are named x, y, z; so they are used by default
            ped_agt.setModel("resources/cylinder_geom.obj");
            ped_agt.setModelScale(0.6, 1.8 ,0.6);

            //Env bounds
            {
                auto pen = visualisation.newLineSketch(1, 1, 1, 0.2f);  // white
                drawBounds(pen, simSpec->envBounds);

                //Visualise obstacles
                if(simSpec->obstacles.size() > 0){
                    auto obsPen = visualisation.newLineSketch(0.3, 0.3, 1, 0.8f);
                    for( auto& obs : simSpec->obstacles){
                        drawBounds(obsPen, obs);
                    }
                }

                //Visualise obstacle regions
                if(simSpec->obstacleRegions.size() > 0){
                    auto obsRegionPen = visualisation.newLineSketch(0.6, 0.6, 1, 0.2f);  // white
                    for( auto & obsRegion: simSpec->obstacleRegions){
                        drawBounds(obsRegionPen, obsRegion.regionBounds);
                    }
                }

                //Visualise agent regions
                if(simSpec->agentRegions.size() > 0){
                    auto agentRegionPen = visualisation.newLineSketch(0.3, 1, 0.3, 0.2f);  // white
                    for( auto & agentRegion: simSpec->agentRegions){
                        drawBounds(agentRegionPen, agentRegion.regionBounds);
                    }
                }
            }

        }
        visualisation.activate();

        // Initialisation
        cudaSim->initialise(fgpuArgs.size(), &fgpuArgs[0]);

        // Generate a population from spec
        {

            int populationSize = simSpec->agents.size();
            int id = 0;
            AgentPopulation population(model->Agent("Pedestrian"), populationSize);
            for(auto& agentSpec: simSpec->agents)
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
                if(agentSpec.goalIndex > -1){

                    instance.setVariable<int>("goalType", simSpec->agentGoals[agentSpec.goalIndex].goalType);
                    instance.setVariable<float>("destx", simSpec->agentGoals[agentSpec.goalIndex].targetLocation.x);
                    instance.setVariable<float>("desty", simSpec->agentGoals[agentSpec.goalIndex].targetLocation.z);
                    instance.setVariable<float>("desiredSpeed", simSpec->agentGoals[agentSpec.goalIndex].desiredSpeed);
                    instance.setVariable<float>("timeDuration", simSpec->agentGoals[agentSpec.goalIndex].timeDuration);
                }

                id += 1;
            }
            cudaSim->setPopulationData(population);
        }

        /**
         * Build RVO data structures
         *
         */
        auto rvoGraph = new RVOGraph();
        std::vector<std::vector<float2>> obstacles;
        for( auto& obs: simSpec->obstacles)
            obstacles.push_back(getLineFromBounds(obs));
        rvoGraph->buildRVO(obstacles);
        rvoGraph->uploadRVOToGPU(getRVOObstaclePointer(), getRVOKDNodePointer());

        /**
         * Upload agent goals
         */
        uploadAgentGoals(simSpec);

        /**
         * Execution
         */
        cudaSim->simulate();

        visualisation.join();

    }
    catch(std::exception* e){
        printf("Error: %s", e->what());
    }

    return 0;
}