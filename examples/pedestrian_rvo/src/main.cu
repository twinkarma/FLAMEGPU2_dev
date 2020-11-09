#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include "flamegpu/flame_api.h"

#include "rvo_kernels.cuh"
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
    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(move, MsgSpatial3D, MsgNone) {

    //Input
    RVOLine orcaLines[ORCA_ARRAY_SIZE];
    Ped neighbours[ORCA_ARRAY_SIZE];
    int orcaLineCount = 0;
    int neighbourCount = 0;
    float timeHorizonObst = 2.0f;
    float timeHorizon = 0.5f;
    float simTimeStep = FLAMEGPU->environment.get<float>("TIME_SCALER");
    float agentRadius = 0.15f;
    float agentMaxSpeed = 3.0f;
    float INTERACTION_RANGE  = FLAMEGPU->environment.get<float>("INTERACTION_RANGE");

    int id = FLAMEGPU->getVariable<int>("id");
    float2 agent_pos = make_float2(fg_getfloat(x), fg_getfloat(y));
    float2 dest_pos = make_float2(fg_getfloat(destx), fg_getfloat(desty));
    float2 agent_vel = make_float2(fg_getfloat(velx),fg_getfloat(vely));

    //Output
    float2 agentObsVector = make_float2(0, 0);
    int obsVectorCount = 0;
    float2 agent_newvel = make_float2(0, 0);

    ///Gets obstacles first
    getObstacles(orcaLines, orcaLineCount, timeHorizonObst, agent_pos, agent_vel, agentRadius, agentMaxSpeed,agentObsVector, obsVectorCount);
    int numObstLines = orcaLineCount;

    if (obsVectorCount > 0)
        agentObsVector = agentObsVector / ((float) obsVectorCount * 2.0f);

    for (const auto &message : FLAMEGPU->message_in(agent_pos.x, agent_pos.y, 0)){
        // Ignore self messages.
        if (message.getVariable<int>("id") != id) {
            const float message_x = message.getVariable<float>("x");
            const float message_y = message.getVariable<float>("y");
            const float message_z = message.getVariable<float>("z");
            const float message_velx = message.getVariable<float>("velx");
            const float message_vely = message.getVariable<float>("vely");
            float2 other_pos = make_float2(message_x, message_y); //Get position
            float2 other_vel = make_float2(message_velx, message_vely);
            float2 replusion_vec = agent_pos - other_pos;
            float separationSq = absSq(replusion_vec); 

            if (separationSq > 0.001f && separationSq < INTERACTION_RANGE * INTERACTION_RANGE) {
                ///Need to sort agent by distance
                Ped p;
                p.pos = other_pos;
                p.vel = other_vel;
                p.separationSq = separationSq;
                addNeighbourSorted(neighbours, neighbourCount, p);

            }

        }

    }

    for (int i = 0;i < neighbourCount;i++) {
        addOrcaAgent(orcaLines, orcaLineCount, timeHorizon, simTimeStep, agent_pos, agent_vel, agentRadius,neighbours[i].pos, neighbours[i].vel, agentRadius);
    }

    float2 prefVel = dest_pos*agentMaxSpeed*0.5f;

    performCollisionAvoidance(orcaLines, orcaLineCount, numObstLines, agentMaxSpeed, prefVel, agent_newvel);

    agent_pos = agent_newvel*simTimeStep + agent_pos;

    FLAMEGPU->setVariable<float>("x", agent_pos.x);
    FLAMEGPU->setVariable<float>("y", agent_pos.y);
    FLAMEGPU->setVariable<float>("velx", agent_newvel.x);
    FLAMEGPU->setVariable<float>("vely", agent_newvel.y);

    return ALIVE;

}

int main(int argc, const char ** argv) {

    


    /**
     * Build RVO data structures
     * 
     */
    auto rvoGraph = new RVOGraph();
    




    /**
     * Create pedestrian model
     * 
     */
    ModelDescription model("pedestrian_rvo");

    /**
     * GLOBALS
     */
    {
        auto env = model.Environment();

            
        env.add("TIME_SCALER", 0.2f);    
        env.add("INTERACTION_RANGE", 5.0f);
   
    }

    /**
     * Messages
     * 
     */
    {
        auto env = model.Environment();
        MsgSpatial3D::Description &message = model.newMessage<MsgSpatial3D>("pedestrian_location");
        // Set the range and bounds.
        message.setRadius(5.0);
        message.setMin(-0.5, -0.5, 0.0);
        message.setMax(0.5, 0.5, 0.05);

        // A message to hold the location of an agent.
        message.newVariable<int>("id");
        // X Y Z are implicit.
        // message.newVariable<float>("x");
        // message.newVariable<float>("y");
        // message.newVariable<float>("z");
        message.newVariable<float>("velx");
        message.newVariable<float>("vely");
    
        
        

    }

    /**
     * Agents
     * 
     */
    {
        AgentDescription &agent = model.newAgent("Pedestrian");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
        agent.newVariable<float>("y");
        agent.newVariable<float>("z");
        agent.newVariable<float>("velx");
        agent.newVariable<float>("vely");
        agent.newVariable<float>("target_speed");
        agent.newVariable<int>("kill");

        //Animation
        agent.newVariable<int>("lod");
        agent.newVariable<float>("animate");
        agent.newVariable<int>("animate_dir");

        //Navigation - a vector to the the destination
        agent.newVariable<float>("destx");
        agent.newVariable<float>("desty");

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
        float envWidth = env.get<float>("MAX_POSITION") - env.get<float>("MIN_POSITION");
        const float INIT_CAM = env.get<float>("MAX_POSITION") * 1.25f;
        visualisation.setInitialCameraLocation(INIT_CAM, INIT_CAM, INIT_CAM);
        visualisation.setCameraSpeed(0.002f * envWidth);
        auto &circ_agt = visualisation.addAgent("Pedestrian");
        // Position vars are named x, y, z; so they are used by default
        circ_agt.setModel(Stock::Models::ICOSPHERE);
        circ_agt.setModelScale(env.get<float>("SEPARATION_RADIUS"));

    }
    visualisation.activate();


    // Initialisation
    cuda_model.initialise(argc, argv);

    // If no xml model file was is provided, generate a population.
    if (cuda_model.getSimulationConfig().input_file.empty()) {
        int populationSize = 3;
        AgentPopulation population(model.Agent("Pedestrian"), populationSize);
        for(unsigned int i = 0; i < populationSize; i++){
            auto instance = population.getNextInstance();
            instance.setVariable<int>("id", i);
            instance.setVariable<float>("x", 0);
            instance.setVariable<float>("y", 0);
            instance.setVariable<float>("z", 0);
            instance.setVariable<float>("velx", 0);
            instance.setVariable<float>("vely", 0);


        }

    }

    /**
     * Execution
     */
    cuda_model.simulate();

    visualisation.join();

    return 0;

}