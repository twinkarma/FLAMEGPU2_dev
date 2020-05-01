#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>


#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"
#include "flamegpu/io/factory.h"
#include "flamegpu/util/nvtx.h"


// @todo - need to abstract __device__ away from the modeller.
// Get the length of a vector
inline __host__ __device__ float vec3Length(const float x, const float y, const float z) {
    return sqrtf(x * x + y * y + z * z);
}

// Method to scale a vector 
__host__ __device__ void vec3Mult(float *x, float *y, float *z, const float multiplier){
    *x *= multiplier;
    *y *= multiplier;
    *z *= multiplier;
}

// Method to scale a vector 
__host__ __device__ void vec3Div(float *x, float *y, float *z, const float divisor){
    *x /= divisor;
    *y /= divisor;
    *z /= divisor;
}

// Method to normalize a vector of 3 points inplace.
__host__ __device__ void vec3Normalize(float* x, float* y, float* z) {
    // Get the length
    float length = vec3Length(*x, *y, *z);
    vec3Div(x, y, z, length);
}

// Bound the agent to the environment
// @todo - this is actually wrapping? Should this be claming instead?
__device__ void boundPosition(float &x, float &y, float &z, const float MIN_POSITION, const float MAX_POSITION) {
    // @todo switch to env vars.
    // Bound x.
    x = (x < MIN_POSITION)? MAX_POSITION: x;
    x = (x > MAX_POSITION)? MIN_POSITION: x;

    y = (y < MIN_POSITION)? MAX_POSITION: y;
    y = (y > MAX_POSITION)? MIN_POSITION: y;

    z = (z < MIN_POSITION)? MAX_POSITION: z;
    z = (z > MAX_POSITION)? MIN_POSITION: z;
}



// Agent function to output the message
FLAMEGPU_AGENT_FUNCTION(outputdata, MsgNone, MsgBruteForce) {
    // Output each agents publicly visible properties.
    FLAMEGPU->message_out.setVariable<int>("id", FLAMEGPU->getVariable<int>("id"));
    FLAMEGPU->message_out.setVariable<float>("x", FLAMEGPU->getVariable<float>("x"));
    FLAMEGPU->message_out.setVariable<float>("y", FLAMEGPU->getVariable<float>("y"));
    FLAMEGPU->message_out.setVariable<float>("z", FLAMEGPU->getVariable<float>("z"));
    FLAMEGPU->message_out.setVariable<float>("fx", FLAMEGPU->getVariable<float>("fx"));
    FLAMEGPU->message_out.setVariable<float>("fy", FLAMEGPU->getVariable<float>("fy"));
    FLAMEGPU->message_out.setVariable<float>("fz", FLAMEGPU->getVariable<float>("fz"));
    return ALIVE;
}
// Agent function to iterate messages and perform the boids behaviours.
FLAMEGPU_AGENT_FUNCTION(inputdata, MsgBruteForce, MsgNone) {
    // Agent properties in local register
    int id = FLAMEGPU->getVariable<int>("id");
    // Agent position
    float agent_x = FLAMEGPU->getVariable<float>("x");
    float agent_y = FLAMEGPU->getVariable<float>("y");
    float agent_z = FLAMEGPU->getVariable<float>("z");
    // Agent velocity
    float agent_fx = FLAMEGPU->getVariable<float>("fx");
    float agent_fy = FLAMEGPU->getVariable<float>("fy");
    float agent_fz = FLAMEGPU->getVariable<float>("fz");

    // Boids percieved center
    float perceived_centre_x = 0.0f;
    float perceived_centre_y = 0.0f;
    float perceived_centre_z = 0.0f;
    int perceived_count = 0;

    // Boids global velocity matching
    float global_velocity_x = 0.0f;
    float global_velocity_y = 0.0f;
    float global_velocity_z = 0.0f;

    // Boids short range avoidance centre
    float collision_centre_x = 0.0f;
    float collision_centre_y = 0.0f;
    float collision_centre_z = 0.0f;
    int collision_count = 0;

    const float INTERACTION_RADIUS = FLAMEGPU->environment.get<float>("INTERACTION_RADIUS");
    const float SEPARATION_RADIUS = FLAMEGPU->environment.get<float>("SEPARATION_RADIUS");
    // Iterate location messages, accumulating relevant data and counts.
    for (const auto &message : FLAMEGPU->message_in) {
        // Ignore self messages.
        if (message.getVariable<int>("id") != id) {
            // Get the message location and velocity.
            const float message_x = message.getVariable<float>("x");
            const float message_y = message.getVariable<float>("y");
            const float message_z = message.getVariable<float>("z");
            const float message_fx = message.getVariable<float>("fx");
            const float message_fy = message.getVariable<float>("fy");
            const float message_fz = message.getVariable<float>("fz");

            // Check interaction radius
            float separation = vec3Length(agent_x - message_x, agent_y - message_y, agent_z - message_z);

            if (separation < (INTERACTION_RADIUS)) {
                // Update the percieved centre
                perceived_centre_x += message_x;
                perceived_centre_y += message_y;
                perceived_centre_z += message_z;
                perceived_count++;

                // Update percieved velocity matching
                global_velocity_x += message_fx;
                global_velocity_y += message_fy;
                global_velocity_z += message_fz;

                // Update collision centre
                if (separation < (SEPARATION_RADIUS)) {  // dependant on model size
                    collision_centre_x += message_x;
                    collision_centre_y += message_y;
                    collision_centre_z += message_z;
                    collision_count += 1;
                }
            }
        }
    }

    // Divide positions/velocities by relevant counts.
    perceived_centre_x /= perceived_count;
    perceived_centre_y /= perceived_count;
    perceived_centre_z /= perceived_count;

    global_velocity_x /= perceived_count;
    global_velocity_y /= perceived_count;
    global_velocity_z /= perceived_count;

    collision_centre_x /= collision_count;
    collision_centre_y /= collision_count;
    collision_centre_z /= collision_count;

    // Total change in velocity
    float velocity_change_x = 0.f;
    float velocity_change_y = 0.f;
    float velocity_change_z = 0.f;

    // Rule 1) Steer towards perceived centre of flock (Cohesion)
    float steer_velocity_x = 0.f;
    float steer_velocity_y = 0.f;
    float steer_velocity_z = 0.f;
    if (perceived_count > 0) {
        const float STEER_SCALE = FLAMEGPU->environment.get<float>("STEER_SCALE");
        steer_velocity_x = (perceived_centre_x - agent_x) * STEER_SCALE;
        steer_velocity_y = (perceived_centre_y - agent_y) * STEER_SCALE;
        steer_velocity_z = (perceived_centre_z - agent_z) * STEER_SCALE;
    }
    velocity_change_x += steer_velocity_x;
    velocity_change_y += steer_velocity_y;
    velocity_change_z += steer_velocity_z;

    // Rule 2) Match neighbours speeds (Alignment)
    float match_velocity_x = 0.f;
    float match_velocity_y = 0.f;
    float match_velocity_z = 0.f;
    if (collision_count > 0) {
        const float MATCH_SCALE = FLAMEGPU->environment.get<float>("MATCH_SCALE");
        match_velocity_x = global_velocity_x * MATCH_SCALE;
        match_velocity_y = global_velocity_y * MATCH_SCALE;
        match_velocity_z = global_velocity_z * MATCH_SCALE;
    }
    velocity_change_x += match_velocity_x;
    velocity_change_y += match_velocity_y;
    velocity_change_z += match_velocity_z;

    // Rule 3) Avoid close range neighbours (Separation)
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;
    float avoid_velocity_z = 0.0f;
    if (collision_count > 0) {
        const float COLLISION_SCALE = FLAMEGPU->environment.get<float>("COLLISION_SCALE");
        avoid_velocity_x = (agent_x - collision_centre_x) * COLLISION_SCALE;
        avoid_velocity_y = (agent_y - collision_centre_y) * COLLISION_SCALE;
        avoid_velocity_z = (agent_z - collision_centre_z) * COLLISION_SCALE;
    }
    velocity_change_x += avoid_velocity_x;
    velocity_change_y += avoid_velocity_y;
    velocity_change_z += avoid_velocity_z;

    // Global scale of velocity change
    const float GLOBAL_SCALE = FLAMEGPU->environment.get<float>("GLOBAL_SCALE");
    velocity_change_x *= GLOBAL_SCALE;
    velocity_change_y *= GLOBAL_SCALE;
    velocity_change_z *= GLOBAL_SCALE;

    // Update agent velocity
    agent_fx += velocity_change_x;
    agent_fy += velocity_change_y;
    agent_fz += velocity_change_z;

    // Bound velocity
    float agent_fscale = vec3Length(agent_fx, agent_fy, agent_fz);
    if (agent_fscale > 1) {
        agent_fx /= agent_fscale;
        agent_fy /= agent_fscale;
        agent_fz /= agent_fscale;
    }

    // Apply the velocity
    const float TIME_SCALE = FLAMEGPU->environment.get<float>("TIME_SCALE");
    agent_x += agent_fx * TIME_SCALE;
    agent_y += agent_fy * TIME_SCALE;
    agent_z += agent_fz * TIME_SCALE;

    // Bound position
    boundPosition(agent_x, agent_y, agent_z, FLAMEGPU->environment.get<float>("MIN_POSITION"), FLAMEGPU->environment.get<float>("MAX_POSITION"));

    // Update global agent memory.
    FLAMEGPU->setVariable<float>("x", agent_x);
    FLAMEGPU->setVariable<float>("y", agent_y);
    FLAMEGPU->setVariable<float>("z", agent_z);

    FLAMEGPU->setVariable<float>("fx", agent_fx);
    FLAMEGPU->setVariable<float>("fy", agent_fy);
    FLAMEGPU->setVariable<float>("fz", agent_fz);

    return ALIVE;
}

int main(int argc, const char ** argv) {
    ModelDescription model("Boids_BruteForce");

    {   // Location message
        MsgBruteForce::Description &message = model.newMessage("location");
        // A message to hold the location of an agent.
        message.newVariable<int>("id");
        message.newVariable<float>("x");
        message.newVariable<float>("y");
        message.newVariable<float>("z");
        message.newVariable<float>("fx");
        message.newVariable<float>("fy");
        message.newVariable<float>("fz");
    }
    {   // Boid agent
        AgentDescription &agent = model.newAgent("Boid");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
        agent.newVariable<float>("y");
        agent.newVariable<float>("z");
        agent.newVariable<float>("fx");
        agent.newVariable<float>("fy");
        agent.newVariable<float>("fz");
        agent.newFunction("outputdata", outputdata).setMessageOutput("location");
        agent.newFunction("inputdata", inputdata).setMessageInput("location");
    }


    /**
     * GLOBALS
     */
    {
        EnvironmentDescription &env = model.Environment();

        // Environment Bounds
        env.add("MIN_POSITION", -0.5f);
        env.add("MAX_POSITION", +0.5f);

        // Initialisation parameter(s)
        env.add("MAX_INITIAL_SPEED", 1.0f);
        env.add("MIN_INITIAL_SPEED", 0.01f);

        // Interaction radius
        env.add("INTERACTION_RADIUS", 0.1f);
        env.add("SEPARATION_RADIUS", 0.005f);

        // Global Scalers
        env.add("TIME_SCALE", 0.0005f);
        env.add("GLOBAL_SCALE", 0.15f);

        // Rule scalers
        env.add("STEER_SCALE", 0.65f);
        env.add("COLLISION_SCALE", 0.75f);
        env.add("MATCH_SCALE", 1.25f);
    }

    /**
     * Control flow
     */     
    {   // Layer #1
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(outputdata);
    }
    {   // Layer #2
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(inputdata);
    }


    /**
     * Create Model Runner
     */
    CUDAAgentModel cuda_model(model);

    /**
     * Create visualisation
     */
#ifdef VISUALISATION
    ModelVis &visualisation = cuda_model.getVisualisation();
    {
        EnvironmentDescription &env = model.Environment();
        float envWidth = env.get<float>("MAX_POSITION") - env.get<float>("MIN_POSITION");
        const float INIT_CAM = env.get<float>("MAX_POSITION") * 1.25f;
        visualisation.setInitialCameraLocation(INIT_CAM, INIT_CAM, INIT_CAM);
        visualisation.setCameraSpeed(0.002f * envWidth);
        auto &circ_agt = visualisation.addAgent("Boid");
        // Position vars are named x, y, z; so they are used by default
        circ_agt.setModel(Stock::Models::ICOSPHERE);
        circ_agt.setModelScale(env.get<float>("SEPARATION_RADIUS"));
    }
    visualisation.activate();
#endif

    // Initialisation
    cuda_model.initialise(argc, argv);

    // If no xml model file was is provided, generate a population.
    // @todo - this doesn't deal with if xml is passed just containing parameters!
    if (cuda_model.getSimulationConfig().xml_input_file.empty()) {
        // Initial population size if not loaded from xml.
        const unsigned int AGENT_COUNT = 2048; // @todo move to environment parametr for flexibility.
        // @todo better RNG / seeding. Multiple distributions from multiple seeds (generated from a single, cli-based seed)
        EnvironmentDescription &env = model.Environment();
        // RNG distributions for iniital agent state.
        std::default_random_engine rng;
        std::uniform_real_distribution<float> pos_dist(env.get<float>("MIN_POSITION"), env.get<float>("MAX_POSITION"));
        std::uniform_real_distribution<float> velocity_dist(-1, 1);
        std::uniform_real_distribution<float> velocity_magnitude(env.get<float>("MIN_INITIAL_SPEED"), env.get<float>("MAX_INITIAL_SPEED"));
        AgentPopulation population(model.Agent("Boid"), AGENT_COUNT);
        for (unsigned int i = 0; i < AGENT_COUNT; i++) {
            AgentInstance instance = population.getNextInstance();
            instance.setVariable<int>("id", i);
            instance.setVariable<float>("x", pos_dist(rng));
            instance.setVariable<float>("y", pos_dist(rng));
            instance.setVariable<float>("z", pos_dist(rng));
            
            // Generate a random velocity direction
            float fx = velocity_dist(rng);
            float fy = velocity_dist(rng);
            float fz = velocity_dist(rng);
            // Generate a random speed between 0 and the maximum initial speed 
            float fmagnitude = velocity_magnitude(rng);
            // Use the random speed for the velocity.
            vec3Normalize(&fx, &fy, &fz);
            vec3Mult(&fx, &fy, &fz, fmagnitude);

            // Set these for the agent.
            instance.setVariable<float>("fx", fx);
            instance.setVariable<float>("fy", fy);
            instance.setVariable<float>("fz", fz);
            printf(
                "new boid %d at (%+1.5f, %+1.5f, %+1.5f), with velocity (%+1.5f, %+1.5f, %+1.5f)\n",
                instance.getVariable<int>("id"),
                instance.getVariable<float>("x"),
                instance.getVariable<float>("y"),
                instance.getVariable<float>("z"),
                instance.getVariable<float>("fx"),
                instance.getVariable<float>("fy"),
                instance.getVariable<float>("fz"));
        }
        cuda_model.setPopulationData(population);
    }

    /**
     * Execution
     */
    cuda_model.simulate();


    /**
     * Export Pop
     */
    // cuda_model.exportData("end.xml");

#ifdef VISUALISATION
    visualisation.join();
#endif
    return 0;
}

