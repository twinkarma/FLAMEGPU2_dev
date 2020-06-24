#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>


#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"
#include "flamegpu/io/factory.h"
#include "flamegpu/util/nvtx.h"

// Agent state variables
#define AGENT_STATE_UNOCCUPIED 0
#define AGENT_STATE_OCCUPIED 1
#define AGENT_STATE_MOVEMENT_REQUESTED 2
#define AGENT_STATE_MOVEMENT_UNRESOLVED 3


// Growback variables
#define SUGAR_GROWBACK_RATE 1
#define SUGAR_MAX_CAPACITY 4

FLAMEGPU_AGENT_FUNCTION(metabolise_and_growback, MsgNone, MsgNone) {
    int sugar_level = FLAMEGPU->getVariable<int>("sugar_level");
    int env_sugar_level = FLAMEGPU->getVariable<int>("env_sugar_level");
    int state = FLAMEGPU->getVariable<int>("state");
    // metabolise if occupied
    if (state == AGENT_STATE_OCCUPIED) {
        // store sugar
        sugar_level += env_sugar_level;
        env_sugar_level = 0;

        // metabolise
        sugar_level -= FLAMEGPU->getVariable<int>("metabolism");

        // check if agent dies
        if (sugar_level == 0) {
            state = AGENT_STATE_UNOCCUPIED;
            FLAMEGPU->setVariable<int>("agent_id", -1);
            sugar_level = 0;
            FLAMEGPU->setVariable<int>("metabolism", 0);
        }
    }

    // growback if unoccupied
    if (state == AGENT_STATE_UNOCCUPIED) {
        if (env_sugar_level < SUGAR_MAX_CAPACITY) {
            env_sugar_level += SUGAR_GROWBACK_RATE;
        }
    }

    // set all active agents to unresolved as they may now want to move
    if (state == AGENT_STATE_OCCUPIED) {
        state = AGENT_STATE_MOVEMENT_UNRESOLVED;
    }
    FLAMEGPU->setVariable<int>("sugar_level", sugar_level);
    FLAMEGPU->setVariable<int>("env_sugar_level", env_sugar_level);
    FLAMEGPU->setVariable<int>("state", state);
    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(output_cell_state, MsgNone, MsgArray2D) {
    int agent_x = 0;  // TODO
    int agent_y = 0;  // TODO
    FLAMEGPU->message_out.setVariable("location_id", FLAMEGPU->getVariable<int>("location_id"));
    FLAMEGPU->message_out.setVariable("state", FLAMEGPU->getVariable<int>("state"));
    FLAMEGPU->message_out.setVariable("env_sugar_level", FLAMEGPU->getVariable<int>("env_sugar_level"));
    FLAMEGPU->message_out.setIndex(agent_x, agent_y);
    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(movement_request, MsgArray2D, MsgArray2D) {
    int best_sugar_level = -1;
    int best_location_id = -1;

    // find the best location to move to
    int state = FLAMEGPU->getVariable<int>("state");
    int agent_x = 0;  // TODO
    int agent_y = 0;  // TODO
    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if occupied then look for empty cells
        if (state == AGENT_STATE_MOVEMENT_UNRESOLVED) {
            // if location is unoccupied then check for empty locations
            if (current_message.getVariable<int>("state") == AGENT_STATE_UNOCCUPIED) {
                // if the sugar level at current location is better than currently stored then update
                if (current_message.getVariable<int>("env_sugar_level") > best_sugar_level) {
                    best_sugar_level = current_message.getVariable<int>("env_sugar_level");
                    best_location_id = current_message.getVariable<int>("location_id");
                }
            }
        }
    }

    // if the agent has found a better location to move to then update its state
    if ((state == AGENT_STATE_MOVEMENT_UNRESOLVED)) {
        // if there is a better location to move to then state indicates a movement request
        state = best_location_id > 0 ? AGENT_STATE_MOVEMENT_REQUESTED : AGENT_STATE_OCCUPIED;
        FLAMEGPU->setVariable<int>("state", state);
    }

    // add a movement request
    FLAMEGPU->message_out.setVariable<int>("agent_id", FLAMEGPU->getVariable<int>("agent_id"));
    FLAMEGPU->message_out.setVariable<int>("location_id", best_location_id);
    FLAMEGPU->message_out.setVariable<int>("sugar_level", FLAMEGPU->getVariable<int>("sugar_level"));
    FLAMEGPU->message_out.setVariable<int>("metabolism", FLAMEGPU->getVariable<int>("metabolism"));
    FLAMEGPU->message_out.setIndex(agent_x, agent_y);

    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(movement_response, MsgArray2D, MsgArray2D) {
    int best_request_id = -1;
    float best_request_priority = -1;
    int best_request_sugar_level = -1;
    int best_request_metabolism = -1;

    int state = FLAMEGPU->getVariable<int>("state");
    int location_id = FLAMEGPU->getVariable<int>("location_id");
    int agent_x = 0;  // TODO
    int agent_y = 0;  // TODO

    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if the location is unoccupied then check for agents requesting to move here
        if (state == AGENT_STATE_UNOCCUPIED) {
            // check if request is to move to this location
            if (current_message.getVariable<int>("location_id") == location_id) {
                // check the priority and maintain the best ranked agent
                float message_priority = 0;  // FLAMEGPU->random.uniform<float>();
                if (message_priority > best_request_priority) {
                    best_request_id = current_message.getVariable<int>("agent_id");
                    best_request_priority = message_priority;
                }
            }
        }
    }

    // if the location is unoccupied and an agent wants to move here then do so and send a response
    if ((state == AGENT_STATE_UNOCCUPIED) && (best_request_id > 0))    {
        FLAMEGPU->setVariable<int>("state", AGENT_STATE_OCCUPIED);
        // move the agent to here
        FLAMEGPU->setVariable<int>("agent_id", best_request_id);
        FLAMEGPU->setVariable<int>("sugar_level", best_request_sugar_level);
        FLAMEGPU->setVariable<int>("metabolism", best_request_metabolism);
    }

    // add a movement response
    FLAMEGPU->message_out.setVariable<int>("location_id", FLAMEGPU->getVariable<int>("location_id"));
    FLAMEGPU->message_out.setVariable<int>("agent_id", best_request_id);
    FLAMEGPU->message_out.setIndex(agent_x, agent_y);

    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(movement_transaction, MsgArray2D, MsgNone) {
    int state = FLAMEGPU->getVariable<int>("state");
    int agent_id = FLAMEGPU->getVariable<int>("agent_id");
    int agent_x = 0;  // TODO
    int agent_y = 0;  // TODO

    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if location contains an agent wanting to move then look for responses allowing relocation
        if (state == AGENT_STATE_MOVEMENT_REQUESTED) {  // if the movement response request came from this location
            if (current_message.getVariable<int>("agent_id") == agent_id) {
                // remove the agent and reset agent specific variables as it has now moved
                FLAMEGPU->setVariable<int>("state", AGENT_STATE_UNOCCUPIED);
                FLAMEGPU->setVariable<int>("agent_id", -1);
                FLAMEGPU->setVariable<int>("sugar_level", 0);
                FLAMEGPU->setVariable<int>("metabolism", 0);
            }
        }
    }

    // if request has not been responded to then agent is unresolved
    if (state == AGENT_STATE_MOVEMENT_REQUESTED) {
        FLAMEGPU->setVariable<int>("state", AGENT_STATE_MOVEMENT_UNRESOLVED);
    }

    return ALIVE;
}
int main(int argc, const char ** argv) {
    NVTX_RANGE("main");
    NVTX_PUSH("ModelDescription");
    ModelDescription model("Sugarscape_example");

    {   // cell_state message
        MsgArray2D::Description &message = model.newMessage<MsgArray2D>("cell_state");
        message.newVariable<int>("location_id");
        message.newVariable<int>("state");
        message.newVariable<int>("env_sugar_level");
        // message.setDimensions(10, 10);
    }
    {   // movement_request message
        MsgArray2D::Description &message = model.newMessage<MsgArray2D>("movement_request");
        message.newVariable<int>("agent_id");
        message.newVariable<int>("location_id");
        message.newVariable<int>("sugar_level");
        message.newVariable<int>("metabolism");
        // message.setDimensions(10, 10);
    }
    {   // movement_response message
        MsgArray2D::Description &message = model.newMessage<MsgArray2D>("movement_response");
        message.newVariable<int>("location_id");
        message.newVariable<int>("agent_id");
        // message.setDimensions(10, 10);
    }
    {   // Per cell agent
        AgentDescription &agent = model.newAgent("agent");
        agent.newVariable<int>("location_id");
        agent.newVariable<int>("agent_id");
        agent.newVariable<int>("state");
        // agent specific variables
        agent.newVariable<int>("sugar_level");
        agent.newVariable<int>("metabolism");
        // environment specific var
        agent.newVariable<int>("env_sugar_level");
        // Functions
        agent.newFunction("metabolise_and_growback", metabolise_and_growback);  // This has function condition if (state != AGENT_STATE_MOVEMENT_UNRESOLVED) max iterations 9
        auto &fn_output_cell_state = agent.newFunction("output_cell_state", output_cell_state);
        {
            fn_output_cell_state.setMessageOutput("cell_state");
        }
        auto &fn_movement_request = agent.newFunction("movement_request", movement_request);
        {
            fn_movement_request.setMessageInput("cell_state");
            fn_movement_request.setMessageOutput("movement_request");
        }
        auto &fn_movement_response = agent.newFunction("movement_response", movement_response);
        {
            fn_movement_request.setMessageInput("movement_request");
            fn_movement_response.setMessageOutput("movement_response");
        }
        auto &fn_movement_transaction = agent.newFunction("movement_transaction", movement_transaction);
        {
            fn_movement_transaction.setMessageInput("movement_response");
        }
    }

    /**
     * GLOBALS
     */
    {
        // EnvironmentDescription &env = model.Environment();
    }

    /**
     * Control flow
     */
    {   // Layer #1
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(metabolise_and_growback);
    }
    {   // Layer #2
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(output_cell_state);
    }
    {   // Layer #3
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(movement_request);
    }
    {   // Layer #4
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(movement_response);
    }
    {   // Layer #5
        LayerDescription &layer = model.newLayer();
        layer.addAgentFunction(movement_transaction);
    }
    NVTX_POP();

    /**
     * Create Model Runner
     */
    NVTX_PUSH("CUDAAgentModel creation");
    CUDAAgentModel cuda_model(model);
    NVTX_POP();

    /**
     * Initialisation
     */
    cuda_model.initialise(argc, argv);
    if (cuda_model.getSimulationConfig().xml_input_file.empty()) {
        // Currently population has not been init, so generate an agent population on the fly
        // const unsigned int SQRT_AGENT_COUNT = 10;
        // const unsigned int AGENT_COUNT = SQRT_AGENT_COUNT * SQRT_AGENT_COUNT;
        // std::default_random_engine rng;
        // std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        // AgentPopulation init_pop(model.Agent("cell"), AGENT_COUNT);
        // for (unsigned int x = 0; x < SQRT_AGENT_COUNT; ++x) {
        //     for (unsigned int y = 0; y < SQRT_AGENT_COUNT; ++y) {
        //         AgentInstance instance = init_pop.getNextInstance();
        //         instance.setVariable<unsigned int, 2>("pos", { x, y });
        //         char is_alive = dist(rng) < 0.4f ? 1 : 0;
        //         instance.setVariable<char>("is_alive", is_alive);  // 40% Chance of being alive
        //     }
        // }
        // cuda_model.setPopulationData(init_pop);
    }

    /**
     * Execution
     */
    cuda_model.simulate();

    /**
     * Export Pop
     */
    // TODO
    return 0;
}
