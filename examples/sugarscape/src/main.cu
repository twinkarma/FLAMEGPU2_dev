#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>


#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"
#include "flamegpu/io/factory.h"
#include "flamegpu/util/nvtx.h"

// Grid Size (the product of these is the agent count)
#define GRID_WIDTH 256
#define GRID_HEIGHT 256

// Agent state variables
#define AGENT_STATUS_UNOCCUPIED 0
#define AGENT_STATUS_OCCUPIED 1
#define AGENT_STATUS_MOVEMENT_REQUESTED 2
#define AGENT_STATUS_MOVEMENT_UNRESOLVED 3


// Growback variables
#define SUGAR_GROWBACK_RATE 1
#define SUGAR_MAX_CAPACITY 4

FLAMEGPU_AGENT_FUNCTION(metabolise_and_growback, MsgNone, MsgNone) {
    int sugar_level = FLAMEGPU->getVariable<int>("sugar_level");
    int env_sugar_level = FLAMEGPU->getVariable<int>("env_sugar_level");
    int state = FLAMEGPU->getVariable<int>("status");
    // metabolise if occupied
    if (state == AGENT_STATUS_OCCUPIED) {
        // store sugar
        sugar_level += env_sugar_level;
        env_sugar_level = 0;

        // metabolise
        sugar_level -= FLAMEGPU->getVariable<int>("metabolism");

        // check if agent dies
        if (sugar_level == 0) {
            state = AGENT_STATUS_UNOCCUPIED;
            FLAMEGPU->setVariable<int>("agent_id", -1);
            sugar_level = 0;
            FLAMEGPU->setVariable<int>("metabolism", 0);
        }
    }

    // growback if unoccupied
    if (state == AGENT_STATUS_UNOCCUPIED) {
        if (env_sugar_level < SUGAR_MAX_CAPACITY) {
            env_sugar_level += SUGAR_GROWBACK_RATE;
        }
    }

    // set all active agents to unresolved as they may now want to move
    if (state == AGENT_STATUS_OCCUPIED) {
        state = AGENT_STATUS_MOVEMENT_UNRESOLVED;
    }
    FLAMEGPU->setVariable<int>("sugar_level", sugar_level);
    FLAMEGPU->setVariable<int>("env_sugar_level", env_sugar_level);
    FLAMEGPU->setVariable<int>("status", state);
    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(output_cell_status, MsgNone, MsgArray2D) {
    unsigned int agent_x = FLAMEGPU->getVariable<unsigned int, 2>("pos", 0);
    unsigned int agent_y = FLAMEGPU->getVariable<unsigned int, 2>("pos", 1);
    FLAMEGPU->message_out.setVariable("location_id", FLAMEGPU->getVariable<int>("location_id"));
    FLAMEGPU->message_out.setVariable("status", FLAMEGPU->getVariable<int>("status"));
    FLAMEGPU->message_out.setVariable("env_sugar_level", FLAMEGPU->getVariable<int>("env_sugar_level"));
    FLAMEGPU->message_out.setIndex(agent_x, agent_y);
    return ALIVE;
}
FLAMEGPU_AGENT_FUNCTION(movement_request, MsgArray2D, MsgArray2D) {
    int best_sugar_level = -1;
    int best_location_id = -1;

    // find the best location to move to
    int status = FLAMEGPU->getVariable<int>("status");
    unsigned int agent_x = FLAMEGPU->getVariable<unsigned int, 2>("pos", 0);
    unsigned int agent_y = FLAMEGPU->getVariable<unsigned int, 2>("pos", 1);
    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if occupied then look for empty cells
        if (status == AGENT_STATUS_MOVEMENT_UNRESOLVED) {
            // if location is unoccupied then check for empty locations
            if (current_message.getVariable<int>("status") == AGENT_STATUS_UNOCCUPIED) {
                // if the sugar level at current location is better than currently stored then update
                if (current_message.getVariable<int>("env_sugar_level") > best_sugar_level) {
                    best_sugar_level = current_message.getVariable<int>("env_sugar_level");
                    best_location_id = current_message.getVariable<int>("location_id");
                }
            }
        }
    }

    // if the agent has found a better location to move to then update its state
    if ((status == AGENT_STATUS_MOVEMENT_UNRESOLVED)) {
        // if there is a better location to move to then state indicates a movement request
        status = best_location_id > 0 ? AGENT_STATUS_MOVEMENT_REQUESTED : AGENT_STATUS_OCCUPIED;
        FLAMEGPU->setVariable<int>("status", status);
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

    int status = FLAMEGPU->getVariable<int>("status");
    int location_id = FLAMEGPU->getVariable<int>("location_id");
    unsigned int agent_x = FLAMEGPU->getVariable<unsigned int, 2>("pos", 0);
    unsigned int agent_y = FLAMEGPU->getVariable<unsigned int, 2>("pos", 1);

    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if the location is unoccupied then check for agents requesting to move here
        if (status == AGENT_STATUS_UNOCCUPIED) {
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
    if ((status == AGENT_STATUS_UNOCCUPIED) && (best_request_id > 0))    {
        FLAMEGPU->setVariable<int>("status", AGENT_STATUS_OCCUPIED);
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
    int status = FLAMEGPU->getVariable<int>("status");
    int agent_id = FLAMEGPU->getVariable<int>("agent_id");
    unsigned int agent_x = FLAMEGPU->getVariable<unsigned int, 2>("pos", 0);
    unsigned int agent_y = FLAMEGPU->getVariable<unsigned int, 2>("pos", 1);

    for (auto current_message : FLAMEGPU->message_in(agent_x, agent_y)) {
        // if location contains an agent wanting to move then look for responses allowing relocation
        if (status == AGENT_STATUS_MOVEMENT_REQUESTED) {  // if the movement response request came from this location
            if (current_message.getVariable<int>("agent_id") == agent_id) {
                // remove the agent and reset agent specific variables as it has now moved
                FLAMEGPU->setVariable<int>("status", AGENT_STATUS_UNOCCUPIED);
                FLAMEGPU->setVariable<int>("agent_id", -1);
                FLAMEGPU->setVariable<int>("sugar_level", 0);
                FLAMEGPU->setVariable<int>("metabolism", 0);
            }
        }
    }

    // if request has not been responded to then agent is unresolved
    if (status == AGENT_STATUS_MOVEMENT_REQUESTED) {
        FLAMEGPU->setVariable<int>("status", AGENT_STATUS_MOVEMENT_UNRESOLVED);
    }

    return ALIVE;
}
FLAMEGPU_EXIT_CONDITION(MovementExitCondition) {
    static unsigned int iterations = 0;
    iterations++;

    // Max iterations 9
    if (iterations < 9) {
        // Agent movements still unresolved
        if (FLAMEGPU->agent("agent").count("status", AGENT_STATUS_MOVEMENT_UNRESOLVED)) {
            return CONTINUE;
        }
    }

    iterations = 0;
    return EXIT;
}
/**
 * Construct the common components of agent shared between both parent and submodel
 */
AgentDescription &makeCoreAgent(ModelDescription &model) {
    AgentDescription &agent = model.newAgent("agent");
    agent.newVariable<unsigned int, 2>("pos");
    agent.newVariable<int>("location_id");
    agent.newVariable<int>("agent_id");
    agent.newVariable<int>("status");
    // agent specific variables
    agent.newVariable<int>("sugar_level");
    agent.newVariable<int>("metabolism");
    // environment specific var
    agent.newVariable<int>("env_sugar_level");
    return agent;
}
int main(int argc, const char ** argv) {
    NVTX_RANGE("main");
    NVTX_PUSH("ModelDescription");
    ModelDescription submodel("Movement_model");
    {  // Define sub model for conflict resolution
        /**
         * Messages
         */
        {   // cell_status message
            MsgArray2D::Description &message = submodel.newMessage<MsgArray2D>("cell_status");
            message.newVariable<int>("location_id");
            message.newVariable<int>("status");
            message.newVariable<int>("env_sugar_level");
            message.setDimensions(GRID_WIDTH, GRID_HEIGHT);
        }
        {   // movement_request message
            MsgArray2D::Description &message = submodel.newMessage<MsgArray2D>("movement_request");
            message.newVariable<int>("agent_id");
            message.newVariable<int>("location_id");
            message.newVariable<int>("sugar_level");
            message.newVariable<int>("metabolism");
            message.setDimensions(GRID_WIDTH, GRID_HEIGHT);
        }
        {   // movement_response message
            MsgArray2D::Description &message = submodel.newMessage<MsgArray2D>("movement_response");
            message.newVariable<int>("location_id");
            message.newVariable<int>("agent_id");
            message.setDimensions(GRID_WIDTH, GRID_HEIGHT);
        }
        /**
         * Agents
         */
        {
            AgentDescription &agent = makeCoreAgent(submodel);
            auto &fn_output_cell_status = agent.newFunction("output_cell_status", output_cell_status);
            {
                fn_output_cell_status.setMessageOutput("cell_status");
            }
            auto &fn_movement_request = agent.newFunction("movement_request", movement_request);
            {
                fn_movement_request.setMessageInput("cell_status");
                fn_movement_request.setMessageOutput("movement_request");
            }
            auto &fn_movement_response = agent.newFunction("movement_response", movement_response);
            {
                fn_movement_response.setMessageInput("movement_request");
                fn_movement_response.setMessageOutput("movement_response");
            }
            auto &fn_movement_transaction = agent.newFunction("movement_transaction", movement_transaction);
            {
                fn_movement_transaction.setMessageInput("movement_response");
            }
        }

        /**
         * Globals
         */
        {
            // EnvironmentDescription &env = model.Environment();
        }

        /**
         * Control flow
         */
        {   // Layer #1
            LayerDescription &layer = submodel.newLayer();
            layer.addAgentFunction(output_cell_status);
        }
        {   // Layer #2
            LayerDescription &layer = submodel.newLayer();
            layer.addAgentFunction(movement_request);
        }
        {   // Layer #3
            LayerDescription &layer = submodel.newLayer();
            layer.addAgentFunction(movement_response);
        }
        {   // Layer #4
            LayerDescription &layer = submodel.newLayer();
            layer.addAgentFunction(movement_transaction);
        }
        submodel.addExitCondition(MovementExitCondition);
    }

    ModelDescription model("Sugarscape_example");

    /**
     * Agents
     */
    {   // Per cell agent
        AgentDescription &agent = makeCoreAgent(model);
        // Functions
        agent.newFunction("metabolise_and_growback", metabolise_and_growback);
    }

    /**
     * Submodels
     */
    SubModelDescription &movement_sub = model.newSubModel("movement_conflict_resolution_model", submodel);
    {
        movement_sub.bindAgent("agent", "agent", true, true);
    }

    /**
     * Globals
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
        layer.addSubModel(movement_sub);
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
    NVTX_PUSH("CUDAAgentModel initialisation");
    cuda_model.initialise(argc, argv);
    if (cuda_model.getSimulationConfig().xml_input_file.empty()) {
        // Currently population has not been init, so generate an agent population on the fly
        const unsigned int AGENT_COUNT = GRID_WIDTH * GRID_HEIGHT;
        std::default_random_engine rng;
        std::uniform_int_distribution<int> dist(0, 1);
        AgentPopulation init_pop(model.Agent("agent"), AGENT_COUNT);
        for (unsigned int x = 0; x < GRID_WIDTH; ++x) {
            for (unsigned int y = 0; y < GRID_HEIGHT; ++y) {
                AgentInstance instance = init_pop.getNextInstance();
                instance.setVariable<unsigned int, 2>("pos", { x, y });
                // TODO: How should these values be init?
                instance.setVariable<int>("location_id", dist(rng));
                instance.setVariable<int>("agent_id", dist(rng));
                instance.setVariable<int>("status", dist(rng));
                // agent specific variables
                instance.setVariable<int>("sugar_level", dist(rng));
                instance.setVariable<int>("metabolism", dist(rng));
                // environment specific var
                instance.setVariable<int>("env_sugar_level", dist(rng));
            }
        }
        cuda_model.setPopulationData(init_pop);
    }
    NVTX_POP();

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
