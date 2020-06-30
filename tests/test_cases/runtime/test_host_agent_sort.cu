#include "gtest/gtest.h"

#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"

namespace test_host_agent_sort {

const unsigned int AGENT_COUNT = 1024;
FLAMEGPU_STEP_FUNCTION(sort_ascending_float) {
    FLAMEGPU->agent("agent").sort<float>("float", HostAgentInstance::Asc);
}
FLAMEGPU_STEP_FUNCTION(sort_descending_float) {
    FLAMEGPU->agent("agent").sort<float>("float", HostAgentInstance::Desc);
}
FLAMEGPU_STEP_FUNCTION(sort_ascending_int) {
  FLAMEGPU->agent("agent").sort<int>("int", HostAgentInstance::Asc);
}
FLAMEGPU_STEP_FUNCTION(sort_descending_int) {
    FLAMEGPU->agent("agent").sort<int>("int", HostAgentInstance::Desc);
}

TEST(HostAgentSort, Ascending_float) {
    // Define model
    ModelDescription model("model");
    AgentDescription &agent = model.newAgent("agent");
    agent.newVariable<float>("float");
    agent.newVariable<float>("spare");
    model.newLayer().addHostFunction(sort_ascending_float);
    std::mt19937 rd;  // Seed does not matter
    std::uniform_real_distribution <float> dist(1, 1000000);

    // Init pop
    AgentPopulation pop(agent, AGENT_COUNT);
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getNextInstance();
        const float t = dist(rd);
        instance.setVariable<float>("float", t);
        instance.setVariable<float>("spare", t+12.0f);
    }
    // Setup Model
    CUDAAgentModel cuda_model(model);
    cuda_model.setPopulationData(pop);
    // Execute step fn
    cuda_model.step();
    // Check results
    cuda_model.getPopulationData(pop);
    EXPECT_EQ(AGENT_COUNT, pop.getCurrentListSize());
    float prev = 1;
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getInstanceAt(i);
        const float f = instance.getVariable<float>("float");
        const float s = instance.getVariable<float>("spare");
        // Agent variables are still aligned
        EXPECT_EQ(s-f, 12.0f);
        // Agent variables are ordered
        EXPECT_GE(f, prev);
        // Store prev
        prev = f;
    }
}
TEST(HostReductionTest, Descending_float) {
    // Define model
    ModelDescription model("model");
    AgentDescription &agent = model.newAgent("agent");
    agent.newVariable<float>("float");
    agent.newVariable<float>("spare");
    model.newLayer().addHostFunction(sort_descending_float);
    std::mt19937 rd;  // Seed does not matter
    std::uniform_real_distribution <float> dist(1, 1000000);

    // Init pop
    AgentPopulation pop(agent, AGENT_COUNT);
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getNextInstance();
        const float t = dist(rd);
        instance.setVariable<float>("float", t);
        instance.setVariable<float>("spare", t+12.0f);
    }
    // Setup Model
    CUDAAgentModel cuda_model(model);
    cuda_model.setPopulationData(pop);
    // Execute step fn
    cuda_model.step();
    // Check results
    cuda_model.getPopulationData(pop);
    EXPECT_EQ(AGENT_COUNT, pop.getCurrentListSize());
    float prev = 1000000;
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getInstanceAt(i);
        const float f = instance.getVariable<float>("float");
        const float s = instance.getVariable<float>("spare");
        // Agent variables are still aligned
        EXPECT_EQ(s-f, 12.0f);
        // Agent variables are ordered
        EXPECT_LE(f, prev);
        // Store prev
        prev = f;
    }
}
TEST(HostAgentSort, Ascending_int) {
    // Define model
    ModelDescription model("model");
    AgentDescription &agent = model.newAgent("agent");
    agent.newVariable<int>("int");
    agent.newVariable<int>("spare");
    model.newLayer().addHostFunction(sort_ascending_int);
    std::mt19937 rd;  // Seed does not matter
    std::uniform_int_distribution <int> dist(0, 1000000);

    // Init pop
    AgentPopulation pop(agent, AGENT_COUNT);
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getNextInstance();
        const int t = i == AGENT_COUNT/2 ? 0 : dist(rd);  // Ensure zero is output atleast once
        instance.setVariable<int>("int", t);
        instance.setVariable<int>("spare", t+12);
    }
    // Setup Model
    CUDAAgentModel cuda_model(model);
    cuda_model.setPopulationData(pop);
    // Execute step fn
    cuda_model.step();
    // Check results
    cuda_model.getPopulationData(pop);
    EXPECT_EQ(AGENT_COUNT, pop.getCurrentListSize());
    int prev = 0;
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getInstanceAt(i);
        const int f = instance.getVariable<int>("int");
        const int s = instance.getVariable<int>("spare");
        // Agent variables are still aligned
        EXPECT_EQ(s-f, 12);
        // Agent variables are ordered
        EXPECT_GE(f, prev);
        // Store prev
        prev = f;
    }
}
TEST(HostReductionTest, Descending_int) {
    // Define model
    ModelDescription model("model");
    AgentDescription &agent = model.newAgent("agent");
    agent.newVariable<int>("int");
    agent.newVariable<int>("spare");
    model.newLayer().addHostFunction(sort_descending_int);
    std::mt19937 rd;  // Seed does not matter
    std::uniform_int_distribution <int> dist(1, 1000000);

    // Init pop
    AgentPopulation pop(agent, AGENT_COUNT);
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getNextInstance();
        const int t = dist(rd);
        instance.setVariable<int>("int", t);
        instance.setVariable<int>("spare", t+12);
    }
    // Setup Model
    CUDAAgentModel cuda_model(model);
    cuda_model.setPopulationData(pop);
    // Execute step fn
    cuda_model.step();
    // Check results
    cuda_model.getPopulationData(pop);
    EXPECT_EQ(AGENT_COUNT, pop.getCurrentListSize());
    int prev = 1000000;
    for (int i = 0; i< static_cast<int>(AGENT_COUNT); i++) {
        AgentInstance instance = pop.getInstanceAt(i);
        const int f = instance.getVariable<int>("int");
        const int s = instance.getVariable<int>("spare");
        // Agent variables are still aligned
        EXPECT_EQ(s-f, 12);
        // Agent variables are ordered
        EXPECT_LE(f, prev);
        // Store prev
        prev = f;
    }
}
}  // namespace test_host_agent_sort
