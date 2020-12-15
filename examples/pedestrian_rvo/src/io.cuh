#ifndef __PEDRVO_IO__
#define __PEDRVO_IO__

#include "simulation_spec.cuh"

/**
 * Parse a steersuite test case format xml into a SimulationSpec
 * @param filePath Path to the xml file
 * @return Shared pointer to a SimulationSpec that contains all information to create a test case
 */
SimulationSpecPtr importSteerBenchXML(std::string filePath);

/**
 * AgentRegions and ObstacleRegions allows the declaration of multiple pedestrian agents or obstacles in single element.
 * This functions expands these regions, creating conrete agents and obstacles and adds them to the agents and obstacles
 * vector respectively.
 * @param env Spec that has agentRegion or obstacleRegion objects
 */
void expandSpecRegions(SimulationSpecPtr env);

/**
 * Create a counter-clockwise line of float2 to represent the boundary
 * @param bounds
 * @return
 */
std::vector<float2> getLineFromBounds(Bounds& bounds);

/**
 * Create an example test case (used for when spec xml is not defined)
 * @return
 */
SimulationSpecPtr createTestSimSpec();
#endif //__PEDRVO_IO__