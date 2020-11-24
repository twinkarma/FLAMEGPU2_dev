#ifndef __PEDRVO_IO__
#define __PEDRVO_IO__

#include "modelspec.cuh"

/**
 * Create a modelspec object from the steerbench XML
 * @param filePath
 * @return Shared pointer to the created ModelEnvSpec object
 */
ModelEnvSpecPtr importSteerBenchXML(std::string filePath);
void expandSteerbenchEnvRegions(ModelEnvSpecPtr env);

/**
 * Create a counter-clockwise line of float2 to represent the boundary
 * @param bounds
 * @return
 */
std::vector<float2> getLineFromBounds(Bounds& bounds);


#endif //__PEDRVO_IO__