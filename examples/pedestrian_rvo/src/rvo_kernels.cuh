#ifndef __RVO_KERNELS__
#define __RVO_KERNELS__

#include "common_cuda_math.h"
#include "rvo_struct.cuh"

#define ORCA_ARRAY_SIZE 30
#define OBSTACLE_DETECT_ARRAY_SIZE 15

 /**
A sufficiently small positive number.
 */
#define RVO_EPSILON 0.00001f


struct RVOLine{
	float2 point;
	float2 direction;
};

struct Ped{
	float2 pos;
	float2 vel;
	float radius;
	float separationSq;
};

struct Obst{
	float distSq;
	int obstIndex;
};

void* getRVOObstaclePointer();
void* getRVOKDNodePointer();
__device__ void addNeighbourSorted(Ped* neighbours, int& neighbourCount, Ped& neighbourToAdd);
__device__ void addNeighbourSorted(Ped* neighbours, int& neighbourCount, Ped& neighbourToAdd);
__device__ void addOrcaAgent(RVOLine* orcaLines, int &orcaLinesCount, float timeHorizon_, float simulationTimestep, float2 position, float2 velocity , float radius_ , float2& otherPosition, float2& otherVelocity, float otherRadius);
__device__ void performCollisionAvoidance(RVOLine* orcaLines, int orcaLinesCount, int numObstLines, float maxSpeed, float2& prefVelocity, float2& newVelocity);
__device__ void getObstacles(RVOLine* orcaLines, int& orcaLinesCount, float timeHorizonObst, float2& agentPos, float2& agentVelocity, float radius, float agentMaxSpeed, float2& agentObstacleVector, int &obstacleVectorCount);

#endif //__RVO_KERNELS__