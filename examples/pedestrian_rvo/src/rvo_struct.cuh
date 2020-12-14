#ifndef __RVO_STRUCT__
#define __RVO_STRUCT__

#include <vector>
#include <memory>
#include "cudahelpers/helper_math.h"

struct RVObstacle {
	int id;
	float2 point;
	float2 unitDir;
	int isConvex;
	size_t nextObstacleIndex;
	size_t prevObstacleIndex;
};

typedef std::vector<RVObstacle*> RVObstacleVec;
typedef std::shared_ptr<RVObstacleVec> RVObstacleVecPtr;



struct RVObstacleGSOA {
	unsigned int size;
	int* id;
	float2* point;
	float2* unitDir;
	int* isConvex;
	int* nextObstacleIndex;
	int* prevObstacleIndex;
};

struct RVOKDNode {

	int leftIndex;

	int rightIndex;

	int siblingIndex;

	int parentIndex;

	int obstacleIndex;

};

struct RVOKDNodeGSOA {
	unsigned int size;
	int* leftIndex;
	int* rightIndex;
	int* siblingIndex;
	int* parentIndex;
	int* obstacleIndex;

};

/**
A node in the Obstacle Graph KD tree
*/
struct ObstacleTreeNode {

	/**
	 * \brief      The left obstacle tree node.
	 */
	ObstacleTreeNode *left;

	/**
	 * \brief      The obstacle number.
	 */
	RVObstacle *obstacle;

	/**
	 * \brief      The right obstacle tree node.
	 */
	ObstacleTreeNode *right;

	int index;

	ObstacleTreeNode(){
	    left = nullptr;
	    obstacle = nullptr;
	    right = nullptr;
	}

};

#endif //__RVO_STRUCT__