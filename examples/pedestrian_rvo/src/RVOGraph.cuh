#ifndef __RVO_GRAPH__
#define __RVO_GRAPH__
#include <vector>
#include <memory>
#include "common_cuda_math.h"
#include "rvo_struct.cuh"


class RVOGraph
{
private:

    RVObstacleVecPtr  rvoObstacles;
    ObstacleTreeNode* obsKdNode;
    int kdNodeCount;


public:
	RVOGraph(void);
	~RVOGraph(void);

	void buildRVO(std::vector<std::vector<float2>>& obstacles);
	void uploadRVOToGPU(void* obstacles_d, void* kdnodes_d);

	RVObstacleGSOA initialiseGPUObstacles(size_t size);
	RVOKDNodeGSOA initialiseGPUKdnodes(size_t size);


	ObstacleTreeNode *recursivelyBuildObstacleTree(RVObstacleVecPtr obstacles, RVObstacleVecPtr globalObstacles, int obsLevel = 0);
	void recursivelyIndexObstacleTreeNode(ObstacleTreeNode * currentNode, int& index);
	void recursivelyBuildGPUObstacleTree( ObstacleTreeNode* node, ObstacleTreeNode* parentNode, bool isLeft, RVOKDNodeGSOA& outputNodeList);

	int getKDTreeDepth(int index, RVOKDNodeGSOA& nodes);
	int addRVOObstacle(std::vector<float2> &vertices, RVObstacleVecPtr obstacles);

};




#endif //__RVO_GRAPH__
