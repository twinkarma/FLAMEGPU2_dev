#include "RVOGraph.cuh"
#include "flamegpu/gpu/CUDAErrorChecking.h"


RVOGraph::RVOGraph(void)
{
    rvoObstacles = nullptr;
    obsKdNode = nullptr;
    kdNodeCount = 0;
}


RVOGraph::~RVOGraph(void)
{
}

void RVOGraph::buildRVO(std::vector<std::vector<float2>>& obstacles){
	
	rvoObstacles = RVObstacleVecPtr(new RVObstacleVec());

	for( auto line : obstacles){
		addRVOObstacle(line, rvoObstacles);
	}

	if(obstacles.size() > 0){
        obsKdNode = recursivelyBuildObstacleTree(rvoObstacles, rvoObstacles);
        recursivelyIndexObstacleTreeNode(obsKdNode, kdNodeCount);
	}


}

void RVOGraph::uploadRVOToGPU(void* obstacles_d, void* kdnodes_d){

    auto gpuObstacles = initialiseGPUObstacles(rvoObstacles->size());
    for( unsigned int i = 0; i < rvoObstacles->size(); i++){
        gpuObstacles.id[i] = (*rvoObstacles)[i]->id;
        gpuObstacles.point[i] = (*rvoObstacles)[i]->point;
        gpuObstacles.unitDir[i] = (*rvoObstacles)[i]->unitDir;
        gpuObstacles.isConvex[i] = (*rvoObstacles)[i]->isConvex;
        gpuObstacles.nextObstacleIndex[i] = (*rvoObstacles)[i]->nextObstacleIndex;
        gpuObstacles.prevObstacleIndex[i] = (*rvoObstacles)[i]->prevObstacleIndex;
    }
    gpuErrchk(cudaMemcpy(obstacles_d, &gpuObstacles, sizeof(RVObstacleGSOA), cudaMemcpyHostToDevice));

    auto gpuKdnodes = initialiseGPUKdnodes(kdNodeCount);
    if(rvoObstacles->size() > 0)
        recursivelyBuildGPUObstacleTree(obsKdNode, 0, true, gpuKdnodes);
    gpuErrchk(cudaMemcpy(kdnodes_d, &gpuKdnodes, sizeof(RVOKDNodeGSOA), cudaMemcpyHostToDevice));
}

RVObstacleGSOA RVOGraph::initialiseGPUObstacles(size_t size){
	RVObstacleGSOA obstacles;
	obstacles.size = size;
	obstacles.id = nullptr;
	obstacles.point = nullptr;
	obstacles.unitDir = nullptr;
	obstacles.isConvex = nullptr;
	obstacles.nextObstacleIndex = nullptr;
	obstacles.prevObstacleIndex = nullptr;

	if(size > 0){
		gpuErrchk(cudaMallocManaged(&obstacles.id, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&obstacles.point, size*sizeof(float2)));
        gpuErrchk(cudaMallocManaged(&obstacles.unitDir, size*sizeof(float2)));
        gpuErrchk(cudaMallocManaged(&obstacles.isConvex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&obstacles.nextObstacleIndex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&obstacles.prevObstacleIndex, size*sizeof(int)));

	}


	return obstacles;
}

RVOKDNodeGSOA RVOGraph::initialiseGPUKdnodes(size_t size){
	RVOKDNodeGSOA kdnodes;
	kdnodes.size = size;
	kdnodes.leftIndex = nullptr;
	kdnodes.rightIndex = nullptr;
	kdnodes.siblingIndex = nullptr;
	kdnodes.parentIndex = nullptr;
	kdnodes.obstacleIndex = nullptr;

	if(size > 0){
        gpuErrchk(cudaMallocManaged(&kdnodes.leftIndex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&kdnodes.rightIndex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&kdnodes.siblingIndex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&kdnodes.parentIndex, size*sizeof(int)));
        gpuErrchk(cudaMallocManaged(&kdnodes.obstacleIndex, size*sizeof(int)));
	}

	return kdnodes;
}




int RVOGraph::addRVOObstacle( std::vector<float2> &vertices , RVObstacleVecPtr obstacles)
{
	if (vertices.size() < 2) {
		return -1;
	}

	const size_t obstacleNo = obstacles->size();

	for (size_t i = 0; i < vertices.size(); ++i) {
		RVObstacle* obstacle = new RVObstacle();
		obstacle->point = vertices[i];

		//If not the first element then links to previous element
		if (i != 0) {
			obstacle->prevObstacleIndex = obstacles->size() - 1;
			(*obstacles)[obstacle->prevObstacleIndex]->nextObstacleIndex = obstacles->size();
		}

		//If last element then links to the first element
		if (i == vertices.size() - 1) {
			obstacle->nextObstacleIndex = obstacleNo;
			(*obstacles)[obstacle->nextObstacleIndex]->prevObstacleIndex = obstacles->size();
		}

		obstacle->unitDir = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

		//obstacle->isConvex = true;

		if (vertices.size() == 2) {
			obstacle->isConvex = 1;
		}
		else {
			bool isConvex = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			obstacle->isConvex = isConvex ? 1 : 0;
		}


		obstacle->id = obstacles->size();

		obstacles->push_back(obstacle);
	}

	return obstacleNo;
}





int RVOGraph::getKDTreeDepth( int index, RVOKDNodeGSOA& nodes )
{
	if(index < 0)
		return 0;

	int leftDepth = getKDTreeDepth(nodes.leftIndex[index], nodes);
	int rightDepth = getKDTreeDepth(nodes.rightIndex[index], nodes);
	int maxDepth = leftDepth > rightDepth ? leftDepth : rightDepth;
	return maxDepth + 1;
}

ObstacleTreeNode * RVOGraph::recursivelyBuildObstacleTree( RVObstacleVecPtr obstacles , RVObstacleVecPtr globalObstacles, int obsLevel)
{
	float RVO_EPSILON = 0.00001f;

	if (obstacles->size() == 0) {
		return nullptr;
	}
	else {
		ObstacleTreeNode *node = new ObstacleTreeNode;

		size_t optimalSplit = 0;
		size_t minLeft = obstacles->size();
		size_t minRight = obstacles->size();

		for (size_t i = 0; i < obstacles->size(); ++i) {
			size_t leftSize = 0;
			size_t rightSize = 0;

			 RVObstacle * obstacleI1 = (*obstacles)[i];
			 RVObstacle * obstacleI2 = (*globalObstacles)[obstacleI1->nextObstacleIndex];

			/* Compute optimal split node. */
			for (size_t j = 0; j < obstacles->size(); ++j) {
				if (i == j) {
					continue;
				}

				RVObstacle *obstacleJ1 = (*obstacles)[j];
				RVObstacle *obstacleJ2 = (*globalObstacles)[obstacleJ1->nextObstacleIndex];

				const float j1LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ1->point);
				const float j2LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ2->point);

				if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
					++leftSize;
				}
				else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
					++rightSize;
				}
				else {
					++leftSize;
					++rightSize;
				}

				if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) >= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
					break;
				}
			}

			if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) < std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
				minLeft = leftSize;
				minRight = rightSize;
				optimalSplit = i;
			}
		}

		/* Build split node. */
		RVObstacleVecPtr leftObstacles( new RVObstacleVec());
		RVObstacleVecPtr rightObstacles( new RVObstacleVec());

		const size_t i = optimalSplit;

		RVObstacle* obstacleI1 = (*obstacles)[i];
		RVObstacle* obstacleI2 = (*globalObstacles)[obstacleI1->nextObstacleIndex];

		for (size_t j = 0; j < obstacles->size(); ++j) {
			if (i == j) {
				continue;
			}

			RVObstacle* obstacleJ1 = (*obstacles)[j];
			RVObstacle* obstacleJ2 = (*globalObstacles)[obstacleJ1->nextObstacleIndex];

			const float j1LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ1->point);
			const float j2LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ2->point);

			if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
				leftObstacles->push_back((*obstacles)[j]);
			}
			else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
				rightObstacles->push_back((*obstacles)[j]);
			}
			else {
				/* Split obstacle j. */
				const float t = det(obstacleI2->point - obstacleI1->point, obstacleJ1->point - obstacleI1->point) / det(obstacleI2->point - obstacleI1->point, obstacleJ1->point - obstacleJ2->point);

				const float2 splitpoint = obstacleJ1->point + t * (obstacleJ2->point - obstacleJ1->point);

				RVObstacle * newObstacle = new RVObstacle();
				newObstacle->point = splitpoint;
				newObstacle->prevObstacleIndex = obstacleJ1->id;
				newObstacle->nextObstacleIndex = obstacleJ2->id;
				newObstacle->isConvex = 1;
				newObstacle->unitDir = obstacleJ1->unitDir;

				newObstacle->id = globalObstacles->size();

				globalObstacles->push_back(newObstacle);

				obstacleJ1->nextObstacleIndex = newObstacle->id;
				obstacleJ2->prevObstacleIndex = newObstacle->id;

				if (j1LeftOfI > 0.0f) {
					leftObstacles->push_back(obstacleJ1);
					rightObstacles->push_back(newObstacle);
				}
				else {
					rightObstacles->push_back(obstacleJ1);
					leftObstacles->push_back(newObstacle);
				}
			}
		}

		node->obstacle = obstacleI1;
		obsLevel += 1;
		node->left = recursivelyBuildObstacleTree(leftObstacles,globalObstacles, obsLevel);
		node->right = recursivelyBuildObstacleTree(rightObstacles, globalObstacles, obsLevel);
		return node;
	}
}


void RVOGraph::recursivelyIndexObstacleTreeNode( ObstacleTreeNode * currentNode, int& index )
{
	currentNode->index = index;
	index++;

	if(currentNode->left != nullptr)
	{
		recursivelyIndexObstacleTreeNode(currentNode->left, index);
	}

	if(currentNode->right != nullptr)
	{
		recursivelyIndexObstacleTreeNode(currentNode->right, index);
	}

}


void RVOGraph::recursivelyBuildGPUObstacleTree( ObstacleTreeNode* node, ObstacleTreeNode* parentNode, bool isLeft, RVOKDNodeGSOA& outputNodeList)
{
	// assert(outputNodeList.size() >  node->index);

	RVOKDNode gNode;
	
	if(node->left != nullptr)
	{
		gNode.leftIndex = node->left->index;
		recursivelyBuildGPUObstacleTree(node->left , node, true,outputNodeList);
	}
	else
		gNode.leftIndex = -1;


	if(node->right != nullptr)
	{
		gNode.rightIndex = node->right->index;
		recursivelyBuildGPUObstacleTree( node->right, node, false, outputNodeList );
	}
	else
		gNode.rightIndex = -1;

	gNode.obstacleIndex = node->obstacle->id;
	
	if(parentNode != nullptr)
	{
		gNode.parentIndex = parentNode->index;

		ObstacleTreeNode* siblingNode = isLeft ? parentNode->right : parentNode->left;
		if(siblingNode != nullptr)
		{
			gNode.siblingIndex = siblingNode->index;
		}
		else
		{
			gNode.siblingIndex = -1;
		}

	}
	else		
	{
	
		gNode.parentIndex = -1;
		gNode.siblingIndex = -1;
	}

	
	outputNodeList.leftIndex[node->index] = gNode.leftIndex;
	outputNodeList.rightIndex[node->index] = gNode.rightIndex;
	outputNodeList.siblingIndex[node->index] = gNode.siblingIndex;
	outputNodeList.parentIndex[node->index] = gNode.parentIndex;
	outputNodeList.obstacleIndex[node->index] = gNode.obstacleIndex;

	

}

	


