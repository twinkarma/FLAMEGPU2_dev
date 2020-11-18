#include "RVOGraph.cuh"
#include "flamegpu/gpu/CUDAErrorChecking.h"


RVOGraph::RVOGraph(void)
{
}


RVOGraph::~RVOGraph(void)
{
}

void RVOGraph::buildRVO(std::vector<std::vector<float2>>& obstacles, void* obstacles_d, void* kdnodes_d){
	
	std::vector<RVObstacle*> rvoObstacles;

	for( auto line : obstacles){
		addRVOObstacle(line, rvoObstacles);
	}

	auto gpuObstacles = initialiseObstacles(rvoObstacles.size());
	for( int i = 0; i < rvoObstacles.size(); i++){
		gpuObstacles.id[i] = rvoObstacles[i]->id;
		gpuObstacles.point[i] = rvoObstacles[i]->point;
		gpuObstacles.unitDir[i] = rvoObstacles[i]->unitDir;
		gpuObstacles.isConvex[i] = rvoObstacles[i]->isConvex;
		gpuObstacles.nextObstacleIndex[i] = rvoObstacles[i]->nextObstacleIndex;
		gpuObstacles.prevObstacleIndex[i] = rvoObstacles[i]->prevObstacleIndex;
	}

    gpuErrchk(cudaMemcpy(obstacles_d, &gpuObstacles, sizeof(gpuObstacles), cudaMemcpyHostToDevice));

	if(obstacles.size() > 0){
        ObstacleTreeNode* obsKdNode = recursivelyBuildObstacleTree(rvoObstacles, rvoObstacles);
        int kdNodeCount = 0;
        recursivelyIndexObstacleTreeNode(obsKdNode, kdNodeCount);
        auto gpuKdnodes = initialiseKdnodes(kdNodeCount); //Pre-allocate the array
        recursivelyBuildGPUObstacleTree(obsKdNode, 0, true, gpuKdnodes);
        gpuErrchk(cudaMemcpy(kdnodes_d, &gpuKdnodes, sizeof(gpuKdnodes), cudaMemcpyHostToDevice));

	}
	else
    {
	    //No nodes if empty
        auto gpuKdnodes = initialiseKdnodes(0); //Pre-allocate the array
        gpuErrchk(cudaMemcpy(kdnodes_d, &gpuKdnodes, sizeof(gpuKdnodes), cudaMemcpyHostToDevice));

    }
}

RVObstacleGSOA RVOGraph::initialiseObstacles(size_t size){
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

RVOKDNodeGSOA RVOGraph::initialiseKdnodes(size_t size){
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



// void RVOGraph::buildRVO( Scene* scene, NavigationDataStructs& navData )
// {
// 	//Creates the boundaries for RVO
// 	std::vector<RVObstacle*> obstacles = buildRVOObstacles(scene);
// 	std::vector<RVObstacle*> obstaclesTemp;
// 	for(auto rvo: obstacles)
// 	{
// 		obstaclesTemp.push_back(rvo);
// 	}
// 	ObstacleTreeNode* obsKdNode = recursivelyBuildObstacleTree(obstaclesTemp, obstacles);
// 	int kdNodeCount = 0;
// 	recursivelyIndexObstacleTreeNode(obsKdNode, kdNodeCount);
// 	for( int i = 0; i < kdNodeCount; i++)
// 	{
// 		auto temp = RVOKDNode();
// 		navData.rVOKDNodes.push_back(temp);
// 	} //Pre-allocate the array
// 	recursivelyBuildGPUObstacleTree(obsKdNode, 0, true, navData.rVOKDNodes);

// 	//Convert the data structures to a GPU one
// 	for( int i = 0; i < obstacles.size(); i++)
// 	{
// 		navData.rVObstacles.push_back(*obstacles[i]);
// 	}
// }

// //Modification: 

// /*
// First pass - process all line segments where an intersection occurs and split the line segments at the intersection point, inserting them at the relevant point in the original line structures. 

// Second pass - any line segment where the midpoint is within any of the un-processed polygons (excluding their parent shape) can be excluded - leaving only those lines which are not within any polygon. 
// This will give a full list of all lines that are not within any polygon - (and therefore form the boundaries of the environment), and can be used as obstacles.
// */


// std::vector<RVObstacle*>  RVOGraph::buildRVOObstacles( Scene* scene)
// {

	

// 	std::vector<RVObstacle*> obstacles;

// 	std::vector<Line*> walkableLines;
// 	std::vector<Line*> testables;

// 	std::vector<LineSegment*> lines;

// 	for( auto stage: scene->Stages())
// 	{
// 		//Iterates through the stage to get the walkables and obstacles
// 		for( Asset* asset : stage->Assets())
// 		{
// 			if(asset->getAssetType() == Data::AssetTypeEnum::WalkableType || asset->getAssetType() == Data::AssetTypeEnum::ObstacleType)
// 			{
// 				Element* elem = asset->getElement();
// 				if(elem != NULL)
// 				{
// 					for(Line* l : elem->Lines())
// 					{
// 						if(asset->getAssetType() == Data::AssetTypeEnum::ObstacleType || !l->isClosedLoop() || l->size() < 3)
// 						{
// 							std::vector<float2> verts;
// 							for(int i = 0; i < l->size(); i++)
// 							{
// 								verts.push_back(make_float2( l->getPoints()[i].x, l->getPoints()[i].y ));
// 							}
// 							addRVOObstacleExpanded(verts, l->isClosedLoop(), obstacles);
// 						}
// 						else
// 						{
// 							walkableLines.push_back(l->cloneToLine());
// 							testables.push_back(l->clone());
// 						}
// 					}
// 				}
// 			}

// 		}
// 	}



// 	for(int a=0;a<walkableLines.size();a++) {
// 		for(int b=0;b<walkableLines.size();b++) {

// 			if (a == b) continue;
// 			Data::Line& l1 = *walkableLines[a];
// 			Data::Line& l2 = *walkableLines[b];


			
// 			//if (l1.intersects(&l2)) {
// 				auto checkIntersections = true;
// 				while(checkIntersections) {

// 					checkIntersections = false;
// 					for(int i=0;i<l1.size();i++) {
// 						bool earlyExit = false;
// 						int current = i;
// 						int next = i == l1.size()-1 ? 0 : i+1;

// 						auto p = l1[current];
// 						auto pn = l1[next];

// 						for(int j=0;j<l2.size();j++) {
// 							int currentL = j;
// 							int nextL = j == l2.size()-1 ? 0 : j+1;

// 							auto c = l2[currentL];
// 							auto cn = l2[nextL];


// 							if (((fabs(p.x - c.x) < 0.001f) && (fabs(p.y - c.y) <  0.001f)) ||
// 									((fabs(pn.x - c.x) < 0.001f) && (fabs(pn.y - c.y) <  0.001f)) ||
// 									((fabs(p.x - cn.x) < 0.001f) && (fabs(p.y - cn.y) <  0.001f)) ||
// 									((fabs(pn.x - cn.x) < 0.001f) && (fabs(pn.y - cn.y) <  0.001f))
// 									){
// 										continue;
// 							}

// 							if (Vector2Helper::linesIntersect(p, pn, c, cn)) {
								
// 								Vector2* point = Vector2Helper::linesIntersectPoint(p, pn, c, cn);

// 								if (point == NULL) { 
// 									continue; 
// 								}
// 								if (((fabs(p.x - point->x) < 0.001f) && (fabs(p.y - point->y) <  0.001f)) ||
// 									((fabs(pn.x - point->x) < 0.001f) && (fabs(pn.y - point->y) <  0.001f)) ||
// 									((fabs(c.x - point->x) < 0.001f) && (fabs(c.y - point->y) <  0.001f))||
// 									((fabs(cn.x - point->x) < 0.001f) && (fabs(cn.y - point->y) <  0.001f))

// 									) {
// 										delete point;
// 									continue;
// 								}


// 								l1.insertPoint(next, *point);
// 								l2.insertPoint(nextL, *point);

// 								delete point;

// 								checkIntersections = true;
// 								earlyExit = true;
// 								break;
// 							}

// 						}
// 						if (earlyExit) break;
// 					}
// 				}
// 			//}
// 		}
// 	}

// 	for(int a=0;a<walkableLines.size();a++) {

// 		Data::Line& l1 = *walkableLines[a];

// 		for(int i=0;i<l1.size();i++) {
// 			bool ignore = false;

// 			int current = i;
// 			int next = i == l1.size()-1 ? 0 : i+1;

// 			auto p = l1[current];
// 			auto pn = l1[next];

// 			for(int b=0;b<testables.size();b++) {
// 				if (a == b) continue;
					

// 				Vector2 midpoint;
// 				midpoint.x = p.x + ((pn.x - p.x) * 0.5f);
// 				midpoint.y = p.y + ((pn.y - p.y) * 0.5f);
						

// 				if (testables[b]->isPointInside(midpoint)) {
// 					ignore = true;
// 					break;
// 				}
// 			}
// 			if (!ignore) {
// 				LineSegment* seg = new LineSegment();
// 				seg->pt1 = p;
// 				seg->pt2 = pn;
// 				lines.push_back(seg);
// 			}
// 		}
// 	}



// 	//Find overlapping lines and combine them
// 	//int reductionIndex = -1;
// 	//std::vector<Line*> reducedLine;
// 	//while(walkableLines.size() > 0)
// 	//{
// 	//	//Takes the first line in the list and remove from the array
// 	//	Line* testLine = (*walkableLines.begin())->clone();
// 	//	
// 	//	walkableLines.erase(walkableLines.begin());
// 	//	//reductionIndex++;

// 	//	while(true)
// 	//	{
// 	//		int overlapCount = 0;

// 	//		auto wlItt = walkableLines.begin();
// 	//		while( wlItt != walkableLines.end() )
// 	//		{
// 	//			bool mergeSuccess = false;
// 	//			Line* currentLine = *wlItt;
// 	//			if(testLine->intersects(currentLine) || testLine->isInside(currentLine) || currentLine->isInside(testLine))
// 	//			{
// 	//				Line* mergedLine = Line::mergePolylines(testLine, currentLine);
// 	//				if(mergedLine != 0)
// 	//				{
// 	//					mergedLine->updateTriangulation();
// 	//					//Delete previous line and swap
// 	//					delete testLine;
// 	//					testLine = mergedLine;

// 	//					overlapCount ++;
// 	//					wlItt = walkableLines.erase(wlItt); //Erase the current iterator
// 	//					mergeSuccess = true;
// 	//				}
// 	//				
// 	//			}

// 	//			//Increment if there's been no overlap or no merge
// 	//			if(!mergeSuccess)
// 	//				wlItt++;

// 	//		}

// 	//		if(overlapCount <= 0)
// 	//			break;
// 	//	}

// 	//	reducedLine.push_back(testLine);
// 	//	
// 	//}

// 	//Adds the merged walkables
// 	/*for( auto l: reducedLine)
// 	{
// 		std::vector<float2> verts;
// 		for(int i = 0; i < l->size(); i++)
// 		{
			
// 			Vector2 vt = l->getPoints()[i];
// 			verts.push_back(make_float2( vt.x, vt.y ));
		
// 		}
// 		addRVOObstacleExpanded(verts, l->isClosedLoop(), obstacles);
// 	}*/

// 	for(auto l : lines) {
// 		std::vector<float2> verts;
// 		verts.push_back(make_float2(l->pt1.x, l->pt1.y));
// 		verts.push_back(make_float2(l->pt2.x, l->pt2.y));

// 		//addRVOObstacleExpanded(verts, false, obstacles);

		
//  			addRVOObstacle(verts, obstacles);
		
// 	}
	

// 	return obstacles;
// }



int RVOGraph::addRVOObstacle( std::vector<float2> &vertices , std::vector<RVObstacle*> &obstacles)
{
	if (vertices.size() < 2) {
		return -1;
	}

	const size_t obstacleNo = obstacles.size();

	for (size_t i = 0; i < vertices.size(); ++i) {
		RVObstacle* obstacle = new RVObstacle();
		obstacle->point = vertices[i];

		//If not the first element then links to previous element
		if (i != 0) {
			obstacle->prevObstacleIndex = obstacles.size() - 1;
			obstacles[obstacle->prevObstacleIndex]->nextObstacleIndex = obstacles.size();
		}

		//If last element then links to the first element
		if (i == vertices.size() - 1) {
			obstacle->nextObstacleIndex = obstacleNo;
			obstacles[obstacle->nextObstacleIndex]->prevObstacleIndex = obstacles.size();
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


		obstacle->id = obstacles.size();

		obstacles.push_back(obstacle);
	}

	return obstacleNo;
}



void RVOGraph::addRVOObstacleExpanded( std::vector<float2> &vertices, bool isClosedLoop, std::vector<RVObstacle*> &obstacles )
{
	float expandBy = 0.01;

	if(vertices.size() > 2)
	{
		for(int i = isClosedLoop? 0 : 1 ; i < vertices.size(); i++) //Goes up to the size to connect the last line
		{


			int p1Index = i-2 < 0 ? vertices.size() + (i - 2)  : i -2;
			int p2Index = i - 1 < 0 ? vertices.size() + (i - 1) : i - 1;
			float2 p1 = vertices[p1Index];
			float2 p2 = vertices[p2Index];
			float2 p3 = vertices[i];
			float2 p4 = vertices[i+1 > vertices.size() - 1 ?  0 : i + 1];

			float2 v1 = p2 - p1;
			float2 v2 = p3 - p2;
			float2 v3 = p4 - p3;

			float2 pn1 = normalize(make_float2(-v1.y, v1.x));
			float2 pn2 = normalize(make_float2(-v2.y, v2.x));
			float2 pn3 = normalize(make_float2(-v3.y, v3.x));

			float2 perp1 = normalize(pn1 + pn2);
			float2 perp2 = normalize(pn2 + pn3);

			if( (i == 1 || i >= vertices.size() -1) && !isClosedLoop )
			{
				perp1 = pn2;
				perp2 = pn2;
			}

			std::vector<float2> oVert;
			oVert.push_back(p2 - perp1*expandBy);
			oVert.push_back(p3 - perp2*expandBy);
			oVert.push_back(p3 + perp2*expandBy);
			oVert.push_back(p2 + perp1*expandBy);

			addRVOObstacle(oVert, obstacles);

		}
	}
	else if(vertices.size() == 2)
	{
// 		for(int i = 1; i < vertices.size(); i++)
// 		{
// 			float2 p2 = vertices[i-1];
// 			float2 p3 = vertices[i];
// 
// 			float2 v2 = p3 - p2;
// 			float2 pn2 = normalize(make_float2(-v2.y, v2.x));
// 
// 			std::vector<float2> oVert;
// 			oVert.push_back(p2 - pn2*expandBy);
// 			oVert.push_back(p3 - pn2*expandBy);
// 			oVert.push_back(p3 + pn2*expandBy);
// 			oVert.push_back(p2 + pn2*expandBy);
// 			addRVOObstacle(oVert, obstacles);
// 		}
	}

	
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



std::vector<float2> RVOGraph::expandObstacle( std::vector<float2> &inputObs, float expandAmount )
{
	std::vector<float2> outputVerts;
	for( int i = 1; i < inputObs.size(); i++)
	{
		
		float2 p1 = inputObs[i-1];
		float2 p2 = inputObs[i];
		float2 vecTo = p2 - p1;
		float2 vecToPerpNorm = normalize(make_float2(-vecTo.y, vecTo.x));
		float clearDist = 0.2;
		outputVerts.push_back(p1 - vecToPerpNorm*clearDist);
		outputVerts.push_back(p2 - vecToPerpNorm*clearDist);
		
	}

	for( int i = inputObs.size() - 2; i >= 0; i--)
	{
		float2 p1 = inputObs[i+1];
		float2 p2 = inputObs[i];
		float2 vecTo = p2 - p1;
		float2 vecToPerpNorm = normalize(make_float2(-vecTo.y, vecTo.x));
		float clearDist = 0.2;
		outputVerts.push_back(p1 - vecToPerpNorm*clearDist);
		outputVerts.push_back(p2 - vecToPerpNorm*clearDist);


	}
	
	return outputVerts;
}




ObstacleTreeNode * RVOGraph::recursivelyBuildObstacleTree( std::vector<RVObstacle*> &obstacles , std::vector<RVObstacle*> &globalObstacles)
{
	float RVO_EPSILON = 0.00001f;

	if (obstacles.size() == 0) {
		return NULL;
	}
	else {
		ObstacleTreeNode *const node = new ObstacleTreeNode;

		size_t optimalSplit = 0;
		size_t minLeft = obstacles.size();
		size_t minRight = obstacles.size();

		for (size_t i = 0; i < obstacles.size(); ++i) {
			size_t leftSize = 0;
			size_t rightSize = 0;

			 RVObstacle * obstacleI1 = obstacles[i];
			 RVObstacle * obstacleI2 = globalObstacles[obstacleI1->nextObstacleIndex];

			/* Compute optimal split node. */
			for (size_t j = 0; j < obstacles.size(); ++j) {
				if (i == j) {
					continue;
				}

				RVObstacle *obstacleJ1 = obstacles[j];
				RVObstacle *obstacleJ2 = globalObstacles[obstacleJ1->nextObstacleIndex];

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
		std::vector<RVObstacle *> leftObstacles(minLeft);
		std::vector<RVObstacle *> rightObstacles(minRight);

		size_t leftCounter = 0;
		size_t rightCounter = 0;
		const size_t i = optimalSplit;

		RVObstacle* obstacleI1 = obstacles[i];
		RVObstacle* obstacleI2 = globalObstacles[obstacleI1->nextObstacleIndex];

		for (size_t j = 0; j < obstacles.size(); ++j) {
			if (i == j) {
				continue;
			}

			RVObstacle* obstacleJ1 = obstacles[j];
			RVObstacle* obstacleJ2 = globalObstacles[obstacleJ1->nextObstacleIndex];

			const float j1LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ1->point);
			const float j2LeftOfI = leftOf(obstacleI1->point, obstacleI2->point, obstacleJ2->point);

			if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
				leftObstacles[leftCounter++] = obstacles[j];
			}
			else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
				rightObstacles[rightCounter++] = obstacles[j];
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

				newObstacle->id = globalObstacles.size();

				globalObstacles.push_back(newObstacle);

				obstacleJ1->nextObstacleIndex = newObstacle->id;
				obstacleJ2->prevObstacleIndex = newObstacle->id;

				if (j1LeftOfI > 0.0f) {
					leftObstacles[leftCounter++] = obstacleJ1;
					rightObstacles[rightCounter++] = newObstacle;
				}
				else {
					rightObstacles[rightCounter++] = obstacleJ1;
					leftObstacles[leftCounter++] = newObstacle;
				}
			}
		}

		node->obstacle = obstacleI1;
		node->left = recursivelyBuildObstacleTree(leftObstacles,globalObstacles);
		node->right = recursivelyBuildObstacleTree(rightObstacles, globalObstacles);
		return node;
	}
}


void RVOGraph::recursivelyIndexObstacleTreeNode( ObstacleTreeNode * currentNode, int& index )
{
	currentNode->index = index;
	index++;

	if(currentNode->left != NULL)
	{
		recursivelyIndexObstacleTreeNode(currentNode->left, index);
	}

	if(currentNode->right != NULL)
	{
		recursivelyIndexObstacleTreeNode(currentNode->right, index);
	}

}


void RVOGraph::recursivelyBuildGPUObstacleTree( ObstacleTreeNode* node, ObstacleTreeNode* parentNode, bool isLeft, RVOKDNodeGSOA& outputNodeList)
{
	// assert(outputNodeList.size() >  node->index);

	RVOKDNode gNode;
	
	if(node->left != NULL)
	{
		gNode.leftIndex = node->left->index;
		recursivelyBuildGPUObstacleTree(node->left , node, true,outputNodeList);
	}
	else
		gNode.leftIndex = -1;


	if(node->right != NULL)
	{
		gNode.rightIndex = node->right->index;
		recursivelyBuildGPUObstacleTree( node->right, node, false, outputNodeList );
	}
	else
		gNode.rightIndex = -1;

	gNode.obstacleIndex = node->obstacle->id;
	
	if(parentNode != NULL)
	{
		gNode.parentIndex = parentNode->index;

		ObstacleTreeNode* siblingNode = isLeft ? parentNode->right : parentNode->left;
		if(siblingNode != NULL)
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

	


