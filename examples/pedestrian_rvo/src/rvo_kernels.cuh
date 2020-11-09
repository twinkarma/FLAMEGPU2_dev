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

__constant__ RVObstacleGSOA rVObstacles_d;
__constant__ RVOKDNodeGSOA rVOKDNodes_d;

struct RVOLine{
	float2 point;
	float2 direction;
};

//Device functions for working out collision avoidance
__device__ bool linearProgram1(RVOLine *lines, int lineCount, int lineNo, float radius, float2 &optVelocity, bool directionOpt, float2 &result)
{
	float dotProduct = dot(lines[lineNo].point, lines[lineNo].direction);
	float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);

	if (discriminant < 0.0f) {
		/* Max speed circle fully invalidates line lineNo. */
		return false;
	}

	float sqrtDiscriminant = sqrtf(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (int i = 0; i < lineNo; i++) {
		float denominator = det(lines[lineNo].direction, lines[i].direction);
		float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

		if (fabsf(denominator) <= RVO_EPSILON) {
			/* Lines lineNo and i are (almost) parallel. */
			if (numerator < 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		float t = numerator / denominator;

		if (denominator >= 0.0f) {
			/* Line i bounds line lineNo on the right. */
			tRight = fminf(tRight, t);
		}
		else {
			/* Line i bounds line lineNo on the left. */
			tLeft = fmaxf(tLeft, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (dot(optVelocity,lines[lineNo].direction) > 0.0f) {
			/* Take right extreme. */
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			/* Take left extreme. */
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
	}
	else {
		/* Optimize closest point. */
		float t = dot(lines[lineNo].direction , (optVelocity - lines[lineNo].point) );

		if (t < tLeft) {
			result = lines[lineNo].point + tLeft * lines[lineNo].direction;
		}
		else if (t > tRight) {
			result = lines[lineNo].point + tRight * lines[lineNo].direction;
		}
		else {
			result = lines[lineNo].point + t * lines[lineNo].direction;
		}
	}

	return true;
}

__device__ int linearProgram2(RVOLine *lines, int lineCount, float radius, float2 &optVelocity, bool directionOpt, float2& result)
{
	if (directionOpt) {
		/*
			* Optimize direction. Note that the optimization velocity is of unit
			* length in this case.
			*/
		result = optVelocity * radius;
	}
	else if (absSq(optVelocity) > sqr(radius)) {
		/* Optimize closest point and outside circle. */
		result = normalize(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (int i = 0; i < lineCount; i++) {
		float lineResult = det(lines[i].direction, lines[i].point - result);
		if (lineResult > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			float2 tempResult = result;

			if (!linearProgram1(lines, lineCount, i, radius, optVelocity, directionOpt, result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return lineCount;
}

 __device__ void linearProgram3( RVOLine *lines, int lineCount, int numObstLines, int beginLine, float radius, float2 &result)
{
	float distance = 0.0f;

	for (int i = beginLine; i < lineCount; ++i) {
		if (det(lines[i].direction, lines[i].point - result) > distance) {
			/* Result does not satisfy constraint of line i. */

			
			RVOLine projLines[ORCA_ARRAY_SIZE];
			int projLinesCount = 0;
			for( int obstIndex = 0; obstIndex < numObstLines; obstIndex++)
			{
				projLines[obstIndex] = lines[obstIndex];
				projLinesCount++;
			}

			for (int j = numObstLines; j < i; ++j) {
				RVOLine line;

				float determinant = det(lines[i].direction, lines[j].direction);

				if (fabsf(determinant) <= RVO_EPSILON) {
					/* Line i and line j are parallel. */
					if ( dot(lines[i].direction, lines[j].direction) > 0.0f) {
						/* Line i and line j point in the same direction. */
						continue;
					}
					else {
						/* Line i and line j point in opposite direction. */
						line.point = 0.5f * (lines[i].point + lines[j].point);
					}
				}
				else {
					line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
				}

				line.direction = normalize(lines[j].direction - lines[i].direction);
				projLines[projLinesCount] = line;
				projLinesCount++;
			}

			float2 tempResult = result;

			auto optVel = make_float2(-lines[i].direction.y, lines[i].direction.x);
			if (linearProgram2(projLines, projLinesCount, radius, optVel, true, result) < projLinesCount) {
				/* This should in principle not happen.  The result is by definition
					* already in the feasible region of this linear program. If it fails,
					* it is due to small floating point error, and the current result is
					* kept.
					*/
				result = tempResult;
			}

			distance = det(lines[i].direction, lines[i].point - result);
		}
	}
}

__device__ void addToLine(RVOLine* lines, int &lineCount, RVOLine &lineToAdd)
{
	if(lineCount >= ORCA_ARRAY_SIZE) return;

	lines[lineCount] = lineToAdd;
	lineCount++;
}

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

__device__ void addNeighbourSorted(Ped* neighbours, int& neighbourCount, Ped& neighbourToAdd)
{
	//Find minimum
	int insertPos = 0;

	while(insertPos < ORCA_ARRAY_SIZE)
	{
		if(insertPos >= neighbourCount)
		{
			//insert if last place
			neighbours[neighbourCount] = neighbourToAdd;
			neighbourCount++;
			break;
		}
		else if(neighbourToAdd.separationSq < neighbours[insertPos].separationSq)
		{
			//Check if dist square is less, if it is then insert

			//Move up from insert pos
			int maxPos = ORCA_ARRAY_SIZE - 1 < neighbourCount  ? ORCA_ARRAY_SIZE -1 : neighbourCount ;
			for(int m = maxPos ; m > insertPos; m--)
			{
				neighbours[m] = neighbours[m-1];
			}
			neighbourCount = maxPos + 1;

			//Insert 
			neighbours[insertPos] = neighbourToAdd;

			break;
		}

		insertPos++;
	}

}

__device__ void addObstacleSorted(Obst* obstacles, int& obstCount, Obst& obstToAdd)
{
	//Find minimum
	int insertPos = 0;

	while(insertPos < OBSTACLE_DETECT_ARRAY_SIZE)
	{
		if(insertPos >= obstCount)
		{
			//insert if last place
			obstacles[obstCount] = obstToAdd;
			obstCount++;
			break;
		}
		else if(obstToAdd.distSq < obstacles[insertPos].distSq)
		{
			//Check if dist square is less, if it is then insert

			//Move up from insert pos
			int maxPos = OBSTACLE_DETECT_ARRAY_SIZE - 1 < obstCount  ? OBSTACLE_DETECT_ARRAY_SIZE -1 : obstCount ;
			for(int m = maxPos ; m > insertPos; m--)
			{
				obstacles[m] = obstacles[m-1];
			}
			obstCount = maxPos + 1;

			//Insert 
			obstacles[insertPos] = obstToAdd;

			break;
		}

		insertPos++;
	}

// 	Put in search order for now
// 		if(obstCount < OBSTACLE_DETECT_ARRAY_SIZE)
// 		{
// 			obstacles[obstCount] = obstToAdd;
// 			obstCount++;
// 		}
	

}

__device__ void addOrcaObstacle(RVOLine* orcaLines, int &orcaLinesCount, float timeHorizonObst_, float2& position, float2& velocity , float radius_ , RVObstacleGSOA& obstacleList, int obstacleIndex)
{
// 	timeHorizonObst_ = 5.0f;
// 	radius_ = 2.0f;
// 	position = make_float2(-17.9814498f, 42.0044695f);
// 	velocity = make_float2(0.622768164f, -0.00117263198f);
// 	RVObstacle tempOp1;
// 	tempOp1.isConvex = true;
// 	tempOp1.point = make_float2(-20.0f, 40.0f);
// 	tempOp1.unitDir = make_float2(0.0f, 1.0f);
// 	RVObstacle tempOp2;
// 	tempOp2.isConvex = true;
// 	tempOp2.point = make_float2(-20, 100.0f);
// 	tempOp2.unitDir = make_float2(-1.0f, 0);
// 	RVObstacle *obstacle1 = &tempOp1;
// 	RVObstacle *obstacle2 = &tempOp2;

	/* Create obstacle ORCA lines. */

	int obs1 = obstacleIndex;
	int obs2 = rVObstacles_d.nextObstacleIndex[obs1];


	float invTimeHorizonObst = 1.0f / timeHorizonObst_;

	float2 relativePosition1 = rVObstacles_d.point[obs1] - position;
	float2 relativePosition2 = rVObstacles_d.point[obs2] - position;
	

	/*
		* Check if velocity obstacle of obstacle is already taken care of by
		* previously constructed obstacle ORCA lines.
		*/
	bool alreadyCovered = false;

	for (int j = 0; j < orcaLinesCount; j++) {
		float d1 = det(invTimeHorizonObst * relativePosition1 - orcaLines[j].point, orcaLines[j].direction) - invTimeHorizonObst * radius_;
		float d2 = det(invTimeHorizonObst * relativePosition2 - orcaLines[j].point, orcaLines[j].direction) - invTimeHorizonObst * radius_;
		if ( d1 >= -RVO_EPSILON && 
			 d2 >=  -RVO_EPSILON) {
			alreadyCovered = true;
			break;
		}
	}

	if (alreadyCovered) {
		return;
	}

	/* Not yet covered. Check for collisions. */

	float distSq1 = absSq(relativePosition1);
	float distSq2 = absSq(relativePosition2);

	float radiusSq = sqr(radius_);

		
	float2 obstacleVector = rVObstacles_d.point[obs2] - rVObstacles_d.point[obs1];
	float s = dot(-relativePosition1,obstacleVector) / absSq(obstacleVector);
	float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

	RVOLine line;

	if (s < 0.0f && distSq1 <= radiusSq) {
		/* Collision with left vertex. Ignore if non-convex. */
		if (rVObstacles_d.isConvex[obs1] > 0) {
			line.point = make_float2(0.0f, 0.0f);
			line.direction = normalize(make_float2(-relativePosition1.y, relativePosition1.x));
			addToLine(orcaLines,orcaLinesCount,line);
		}

		return;
	}
	else if (s > 1.0f && distSq2 <= radiusSq) {
		/* Collision with right vertex. Ignore if non-convex
			* or if it will be taken care of by neighoring obstace */
		if (rVObstacles_d.isConvex[obs2] > 0&& det(relativePosition2, rVObstacles_d.unitDir[obs2]) >= 0.0f) {
			line.point = make_float2(0.0f, 0.0f);
			line.direction = normalize(make_float2(-relativePosition2.y, relativePosition2.x));
			addToLine(orcaLines,orcaLinesCount,line);
		}

		return;
	}
	else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
		/* Collision with obstacle segment. */
		line.point = make_float2(0.0f, 0.0f);
		line.direction = -rVObstacles_d.unitDir[obs1];
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}

	/*
		* No collision.
		* Compute legs. When obliquely viewed, both legs can come from a single
		* vertex. Legs extend cut-off line when nonconvex vertex.
		*/

	float2 leftLegDirection, rightLegDirection;

	if (s < 0.0f && distSqLine <= radiusSq) {
		/*
			* Obstacle viewed obliquely so that left vertex
			* defines velocity obstacle.
			*/
		if (!rVObstacles_d.isConvex[obs1] > 0) {
			/* Ignore obstacle. */
			return;
		}

		obs2 = obs1;

		float leg1 = sqrtf(distSq1 - radiusSq);
		leftLegDirection = make_float2(relativePosition1.x * leg1 - relativePosition1.y * radius_ , relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
		rightLegDirection = make_float2(relativePosition1.x * leg1 + relativePosition1.y * radius_ , -relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
	}
	else if (s > 1.0f && distSqLine <= radiusSq) {
		/*
			* Obstacle viewed obliquely so that
			* right vertex defines velocity obstacle.
			*/
		if (!rVObstacles_d.isConvex[obs2] > 0) {
			/* Ignore obstacle. */
			return;
		}

		obs1 = obs2;

		float leg2 = sqrtf(distSq2 - radiusSq);
		leftLegDirection = make_float2(relativePosition2.x * leg2 - relativePosition2.y * radius_, relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
		rightLegDirection = make_float2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
	}
	else {
		/* Usual situation. */
		if (rVObstacles_d.isConvex[obs1] > 0) {
			float leg1 = sqrtf(distSq1 - radiusSq);
			leftLegDirection = make_float2(relativePosition1.x * leg1 - relativePosition1.y * radius_, relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
		}
		else {
			/* Left vertex non-convex; left leg extends cut-off line. */
			leftLegDirection = -rVObstacles_d.unitDir[obs1];
		}

		if (rVObstacles_d.isConvex[obs2] > 0) {
			float leg2 = sqrtf(distSq2 - radiusSq);
			rightLegDirection = make_float2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
		}
		else {
			/* Right vertex non-convex; right leg extends cut-off line. */
			rightLegDirection = rVObstacles_d.unitDir[obs1];
		}
	}

	/*
		* Legs can never point into neighboring edge when convex vertex,
		* take cutoff-line of neighboring edge instead. If velocity projected on
		* "foreign" leg, no constraint is added.
		*/

	//Debug testing
// 	RVObstacle lnTemp;
// 	lnTemp.isConvex = true;
// 	lnTemp.point = make_float2(-20.0f, 10.0f);
// 	lnTemp.unitDir = make_float2(0.0f, 1.0f);
// 	RVObstacle *leftNeighbor = &lnTemp;

	int leftNeighbourIndex = rVObstacles_d.prevObstacleIndex[obs1];

	bool isLeftLegForeign = false;
	bool isRightLegForeign = false;

	if (rVObstacles_d.isConvex[obs1] > 0 && det(leftLegDirection, -rVObstacles_d.unitDir[leftNeighbourIndex]) >= 0.0f) {
		/* Left leg points into obstacle. */
		leftLegDirection = -rVObstacles_d.unitDir[leftNeighbourIndex];
		isLeftLegForeign = true;
	}

	if (rVObstacles_d.isConvex[obs2] > 0 && det(rightLegDirection, rVObstacles_d.unitDir[obs2]) <= 0.0f) {
		/* Right leg points into obstacle. */
		rightLegDirection = rVObstacles_d.unitDir[obs2];
		isRightLegForeign = true;
	}

	/* Compute cut-off centers. */
	float2 leftCutoff = invTimeHorizonObst * (rVObstacles_d.point[obs1] - position);
	float2 rightCutoff = invTimeHorizonObst * (rVObstacles_d.point[obs2] - position);
	float2 cutoffVec = rightCutoff - leftCutoff;

	/* Project current velocity on velocity obstacle. */

	/* Check if current velocity is projected on cutoff circles. */
	float t = (obs1 == obs2 ? 0.5f : dot((velocity - leftCutoff), cutoffVec) / absSq(cutoffVec));
	float tLeft = dot((velocity - leftCutoff), leftLegDirection);
	float tRight = dot((velocity - rightCutoff), rightLegDirection);

	/*if(i ==0)
	{
		printf ("New \n");
		printf("Velocity: %f %f\n", velocity.x, velocity.y);
		printf("s: %f distsq1: %f distsq2: %f radiussq: %f\n", s, distSq1, distSq2, radiusSq);
		printf("left legdir: %f %f right leg dir: %f %f\n", leftLegDirection.x, leftLegDirection.y, rightLegDirection.x, rightLegDirection.y);
		printf("left cutoff: %f %f rightcutoff: %f %f cutoffvec: %f %f \n", leftCutoff.x, leftCutoff.y, rightCutoff.x, rightCutoff.y, cutoffVec.x, cutoffVec.y);
		printf("t: %f tLeft: %f tRight: %f \n", t, tLeft, tRight);

	}*/


	if ((t < 0.0f && tLeft < 0.0f) || (obs1 == obs2 && tLeft < 0.0f && tRight < 0.0f)) {
		/* Project on left cut-off circle. */
		float2 unitW = normalize(velocity - leftCutoff);

		line.direction = make_float2(unitW.y, -unitW.x);
		line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}
	else if (t > 1.0f && tRight < 0.0f) {
		/* Project on right cut-off circle. */
		float2 unitW = normalize(velocity - rightCutoff);

		line.direction = make_float2(unitW.y, -unitW.x);
		line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}

	/*
		* Project on left leg, right leg, or cut-off line, whichever is closest
		* to velocity.
		*/
	//0x7f800000 is infinity
	float distSqCutoff = ((t < 0.0f || t > 1.0f || obs1 == obs2) ? 0x7f800000 : absSq(velocity - (leftCutoff + t * cutoffVec)));
	float distSqLeft = ((tLeft < 0.0f) ? 0x7f800000 : absSq(velocity - (leftCutoff + tLeft * leftLegDirection)));
	float distSqRight = ((tRight < 0.0f) ? 0x7f800000 : absSq(velocity - (rightCutoff + tRight * rightLegDirection)));

	if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
		/* Project on cut-off line. */
		line.direction = -rVObstacles_d.unitDir[obs1];
		line.point = leftCutoff + radius_ * invTimeHorizonObst * make_float2(-line.direction.y, line.direction.x);
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}
	else if (distSqLeft <= distSqRight) {
		/* Project on left leg. */
		if (isLeftLegForeign) {
			return;
		}

		line.direction = leftLegDirection;
		line.point = leftCutoff + radius_ * invTimeHorizonObst * make_float2(-line.direction.y, line.direction.x);
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}
	else {
		/* Project on right leg. */
		if (isRightLegForeign) {
			return;
		}

		line.direction = -rightLegDirection;
		line.point = rightCutoff + radius_ * invTimeHorizonObst * make_float2(-line.direction.y, line.direction.x);
		addToLine(orcaLines,orcaLinesCount,line);
		return;
	}

}

__device__ void addOrcaAgent(RVOLine* orcaLines, int &orcaLinesCount, float timeHorizon_, float simulationTimestep, float2 position, float2 velocity , float radius_ , float2& otherPosition, float2& otherVelocity, float otherRadius)
{
	float invTimeHorizon = 1.0f / timeHorizon_;

	/* Create agent ORCA lines. */

	float2 relativePosition =otherPosition - position;
	float2 relativeVelocity = velocity -otherVelocity;
	float distSq = absSq(relativePosition);
	float combinedRadius = radius_ + otherRadius;
	float combinedRadiusSq = sqr(combinedRadius);

	RVOLine line;
	float2 u;

	if (distSq > combinedRadiusSq) {
		/* No collision. */
		float2 w = relativeVelocity - invTimeHorizon * relativePosition;
		/* Vector from cutoff center to relative velocity. */
		float wLengthSq = absSq(w);

		float dotProduct1 = dot(w,relativePosition);

		if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
			/* Project on cut-off circle. */
			float wLength = sqrtf(wLengthSq);
			float2 unitW = w / wLength;

			line.direction = make_float2(unitW.y, -unitW.x);
			u = (combinedRadius * invTimeHorizon - wLength) * unitW;
		}
		else {
			/* Project on legs. */
			float leg = sqrtf(distSq - combinedRadiusSq);

			if (det(relativePosition, w) > 0.0f) {
				/* Project on left leg. */
				line.direction = make_float2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
			}
			else {
				/* Project on right leg. */
				line.direction = (make_float2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg)*-1.0) / distSq;
			}

			float dotProduct2 = dot(relativeVelocity, line.direction);

			u = dotProduct2 * line.direction - relativeVelocity;
		}
	}
	else {
		/* Collision. Project on cut-off circle of time timeStep. */
		float invTimeStep = 1.0f / simulationTimestep;

		/* Vector from cutoff center to relative velocity. */
		float2 w = relativeVelocity - invTimeStep * relativePosition;

		float wLength = abs(w);
		float2 unitW = w / wLength;

		line.direction = make_float2(unitW.y, -unitW.x);
		u = (combinedRadius * invTimeStep - wLength) * unitW;
	}

	line.point = velocity + 0.5f * u;
	addToLine(orcaLines,orcaLinesCount,line);
	

}

__device__ void performCollisionAvoidance(RVOLine* orcaLines, int orcaLinesCount, int numObstLines, float maxSpeed, float2& prefVelocity, float2& newVelocity)
{
	
	int lineFail = linearProgram2(orcaLines, orcaLinesCount, maxSpeed, prefVelocity, false, newVelocity);

	if (lineFail < orcaLinesCount) {
		linearProgram3(orcaLines, orcaLinesCount, numObstLines, lineFail, maxSpeed, newVelocity);
	}

}


__device__ int queryObstacleTree(Obst* obstacles, int& obstCount, float2& agentPos, float rangeSq, float2& agentObstacleVector , int &obstacleVectorCount)
{
	//Stackless KD tree traversal
	int currentIndex = 0;
	int nextIndex = 0;
	unsigned long bitstack = 0;

	while(true)
	{
		int obs1 = rVOKDNodes_d.obstacleIndex[currentIndex];
		int obs2 = rVObstacles_d.nextObstacleIndex[obs1];
		float2 obsVec = rVObstacles_d.point[obs2] - rVObstacles_d.point[obs1];
		float agentLeftOfLine = leftOf(rVObstacles_d.point[obs1],rVObstacles_d.point[obs2], agentPos);
		float distSqLine = sqr(agentLeftOfLine) / absSq(rVObstacles_d.point[obs2] - rVObstacles_d.point[obs1]);
		float distSq = distSqPointLineSegment(rVObstacles_d.point[obs1], rVObstacles_d.point[obs2], agentPos);
		float wallDetectionRange = 2.5f;
		if(distSq < wallDetectionRange * wallDetectionRange )
		{
			//Perp vector
			obsVec = make_float2(-obsVec.y, obsVec.x);
			if(agentLeftOfLine < 0 ) obsVec *= -1;//Reverse if on other side
			agentObstacleVector +=  normalize(obsVec) * expf(-distSq);
			obstacleVectorCount ++;
		}

		if(distSqLine < rangeSq && agentLeftOfLine < 0.0f && distSq < rangeSq)
		{
			Obst oTemp;
			oTemp.distSq = distSq;
			oTemp.obstIndex = obs1;
			addObstacleSorted(obstacles, obstCount, oTemp);
		}

		if(rVOKDNodes_d.leftIndex[currentIndex] >= 0 || rVOKDNodes_d.rightIndex[currentIndex] >= 0)
		{
			//Is not leaf

			//Always go on the side that the agent's on first
			int nearIndex = agentLeftOfLine >= 0.0f ? rVOKDNodes_d.leftIndex[currentIndex]: rVOKDNodes_d.rightIndex[currentIndex];
			int otherIndex =  agentLeftOfLine < 0.0f ? rVOKDNodes_d.leftIndex[currentIndex]: rVOKDNodes_d.rightIndex[currentIndex];


			bitstack = bitstack << 1; //Shift bit to the left

			if(distSqLine < rangeSq && nearIndex >= 0 && otherIndex >= 0)
			{
				bitstack = bitstack | 1; //Mark node to backtrack
				currentIndex = nearIndex;
			}
			else
			{
				currentIndex = nearIndex >= 0 ? nearIndex : otherIndex;
			}

			

			continue;
		}

		//Backtrack
		unsigned long btest = bitstack  & 1;
		while(btest == 0)
		{
			if(bitstack == 0)
				return -1; //Terminate, no hit!

			currentIndex = rVOKDNodes_d.parentIndex[currentIndex];
			bitstack = bitstack >> 1;
			btest = bitstack  & 1;
		}
		currentIndex = rVOKDNodes_d.siblingIndex[currentIndex];
		bitstack = bitstack ^ 1;

		if(currentIndex < 0)
			return -1; //If index is < 0 then no hit at root
	}



	
}

__device__ void getObstacles(RVOLine* orcaLines, int& orcaLinesCount, float timeHorizonObst, float2& agentPos, float2& agentVelocity, float radius, float agentMaxSpeed, float2& agentObstacleVector, int &obstacleVectorCount){

	
	Obst nearestObstacles[OBSTACLE_DETECT_ARRAY_SIZE];
	int nearestObstCount = 0;
	
	//Get all the obstacles within the index
	
	float rangeSq = powf(timeHorizonObst*agentMaxSpeed + radius, 2);

	queryObstacleTree(nearestObstacles, nearestObstCount,  agentPos, rangeSq, agentObstacleVector, obstacleVectorCount);

	//Add obstacles to lines
	for( int i = 0; i < nearestObstCount; i++)
	{
		addOrcaObstacle(orcaLines, orcaLinesCount,  timeHorizonObst, agentPos, agentVelocity, radius, rVObstacles_d,   nearestObstacles[i].obstIndex);
	}
}

void* getRVOObstaclePointer(){
	void* addr;
	cudaGetSymbolAddress(&addr, rVObstacles_d);
	return addr;
}

void* getRVOKDNodePointer(){
	void* addr;
	cudaGetSymbolAddress(&addr, rVOKDNodes_d);
	return addr;
}

#endif //__RVO_KERNELS__