#ifndef __COMMON_CUDA_MATHS__
#define __COMMON_CUDA_MATHS__

#include "cudahelpers/helper_math.h"

inline  __device__ float abs(float2 &vector)
{
	return sqrtf(dot(vector,vector));
}




inline __host__ __device__ float  sqr(float num)
{
	return powf(num,2);

}

inline __host__ __device__ float absSq(const float2 &vec)
{
	return dot(vec,vec);
}

inline __host__ __device__ float distSqr( float x, float y, const float2& dest )
{
	return powf(x - dest.x,2) + powf(y - dest.y, 2);
}

inline __host__ __device__ float  det(const float2 &vector1, const float2 &vector2)
{
	return vector1.x * vector2.y - vector1.y * vector2.x;
}

inline float  operator *(float2 &vec1, float2 &vec2)
{
	return dot(vec1, vec2);
}

/**
* \brief      Computes the squared distance from a line segment with the
*             specified endpoints to a specified point.
* \param      a               The first endpoint of the line segment.
* \param      b               The second endpoint of the line segment.
* \param      c               The point to which the squared distance is to
*                             be calculated.
* \return     The squared distance from the line segment to the point.
*/
inline __host__ __device__  float distSqPointLineSegment(const float2 &a,const  float2 &b, const float2 &c)
{
	const float r = dot((c - a), (b - a)) / absSq(b - a);

	if (r < 0.0f) {
		return absSq(c - a);
	}
	else if (r > 1.0f) {
		return absSq(c - b);
	}
	else {
		return absSq(c - (a + r * (b - a)));
	}
}

/**
* \brief      Computes the signed distance from a line connecting the
*             specified points to a specified point.
* \param      a               The first point on the line.
* \param      b               The second point on the line.
* \param      c               The point to which the signed distance is to
*                             be calculated.
* \return     Positive when the point c lies to the left of the line ab.
*/
inline __host__ __device__ float leftOf(const float2 &a,const  float2 &b,const float2 &c)
{
	return det(a - c, b - a);
}



/************************************************************************
* Gets the point on the line nearest to P, when segmentClamp is false it tests for the whole plane
************************************************************************/
inline __host__ __device__ float2 getClosestPoint(const float2& A,const float2& B,const  float2& P, bool segmentClamp)
{
	float2 AP = P - A;
	float2 AB = B - A;
	float ab2 = AB.x*AB.x + AB.y*AB.y;
	float ap_ab = AP.x*AB.x + AP.y*AB.y;
	float t = ap_ab / ab2;
	if (segmentClamp)
	{
		if (t < 0.0f) t = 0.0f;
		else if (t > 1.0f) t = 1.0f;
	}

	return  A + AB * t;	 
}


inline __host__ __device__  bool notEqual( float2 p1, float2 p2 )
{
	return (p1.x != p2.x && p1.y != p2.y );
}

inline __host__ __device__ int get_xline_intersection( float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y )
{
	float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
	s10_x = p1_x - p0_x;
	s10_y = p1_y - p0_y;
	s32_x = p3_x - p2_x;
	s32_y = p3_y - p2_y;

	denom = s10_x * s32_y - s32_x * s10_y;
	if (denom == 0)
		return 0; // Collinear
	bool denomPositive = denom > 0;

	s02_x = p0_x - p2_x;
	s02_y = p0_y - p2_y;
	s_numer = s10_x * s02_y - s10_y * s02_x;
	if ((s_numer < 0) == denomPositive)
		return 0; // No collision

	t_numer = s32_x * s02_y - s32_y * s02_x;
	if ((t_numer < 0) == denomPositive)
		return 0; // No collision

	if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
		return 0; // No collision
	// Collision detected
	t = t_numer / denom;
	if (i_x != NULL)
		*i_x = p0_x + (t * s10_x);
	if (i_y != NULL)
		*i_y = p0_y + (t * s10_y);

	return 1;
}

inline __host__ __device__  int get_line_intersection( float2 p1, float2 p2,  float2 p3, float2 p4, float2& output )
{
	return get_xline_intersection(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, &output.x, &output.y);
}




inline __host__ __device__  float2 rotate( float2 vec, float angle )
{
	return make_float2( vec.x * cosf(angle) - vec.y * sinf(angle),
		vec.y*cosf(angle) + vec.x *sinf(angle));
}

inline __host__ __device__ bool inlineBounds( float2& p1, float2& p2, float radius, float2& pos )
{
	bool inBounds = true;
	float2 pVec = p2 - p1;
	float2 pVecPerp = normalize(make_float2(-pVec.y, pVec.x))*radius;
	inBounds = leftOf(p2 + pVecPerp, p1 + pVecPerp, pos) >= 0 ? inBounds : false;
	inBounds = leftOf(p1 - pVecPerp, p2 - pVecPerp, pos) >= 0 ? inBounds : false;
	inBounds = leftOf( p1 + pVecPerp ,p1 - pVecPerp, pos) >= 0 ? inBounds : false;
	inBounds = leftOf( p2 - pVecPerp,p2 + pVecPerp,  pos) >= 0 ? inBounds : false;
	return inBounds;
}

inline __host__ __device__ bool rectCircleIntersect(const float2& circlePos,const float radius,const float2& r1,const float2& r2,const float2& r3,const float2& r4 )
{
	//Circle encloses a rectange if:
	// Circle middle is inside rectangle
	//	Circle radius is with the closest point

	float l1 = leftOf(r1, r2, circlePos);
	float l2 = leftOf(r2, r3, circlePos);
	float l3 = leftOf(r3, r4, circlePos);
	float l4 = leftOf(r4, r1, circlePos);

	if(l1 <=0 && l2 <=0 && l3 <=0 && l4 <= 0)
	{
		//Mid point inside 
		return true; 
	}
	else
	{
		float ld1 = distSqPointLineSegment(r1, r2, circlePos);
		float ld2 = distSqPointLineSegment(r2, r3, circlePos);
		float ld3 = distSqPointLineSegment(r3, r4, circlePos);
		float ld4 = distSqPointLineSegment(r4, r1, circlePos);
		float radSqr = radius*radius;

		if(ld1 <= radSqr || ld2 <= radSqr || ld3 <= radSqr || ld4 <= radSqr)
			return true; //Radius within the nearest point on line
	}


	return false;

}


inline __host__ __device__	bool rectLineCircleIntersect( float2& circlePos, float radius, float2& p1, float2& p2 , float rRadius)
{
	float2 pVec = p2 - p1;
	float2 pVecPerp = normalize(make_float2(-pVec.y, pVec.x))*rRadius;
	return rectCircleIntersect(circlePos, radius, p1 + pVecPerp, p2 + pVecPerp, p2 - pVecPerp, p1 - pVecPerp);
}


inline __host__ __device__ bool circleCircleIntersect( float2& c1, float c1radius, float2& c2, float c2radius )
{
	return distSqr(c1.x, c1.y, c2) <= powf(c1radius+ c2radius, 2.0f);
}




#endif