#ifndef __COMMON_MATH__
#define __COMMON_MATH__



inline float absSq(const Vector2 &vec )
{
	return glm::dot(vec, vec);
}

inline float distSqPointLineSegment(const Vector2 &a,const Vector2 &b, const Vector2 &c )
{
	const float r = glm::dot((c - a), (b - a)) / absSq(b - a);

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

inline Vector2 rotate( Vector2 vec, float angle )
{
	return Vector2( vec.x * cosf(angle) - vec.y * sinf(angle),
		vec.y*cosf(angle) + vec.x *sinf(angle));
}

inline int get_line_intersection( float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y )
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

inline int get_line_intersection( Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, Vector2& output )
{
	return get_line_intersection(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, &output.x, &output.y);
}



#endif