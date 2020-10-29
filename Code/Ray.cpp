#pragma once

#include "Ray.h"
#include <vector>
#include "vec3.h"


std::vector<float> Ray::rayTriangleIntersection(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2) {
	Vec3<float> e0 = p1 - p0;
	Vec3<float> e1 = p2 - p0;
	Vec3<float> n = normalize(cross(e0, e1));
	Vec3<float> q = cross(m_direction, e1);
	float a = dot(e0, q);

	if (dot(n, m_direction) >= 0 || abs(a) < epsilon)
		return { NULL };

	Vec3<float> s = (m_point - p0) / a;
	Vec3<float> r = cross(s, e0);
	float b0 = dot(s, q);
	float b1 = dot(r, m_direction);
	float b2 = 1 - b0 - b1;
	if (b0 < 0 || b1 < 0 || b2 < 0)
		return { NULL };

	float t = dot(e1, r);
	if (t >= 0)
		return { b2, b0, b1, t };

	return { NULL };
}




