#pragma once

#ifndef Rayh
#define Rayh

#include <vector>
#include "vec3.h"
#define epsilon 1e-6

class Ray {
public:
	Ray(Vec3<float> p, Vec3<float> dir) :m_point(p), m_direction(dir) { }

	const Vec3f& point() const { return m_point; };

	Vec3f& point() { return m_point; };

	const Vec3f& direction() const { return m_direction; };

	Vec3f& direction() { return m_direction; };

	std::vector<float> rayTriangleIntersection(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2);

private:
	Vec3<float> m_point;
	Vec3f m_direction;
};


#endif