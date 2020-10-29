#pragma once

#ifndef LightSourceh
#define LightSourceh
#include "Vec3.h"
#include "ray.h"



class LightSource {
public:


	LightSource(const Vec3f& center = Vec3f(1.f, 1.f, 2.f),
		const Vec3f& color = Vec3f(1.f, 1.f, 1.f),
		const float intensity = 18.0,
		const Vec3f& orientation = Vec3f(-1.f, -1.f, -1.f),
		const float size = 0.2, const Vec3f& axe1_ = Vec3f(1.f, 1.f, -1.f)) :
		m_color(color), m_intensity(intensity), m_position(center), m_size(size) {
		m_orientation = normalize(orientation);
		axe1 = normalize(axe1_);
		axe2 = normalize(cross(axe1_, m_orientation));
	};


	inline Vec3f get_position() { 

		if (m_size == 0) return m_position;
		float x = (float)((rand() % 1000) / 500.f) - 1;
		float y = (float)(rand() % 1000) / 500.f - 1;
		return m_position+x*m_size*axe1+y*m_size*axe2; 
	};

	inline const float& get_intensity() const { return m_intensity; }

	inline float ray_intersect(Ray ray) {
		Vec3f P00 = m_position - axe1 - axe2;
		Vec3f P01 = m_position - axe1 + axe2;
		Vec3f P10 = m_position + axe1 - axe2;
		Vec3f P11 = m_position + axe1 + axe2;

		std::vector<float> inter1 = ray.rayTriangleIntersection(P00, P01, P10);
		if (inter1[0] != NULL) {
			return inter1[3];
		}
		std::vector<float> inter2 = ray.rayTriangleIntersection(P11, P10, P01);
		if (inter2[0] != NULL) {
			return inter2[3];
		}
		return -1; //Pas d'intersection
	}

private:
	Vec3f m_position;
	Vec3f m_orientation;
	Vec3f axe1;
	Vec3f axe2;
	float m_size;
	Vec3f m_color;
	float m_intensity;
};


#endif