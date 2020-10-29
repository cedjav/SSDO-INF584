#pragma once

#ifndef Camerah
#define Camerah


#include "vec3.h"

class camera {
public:
	camera() {
		m_position = Vec3<float>(0, 0, 0);
		m_direction = Vec3<float>(0, 0, -1);
		m_h_FoV=0.5;
		m_w_FoV=0.75;
		m_woverh=1.5;
	}

	camera(Vec3<float> pos, Vec3<float> dir, float angle, float woverh): m_position(pos),m_direction(dir), m_woverh(woverh) {
		m_h_FoV = angle*3.14159/180;
		m_w_FoV = atan(tan(m_h_FoV * 0.5) * m_woverh) * 2;
	}


	const Vec3<float>& position() const { return m_position; }

	inline Vec3<float>& position() { return m_position; }

	inline const Vec3<float>& direction() const { return m_direction; }

	inline Vec3<float>& direction() { return m_direction; }

	inline const float& get_h_FoV() const { return m_h_FoV; };

	inline const float& get_w_FoV() const { return m_w_FoV; };

	inline const float& get_woverh() const {return m_woverh; }


private:
	Vec3<float> m_position;
	Vec3<float> m_direction;
	float m_h_FoV;
	float m_w_FoV;
	float m_woverh;
};
#endif