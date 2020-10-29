#pragma once

#ifndef WorleyN
#define WorleyN

#include "vec3.h"
#include <vector>


class WorleyNoise
{
public:
	
	WorleyNoise() {
		m_nbpoints = 0;
	}

	WorleyNoise(const int nbpoints, Vec3<float> min, Vec3<float> max) : m_nbpoints(nbpoints) {
		m_points.resize(m_nbpoints);
		for (int i = 0; i < m_nbpoints; i++) {
			float x = min[0] + (float)(rand() % 10000) / 10000 * (max[0] - min[0]);
			float y = min[1] + (float)(rand() % 10000) / 10000 * (max[1] - min[1]);
			float z = min[2] + (float)(rand() % 10000) / 10000 * (max[2] - min[2]);
			m_points[i] = Vec3<float>(x, y, z);
		}
	}

	WorleyNoise(const int nbpoints, Vec3<float> min, Vec3<float> max, Vec3<float> min1, Vec3<float> max1) : m_nbpoints(nbpoints) {
		m_points.resize(m_nbpoints*2);
		for (int i = 0; i < m_nbpoints; i++) {
			float x = min[0] + (float)(rand() % 10000) / 10000 * (max[0] - min[0]);
			float y = min[1] + (float)(rand() % 10000) / 10000 * (max[1] - min[1]);
			float z = min[2] + (float)(rand() % 10000) / 10000 * (max[2] - min[2]);
			m_points[i] = Vec3<float>(x, y, z);
		}

		for (int i = 0; i < m_nbpoints; i++) {
			float x = min1[0] + (float)(rand() % 10000) / 10000 * (max1[0] - min1[0]);
			float y = min1[1] + (float)(rand() % 10000) / 10000 * (max1[1] - min1[1]);
			float z = min1[2] + (float)(rand() % 10000) / 10000 * (max1[2] - min1[2]);
			m_points[i] = Vec3<float>(x, y, z);
		}
	}



	float Noise(Vec3<float> position) {
		//Brut force implementation to compute the second nearest distance
		float dist1 = 999999.f;
		float dist2 = 999999.f;
		for (int i = 0; i < m_nbpoints; i++) {
			float d = (position - m_points[i]).squaredLength();
			if (d < dist1) {
				dist2 = dist1; dist1 = d;
			}
			else if (d < dist2)
				dist2 = d;
		}
		return sqrt(dist2);
	}


private:
	int m_nbpoints;
	std::vector<Vec3<float>> m_points;

};


#endif