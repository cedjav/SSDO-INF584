#pragma once

#ifndef Meshh
#define Meshh

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include "vec3.h"
#include "Lightsource.h"
#include "Material.h"
#include "camera.h"
#include "Accelerating.h"
//#include "BVH.h"


class mesh {
public:
	mesh() {
		m_vertex.resize(0);
		m_normales.resize(0);
		m_triangle.resize(0);
		m_nb_vertex = 0;
		m_nb_triangle = 0;
	}

	void add_triangle(Vec3<float> v1, Vec3<float> v2, Vec3<float> v3) {
		int t1(-1), t2(-1), t3(-1);
		for (int i = 0; i < m_vertex.size(); i++)
			if (m_vertex[i] == v1) { t1 = i; break; }
		if (t1 == -1) {
			t1 = m_vertex.size(); m_vertex.push_back(v1); m_nb_vertex++;
		}

		for (int i = 0; i < m_vertex.size(); i++)
			if (m_vertex[i] == v2) { t2 = i; break; }
		if (t2 == -1) { 
			t2 = m_vertex.size(); m_vertex.push_back(v2);m_nb_vertex++;
		}

		for (int i = 0; i < m_vertex.size(); i++)
			if (m_vertex[i] == v3) { t3 = i; break; }
		if (t3 == -1) { 
			t3 = m_vertex.size(); m_vertex.push_back(v3);m_nb_vertex++;
		}

		Vec3<int> new_triangle(t1, t2, t3);
		m_triangle.push_back(new_triangle);
		m_nb_triangle++;
	}

	void add_vertex(float x, float y, float z) {
		Vec3<float> new_vertex{ x,y,z };
		m_vertex.push_back(new_vertex);
		m_nb_vertex++;
	}		 

	void load_off(char* filename) {
		std::ifstream myfile(filename);
		if (myfile.is_open()) {
			int nb_v{ 0 }, nb_f{ 0 }, nothing;
			std::string off;
			myfile >> off;
			myfile >> nb_v >> nb_f >> nothing;
			for (int i = 0;i < nb_v;i++) {
				float x, y, z;
				myfile >> x >> y >> z;
				add_vertex(x, y, z);
			}
			for (int i = 0;i < nb_f;i++) {
				int x, y, z;
				myfile >> nothing >> x >> y >> z;
				if (nothing == 3) {
					Vec3<int> new_triangle(x, y, z);
					m_triangle.push_back(new_triangle);
					m_nb_triangle++;
				}
			}
			myfile.close();
			recomputenormales();
		}
	}

	void recomputenormales() {
		m_normales.resize(0);
		for (int i = 0; i < m_nb_vertex;i++)
			m_normales.push_back(Vec3f(0.0f, 0.0f, 0.0f));
		for (int i = 0; i < m_nb_triangle; i++) {
			int t0 = m_triangle[i][0];
			int t1 = m_triangle[i][1];
			int t2 = m_triangle[i][2];
			Vec3f vecteur1 = m_vertex[t1] - m_vertex[t0];
			Vec3f vecteur2 = m_vertex[t2] - m_vertex[t0];
			Vec3f normaleface = cross(vecteur1, vecteur2);
			for (int j = 0;j < 3;j++)
				m_normales[m_triangle[i][j]]+= normaleface;
		}
		for (int i = 0; i < m_nb_vertex;i++)
			m_normales[i].normalize();

	}

	inline int get_nb_v() { return m_nb_vertex; }
		 
	inline int get_nb_f() { return m_nb_triangle; }

	Vec3<Vec3<float>> get_triangle_coord(int num) {
		int t0 = m_triangle[num][0];
		int t1 = m_triangle[num][1];
		int t2 = m_triangle[num][2];

		Vec3<float>	v0 = m_vertex[t0];
		Vec3<float>	v1 = m_vertex[t1];
		Vec3<float>	v2 = m_vertex[t2];

		return { v0,v1,v2 };
	}

	void compute_AABB() {
		Vec3f min = Vec3f(1e6f, 1e6f, 1e6f);
		Vec3f max = -min;

		for (int i = 0;i < m_nb_vertex;i++) {
			if (m_vertex[i][0] < min[0]) min[0] = m_vertex[i][0];
			if (m_vertex[i][1] < min[1]) min[1] = m_vertex[i][1];
			if (m_vertex[i][2] < min[2]) min[2] = m_vertex[i][2];
			if (m_vertex[i][0] > max[0]) max[0] = m_vertex[i][0];
			if (m_vertex[i][1] > max[1]) max[1] = m_vertex[i][1];
			if (m_vertex[i][2] > max[2]) max[2] = m_vertex[i][2];
		}
		//std::cout << "MIN  "<<min << "MAX  " << max << std::endl;
		m_AABB = AABB(min, max);
	}

	inline const std::vector<Vec3f>& vertexPositions() const { return m_vertex; }

	inline std::vector<Vec3f>& vertexPositions() { return m_vertex; }

	inline const std::vector<Vec3f>& vertexNormals() const { return m_normales; }

	inline std::vector<Vec3f>& vertexNormals() { return m_normales; }

	inline const std::vector<Vec3i>& indexedTriangles() const { return m_triangle; }

	inline std::vector<Vec3i>& indexedTriangles() { return m_triangle; }

	inline const AABB get_AABB() { return m_AABB; }

	void compute_barycentre() {
		m_barycentres_faces.resize(m_nb_triangle);
		for (int i = 0; i < m_nb_triangle; i++) {
			int t0 = m_triangle[i][0];
			int t1 = m_triangle[i][1];
			int t2 = m_triangle[i][2];
			//Vec3f bar= (m_vertex[t0] + m_vertex[t1] + m_vertex[t2]);
			//bar = bar / 3;


			m_barycentres_faces[i]= (m_vertex[t0] + m_vertex[t1] + m_vertex[t2])/3;
		}
	}

	inline const std::vector<Vec3f>& barycentrePositions() const { return m_barycentres_faces; }

	inline const Material& material() const { return m_material; }

	inline Material& material() { return m_material; }

private:

	Vec3f triangleNormal(size_t i) const {
		return normalize(cross(m_vertex[m_triangle[i][1]] - m_vertex[m_triangle[i][0]],
				m_vertex[m_triangle[i][2]] - m_vertex[m_triangle[i][0]]));
	}

	std::vector<Vec3f> m_vertex;
	std::vector<Vec3f> m_normales;
	std::vector<Vec3i> m_triangle;
	std::vector<Vec3f> m_barycentres_faces;
	int m_nb_vertex;
	int m_nb_triangle;
	AABB m_AABB;
	Material m_material;
};

#endif