#pragma once

#ifndef Accelerate
#define Accelerate

#include "vec3.h"
#include "Ray.h"
#include "Mesh.h"
#include <vector>

class AABB {
public:

	AABB() {
		m_min = Vec3f(0.0f, 0.0f, 0.0f);
		m_max = Vec3f(0.0f, 0.0f, 0.0f);
	}

	AABB(Vec3f min, Vec3f max) {
		if ((min[0] > max[0]) || (min[1] > max[1]) || (min[2] > max[2]))
			throw(1); // A affiner
		m_min = min;
		m_max = max;
	}
	
	void print_coord() {
		std::cout << "MIN  " << m_min << " MAX   " << m_max << std::endl;
	}

	inline const Vec3f get_min() { return m_min; }

	inline const Vec3f get_max() { return m_max; }

	/*
	bool rayRectangleIntersection(const Ray ray, const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, float &alpha, float &beta,float &t) {
		Vec3<float> e0 = p1 - p0;
		Vec3<float> e1 = p2 - p0;
		Vec3<float> q = cross(ray.direction(), e1);

		float a = dot(e0, q);
		if (abs(a) < epsilon) return false;

		Vec3<float> s = (ray.point() - p0) / a;
		alpha = dot(s, q);
		if (alpha < 0 || alpha > 1) return false;

		Vec3<float> r = cross(s, e0);
		beta = dot(r, ray.direction());
		if (beta < 0 || beta >1) return false;

		t = dot(e1, r);
		if (t<0)return false;

		return true;
	}

	bool rayAABBIntersection_old(const Ray ray, Vec3f& entry, Vec3f& exit) {
		std::vector <Vec3f> P;
		P.resize(8);

		P[0] = m_min;
		P[1] = Vec3f(m_min[0], m_min[1], m_max[2]);
		P[2] = Vec3f(m_min[0], m_max[1], m_min[2]);
		P[3] = Vec3f(m_max[0], m_min[1], m_min[2]);
		P[4] = Vec3f(m_max[0], m_max[1], m_min[2]);
		P[5] = Vec3f(m_min[0], m_max[1], m_max[2]);
		P[6] = Vec3f(m_max[0], m_min[1], m_max[2]);
		P[7] = m_max;

		std::vector<float> alpha;
		std::vector<float> beta;
		std::vector<float> dist;
		std::vector<bool> face;

		alpha.resize(6);
		beta.resize(6);
		dist.resize(6);
		face.resize(6);

		face[0] = rayRectangleIntersection(ray, P[0], P[1], P[3], alpha[0], beta[0], dist[0]);
		face[1] = rayRectangleIntersection(ray, P[0], P[2], P[3], alpha[1], beta[1],dist[1]);
		face[2] = rayRectangleIntersection(ray, P[0], P[1], P[2], alpha[2], beta[2],dist[2]);
		face[3] = rayRectangleIntersection(ray, P[2], P[5], P[4], alpha[3], beta[3],dist[3]);
		face[4] = rayRectangleIntersection(ray, P[1], P[5], P[6], alpha[4], beta[4],dist[4]);
		
		int nbhit = face[0] + face[1] + face[2] + face[3] + face[4];
		if (nbhit ==0) return false;

		// On ne calcule la 6ème face qui si on a déjà un hit sur les 5 premières
		face[5] = rayRectangleIntersection(ray, P[3], P[6], P[4], alpha[5], beta[5], dist[5]);

		// On veut stocker les numéros des faces d'entrées dans i0 et i1
		int i0 = -1;
		int i1 = -1;
		for (int i = 0;i < 6;i++) 
			if (face[i]) {
				if (i0 == -1)	i0 = i;
				else i1 = i;
			}

		// On veut entry = face[i0] et sortie = face[i1]
		if (dist[i0] > dist[i1]) {
			int k = i0;
			i0=i1;
			i1 = k;
		}

		// Maintenant, il faut repasser aux faces
		if (i0 == 0) entry = P[0] + alpha[0] * (P[1] - P[0]) + beta[0] * (P[3] - P[0]);
		if (i1 == 0) exit = P[0] + alpha[0] * (P[1] - P[0]) + beta[0] * (P[3] - P[0]);

		if (i0 == 1) entry = P[0] + alpha[1] * (P[2] - P[0]) + beta[1] * (P[3] - P[0]);
		if (i1 == 1) exit = P[0] + alpha[1] * (P[2] - P[0]) + beta[1] * (P[3] - P[0]);

		if (i0 == 2) entry = P[0] + alpha[2] * (P[1] - P[0]) + beta[2] * (P[2] - P[0]);
		if (i1 == 2) exit = P[0] + alpha[2] * (P[1] - P[0]) + beta[2] * (P[2] - P[0]);

		if (i0 == 3) entry = P[2] + alpha[3] * (P[5] - P[2]) + beta[3] * (P[4] - P[2]);
		if (i1 == 3) exit = P[2] + alpha[3] * (P[5] - P[2]) + beta[3] * (P[4] - P[2]);

		if (i0 == 4) entry = P[1] + alpha[4] * (P[5] - P[1]) + beta[4] * (P[6] - P[1]);
		if (i1 == 4) exit = P[1] + alpha[4] * (P[5] - P[1]) + beta[4] * (P[6] - P[1]);

		if (i0 == 5) entry = P[3] + alpha[5] * (P[6] - P[3]) + beta[5] * (P[4] - P[3]);
		if (i1 == 5) exit = P[3] + alpha[5] * (P[6] - P[3]) + beta[5] * (P[4] - P[3]);

		std::cout << entry << std::endl;
		std::cout << exit << std::endl;

		return true;

	}


	*/


	bool rayAABBIntersection(const Ray ray, Vec3f& entry, Vec3f& exit) {
		Vec3f origine = ray.point();
		Vec3f direction = ray.direction();
		Vec3f inter1;
		Vec3f inter2;
		int nbinter = 0;
		
		// Test d'intersection sur z=cte
		if (abs(direction[2]) > epsilon) {
			float tzmin=(m_min[2]-origine[2])/direction[2];
			Vec3f Intersection_Zmin = origine + tzmin * direction;
			if (Intersection_Zmin[0] > m_min[0] && Intersection_Zmin[0] < m_max[0] && Intersection_Zmin[1]>m_min[1] && Intersection_Zmin[1] < m_max[1]) {
				nbinter++;
				inter1 = Intersection_Zmin;
			}
			float tzmax = (m_max[2] - origine[2]) / direction[2];
			Vec3f Intersection_Zmax = origine + tzmax * direction;
			if (Intersection_Zmax[0] > m_min[0] && Intersection_Zmax[0] < m_max[0] && Intersection_Zmax[1]>m_min[1] && Intersection_Zmax[1] < m_max[1]) {
				if (nbinter==0) inter1 = Intersection_Zmax;
				else inter2 = Intersection_Zmax;
				nbinter++;
			}
		}

		// Test d'intersection sur y=cte
		if (abs(direction[1]) > epsilon) {
			float tymin = (m_min[1] - origine[1]) / direction[1];
			Vec3f Intersection_Ymin = origine + tymin * direction;
			if (Intersection_Ymin[0] > m_min[0] && Intersection_Ymin[0] < m_max[0] && Intersection_Ymin[2]>m_min[2] && Intersection_Ymin[2] < m_max[2]) {
				if (nbinter == 0) inter1 = Intersection_Ymin;
				else inter2 = Intersection_Ymin;
				nbinter++;
			}
			float tymax = (m_max[1] - origine[1]) / direction[1];
			Vec3f Intersection_Ymax = origine + tymax * direction;
			if (Intersection_Ymax[0] > m_min[0] && Intersection_Ymax[0] < m_max[0] && Intersection_Ymax[2]>m_min[2] && Intersection_Ymax[2] < m_max[2]) {
				if (nbinter == 0) inter1 = Intersection_Ymax;
				else inter2 = Intersection_Ymax;
				nbinter++;
			}
		}

		// Test d'intersection sur x=cte
		if (abs(direction[0]) > epsilon) {
			float txmin = (m_min[0] - origine[0]) / direction[0];
			Vec3f Intersection_Xmin = origine + txmin * direction;
			if (Intersection_Xmin[1] > m_min[1] && Intersection_Xmin[1] < m_max[1] && Intersection_Xmin[2]>m_min[2] && Intersection_Xmin[2] < m_max[2]) {
				if (nbinter == 0) inter1 = Intersection_Xmin;
				else inter2 = Intersection_Xmin;
				nbinter++;
			}
			float txmax = (m_max[0] - origine[0]) / direction[0];
			Vec3f Intersection_Xmax = origine + txmax * direction;
			if (Intersection_Xmax[1] > m_min[1] && Intersection_Xmax[1] < m_max[1] && Intersection_Xmax[2]>m_min[2] && Intersection_Xmax[2] < m_max[2]) {
				if (nbinter == 0) inter1 = Intersection_Xmax;
				else inter2 = Intersection_Xmax;
				nbinter++;
			}
		}

		if (nbinter < 2) return false;

		if (length(origine - inter1) < length(origine - inter2)) {
			entry = inter1;
			exit = inter2;
		}
		else {
			entry = inter2;
			exit = inter1;
		}

		return true;

	}


	// Fast version : only YES or NO
	bool rayAABBIntersection(const Ray ray) {
		Vec3f origine = ray.point();
		Vec3f direction = ray.direction();
		Vec3f inter1;
		Vec3f inter2;
		//int nbinter = 0;

		// Test d'intersection sur z=cte
		if (abs(direction[2]) > epsilon) {
			float tzmin = (m_min[2] - origine[2]) / direction[2];
			Vec3f Intersection_Zmin = origine + tzmin * direction;
			if (Intersection_Zmin[0] > m_min[0] && Intersection_Zmin[0] < m_max[0] && Intersection_Zmin[1]>m_min[1] && Intersection_Zmin[1] < m_max[1])
				return true;
			float tzmax = (m_max[2] - origine[2]) / direction[2];
			Vec3f Intersection_Zmax = origine + tzmax * direction;
			if (Intersection_Zmax[0] > m_min[0] && Intersection_Zmax[0] < m_max[0] && Intersection_Zmax[1]>m_min[1] && Intersection_Zmax[1] < m_max[1]) 
				return true;
		}

		// Test d'intersection sur y=cte
		if (abs(direction[1]) > epsilon) {
			float tymin = (m_min[1] - origine[1]) / direction[1];
			Vec3f Intersection_Ymin = origine + tymin * direction;
			if (Intersection_Ymin[0] > m_min[0] && Intersection_Ymin[0] < m_max[0] && Intersection_Ymin[2]>m_min[2] && Intersection_Ymin[2] < m_max[2]) 
				return true;
			float tymax = (m_max[1] - origine[1]) / direction[1];
			Vec3f Intersection_Ymax = origine + tymax * direction;
			if (Intersection_Ymax[0] > m_min[0] && Intersection_Ymax[0] < m_max[0] && Intersection_Ymax[2]>m_min[2] && Intersection_Ymax[2] < m_max[2]) 
				return true;
		}

		// Test d'intersection sur x=cte
		if (abs(direction[0]) > epsilon) {
			float txmin = (m_min[0] - origine[0]) / direction[0];
			Vec3f Intersection_Xmin = origine + txmin * direction;
			if (Intersection_Xmin[1] > m_min[1] && Intersection_Xmin[1] < m_max[1] && Intersection_Xmin[2]>m_min[2] && Intersection_Xmin[2] < m_max[2]) 
				return true;
			float txmax = (m_max[0] - origine[0]) / direction[0];
			Vec3f Intersection_Xmax = origine + txmax * direction;
			if (Intersection_Xmax[1] > m_min[1] && Intersection_Xmax[1] < m_max[1] && Intersection_Xmax[2]>m_min[2] && Intersection_Xmax[2] < m_max[2]) 
				return true;
		}
		return false;
	}

	/*
	bool rayAABBIntersection_old(const Ray ray) {
		std::vector <Vec3f> P;
		P.resize(8);

		P[0] = m_min;
		P[1] = Vec3f(m_min[0], m_min[1], m_max[2]);
		P[2] = Vec3f(m_min[0], m_max[1], m_min[2]);
		P[3] = Vec3f(m_max[0], m_min[1], m_min[2]);
		P[4] = Vec3f(m_max[0], m_max[1], m_min[2]);
		P[5] = Vec3f(m_min[0], m_max[1], m_max[2]);
		P[6] = Vec3f(m_max[0], m_min[1], m_max[2]);
		P[7] = m_max;

		std::vector<float> alpha;
		std::vector<float> beta;
		std::vector<float> dist;
		std::vector<bool> face;

		alpha.resize(6);
		beta.resize(6);
		dist.resize(6);
		face.resize(6);

		face[0] = rayRectangleIntersection(ray, P[0], P[1], P[3], alpha[0], beta[0], dist[0]);
		face[1] = rayRectangleIntersection(ray, P[0], P[2], P[3], alpha[1], beta[1], dist[1]);
		face[2] = rayRectangleIntersection(ray, P[0], P[1], P[2], alpha[2], beta[2], dist[2]);
		face[3] = rayRectangleIntersection(ray, P[2], P[5], P[4], alpha[3], beta[3], dist[3]);
		face[4] = rayRectangleIntersection(ray, P[1], P[5], P[6], alpha[4], beta[4], dist[4]);
		face[5] = rayRectangleIntersection(ray, P[3], P[6], P[4], alpha[5], beta[5], dist[5]);

		int nbhit = face[0] + face[1] + face[2] + face[3] + face[4] + face[5];
		if (nbhit < 2) return false;
		return true;
	}
		   */

private:
	Vec3f m_min;
	Vec3f m_max;
};



#endif