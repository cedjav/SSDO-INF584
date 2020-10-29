#pragma once

#ifndef ZZBuffer
#define ZZBuffer

#include <iostream>
#include <fstream>
#include <vector>
#include "vec3.h"


class Z_Buffer {
public:

	Z_Buffer::Z_Buffer() {
		Vec3f vide = Vec3f(0, 0, 0);
		m_position.push_back(vide);
		m_normal.push_back(vide);
		m_depth.push_back(0);
		m_nbcol = 1;
		m_nbrow = 1;
	}

	Z_Buffer::Z_Buffer(size_t colomns, size_t rows) :m_nbcol(colomns), m_nbrow(rows) {
		Vec3f vide = Vec3f(0, 0, 0);
		for (int i = 0; i < m_nbcol * m_nbrow; i++) {
			m_position.push_back(vide);
			m_normal.push_back(vide);
			m_depth.push_back(0);
		}
	}	 


	int get_size() {
		return m_nbcol * m_nbrow;
	}

	int get_nbcols() {
		return m_nbcol;
	}

	int get_nbrows() {
		return m_nbrow;
	}

	inline const std::vector<Vec3f>& position() const { return m_position; }

	inline std::vector<Vec3f>& position() { return m_position; }

	inline const std::vector<Vec3f>& normal() const { return m_normal; }

	inline std::vector<Vec3f>& normal() { return m_normal; }

	inline const std::vector<float>& depth() const { return m_depth; }

	inline std::vector<float>& depth() { return m_depth; }

	inline Vec3f get_position(int col, int row) {
		return m_position[row + col * m_nbrow];
	}

	inline Vec3f get_normal(int col, int row) {
		return m_normal[row + col * m_nbrow];
	}

	inline float get_depth(int col, int row) {
		return m_depth[row + col * m_nbrow];
	}

	inline void set_position(int col, int row, Vec3f color) {
		if (row < 0 || row >= m_nbrow || col < 0 || col >= m_nbcol) {
			perror("Depassement de la taille du Z_Buffer dans set_position");
			exit(1);
		}
		m_position[row + col * m_nbrow] = color;
	}

	inline void set_normal(int col, int row, Vec3f color) {
		if (row < 0 || row >= m_nbrow || col < 0 || col >= m_nbcol) {
			perror("Depassement de la taille du Z_Buffer dans set_normal");
			exit(1);
		}
		m_normal[row + col * m_nbrow] = color;
	}

	inline void set_depth(int col, int row, float depth ) {
		if (row < 0 || row >= m_nbrow || col < 0 || col >= m_nbcol) {
			perror("Depassement de la taille du Z_Buffer dans set_depth");
			exit(1);
		}
		m_depth[row + col * m_nbrow] = depth;
	}

	inline int get_mesh_index(int col, int row) {
		return m_mesh_index[row + col * m_nbrow];
	}

	inline void set_mesh_index(int col, int row, int mesh_index) {
		m_mesh_index[row + col * m_nbrow] = mesh_index;
	}


	Z_Buffer::Z_Buffer(int w, int h, scene& scene, BVH& mon_BVH) {
		std::cout << "Calcul du ZBuffer";
		m_nbcol = w;
		m_nbrow = h;
		m_depth.resize(w * h);
		m_position.resize(w * h);
		m_normal.resize(w * h);
		m_mesh_index.resize(w * h);
		camera camera = scene.get_camera();
		const Vec3<float> c = camera.position();
		Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
		Vec3<float> axey = normalize(cross(axex, camera.direction()));
		const Vec3<float> p0 = camera.position() + camera.direction();

		for (int x = 0; x < w; x++) {
			if (x % 10 == 0) std::cout << ".";
			for (int y = 0; y < h; y++) {
				// Le centre de l'écran est à c + direction. 
				Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f * x) / w) + axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * y) / h);
				Vec3<float> vecteur = p - c;
				vecteur.normalize();
				Ray ray(c, vecteur);
				float e = 1e8;
				int goodray = -1;
				// int goodray = -1; [Pas besoin de garder d'info sur le Goodray]
				std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
				for (int i = 0; i < all_rays.size();i++)
					if (all_rays[i].t < e) {
						e = all_rays[i].t;
						goodray = i;
					}
				if (goodray != -1) {
					set_depth(x, y, e);	 //Division par 4 pour avoir un meilleur rendu dans le cadre de notre image
					set_position(x, y, c + all_rays[goodray].t * vecteur);

					const auto& mesh = scene.meshes()[all_rays[goodray].mesh];
					const auto& P = mesh.vertexPositions();
					const auto& N = mesh.vertexNormals();
					const Vec3i& triangle = mesh.indexedTriangles()[all_rays[goodray].triangle];
					// Normal de Phong à remettre pour petits objets mais pas efficace pour ma scene avec des angles droits
					//Vec3f hitNormal = normalize((1.f - u - v) * N[triangle[0]] + u * N[triangle[1]] + v * N[triangle[2]]);
					Vec3f hitNormal = normalize(cross(P[triangle[1]] - P[triangle[0]], P[triangle[2]] - P[triangle[0]]));
					set_normal(x, y, hitNormal);
					set_mesh_index(x, y, all_rays[goodray].mesh);
				}
				else {
					set_depth(x, y, -1);
				}

			}
		}
		std::cout << "Calcul ZBuffer OK" << std::endl;
	}




private:
	std::vector<Vec3f> m_position;
	std::vector<Vec3f> m_normal;
	std::vector<float> m_depth;
	std::vector<int> m_mesh_index;
	size_t m_nbcol;
	size_t m_nbrow;
};



#endif