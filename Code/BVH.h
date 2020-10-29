#pragma once

#ifndef BVHfile
#define BVHfile

#include "vec3.h"
#include "Accelerating.h"
#include "Mesh.h"
#include "Ray.h"
#include <vector>

struct rep_intersection {
	float u;
	float v;
	float t;
	int mesh;
	int triangle;
};

typedef struct rep_intersection rep_intersection;



class BVH {
public:

	float valeur_min(float a, float b, float c) {
		if (a < b) {
			if (a < c) return a;
			else return c;
		}
		if (b < c) return  b;
		return c;
	}

	BVH() : m_left(NULL), m_right(NULL) {}

	/*
	// Constructeur à n'appeler qu'une seule fois : il fabrique un vecteur des 
	BVH(mesh& mesh) {
		mesh.compute_AABB();
		mesh.compute_barycentre();
		std::vector<int> data;
		for (int i = 0; i < mesh.get_nb_f(); i++) 
			data.push_back(i);

		BVH* reponse= BVHbuild(mesh, data);

		m_right = reponse->m_right;
		m_left = reponse->m_left;
		m_numtr = reponse->m_numtr;
		m_nb_prim = reponse->m_nb_prim;
		m_AABB = mesh.get_AABB(); // On utilise la bounding box du mesh et pas la réunion des deux du dessous
		// C'est peut-être pareil mais je ne suis pas sûr, alors...
	}
	*/

	BVH(scene& scene) {
		std::vector<int> data_mesh;
		std::vector<int> data_triangle;

		for (int i = 0; i < scene.meshes().size(); i++) {
			mesh &current_mesh = scene.meshes()[i];
			current_mesh.compute_barycentre();

			for (int j = 0; j < current_mesh.get_nb_f(); j++) {
				data_mesh.push_back(i);
				data_triangle.push_back(j);
			}
		}
		BVH* reponse = BVHbuildscene(scene, data_mesh, data_triangle);

		m_right = reponse->m_right;
		m_left = reponse->m_left;
		m_numtr = reponse->m_numtr;
		m_nummesh = reponse->m_nummesh;
		m_nb_prim = reponse->m_nb_prim;
		m_AABB = reponse->m_AABB;
	}


	BVH* BVHbuildscene(scene& scene, std::vector<int> data_mesh, std::vector<int> data_triangle) {
		BVH* reponse = new BVH;

		int taille = data_mesh.size();
		reponse->m_nb_prim = taille;
		// Si on a atteint une feuille
		if (taille == 1) {
			reponse->m_numtr = data_triangle[0];
			reponse->m_nummesh = data_mesh[0];
			reponse->m_left = NULL;
			reponse->m_right = NULL;
			Vec3<Vec3<float>> points = scene.meshes()[data_mesh[0]].get_triangle_coord(data_triangle[0]);
			float xmin = valeur_min(points[0][0], points[1][0], points[2][0]);
			float ymin = valeur_min(points[0][1], points[1][1], points[2][1]);
			float zmin = valeur_min(points[0][2], points[1][2], points[2][2]);

			float xmax = -valeur_min(-points[0][0], -points[1][0], -points[2][0]);
			float ymax = -valeur_min(-points[0][1], -points[1][1], -points[2][1]);
			float zmax = -valeur_min(-points[0][2], -points[1][2], -points[2][2]);

			reponse->m_AABB = AABB(Vec3f(xmin, ymin, zmin), Vec3f(xmax, ymax, zmax));
			//std::cout << "Feuille " << xmin << " " << ymin << " " << zmin << "          " << xmax << " " << ymax << " " << zmax << std::endl;
		}
		else {
			int axe = -1;
			Vec3f min = Vec3f(1e6f, 1e6f, 1e6f);
			Vec3f max = -min;

			// On commence par parcourir toutes les primitives pour trouver la bounding box
			for (int i = 0; i < data_mesh.size(); i++) {
				Vec3<Vec3<float>> points = scene.meshes()[data_mesh[i]].get_triangle_coord(data_triangle[i]);
				for (int j = 0; j < 3; j++) {
					if (points[j][0] < min[0]) min[0] = points[j][0];
					if (points[j][1] < min[1]) min[1] = points[j][1];
					if (points[j][2] < min[2]) min[2] = points[j][2];
					if (points[j][0] > max[0]) max[0] = points[j][0];
					if (points[j][1] > max[1]) max[1] = points[j][1];
					if (points[j][2] > max[2]) max[2] = points[j][2];
				}
			}
			reponse->m_AABB = AABB(min, max);

			// On va couper selon l'axe le plus grand et définir deux BVH filles
			float taillex = max[0] - min[0];
			float tailley = max[1] - min[1];
			float taillez = max[2] - min[2];
			if (taillex > tailley) {
				if (taillex > taillez) axe = 0;
				else axe = 2;
			}
			else if (taillez > tailley) axe = 2;
			else axe = 1;

			// Trier les primitives du mesh
			// Brut force et on fera un Quick Sort si on a le temps plus tard

			// Remplacer par sdt::sort

			for (int i = 0;i < data_mesh.size() - 1; i++) {
				//std::cout << i << " ";
				for (int j = 1; j < data_mesh.size(); j++) {
					//mesh& current_meshi = scene.meshes()[data_mesh[i]];
					//Vec3f pointi = current_meshi.barycentrePositions()[data_triangle[i]];
					//mesh& current_meshj = scene.meshes()[data_mesh[j]];
					//Vec3f pointj = current_meshj.barycentrePositions()[data_triangle[j]];

					if (scene.meshes()[data_mesh[i]].barycentrePositions()[data_triangle[i]][axe] > scene.meshes()[data_mesh[j]].barycentrePositions()[data_triangle[j]][axe]) {
					//if (pointi[axe]>pointj[axe]) {
						int k = data_mesh[i];
						data_mesh[i] = data_mesh[j];
						data_mesh[j] = k;

						k = data_triangle[i];
						data_triangle[i] = data_triangle[j];
						data_triangle[j] = k;
					}
				}
			}

			//for (int i = 0;i < data_mesh.size() - 1; i++) {
			//	std::cout << scene.meshes()[data_mesh[i]].barycentrePositions()[data_triangle[i]][axe] << std::endl;
			//}



			// Repérer la médiane
			int mediane = data_triangle.size() / 2;

			// Créer deux vectors de primitives disjoins

			std::vector<int> prim_right_mesh;
			std::vector<int> prim_left_mesh;
			std::vector<int> prim_right_triangle;
			std::vector<int> prim_left_triangle;

			for (int i = 0;i < mediane;i++) {
				prim_right_mesh.push_back(data_mesh[i]);
				prim_right_triangle.push_back(data_triangle[i]);
			}

			for (int i = mediane;i < data_triangle.size();i++) {
				prim_left_mesh.push_back(data_mesh[i]);
				prim_left_triangle.push_back(data_triangle[i]);
			}
			
			//std::cout << "Taille des datas : " << data_triangle.size() << " - Coupure sur axe " << axe << "  Taille en dessous : " << prim_left_mesh.size() << "  " << prim_right_mesh.size() << "  Min " << min << "  et max " << max << std::endl;

			// Appeler récursivement

			reponse->m_left = BVHbuildscene(scene, prim_left_mesh, prim_left_triangle);
			reponse->m_right = BVHbuildscene(scene, prim_right_mesh, prim_right_triangle);
			reponse->m_numtr = -1;
			reponse->m_nummesh = -1;
		}
		return reponse;
	}



	BVH* BVHbuild(mesh& mesh, std::vector<int> data) {
		BVH* reponse = new BVH;
		// shared_ptr<BVH> myBVH = make_shared<BVH> ();
		int taille = data.size();
		reponse->m_nb_prim = taille;
		// Si on a atteint une feuille
		if (taille == 1) {
			reponse->m_numtr = data[0];
			reponse->m_left = NULL;
			reponse->m_right = NULL;
			Vec3<Vec3<float>> points = mesh.get_triangle_coord(data[0]);
			float xmin = valeur_min(points[0][0], points[1][0], points[2][0]);
			float ymin = valeur_min(points[0][1], points[1][1], points[2][1]);
			float zmin = valeur_min(points[0][2], points[1][2], points[2][2]);

			float xmax = -valeur_min(-points[0][0], -points[1][0], -points[2][0]);
			float ymax = -valeur_min(-points[0][1], -points[1][1], -points[2][1]);
			float zmax = -valeur_min(-points[0][2], -points[1][2], -points[2][2]);

			reponse->m_AABB = AABB(Vec3f(xmin, ymin, zmin), Vec3f(xmax, ymax, zmax));
			std::cout << "Feuille " << xmin << " " << ymin << " " << zmin << "          " << xmax << " " << ymax << " " << zmax << std::endl;
		}
		else {
			int axe = -1;
			Vec3f min = Vec3f(1e6f, 1e6f, 1e6f);
			Vec3f max = -min;

			// On commence par parcourir toutes les primitives pour trouver la bounding box
			for (int i = 0; i < data.size(); i++) {
				Vec3<Vec3<float>> points = mesh.get_triangle_coord(data[i]);
				for (int j = 0; j < 3; j++) {
					if (points[j][0] < min[0]) min[0] = points[j][0];
					if (points[j][1] < min[1]) min[1] = points[j][1];
					if (points[j][2] < min[2]) min[2] = points[j][2];
					if (points[j][0] > max[0]) max[0] = points[j][0];
					if (points[j][1] > max[1]) max[1] = points[j][1];
					if (points[j][2] > max[2]) max[2] = points[j][2];
				}
			}
			reponse->m_AABB = AABB(min, max);

			// On va couper selon l'axe le plus grand et définir deux BVH filles
			float taillex = max[0] - min[0];
			float tailley = max[1] - min[1];
			float taillez = max[2] - min[2];
			if (taillex > tailley) {
				if (taillex > taillez) axe = 0;
				else axe = 2;
			}
			else if (taillez > tailley) axe = 2;
			else axe = 1;

			// Trier les primitives du mesh
			// Brut force et on fera un Quick Sort si on a le temps plus tard

			// Remplacer par sdt::sort

			for (int i = 0;i < data.size()-1; i++) {
				for (int j = 1; j < data.size(); j++) {
					if (mesh.barycentrePositions()[data[i]][axe] > mesh.barycentrePositions()[data[j]][axe]) {
						int k = data[i];
						data[i] = data[j];
						data[j] = k;
					}
				}
			}

			// Repérer la médiane
			int mediane = data.size() / 2;

			// Créer deux vectors de primitives disjoins

			std::vector<int> prim_right;
			std::vector<int> prim_left;

			for (int i = 0;i < mediane;i++)
				prim_right.push_back(data[i]);

			for (int i = mediane;i < data.size();i++)
				prim_left.push_back(data[i]);
			//std::cout << "Taille des datas : " << data.size() << " - Coupure sur axe " << axe << "  Taille en dessous : " << prim_left.size() << "  " << prim_right.size() << "  Min "<<min<<"  et max "<<max<<std::endl;
			
			// Appeler les constructeurs

			reponse->m_left = BVHbuild(mesh, prim_left);
			reponse->m_right = BVHbuild(mesh, prim_right);
			reponse->m_numtr = -1;
		}
		return reponse;
	}


	inline std::vector<rep_intersection> rayintersect(Ray &ray,scene &scene) {
		bool test = m_AABB.rayAABBIntersection(ray);
		if (!test) return {};

		if (m_nb_prim == 1) {
			Vec3<Vec3<float>> &points = scene.meshes()[m_nummesh].get_triangle_coord(m_numtr);
			const Vec3f& p0 = points[0];
			const Vec3f& p1 = points[1];
			const Vec3f& p2 = points[2];
			std::vector<float> xx = ray.rayTriangleIntersection(p0, p1,p2);
			std::vector<rep_intersection> reponse;
			if (xx[0] == NULL) {
				//reponse.resize(0);
				//return (reponse);
				return {};
			}
			else {
				reponse.resize(1);
				reponse[0].u = xx[1];
				reponse[0].v = xx[2];
				reponse[0].t = xx[3];
				reponse[0].mesh = m_nummesh;
				reponse[0].triangle = m_numtr;

				//std::cout << xx[1]<<" "<<xx[2]<<" "<<xx[3]<<" "<< m_nummesh <<" "<< m_numtr << std::endl;
				return reponse;

			}
		}



		std::vector<rep_intersection> Rep_Right= (*m_right).rayintersect(ray, scene);
		//std::cout << Rep_Right.size();
		std::vector<rep_intersection> Rep_Left = (*m_left).rayintersect(ray, scene);
		//std::cout << Rep_Left.size();
		//std::cout << "Taille BVH " << m_nb_prim << " Right : " << Rep_Right.size() << " Left" << Rep_Left.size() << std::endl;
		 
		for (int i = 0;i < Rep_Left.size();i++)
			Rep_Right.push_back(Rep_Left[i]);
		
		return Rep_Right;
	}


	// Stratégie : essayer de perdre le plus vite possibleavec un true (sinon, on doit continuer à tout tester)
	inline bool does_intersect(Ray& ray, scene& scene) {
		bool test = m_AABB.rayAABBIntersection(ray);
		if (!test) return false;

		if (m_nb_prim == 1) {
			Vec3<Vec3<float>>& points = scene.meshes()[m_nummesh].get_triangle_coord(m_numtr);
			const Vec3f& p0 = points[0];
			const Vec3f& p1 = points[1];
			const Vec3f& p2 = points[2];
			std::vector<float> xx = ray.rayTriangleIntersection(p0, p1, p2);
			if (xx[0] == NULL) return false;
		   	else return true;
		}

		if ((*m_right).does_intersect(ray, scene)) return true;
		if ((*m_left).does_intersect(ray, scene)) return true;
		return false;
	}



private:
	BVH *m_right;
	BVH *m_left;
	int m_numtr;
	int m_nummesh;
	int m_nb_prim;
	AABB m_AABB;

};

#endif
