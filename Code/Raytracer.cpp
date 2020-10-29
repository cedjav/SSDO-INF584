#pragma once

#include "vec3.h"
#include "image.h"
#include "Camera.h"
#include "scene.h"
#include "Ray.h"
#include "BVH.h"
#include "ZBuffer.h"

float valeur_max(float a, float b, float c) {
	if (a > b) {
		if (a > c) return a;
		else return c;
	}
	if (b > c) return  b;
	return c;
}

inline Vec3f shade2(const scene& scene, const size_t& meshIndex, const size_t& triangleIndex, 
	const float& u, const float& v, const Ray& ray,const Vec3f &light_position,const float distance_deja_parcourue, float intensity) {
	const auto& mesh = scene.meshes()[meshIndex];
	const auto& P = mesh.vertexPositions();
	const auto& N = mesh.vertexNormals();
	const Vec3i& triangle = mesh.indexedTriangles()[triangleIndex];

	// Normal de Phong à remettre pour petits objets
	//Vec3f hitNormal = normalize((1.f - u - v) * N[triangle[0]] + u * N[triangle[1]] + v * N[triangle[2]]);
	Vec3f hitNormal = cross(P[triangle[1]] - P[triangle[0]], P[triangle[2]] - P[triangle[0]]);
	Vec3f intersection = (1.f - u - v) * P[triangle[0]] + u * P[triangle[1]] + v * P[triangle[2]];

	Vec3f wi = normalize(light_position - intersection);
	Vec3f wo = normalize(scene.get_camera().position() - intersection);

	Material material = mesh.material();
	Vec3f reponse = material.evaluationColorResponse(hitNormal,wi,wo);

	float distance = length(light_position - intersection)+ distance_deja_parcourue;
	return intensity * reponse / (distance * distance);
}

inline bool isvisibleBVH(Ray& ray, scene& scene, BVH & mon_BVH,float distance_to_light) {
	std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
	// On pourrait faire mieux en modifiant rayintersect pour une routine qui répond False dès qu'elle trouve une intersection
	for (int i = 0; i < all_rays.size(); i++) {
		if (all_rays[i].t< distance_to_light)
			return false;
	}
	return true;
}

// Avec la BVH - 1er mars 2020
void rayTrace5BVH(Image& im, scene& scene, BVH& mon_BVH, int Ktotal) {	// Take the bakground image and the scene as input

	int w = im.get_nbcols();
	int h = im.get_nbrows();
	//int nbmesh = scene.get_n_meshes();
	Vec3f Black = Vec3f(0, 0, 0);

	camera camera = scene.get_camera();
	const Vec3<float> c = camera.position();
	const Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
	const Vec3<float> axey = normalize(cross(axex, camera.direction()));
	const Vec3<float> p0 = camera.position() + camera.direction();

	for (int x = 0; x < w; x++) {

		if (x % 10 == 0) std::cout << ".";
		//std::cout << x << " ";
#pragma omp parallel for
		for (int y = 0; y < h; y++) {
			//std::cout << y << " ";

			Vec3f couleur_total = Vec3f(0, 0, 0);
			Vec3f couleur = Vec3f(0, 0, 0);

			for (int k = 0;k < Ktotal;k++) {

				int ligh_number = (rand() % scene.get_n_lightsources());
				const Vec3<float> light = scene.lightsource()[ligh_number].get_position();
				float light_intensity = scene.lightsource()[ligh_number].get_intensity();

				float xplus = (float)(rand() % 100) / 100 - 0.50; //entre -0.5 et +.5
				float yplus = (float)(rand() % 100) / 100 - 0.50; //entre -0.5 et +.5

				// Le centre de l'écran est à c + direction. 
				const Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f * (x + xplus)) / w)
					+ axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * (y + yplus)) / h);
				const Vec3<float> vecteur = normalize(p - c);
				Ray ray(c, vecteur);
				float e = 1e8;
				int goodray = -1;
				std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
				for (int i = 0; i < all_rays.size();i++) {
					float d = all_rays[i].t;
					if (d < e) {
						e=d;
						goodray = i;
					}
				}
				// Il faut maintenant tester si la lumière est visible depuis l'intersection
				if (goodray != -1) {
					const Vec3f intersection = c + all_rays[goodray].t * vecteur;

					Vec3f direction_to_light = light - intersection;
					const float distance_lo_light = length(direction_to_light);
					direction_to_light = direction_to_light / distance_lo_light;

					Ray raytolight = Ray(intersection + 0.0001f * direction_to_light, direction_to_light);
					if (isvisibleBVH(raytolight, scene, mon_BVH, distance_lo_light - 0.0001f))
						couleur_total += shade2(scene, all_rays[goodray].mesh,
							all_rays[goodray].triangle, all_rays[goodray].u, all_rays[goodray].v, ray, light, 0, light_intensity);
				}
			}
			couleur_total /= Ktotal;
			im.set_pixel(x, y, couleur_total);

		}
	}
}

//Attention direction doit être normalisé
Vec3f color_response(const Vec3f &origine, const Vec3f &direction, 
	const Vec3f &light, const int &depth,const float &distance_deja_parcourue, scene &scene, BVH &mon_BVH, float light_intensity) {

	Vec3f couleur1 = Vec3f(0., 0., 0.);
	Vec3f couleur2 = Vec3f(0., 0., 0.);
	Vec3i triangles;

	Ray ray(origine, direction);	   
	float e = 1e8;
	int goodray = -1;
	std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);

	for (int i = 0; i < all_rays.size();i++) {
		float d = all_rays[i].t;
		if (d < e) {
			e = d;
			goodray = i;
		}
	}
	//std::cout << "...et Goodray=" << goodray;
	if (goodray != -1) {		 // Mon rayon a intersecté la scene ; il faut relancer
		const Vec3f intersection = origine + all_rays[goodray].t * direction;

		Vec3f direction_to_light = light - intersection;
		const float distance_lo_light = length(direction_to_light);
		direction_to_light = direction_to_light / distance_lo_light;
		Ray raytolight = Ray(intersection + 0.0001f * direction_to_light, direction_to_light);

		// Direct lighting
		if (isvisibleBVH(raytolight, scene, mon_BVH, distance_lo_light - 0.001f));
			couleur1 = shade2(scene, all_rays[goodray].mesh, all_rays[goodray].triangle, 
				all_rays[goodray].u, all_rays[goodray].v, ray,light, distance_deja_parcourue, light_intensity);

		if (depth < 3) {		// Additionnal indirect lighting
			mesh &current_mesh = scene.meshes()[all_rays[goodray].mesh];
			Vec3<Vec3f> triangles_coord= current_mesh.get_triangle_coord(all_rays[goodray].triangle);
			Vec3f v0 = triangles_coord[0];
			Vec3f v1 = triangles_coord[1];
			Vec3f v2 = triangles_coord[2];
			Vec3f vecteur1 = v1 - v0;
			Vec3f vecteur2 = v2 - v0;
			Vec3f normal = normalize(cross(vecteur1, vecteur2));

			if (length(normal) == 0) normal = Vec3f(1, 0, 0);
			float xx = (float)(rand() % 1000) /1000;
			float yy = (float)(rand() % 1000) /1000;
			float zz = (float)(rand() % 1000) /1000;
			Vec3f axe1 = normalize(cross(Vec3f(xx,yy,zz), normal));
			Vec3f axe2 = normalize(cross(axe1, normal));

			float angle1 = (float)(rand() % 3142) / 1000 - (3.142 / 2); // Angle 1 entre -pi/2 et +pi/2
			float angle2 = (float)(rand() % 3142) / 1000 - (3.142 / 2); // Angle 1 entre -pi/2 et +pi/2
			Vec3f newdirection = sin(angle1) * axe1 + cos(angle1) * axe2;
			newdirection = normalize(newdirection * sin(angle2) + normal * cos(angle2));

			float distance_a_passer;
			if (depth == 0)
				distance_a_passer = all_rays[goodray].t;
			else
				distance_a_passer = distance_deja_parcourue;

			couleur2 = color_response(intersection+ (newdirection/10000), newdirection, light, depth + 1, 
				distance_a_passer,scene, mon_BVH, light_intensity);
		}
	}

	return couleur1 + couleur2;

}

// Avec la BVH - 6 mars 2020
void rayTrace5BVH_GI(Image& im, scene& scene, BVH& mon_BVH, int Ktotal) {	// Take the bakground image and the scene as input
	int w = im.get_nbcols();
	int h = im.get_nbrows();
	int nbmesh = scene.get_n_meshes();
	Vec3f Black = Vec3f(0, 0, 0);

	camera camera = scene.get_camera();
	const Vec3<float> c = camera.position();
	const Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
	const Vec3<float> axey = normalize(cross(axex, camera.direction()));
	const Vec3<float> p0 = camera.position() + camera.direction();

	for (int x = 0; x < w; x++) {
		if (x % 10 == 0) std::cout << ".";
		//std::cout << "  x= " << x << std::endl;
#pragma omp parallel for
		for (int y = 0; y < h; y++) {
			//std::cout << "y= " << y << std::endl;

			Vec3f couleur_total = Vec3f(0, 0, 0);
			Vec3f couleur = Vec3f(0, 0, 0);

			for (int k = 0;k < Ktotal;k++) {
				//std::cout << "  k  " << k<<std::endl;
				float xplus = (float)(rand() % 100) / 100 - 0.50; //entre -0.5 et +.5
				float yplus = (float)(rand() % 100) / 100 - 0.50; //entre -0.5 et +.5

				int ligh_number = (rand() % scene.get_n_lightsources());
				const Vec3<float> light = scene.lightsource()[ligh_number].get_position();
				float light_intensity = scene.lightsource()[ligh_number].get_intensity();

				// Le centre de l'écran est à c + direction. 
				const Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f * (x + xplus)) / w)
					+ axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * (y + yplus)) / h);
				const Vec3<float> vecteur = normalize(p - c);

				couleur = color_response(c, vecteur, light, 0, 0,scene, mon_BVH, light_intensity);
				couleur_total += couleur;
				//std::cout << " k= " << k << "  couleur =" << couleur << std::endl;
			}
			couleur_total /= Ktotal;
			couleur_total /= 2; //Empirique
			//std::cout << "Couleur total = "<< couleur_total << std::endl;
			im.set_pixel(x, y, couleur_total);
			//std::cout << im.get_pixel(x, y) << std::endl;
		}
	}
}

// Calcul de l'AO - 20 mars 2020
void AO(Image& im, scene& scene, BVH& mon_BVH, int Ktotal) {	// Take the bakground image and the scene as input
	
	int w = im.get_nbcols();
	int h = im.get_nbrows();
	int nbmesh = scene.get_n_meshes();
	Vec3f Black = Vec3f(0, 0, 0);

	camera camera = scene.get_camera();
	const Vec3<float> c = camera.position();
	const Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
	const Vec3<float> axey = normalize(cross(axex, camera.direction()));
	const Vec3<float> p0 = camera.position() + camera.direction();


	for (int x = 0; x < w; x++) {

		if (x % 10 == 0) std::cout << ".";
		//std::cout << x << " ";
#pragma omp parallel for
		for (int y = 0; y < h; y++) {
			//std::cout << y << " ";

			Vec3f couleur_total = Vec3f(0, 0, 0);
			Vec3f couleur = Vec3f(0, 0, 0);

			// Stratégie : on reprend essentiellement le même code mais pour chaque pixel on tire un seul rayon, on touche la scene et là on lance K rayons pour calculer l'AO
			//const Vec3<float> light = scene.lightsource().get_position();
			
			// Le centre de l'écran est à c + direction. 
				
			const Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f *x ) / w) + axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * y ) / h);
			const Vec3<float> vecteur = normalize(p - c);
			Ray ray(c, vecteur);
			float e = 1e8;
			int goodray = -1;
			std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
			for (int i = 0; i < all_rays.size();i++) {
				float d = all_rays[i].t;
				if (d < e) {
					e = d;
					goodray = i;
				}
			}
			float AO = 0;
			if (goodray != -1) {
				const Vec3f intersection = c + all_rays[goodray].t * vecteur;	 // On a trouvé le point d'intersection entre de notre rayon sur la scene ; les choses sérieuses commencent
				const auto& mesh = scene.meshes()[all_rays[goodray].mesh];
				const auto& P = mesh.vertexPositions();
				const auto& N = mesh.vertexNormals();
				const Vec3i& triangle = mesh.indexedTriangles()[all_rays[goodray].triangle];
				// Normal de Phong à remettre pour petits objets mais pas efficace pour ma scene avec des angles droits
				//Vec3f hitNormal = normalize((1.f - u - v) * N[triangle[0]] + u * N[triangle[1]] + v * N[triangle[2]]);
				Vec3f hitNormal = normalize(cross(P[triangle[1]] - P[triangle[0]], P[triangle[2]] - P[triangle[0]]));
				if (length(hitNormal) == 0) hitNormal = Vec3f(1, 0, 0);

				// C'est ici qu'on va tirer K rayons et tester les intersections pour voir si on retouche la scène.
				// Je reprends le code déjà écrit dans RayTrace_GI pour chercher une direction aléatoire dans l'angle solide (pas hyper propre mais opérationnel)
	
				for (int k = 0;k < Ktotal;k++) {
					float xx = (float)(rand() % 1000) / 1000;
					float yy = (float)(rand() % 1000) / 1000;
					float zz = (float)(rand() % 1000) / 1000;
					Vec3f axe1 = normalize(cross(Vec3f(xx, yy, zz), hitNormal));
					Vec3f axe2 = normalize(cross(axe1, hitNormal));
					// J'ai maintenant une base orthonormée axe1,axe2,hitNormal

					float angle1 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 1 entre -pi/2 et +pi/2
					float angle2 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 2 entre -pi/2 et +pi/2
					Vec3f newdirection = sin(angle1) * axe1 + cos(angle1) * axe2;
					newdirection = normalize(newdirection * sin(angle2) + hitNormal * cos(angle2));
					Ray ray(intersection + newdirection * 0.0001, newdirection);
					//std::vector<rep_intersection> test_intersection = mon_BVH.rayintersect(ray, scene);
					//if (test_intersection.size() == 0)
					//	AO=AO+1;
					if  (!mon_BVH.does_intersect(ray, scene)) 
						AO =AO+1;
  				}
			}
			
			im.set_pixel(x, y, Vec3f(AO/ Ktotal,AO/ Ktotal,AO/ Ktotal));
		}
	}
}

// Calcul du ZBuffer - 21 mars 2020	à partir du code de janvier
// Le ZBuffer sera stocké dans l'image im (image grise = 3 fois la même chose dans chaque Vec3f de chaque pixel)
// Deux tiers de l'espace mémoire sont redondants  mais j'évite la création d'une nouvelle datastructure et d'éventuelles erreurs liées
// En plus, la visualisation sera facile !
void compute_Zbuffer(Image& im, scene& scene,BVH& mon_BVH) {	// Take the bakground image and the scene as input
	std::cout << "Calcul du ZBuffer";
	int w = im.get_nbcols();
	int h = im.get_nbrows();
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
			// int goodray = -1; [Pas besoin de garder d'info sur le Goodray]
			std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
			for (int i = 0; i < all_rays.size();i++) 
				if (all_rays[i].t < e) 
					e = all_rays[i].t;
			im.set_pixel(x, y, Vec3f(e/4,e/4,e/4));	 //Division par 4 pour avoir un meilleur rendu dans le cadre de notre image
		}
	}
	std::cout << "Calcul ZBuffer OK" << std::endl;
}

void compute_SSAO(Image& im, scene& scene, BVH& mon_BVH,int KMax) {

	int w = im.get_nbcols();
	int h = im.get_nbrows();
	Image ZBuffer(w, h);
	const float Radius = 20.f;
	compute_Zbuffer(ZBuffer, scene, mon_BVH);

	for (int x0 = 0; x0 < w; x0++) {
		if (x0 % 10 == 0) std::cout << ".";
		for (int y0 = 0; y0 < h; y0++) {

			// Stratégie 2 : prendre KMax pixel "autour" du pixel actuel dans le ZBuffer et 
			// compter les points qui sont plus près (pour être "équivalent" à AO)
			// NB : la stratégie 1 de comparaison entre la distance et la distance moyenne ne donne pas le résultat attendu
			float dist0 = ZBuffer.get_pixel(x0, y0)[0];
			float SSAO = 0;
			for (int k = 0; k < KMax; k++) {
				float xx = (float)(rand() % 1000) / 1000 - .5;	 //Entre -0.5 et +0.5
				float yy = (float)(rand() % 1000) / 1000 - .5;	 //Entre -0.5 et +0.5
				int x = x0 + (xx * Radius);
				int y = y0 + (yy * Radius);
				if (x < 0) x = 0;
				if (y < 0) y = 0;
				if (x >= w) x = w - 1;
				if (y >= h) y = h - 1;
				//if (ZBuffer.get_pixel(x, y)[0] > dist0)	 // Stratégie 2a
				//if (ZBuffer.get_pixel(x, y)[0] > dist0-.001) // Stratégie 2b
					if (ZBuffer.get_pixel(x, y)[0] > dist0 - .01) // Stratégie 2c
					SSAO += 1;
			}
			im.set_pixel(x0,y0,Vec3f(SSAO/KMax, SSAO / KMax, SSAO / KMax));
		}
	}
}


// La routine compute_SSDO_NoBounce ne sert plus car il suffit d'appeler "compute_SSDO_ZBuff" avec les mêmes paramètres et false
// Mais je laisse tout de même au cas où...
void compute_SSDO_NoBounce(Image& im, scene& scene, BVH& mon_BVH, int KMax) {

	int w = im.get_nbcols();
	int h = im.get_nbrows();
	Image mon_ZBuffer(w, h);
	compute_Zbuffer(mon_ZBuffer, scene, mon_BVH);

	const float RMax = .05f;

	camera camera = scene.get_camera();
	const Vec3<float> c = camera.position();
	const Vec3f camera_to_screen_normalise = normalize(camera.direction());
	float distance_camera_to_screen = (camera.direction().length());
	const Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
	const Vec3<float> axey = normalize(cross(axex, camera.direction()));
	const Vec3<float> p0 = camera.position() + camera.direction();
	
	const float tan_x = tan(camera.get_w_FoV() / 2);  //Pour éviter de le recalculer des millions de fois
	const float tan_y = tan(camera.get_w_FoV() / 2);  //Pour éviter de le recalculer des millions de fois

	for (int x0 = 0; x0 < w; x0++) {
		if (x0 % 10 == 0) std::cout << ".";
		for (int y0 = 0; y0 < h; y0++) {

			float dist0 = mon_ZBuffer.get_pixel(x0, y0)[0];
			Vec3f couleur_total = Vec3f(0, 0, 0);

			// Le centre de l'écran est à c + direction. 
			// Pour gagner du temps, on ne tire qu'un rayon jusqu'à trouver l'intersection avec la scene ; après on en relancera plusieurs
			const Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f * x0) / w)+ axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * y0) / h);
			const Vec3<float> vecteur = normalize(p - c);
			Ray ray(c, vecteur);
			float e = 1e8;
			int goodray = -1;
			std::vector<rep_intersection> all_rays = mon_BVH.rayintersect(ray, scene);
			for (int i = 0; i < all_rays.size();i++) 
				if (all_rays[i].t < e) {
					e = all_rays[i].t;
					goodray = i;
				}
			if (goodray != -1) {
				const Vec3f intersection = c + all_rays[goodray].t * vecteur;	 // On a trouvé le point d'intersection entre de notre rayon sur la scene ; les choses sérieuses commencent
				const auto& mesh = scene.meshes()[all_rays[goodray].mesh];
				const auto& P = mesh.vertexPositions();
				const auto& N = mesh.vertexNormals();
				const Vec3i& triangle = mesh.indexedTriangles()[all_rays[goodray].triangle];
				// Normal de Phong à remettre pour petits objets mais pas efficace pour ma scene avec des angles droits
				//Vec3f hitNormal = normalize((1.f - u - v) * N[triangle[0]] + u * N[triangle[1]] + v * N[triangle[2]]);
				Vec3f hitNormal = normalize(cross(P[triangle[1]] - P[triangle[0]], P[triangle[2]] - P[triangle[0]]));
				if (length(hitNormal) == 0) hitNormal = Vec3f(1, 0, 0);
				Vec3f wo = -vecteur;
				Material material = mesh.material();

				// C'est ici qu'on va tirer K rayons et tester les intersections pour voir si on retouche la scène.
				// Je reprends le code déjà écrit dans RayTrace_GI pour chercher une direction aléatoire dans l'angle solide (pas hyper propre mais opérationnel)

	
				for (int k = 0;k < KMax;k++) {
					float xx = (float)(rand() % 1000) / 1000;
					float yy = (float)(rand() % 1000) / 1000;
					float zz = (float)(rand() % 1000) / 1000;
					Vec3f axe1 = normalize(cross(Vec3f(xx, yy, zz), hitNormal));
					Vec3f axe2 = normalize(cross(axe1, hitNormal));
					// J'ai maintenant une base orthonormée axe1,axe2,hitNormal

					float angle1 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 1 entre -pi/2 et +pi/2
					float angle2 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 2 entre -pi/2 et +pi/2
					Vec3f newdirection = sin(angle1) * axe1 + cos(angle1) * axe2;
					newdirection = normalize(newdirection * sin(angle2) + hitNormal * cos(angle2));
					//std::cout << newdirection << std::endl;
					float lamba = RMax * (float)(rand() % 1000) / 1000; //Entre 0 et RMax
					Vec3f point = intersection + newdirection * lamba;

					// Il faut maintenant voir si on a un occluder ou pas 
					// Et commencer par trouver le point sur l'écran
					Vec3f camera_to_point = point - c;
					float distance_point_to_screen = dot(camera_to_point, camera_to_screen_normalise);
					Vec3f camera_to_sceen = camera_to_point / distance_point_to_screen * distance_camera_to_screen;
					int x1 = -(dot(-axex, camera_to_sceen) / tan_x - 1) * w / 2;
					int y1 = -(dot(axey, camera_to_sceen) / tan_y - 1) * h / 2;
					if (x1 < 0) x1 = 0;
					if (x1 >= w) x1 = w - 1;
					if (y1 < 0) y1 = 0;
					if (y1 >= h) y1 = h - 1;

					float dist1 = mon_ZBuffer.get_pixel(x1, y1)[0];
					if (dist1 < length(camera_to_point)) //Sinon il y a occlusion
					{
						Ray ray = Ray(intersection, newdirection);
						int ligh_number = (rand() % scene.get_n_lightsources());
						LightSource& Light1 = scene.lightsource()[ligh_number];
						float dist1 = Light1.ray_intersect(ray);
						if (dist1 > 0) {
							float intensity = Light1.get_intensity();
							Vec3f reponse = material.evaluationColorResponse(hitNormal, newdirection, wo);
							couleur_total += intensity * reponse / (dist1 * dist1);
						}
					}
				}
			}
			couleur_total /= KMax;
			im.set_pixel(x0, y0, couleur_total);
		}
	}
}

void compute_SSDO_ZBuff(Image& im, scene& scene, BVH& mon_BVH, int KMax,bool bounce) {

	int w = im.get_nbcols();
	int h = im.get_nbrows();
	Image OneBounce(w, h);
	Z_Buffer mon_ZBuffer = Z_Buffer(w,h, scene, mon_BVH);

	// Datastructure pour stocker les coordonnées des occluders pour chaque point de l'écran
	std::vector<std::vector<int>> vec_x1;
	std::vector<std::vector<int>> vec_y1;

	const float RMax = .1f;
	const float max_bleeding = .25f;

	camera camera = scene.get_camera();
	const Vec3<float> c = camera.position();
	const Vec3f camera_to_screen_normalise = normalize(camera.direction());
	float distance_camera_to_screen = (camera.direction().length());
	const Vec3<float> axex = normalize(cross(camera.direction(), { 0,1,0 }));
	const Vec3<float> axey = normalize(cross(axex, camera.direction()));
	const Vec3<float> p0 = camera.position() + camera.direction();

	const float tan_x = tan(camera.get_w_FoV() / 2);  //Pour éviter de le recalculer des millions de fois
	const float tan_y = tan(camera.get_w_FoV() / 2);  //Pour éviter de le recalculer des millions de fois

	std::cout << "Passe 1 - Direct lighting " << std::endl;

	for (int x0 = 0; x0 < w; x0++) {
		if (x0 % 10 == 0) std::cout << ".";
		for (int y0 = 0; y0 < h; y0++) {

			std::vector<int> pour_vec_x1;
			std::vector<int> pour_vec_y1;
			std::vector<Vec3f> pour_vec_p;

			Vec3f couleur_total = Vec3f(0, 0, 0);

			const Vec3<float> p = p0 + -axex * tan(camera.get_w_FoV() / 2) * (1.0f - (2.0f * x0) / w) + axey * tan(camera.get_h_FoV() / 2) * (1.0f - (2.0f * y0) / h);
			const Vec3<float> vecteur = normalize(p - c);

			float dist0 = mon_ZBuffer.get_depth(x0, y0);
			Vec3f& intersection = mon_ZBuffer.get_position(x0, y0);
			Vec3f& hitNormal = mon_ZBuffer.get_normal(x0, y0);
			if (length(hitNormal) == 0) hitNormal = Vec3f(1, 0, 0);
			// J'ai maintenant une base orthonormée axe1,axe2,hitNormal

			auto& mesh = scene.meshes()[mon_ZBuffer.get_mesh_index(x0, y0)];
			Material& material = mesh.material();

			float xx = (float)(rand() % 1000) / 1000;
			float yy = (float)(rand() % 1000) / 1000;
			float zz = (float)(rand() % 1000) / 1000;
			Vec3f axe1 = normalize(cross(Vec3f(xx, yy, zz), hitNormal));
			Vec3f axe2 = normalize(cross(axe1, hitNormal));

			// C'est ici qu'on va tirer K rayons et tester les intersections pour voir si on retouche la scène.
			// Je reprends le code déjà écrit dans RayTrace_GI pour chercher une direction aléatoire dans l'angle solide (pas hyper propre mais opérationnel)

			for (int k = 0;k < KMax;k++) {
				float angle1 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 1 entre -pi/2 et +pi/2
				float angle2 = (float)(rand() % 3140) / 1000 - (3.14 / 2); // Angle 2 entre -pi/2 et +pi/2
				Vec3f newdirection = sin(angle1) * axe1 + cos(angle1) * axe2;
				newdirection = normalize(newdirection * sin(angle2) + hitNormal * cos(angle2));

				float lamba = RMax * (float)(rand() % 1000) / 1000; //Entre 0 et RMax
				Vec3f point = intersection + newdirection * lamba;

				// Il faut maintenant voir si on a un occluder ou pas 
				// Et commencer par trouver le point sur l'écran
				Vec3f camera_to_point = point - c;
				float distance_point_to_screen = dot(camera_to_point, camera_to_screen_normalise);
				Vec3f camera_to_sceen = camera_to_point / distance_point_to_screen * distance_camera_to_screen;
				int x1 = -(dot(-axex, camera_to_sceen) / tan_x - 1) * w / 2;
				int y1 = -(dot(axey, camera_to_sceen) / tan_y - 1) * h / 2;
				if (x1 < 0) x1 = 0;
				if (x1 >= w) x1 = w - 1;
				if (y1 < 0) y1 = 0;
				if (y1 >= h) y1 = h - 1;

				float dist1 = mon_ZBuffer.get_depth(x1, y1);
				if (dist1 > length(camera_to_point)) //Sinon il y a occlusion
				{
					Ray ray = Ray(intersection, newdirection);		
					int ligh_number = (rand() % scene.get_n_lightsources());
					LightSource& Light1 = scene.lightsource()[ligh_number];
					float dist1 = Light1.ray_intersect(ray);
					if (dist1 > 0) {
						float intensity = Light1.get_intensity();
						Vec3f reponse = material.evaluationColorResponse(hitNormal, newdirection, -vecteur);
						//std::cout << intensity << "   "<< dist1 << "   " << reponse<<std::endl;

						couleur_total += intensity * reponse / (dist1 * dist1);
					}		
				}
				else { // Il y a occlusion : il faut stocker des informations pour la suite
					pour_vec_x1.push_back(x1);
					pour_vec_y1.push_back(y1);
				}
			}
			couleur_total /= KMax;
			im.set_pixel(x0, y0, couleur_total*2);
			vec_x1.push_back(pour_vec_x1);
			vec_y1.push_back(pour_vec_y1);
		}
	}
	im.save_to_PPM("SSDO_No Bounce_after pass 1.ppm");
	if (bounce) {  //Sinon pas de seconde passe
		std::cout << std::endl<< "Passe 2 - Indirect lighting One Bounce " << std::endl;

		for (int x0 = 0; x0 < w; x0++) {
			if (x0 % 10 == 0) std::cout << ".";
			for (int y0 = 0; y0 < h; y0++) {

				Vec3f couleur_total = Vec3f(0, 0, 0);

				// On récupère les champs stockés en première passe
				std::vector<int> pour_vec_x1 = vec_x1[x0 * h + y0];
				std::vector<int> pour_vec_y1 = vec_y1[x0 * h + y0];
				//std::vector<Vec3f> pour_vec_p = vec_p[x0 * h + y0];

				Vec3f& point_0 = mon_ZBuffer.get_position(x0, y0);	 
				Vec3f& hitNormal_0 = mon_ZBuffer.get_normal(x0, y0);
				Vec3f point_a_camera = c- point_0;
				float cos2 = dot(hitNormal_0, point_a_camera);
				float distance_p0_c = length(point_a_camera);

				for (int k = 0;k < pour_vec_x1.size(); k++) {
					int x1 = pour_vec_x1[k];
					int y1 = pour_vec_y1[k];
					Vec3f point_1 = mon_ZBuffer.get_position(x1, y1);
					Vec3f& hitNormal_1 = mon_ZBuffer.get_normal(x1, y1);

					Vec3f Lum = im.get_pixel(x1, y1);
					Vec3f point_a_point = point_1 - point_0;
					float distance = length(point_a_point);
					if (distance < (2.5*RMax) ) distance = 2.5* RMax ; //Distance mini
					//if (distance < 1) distance = 1;
					float cos1 = dot(hitNormal_1, point_a_point) / distance;

					if (cos1 < 0) cos1 = 0;
					if (cos2 < 0) cos2 = 0;
					couleur_total += Lum * cos1 * cos2 * RMax * RMax / (distance * distance);

				}
				couleur_total /= KMax;
				couleur_total *= 20;		 // Paramètre à ajuster manuellement

				// Je cherche à contrôler le bleeding pour qu'on ne voit pas que cela...
				float max_channel = valeur_max(couleur_total[0], couleur_total[1], couleur_total[2]);
				if (max_channel > max_bleeding)
					couleur_total *= (max_bleeding / max_channel);

				OneBounce.set_pixel(x0, y0, couleur_total);
				}
		}
		OneBounce.save_to_PPM("SSDO_One_Bounce_Only the bounce.ppm");
		for (int x0 = 0; x0 < w; x0++) {
			for (int y0 = 0; y0 < h; y0++) {
				Vec3f Im1 = im.get_pixel(x0, y0);
				Vec3f Im2 = OneBounce.get_pixel(x0, y0);
				im.set_pixel(x0, y0, Im1 + Im2);
			}
		}

	}
}