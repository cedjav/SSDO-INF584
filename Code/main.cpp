#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "vec3.h"
#include "image.h"
#include "camera.h"
#include "Ray.h"
#include "Lightsource.h"
#include "Material.h"
#include "Mesh.h"
#include "scene.h"
#include "BVH.h"
#include "RayTracer.h"
#include "WorleyNoise.h"
#include "Accelerating.h"
#include <ctime>


const int width = 200;
const int height = 200;
#define epsilon 1e-6

// Fabrique un mur rectangulaire
mesh build_wall(Vec3f p00, Vec3f dir1, Vec3f dir2) {
	mesh mon_mesh;
	Vec3f p10 = p00 + dir1;
	Vec3f p01 = p00 + dir2;
	Vec3f p11 = p00 + dir1 + dir2;

	mon_mesh.add_triangle(p00, p10, p01);
	mon_mesh.add_triangle(p01, p10, p11);
	mon_mesh.recomputenormales();
	mon_mesh.compute_barycentre();
	return mon_mesh;
}

// Fabrique un parrallelepidède rectangle
void build_boxrectangle(mesh& cube, Vec3f p00, Vec3f dir1, Vec3f dir2, Vec3f dir3) {
	cube.vertexPositions()[0] = p00;
	cube.vertexPositions()[1] = p00 + dir1;
	cube.vertexPositions()[2] = p00 + dir2;
	cube.vertexPositions()[3] = p00 + dir1 + dir2;

	cube.vertexPositions()[4] = p00 + dir3;
	cube.vertexPositions()[5] = p00 + dir3 + dir1;
	cube.vertexPositions()[6] = p00 + dir3 + dir2;
	cube.vertexPositions()[7] = p00 + dir3 + dir1 + dir2;

	cube.recomputenormales();
	cube.compute_barycentre();
}

// Fabrication d'un parrallelepidède rectangle un peu rétréci en haut comme le grand bloc blanc du haut de la page  de l'article
// On passe un cube (pour ne pas recalculer la topologie en entrée)
void build_box_presque_rectangle(mesh& cube,Vec3f p00, Vec3f dir1, Vec3f dir2, Vec3f dir3, float retrecissement) {
	cube.vertexPositions()[0] = p00;
	cube.vertexPositions()[1] = p00+dir1;
	cube.vertexPositions()[2] = p00+dir2;
	cube.vertexPositions()[3] = p00+dir1+dir2;

	cube.vertexPositions()[4] = p00 + dir3 + (retrecissement * dir1) ;
	cube.vertexPositions()[5] = p00 + dir3 + (1 - retrecissement) * dir1 ;
	cube.vertexPositions()[6] = p00 + dir3 + (retrecissement * dir1) + dir2;
	cube.vertexPositions()[7] = p00 + dir3 + (1 - retrecissement) * dir1+dir2;

	cube.recomputenormales();
	cube.compute_barycentre();
}

scene load_scene1() {
	Material material_rouge(1.00f, Vec3f(0.9f, 0.1f, 0.1f), 0.5f, Vec3f(1.0f, 1.0f, 1.0f));
	Material material_vert(1.00f, Vec3f(0.1f, 0.9f, 0.1f), 0.5f, Vec3f(1.0f, 1.0f, 1.0f));
	Material material_clair(1.00f, Vec3f(0.95f, 0.95f, 0.95f), 0.5f, Vec3f(1.0f, 0.9f, 0.7f));

	// Construction des murs 
	float zcte = -2.f;
	float xcte = 1.0f;
	float ycte = 1.0f;

	Vec3f derriere_bas_gauche = Vec3f(-xcte, -ycte, zcte);
	Vec3f derriere_bas_droite = Vec3f(xcte, -ycte, zcte);
	Vec3f derriere_haut_gauche = Vec3f(-xcte, ycte, zcte);
	Vec3f derriere_haut_droite = Vec3f(xcte, ycte, zcte);

	Vec3f devant_bas_gauche = Vec3f(-xcte, -ycte, -zcte);
	Vec3f devant_bas_droite = Vec3f(xcte, -ycte, -zcte);
	Vec3f devant_haut_gauche = Vec3f(-xcte, ycte, -zcte);
	Vec3f devant_haut_droite = Vec3f(xcte, ycte, -zcte);

	Vec3f droite = derriere_bas_droite - derriere_bas_gauche;
	Vec3f haut = derriere_haut_gauche - derriere_bas_gauche;
	Vec3f devant = devant_bas_gauche - derriere_bas_gauche;

	mesh mur_fond = build_wall(derriere_bas_gauche, droite, haut);
	mesh mur_gauche = build_wall(derriere_bas_gauche, haut, devant);
	mesh mur_bas = build_wall(derriere_bas_gauche, devant, droite);
	mesh mur_droite = build_wall(derriere_haut_droite, -haut, devant);
	mesh mur_haut = build_wall(derriere_haut_droite, devant, -droite);


	// Murs comme dans le slide 20 de la séance GI
	mur_fond.material() = material_clair;
	mur_gauche.material() = material_rouge;
	mur_bas.material() = material_clair;
	mur_droite.material() = material_vert;
	mur_haut.material() = material_clair;


	// Création des deux cubes pour les mettre dans la scene
	mesh cube;
	cube.load_off("../cube_tri.OFF");
	mesh cube1 = cube;
	for (int j = 0;j < cube1.get_nb_v();j++) {
		cube1.vertexPositions()[j] /= 3;
		cube1.vertexPositions()[j][0] -= 0.1;
		cube1.vertexPositions()[j][1] -= 1;
		cube1.vertexPositions()[j][2] -= 1.3;

		//cube1.vertexPositions()[j][0] += (xcte / 4);
		//cube1.vertexPositions()[j][1] -= ycte;
		float oldx = cube1.vertexPositions()[j][0];
		float oldz = cube1.vertexPositions()[j][2];
		float angle = -(30.f / 180.f) * 3.14159; //rotation de 20 degré autour de z
		// Je ne trouve pas de rotation matricielle facile dans Vec3 alors je fais à la main :
		cube1.vertexPositions()[j][0] = cos(angle) * oldx + sin(angle) * oldz;
		cube1.vertexPositions()[j][2] = -sin(angle) * oldx + cos(angle) * oldz;
	}
	cube1.recomputenormales();
	cube1.compute_barycentre();
	cube1.material() = material_rouge;


	mesh cube2 = cube;
	for (int j = 0;j < cube1.get_nb_v();j++) {
		cube2.vertexPositions()[j][0] = (cube2.vertexPositions()[j][0] / 2) - .25;
		cube2.vertexPositions()[j][1] = (cube2.vertexPositions()[j][1] * 1.3) - ycte + .1;
		cube2.vertexPositions()[j][2] = (cube2.vertexPositions()[j][2] / 2) - 1.5;

		float oldx = cube2.vertexPositions()[j][0];
		float oldz = cube2.vertexPositions()[j][2];
		float angle = (20.f / 180.f) * 3.14159; //rotation de 20 degré autour de z
		// Je ne trouve pas de rotation matricielle facile dans Vec3 alors je fais à la main :
		cube2.vertexPositions()[j][0] = cos(angle) * oldx + sin(angle) * oldz;
		cube2.vertexPositions()[j][2] = -sin(angle) * oldx + cos(angle) * oldz;
	}
	cube2.recomputenormales();
	cube2.compute_barycentre();
	cube2.material() = material_clair;

	// Mise en place de l'image de la lumière et de la caméra
	camera ma_camera3(Vec3<float>(0, -.3, 1.5), Vec3<float>(0, 0.0, -1), 50.0f, (float)width / height);
	float light_size = .3f;
	LightSource lightsource3(Vec3f(0.f, ycte - 0.2f, zcte / 4), Vec3f(1.f, 1.f, 1.f), 4.f, Vec3f(0.f, -1.f, 0.f), light_size, Vec3f(1.f, 0.f, 0.f));
	//LightSource lightsource3(Vec3f(0.f, 0, 2.f), Vec3f(1.f, 1.f, 1.f), 5.f, Vec3f(0.f, 0.f,-1.f), light_size);
	scene ma_scene(mur_fond, ma_camera3, lightsource3);
	ma_scene.add_mesh(mur_gauche);
	ma_scene.add_mesh(mur_bas);
	ma_scene.add_mesh(mur_droite);
	ma_scene.add_mesh(mur_haut);
	ma_scene.add_mesh(cube1);
	ma_scene.add_mesh(cube2);

	return ma_scene;

}

// Lumière en haut
scene load_scene2() {
		Material material_rouge(1.00f, Vec3f(0.9f, 0.1f, 0.1f), 0.5f, Vec3f(1.0f, 1.0f, 1.0f));
		Material material_vert(1.00f, Vec3f(0.1f, 0.9f, 0.1f), 0.5f, Vec3f(1.0f, 1.0f, 1.0f));
		Material material_clair(1.00f, Vec3f(0.95f, 0.95f, 0.95f), 0.5f, Vec3f(1.0f, 0.9f, 0.7f));
		Material material_bleu(1.00f, Vec3f(0.1f, 0.1f, 0.95f), 0.5f, Vec3f(1.0f, 1.0f, 1.0f));

		// Construction des murs 
		float zcte = -2.f;
		float xcte = 1.0f;
		float ycte = 1.0f;

		Vec3f derriere_bas_gauche = Vec3f(-xcte, -ycte, zcte);
		Vec3f derriere_bas_droite = Vec3f(xcte, -ycte, zcte);
		Vec3f derriere_haut_gauche = Vec3f(-xcte, ycte, zcte);
		Vec3f derriere_haut_droite = Vec3f(xcte, ycte, zcte);

		Vec3f devant_bas_gauche = Vec3f(-xcte, -ycte, -zcte);
		Vec3f devant_bas_droite = Vec3f(xcte, -ycte, -zcte);
		Vec3f devant_haut_gauche = Vec3f(-xcte, ycte, -zcte);
		Vec3f devant_haut_droite = Vec3f(xcte, ycte, -zcte);

		Vec3f droite = derriere_bas_droite - derriere_bas_gauche;
		Vec3f haut = derriere_haut_gauche - derriere_bas_gauche;
		Vec3f devant = devant_bas_gauche - derriere_bas_gauche;

		mesh mur_fond = build_wall(derriere_bas_gauche, droite, haut);
		mesh mur_gauche = build_wall(derriere_bas_gauche, haut, devant);
		mesh mur_bas = build_wall(derriere_bas_gauche, devant, droite);
		mesh mur_droite = build_wall(derriere_haut_droite, -haut, devant);
		mesh mur_haut = build_wall(derriere_haut_droite, devant, -droite);

		// 5 Murs comme dans le slide 20 de la séance GI
		mur_fond.material() = material_clair;
		mur_gauche.material() = material_rouge;
		mur_bas.material() = material_clair;
		mur_droite.material() = material_vert;
		mur_haut.material() = material_clair;

		// Mise en place de l'image de la lumière et de la caméra
		camera ma_camera3(Vec3<float>(0, -.3, 1.5), Vec3<float>(0, 0.0, -1), 50.0f, (float)width / height);
		float light_size = .5f;
		LightSource lightsource3(Vec3f(0.f, ycte - 0.2f, +.5f), Vec3f(1.f, 1.f, 1.f), 2.f, Vec3f(0.f, -1.f, 0.f), light_size, Vec3f(1.f, 0.f, 0.f));
		scene ma_scene(mur_fond, ma_camera3, lightsource3);
		ma_scene.add_mesh(mur_gauche);
		ma_scene.add_mesh(mur_bas);
		ma_scene.add_mesh(mur_droite);
		ma_scene.add_mesh(mur_haut);

		// Chargement d'un cube pour avoir la topologie 
		mesh cube;
		cube.load_off("../cube_tri.OFF");

		// Long cube du dessus
		Vec3f P0 = Vec3f(-.4f, -.65f, .2f);
		Vec3f dir1 = Vec3f(0.2f, 0, 0);
		Vec3f dir2 = Vec3f(0.f, 0.f, -.6f);
		Vec3f dir3 = Vec3f(0, .1f, 0);
		mesh cube1 = cube, cube2 = cube, cube3 = cube, cube4 = cube;
		build_box_presque_rectangle(cube1, P0, dir1, dir2, dir3, .1f);
		build_box_presque_rectangle(cube2, P0 + dir1, dir1, dir2, dir3, .1f);
		build_box_presque_rectangle(cube3, P0 + dir1 *2, dir1, dir2, dir3, .1f);
		build_box_presque_rectangle(cube4, P0 + dir1 *3, dir1, dir2, dir3, .1f);
		cube1.material() = material_clair;
		cube2.material() = material_clair;
		cube3.material() = material_clair;
		cube4.material() = material_clair;
		ma_scene.add_mesh(cube1);
		ma_scene.add_mesh(cube2);
		ma_scene.add_mesh(cube3);
		ma_scene.add_mesh(cube4);

		// Plus petit cube de devant
		mesh cube5 = cube, cube6 = cube, cube7 = cube, cube8 = cube;
		Vec3f dir2bis = Vec3f(0, -.2f, 0);
		Vec3f dir3bis = Vec3f(0.f, 0, .1f);
		build_box_presque_rectangle(cube5, P0+dir1, -dir1, dir2bis, dir3bis, .1f);
		build_box_presque_rectangle(cube6, P0 + dir1*2, -dir1, dir2bis, dir3bis, .1f);
		build_box_presque_rectangle(cube7, P0 + dir1 *3, -dir1, dir2bis, dir3bis, .1f);
		build_box_presque_rectangle(cube8, P0 + dir1 *4, -dir1, dir2bis, dir3bis, .1f);
		cube5.material() = material_rouge;
		cube6.material() = material_clair;
		cube7.material() = material_vert;
		cube8.material() = material_clair;
		ma_scene.add_mesh(cube5);
		ma_scene.add_mesh(cube6);
		ma_scene.add_mesh(cube7);
		ma_scene.add_mesh(cube8);

		// Mise en place de mésostructure : petits cubes bleus
		// Première rangée sur cube 2
		mesh rect_sur_haut= cube;
		rect_sur_haut.material() = material_bleu;
		Vec3f P1 = P0 + dir1 * 1.15 + dir2 * .05 + dir3;
		build_boxrectangle(rect_sur_haut, P1 + dir1 * .00, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .13, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .26, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		// Deuxième rangée sur cube 2
		P1 = P0 + dir1 * 1.6 + dir2 * .2 + dir3;
		build_boxrectangle(rect_sur_haut, P1 + dir1 * .00, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .04, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .08, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);


		// Première rangée sur cube 3
		P1 = P0 + dir1 * 2.15 + dir2 * .05 + dir3;
		build_boxrectangle(rect_sur_haut, P1 + dir1 * .00, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .13, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .26, dir1 * .1, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);


		// Deuxième rangée sur cube 3
		P1 = P0 + dir1 * 2.6 + dir2 * .2 + dir3;
		build_boxrectangle(rect_sur_haut, P1 + dir1 * .00, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .04, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 + dir1 * .08, dir1 * .02, dir2 * .1, dir3 * .3);
		ma_scene.add_mesh(rect_sur_haut);


		// Petits cube rouge
		rect_sur_haut.material() = material_rouge;
		P1 = P0  + dir1*1.8 + dir3bis + dir2bis * .2;

		build_boxrectangle(rect_sur_haut, P1 - dir1 * .10 + dir2bis * .00, -dir1 * .10, dir2bis * .3, dir3bis * .5);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 - dir1 * .23 + dir2bis * .05, -dir1 * .10, dir2bis * .3, dir3bis * .5);
		ma_scene.add_mesh(rect_sur_haut);

		build_boxrectangle(rect_sur_haut, P1 - dir1 * .36 + dir2bis * .10, -dir1 * .10, dir2bis * .3, dir3bis * .5);
		ma_scene.add_mesh(rect_sur_haut);



		return ma_scene;

}	 // Lumière en haut = rouge

int main(int argc, char*argv[])
{
	scene ma_scene=load_scene2();
	LightSource env_map(Vec3f(0, -.3, 10), Vec3f(1.f, 1.f, 1.f), 500.f, Vec3f(0.f, 0.f, -1.f), 30, Vec3f(1.f, 0.f, 0.f));
	ma_scene.add_lightsource(env_map);


	// Etapes 1 à 3 : code généré pour le TD : BVH, DI avec GI

	// Etape 1 : calcul de la BVH de la scene... meme si cela n'amène pas énormément de gain sur une scene qui reste très simple
	std::clock_t start3 = std::clock();
	std::cout << "Lancement de la construction de la BVH de la scene...";
	BVH mon_BVH = BVH(ma_scene);
	std::cout << "BVH OK avec duree " << std::clock() - start3 << std::endl;


	// Etape 2 : calcul en Direct Illumination avec 256 rays (les lumières sont aera lights) 
	Image Mon_image_DI(width, height);
	start3 = std::clock();
	std::cout << "Calcul de la DI avec 256 rays" << std::endl;
	rayTrace5BVH(Mon_image_DI, ma_scene, mon_BVH, 256);
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;
	Mon_image_DI.save_to_PPM("Compute_DI.ppm");


	// Etape 3 : calcul en GI avec 1-4-16-64-256 rays
	
	Image Mon_image_GI(width, height);		  
	std::cout << "BVH + GI 1 sample" << std::endl;
	start3 = std::clock();
	rayTrace5BVH_GI(Mon_image_GI, ma_scene, mon_BVH, 1);
	Mon_image_GI.save_to_PPM("rayTrace_BVH_withGI - Kotal = 1.ppm");
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;

	std::cout << "BVH + GI 4 sample" << std::endl;
	start3 = std::clock();
	rayTrace5BVH_GI(Mon_image_GI, ma_scene, mon_BVH, 4);
	Mon_image_GI.save_to_PPM("rayTrace_BVH_withGI - Kotal = 4.ppm");
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;

	std::cout << "BVH + GI 16 samples" << std::endl;
	start3 = std::clock();
	rayTrace5BVH_GI(Mon_image_GI, ma_scene, mon_BVH, 16);
	Mon_image_GI.save_to_PPM("rayTrace_BVH_withGI - Kotal = 16.ppm");
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;

	std::cout << "BVH + GI 64 samples" << std::endl;
	start3 = std::clock();
	rayTrace5BVH_GI(Mon_image_GI, ma_scene, mon_BVH, 64);
	Mon_image_GI.save_to_PPM("rayTrace_BVH_withGI - Kotal = 64.ppm");
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;

		std::cout << "BVH + GI 256 samples" << std::endl;
	start3 = std::clock();
	rayTrace5BVH_GI(Mon_image_GI, ma_scene, mon_BVH, 256);
	Mon_image_GI.save_to_PPM("rayTrace_BVH_withGI - Kotal = 256.ppm");
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;
	
	

	// Etape 4 : calcul de l'Ambiant Occlusion avec 256 rays 
	Image Mon_image_AO(width, height);
	std::cout << "Calcul de l'AO" << std::endl;
	start3 = std::clock();
	AO(Mon_image_AO, ma_scene, mon_BVH, 256);	 
	std::cout << "AO OK avec Duree : " << std::clock() - start3 << std::endl;
	Mon_image_AO.save_to_PPM("Compute_AO.ppm");

	// Etape 4bis : on calcule AO/2 + DI
	// C'est ce qui donne le rendu le plus réaliste au final (j'ai essayé d'autres combinaisons)
	Image Mon_image_AODI(width, height);
	for (int x = 0;x < width;x++)
		for (int y = 0; y < height; y++) {
			Vec3f AO = Mon_image_AO.get_pixel(x, y);
			Vec3f DI = Mon_image_DI.get_pixel(x, y);
			Mon_image_AODI.set_pixel(x, y, AO / 2 + DI); 
		}
	Mon_image_AODI.save_to_PPM("DI+AOx.5.ppm");

			


	// Etape 5 : calcul de la SSAO
	// Etape 5A : calcul du Z-Buffer : ici le Z-Buffer est une image en niveau de gris car R G et B stockent tous les mêmes informations
	// de distance (en fait distance / 4) pour avoir qqch de "joli"	à afficher dans cette scene

	Image Mon_image_ZBuffer(width, height); 
	std::cout << "Calcul du ZBuffer" << std::endl;
	start3 = std::clock();
	compute_Zbuffer(Mon_image_ZBuffer, ma_scene, mon_BVH);
	std::cout << "ZBuffer OK avec duree : " << std::clock() - start3 << std::endl;
	Mon_image_ZBuffer.save_to_PPM("Compute_ZBuffer.ppm");

	// Etape 5B : calcul de la SSAO. La SSAO recalcule le Z-Buffer ; l'étape 5A c'était juste pour sauvegarder l'image
	Image Mon_image_SSAO(width, height);
	std::cout << "Calcul de la SSAO" << std::endl;
	start3 = std::clock();
	compute_SSAO(Mon_image_SSAO, ma_scene, mon_BVH, 64);	  // Le dernier paramètre est le nombre de rayon tiré pour chaque point de l'image
	std::cout << "SSAO OK avec duree : " << std::clock() - start3 << std::endl;
	Mon_image_SSAO.save_to_PPM("SSAO.ppm");

	// Etape 5C : Pour combiner SSAO et DI je fais la multiplication et un peu d'addition
	Image Mon_image_SSAODI(width, height);
	for (int x = 0;x < width;x++)
		for (int y = 0; y < height; y++) {

			Vec3f SSAO = Mon_image_SSAO.get_pixel(x, y);
			Vec3f DI = Mon_image_DI.get_pixel(x, y);
			Vec3f Produit = (SSAO + Vec3f(.4f, .4f, .4f)) * DI;  // Cote mal taillée pour faire un mix en tre multiplication et addition
			Mon_image_SSAODI.set_pixel(x, y, Produit);
		}
	Mon_image_SSAODI.save_to_PPM("DI x SSAO.ppm");




	// Etape 6 : Calcul de la SSDO avec et sans rebond
	Image Mon_Image_SSDO_NoBounce(width, height);

	// Etape 6A : sans rebond
	// Ne sert en fait à rien car c'est déjà calculé comme première de la suite. 
	//std::cout << "Calcul de la SSDO sans Bounce avec BVH" << std::endl;
	//start3 = std::clock();
	//compute_SSDO_NoBounce(Mon_Image_SSDO_NoBounce, ma_scene, mon_BVH, 1024);   // 1024 rays
	//std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;
	//Mon_Image_SSDO_NoBounce.save_to_PPM("SSDO_No Bounce.ppm");	  

	// A noter : pour SSDO, il faut stocker plus d'information et, je créerai une classe Z-Buffer qui contient plus d'info
	// La routine de RayTracing fait des sauvegarde intermédiaire
	Image Mon_Image_SSDO_OneBounce(width, height);
	std::cout << "Calcul de la SSDO sans Bounce avec BVH et class ZBuffer complete" << std::endl;
	start3 = std::clock();
	compute_SSDO_ZBuff(Mon_Image_SSDO_OneBounce, ma_scene, mon_BVH, 1024,true);	  //1024 rays
	std::cout << "OK avec duree : " << std::clock() - start3 << std::endl;
	Mon_Image_SSDO_OneBounce.save_to_PPM("SSDO_Direct_plus_OneBounce.ppm");

	// Essaie de réduction du bruit	: peu convaincant et non évoqué dans le rapport
	//Image Mon_Image_SSDO_NoBounce_Blured(width, height);
	//Mon_Image_SSDO_NoBounce.Blur(Mon_Image_SSDO_NoBounce_Blured);
	//Mon_Image_SSDO_NoBounce_Blured.save_to_PPM("SSDO_No Bounce_Blured.ppm");

	return 0;
}