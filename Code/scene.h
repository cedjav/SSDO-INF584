#pragma once

#ifndef Sceneh
#define Sceneh

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include "vec3.h"
#include "Lightsource.h"
#include "Material.h"
#include "camera.h"
#include "Mesh.h"


class scene {
public:
	scene() {
		m_meshes.resize(0);
		m_nb_meshes = 0;
		m_nb_lightsources = 0;
	};

	scene(mesh mesh, camera camera) :m_camera(camera) {
		m_meshes.resize(0);
		mesh.compute_AABB();
		m_meshes.push_back(mesh);
		LightSource light = LightSource();
		m_lightsource.push_back(light);
		m_nb_meshes = 1;
		m_nb_lightsources = 1;
	};

	scene(mesh mesh, camera camera, LightSource lightsource) :m_camera(camera) {
		m_meshes.resize(0);
		mesh.compute_AABB();
		m_meshes.push_back(mesh);
		m_lightsource.push_back(lightsource);
		m_nb_meshes = 1;
		m_nb_lightsources = 1;
	};

	scene(mesh mesh, camera camera, LightSource lightsource,Material material) :m_camera(camera) {
		m_meshes.resize(0);
		mesh.compute_AABB();
		m_meshes.push_back(mesh);
		m_lightsource.push_back(lightsource);
		m_nb_meshes = 1;	
		m_nb_lightsources = 1;
	};

	void add_mesh(mesh mesh) {
		mesh.compute_AABB();
		m_meshes.push_back(mesh);
		m_nb_meshes++;
	};

	void add_lightsource(LightSource light) {
		m_lightsource.push_back(light);
		m_nb_lightsources++;
	};

	void set_camera(camera cam) { m_camera = cam; }

	inline size_t get_n_meshes() { return m_nb_meshes; }

	inline size_t get_n_lightsources() { return m_nb_lightsources; }

	inline const camera& get_camera() const { return m_camera; }

	inline const std::vector<mesh>& meshes() const { return m_meshes; }

	inline std::vector<mesh>& meshes() { return m_meshes; }

	inline const std::vector < LightSource>& lightsource() const { return m_lightsource; }

	inline std::vector <LightSource>& lightsource() { return m_lightsource; }




public:
	camera m_camera;
	std::vector<mesh> m_meshes;
	std::vector<LightSource> m_lightsource;

	int m_nb_meshes;
	int m_nb_lightsources;

};


#endif