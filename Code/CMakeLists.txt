cmake_minimum_required(VERSION 3.0)

project(SSDO_Cedric_JAVAULT)

add_executable (
	SSDO_Cedric_JAVAULT
	main.cpp
	Ray.cpp
	Raytracer.cpp
	WorleyNoise.cpp
	Accelerating.h
	BVH.h
	Camera.h
	image.h
	LightSource.h
	Material.h
	Mesh.h
	Ray.h
	RayTracer.h
	scene.h
	vec3.h
	WorleyNoise.h
	ZBuffer.h
)