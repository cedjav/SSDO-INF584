#pragma once

#ifndef RayTracerh
#define RayTracerh


void rayTrace5BVH(Image& im, scene& scene, BVH& mon_BVH, int Ktotal);
void AO(Image& im, scene& scene, BVH& mon_BVH, int Ktotal);
void rayTrace5BVH_GI(Image& im, scene& scene, BVH& mon_BVH, int Ktotal);

inline bool isvisible(Ray& ray, scene& scene);
inline bool isvisibleBVH(Ray& ray, scene& scene, BVH& mon_BVH, float distance_to_light);
Vec3f color_response(const Vec3f& origine, const Vec3f& direction,
	const Vec3f& light, const int& depth, const float& distance_deja_parcourue, scene& scene, BVH& mon_BVH, float intensity);
void compute_Zbuffer(Image& im, scene& scene, BVH& mon_BVH);

inline Vec3f shade2(const scene& scene, const size_t& meshIndex, const size_t& triangleIndex,
	const float& u, const float& v, const Ray& ray, const Vec3f& light_position, const float distance_deja_parcourue, float intensity);

void compute_SSAO(Image& im, scene& scene, BVH& mon_BVH, int KMax);
void compute_SSDO_NoBounce(Image& im, scene& scene, BVH& mon_BVH, int KMax);
//void compute_SSDO_NoBounce_ZBuff(Image& im, scene& scene, BVH& mon_BVH, int KMax);
void compute_SSDO_ZBuff(Image& im, scene& scene, BVH& mon_BVH, int KMax, bool bounce);



#endif