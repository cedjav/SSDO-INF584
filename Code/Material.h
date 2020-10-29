#pragma once

#ifndef Materialh
#define Materialh

#include "Vec3.h"
#include "WorleyNoise.h"

class Material {
public:
	Material(const float diffuse = 0.00f, const Vec3f& color = Vec3f(0.8f, 0.5f, 0.4f),const float alpha = 0.5f, const Vec3f F0=Vec3f(1.0f,0.86f,0.57f)) : 
		m_diffuse(diffuse), m_color(color), m_alpha(alpha),m_F0(F0),m_alphacarre(alpha*alpha) {};

	virtual ~Material() {};

	void createAlbedoNoise(int nb_points, Vec3f min, Vec3f max) {
		m_WorleyNoise_albedo = WorleyNoise(nb_points, min, max);
		m_WorleyNoise_roughness = WorleyNoise(nb_points, min, max);
		m_WorleyNoise_metallicness = WorleyNoise(nb_points, min, max);

	}

	void createAlbedoNoise(int nb_points, Vec3f min, Vec3f max, Vec3f min1, Vec3f max1) {
		m_WorleyNoise_albedo = WorleyNoise(nb_points, min, max,min1,max1);
		m_WorleyNoise_roughness = WorleyNoise(nb_points, min, max, min1, max1);
		m_WorleyNoise_metallicness = WorleyNoise(nb_points, min, max, min1, max1);

	}

	inline Vec3f evaluationColorResponse(const Vec3f& normal, const Vec3f& wi) {
		float produit_scalaire = dot(normal,wi);
		if (produit_scalaire < 0) produit_scalaire = 0;
		float reponse = m_diffuse * produit_scalaire;
		return reponse * m_color;
	};

	inline Vec3f evaluationColorResponse(const Vec3f& normal, const Vec3f& wi, const Vec3f& wo) {
		float produit_scalaire = dot(normal,wi);
		if (produit_scalaire < 0) return Vec3f(0, 0, 0);

		const Vec3f wh = normalize((wi + wo) / 2.0f);

		float d = m_alphacarre / 
			(3.14159f * pow((1 + (m_alphacarre - 1) * pow(dot(normal, wh),2)), 2));
		float scal = dot(wi, wh);
		if (scal < 0) scal = 0;
		float scalpow= pow(1 - scal, 5);

		Vec3f f = Vec3f(m_F0[0] + (1 - m_F0[0]) * scalpow,
			m_F0[1] + (1 - m_F0[1]) * scalpow,
			m_F0[2] + (1 - m_F0[2]) * scalpow);

		float g = 1; // Approximation
		Vec3f global = Vec3f(m_diffuse, m_diffuse, m_diffuse) + d * f * g * produit_scalaire;
		return global *f* m_color;
	};

	inline Vec3f evaluationColorResponse(const Vec3f& normal, const Vec3f& wi, const Vec3f& wo, const Vec3f& position) {
		float produit_scalaire = dot(normal, wi);
		if (produit_scalaire < 0) return Vec3f(0, 0, 0);

		const Vec3f wh = normalize((wi + wo) / 2.0f);

		float alphacarre = m_WorleyNoise_roughness.Noise(position);
		if (alphacarre > 1) alphacarre = 1;
		if (alphacarre > 0) alphacarre = 0;
		float d = alphacarre /
			(3.14159f * pow((1 + (alphacarre - 1) * pow(dot(normal, wh), 2)), 2));
		float scal = dot(wi, wh);
		if (scal < 0) scal = 0;
		float scalpow = pow(1 - scal, 5);

		float F0 = m_WorleyNoise_metallicness.Noise(position);
		if (F0 > 1) F0 = 1;
		float f = F0 + (1 - F0) * scalpow;

		float g = 1; // Approximation
		
		float reponse = (m_diffuse + d * f * g) * produit_scalaire;

		float noise = m_WorleyNoise_albedo.Noise(position);
		Vec3f color = Vec3f(.5f - noise, .7f + noise, .8f);
		if (position[2] > -1.9f) color /= 5;		// Pour gérer deux matériaux : le fond et le visage!
		return reponse * color;
	};


protected:
	float m_diffuse;
	float m_alpha;
	Vec3f m_color;
	Vec3f m_F0;
	float m_alphacarre;
	WorleyNoise m_WorleyNoise_albedo;
	WorleyNoise m_WorleyNoise_roughness;
	WorleyNoise m_WorleyNoise_metallicness;
};


#endif