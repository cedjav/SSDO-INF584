#pragma once

#ifndef Imageh
#define Imageh

#include <iostream>
#include <fstream>
#include <vector>


class Image {	// Use a one dimension array
public:

	Image() :m_nbcol(400), m_nbrow(400) {
		Vec3f vide = Vec3f(0, 0, 0);
		for (int i = 0; i < m_nbcol * m_nbrow; i++)
			m_pixel.push_back(vide);
	}

	Image(size_t colomns, size_t rows) :m_nbcol(colomns), m_nbrow(rows) {
		Vec3f vide=Vec3f(0,0,0);
		for (int i = 0; i < m_nbcol * m_nbrow; i++)
			m_pixel.push_back(vide);
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

	Vec3f get_pixel(int col, int row) {
		if (row < 0 || row >= m_nbrow || col < 0 || col >= m_nbcol) {
			perror("Depassement de la taille de l'image!!!!!");
			exit(1);
		}
		return m_pixel[row + col * m_nbrow];
	}

	void set_pixel(int col, int row, Vec3f color) {
		if (row < 0 || row >= m_nbrow || col < 0 || col >= m_nbcol) {
			perror("Depassement de la taille de l'image xxx");
			exit(1);
		}
		m_pixel[row + col * m_nbrow] = color;
	}

	float valeur_min(float a, float b, float c) {
		if (a < b) {
			if (a < c) return a;
			else return c;
		}
		if (b < c) return  b;
		return c;
	}

	void Image::save_to_PPM_NoSaturation(const char* filename) {
		std::ofstream out(filename, std::ofstream::out);

		out << "P3" << std::endl;
		out << m_nbcol << " " << m_nbrow << std::endl;
		out << "255" << std::endl;
		for (int i = 0; i < m_nbrow; i++) {
			for (int j = 0; j < m_nbcol; j++) {
				Vec3f Pixel = m_pixel[i + j * m_nbrow];
				int red = Pixel[0]* 255;
				int green = Pixel[1] * 255;
				int blue = Pixel[2] * 255;
				if (red > 255) red = 255;
				if(red < 0) red = 0;
				if (green > 255) green = 255;
				if (green < 0) green = 0;
				if (blue > 255) blue = 255;	  
				if (blue < 0) blue = 0;

				out << red << " " << green << " " << blue << std::endl;
			}
			out << std::endl;
		}
		out.close();
	}

	void Image::save_to_PPM(const char* filename) {
		std::ofstream out(filename, std::ofstream::out);

		out << "P3" << std::endl;
		out << m_nbcol << " " << m_nbrow << std::endl;
		out << "255" << std::endl;
		for (int i = 0; i < m_nbrow; i++) {
			for (int j = 0; j < m_nbcol; j++) {
				Vec3f Pixel = m_pixel[i + j * m_nbrow];
				// Pour éviter qu'un rouge saturé ne devienne blanc...
				float val_max = -valeur_min(-Pixel[0], -Pixel[1], -Pixel[2]);
				if (val_max < 1) val_max = 1;
				int red = Pixel[0] / val_max * 255;
				int green = Pixel[1] / val_max * 255;
				int blue = Pixel[2] / val_max * 255;
				if (red > 255) red = 255;
				if (red < 0) red = 0;
				if (green > 255) green = 255;
				if (green < 0) green = 0;
				if (blue > 255) blue = 255;
				if (blue < 0) blue = 0;

				out << red << " " << green << " " << blue << std::endl;
			}
			out << std::endl;
		}
		out.close();
	}

	void Image::fill_Background(Vec3f haut, Vec3f bas) {
		for (int i = 0; i < m_nbrow; i++) {
			Vec3f color=(haut*(m_nbrow - i - 1) + bas* i) / (m_nbrow - 1);
			for (int j = 0; j < m_nbcol; j++)
				set_pixel(j, i, color);
		}
	}

	// Masque de Gauss 3x3
	void Image::Blur(Image& output) {
		Vec3f Pixel;

		for (int i = 1; i < m_nbrow-1; i++) {

			for (int j = 1; j < m_nbcol-1; j++) {

				Pixel = get_pixel(i - 1, j - 1) * 1
					+ get_pixel(i - 1, j) * 2
					+ get_pixel(i - 1, j + 1) * 1
					+ get_pixel(i, j - 1) * 2
					+ get_pixel(i, j) * 4
					+ get_pixel(i, j + 1) * 2
					+ get_pixel(i + 1, j - 1) * 1
					+ get_pixel(i + 1, j) * 2
					+ get_pixel(i + 1, j + 1) * 1;
				output.set_pixel(i, j, Pixel/16);
			}

			// Traitement de j=0
			Pixel = get_pixel(i - 1 , 0) * 2
				+ get_pixel(i - 1 , 1 ) * 1
				+ get_pixel(i , 0 ) * 4
				+ get_pixel(i , 1 ) * 2
				+ get_pixel(i + 1 , 0 ) * 2
				+ get_pixel(i + 1 , 1 ) * 1;
			output.set_pixel(i, 0, Pixel / 12);

			// Traitement de j=m_nbcol-1
			Pixel = get_pixel(i - 1, m_nbcol - 2 ) * 1
				+ get_pixel(i - 1 , m_nbcol - 1 ) * 2
				+ get_pixel(i , m_nbcol - 2 )* 2
				+ get_pixel(i , m_nbcol - 1 ) * 4
				+ get_pixel(i + 1 , m_nbcol - 2 ) * 1
				+ get_pixel(i + 1 , m_nbcol - 1 ) * 2;
			output.set_pixel(i, m_nbcol - 1, Pixel / 12);
		}
		for (int j = 1; j < m_nbcol - 1; j++) {
			// Traitement de i=0
			Pixel = get_pixel(0, j - 1) * 2
				+ get_pixel(0, j) * 4
				+ get_pixel(0, j + 1) * 2
				+ get_pixel(1, j - 1) * 1
				+ get_pixel(1, j) * 2
				+ get_pixel(1, j + 1) * 1;
			output.set_pixel(0, j, Pixel / 12);

			// Traitement de i=m_nbrow-1
			Pixel = get_pixel(m_nbrow - 2, j - 1) * 1
				+ get_pixel(m_nbrow - 2, j) * 2
				+ get_pixel(m_nbrow - 2, j + 1) * 1
				+ get_pixel(m_nbrow - 1, j - 1) * 2
				+ get_pixel(m_nbrow - 1, j) * 4
				+ get_pixel(m_nbrow - 1, j + 1) * 2;
			output.set_pixel(m_nbrow - 1, j, Pixel / 12);
		}
		// Traitement des coins
		// i=0 et j=0
		Pixel = get_pixel(0, 0) * 4
			+ get_pixel(0, 1) * 2
			+ get_pixel(1, 0) * 2
			+ get_pixel(1, 1) * 1;
		output.set_pixel(0,0, Pixel / 9);

		// i=m_nbrow-1 et  j=0
		Pixel = get_pixel(m_nbrow - 2, 0) * 2
			+ get_pixel(m_nbrow - 2, 1) * 1
			+ get_pixel(m_nbrow - 1, 0) * 4
			+ get_pixel(m_nbrow - 1, 1) * 2;
		output.set_pixel(m_nbrow - 1,0, Pixel / 9);

		// i=0 et  j=m_nbcol-1
		Pixel = get_pixel(0,m_nbcol - 2) * 2
			+ get_pixel(0,m_nbcol -1) * 4
			+ get_pixel(1, m_nbcol - 2)  * 1
			+ get_pixel(1, m_nbcol -1) * 2;
		output.set_pixel(0, m_nbcol - 1, Pixel / 9);

		// i=m_nbrow-1 et  j=m_nbcol-1
		Pixel = get_pixel(m_nbrow - 2, m_nbcol - 2) * 1
			+ get_pixel(m_nbrow - 2, m_nbcol - 1) * 2
			+ get_pixel(m_nbrow - 1, m_nbcol - 2) * 2
			+ get_pixel(m_nbrow - 1, m_nbcol - 1) * 4;
		output.set_pixel(m_nbrow - 1, m_nbcol - 1, Pixel / 9);
	}


private:
	std::vector<Vec3f> m_pixel;
	size_t m_nbcol;
	size_t m_nbrow;
};



#endif