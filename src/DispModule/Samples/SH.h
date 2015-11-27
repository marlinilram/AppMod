//////////////////////////////////////////////////////////////////////////////////////////
//	SH.h
//	Functions to evaluate a spherical harmonic basis function at a given point
//	Downloaded from: www.paulsprojects.net
//	Created:	21st September 2003
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef SH_H
#define SH_H

#include "Eigen\Eigen"

//Evaluate an Associated Legendre Polynomial P(l, m) at x
double P(int l, int m, double x);

//Calculate the normalisation constant for an SH function
double K(int l, int m);

//Sample a spherical harmonic basis function Y(l, m) at a point on the unit sphere
double SH(int l, int m, double theta, double phi);

void SHRotationMatrix(Eigen::MatrixXf& mat_out, float sin_alph, float cos_alph);
void rotateSH(Eigen::Matrix3f& rot_mat, Eigen::VectorXf& coef_in, Eigen::VectorXf& coef_out);


//Calculate n!
int Factorial(int n);

#endif