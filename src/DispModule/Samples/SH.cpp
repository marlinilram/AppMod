//////////////////////////////////////////////////////////////////////////////////////////
//	SH.cpp
//	Functions to evaluate a spherical harmonic basis function at a given point
//	Downloaded from: www.paulsprojects.net
//	Created:	21st September 2003
//  Modified:   11/28/2014 Lin Ma majcjc@gmail.com
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	
#include <math.h>
#include "SH.h"

//Evaluate an Associated Legendre Polynomial P(l, m) at x
double P(int l, int m, double x)
{
	//First generate the value of P(m, m) at x
	double pmm=1.0;

	if(m>0)
	{
		double sqrtOneMinusX2=sqrt(1.0-x*x);

		double fact=1.0;

		for(int i=1; i<=m; ++i)
		{
			pmm*=(-fact)*sqrtOneMinusX2;
			fact+=2.0;
		}
	}

	//If l==m, P(l, m)==P(m, m)
	if(l==m)
		return pmm;

	//Use rule 3 to calculate P(m+1, m) from P(m, m)
	double pmp1m=x*(2.0*m+1.0)*pmm;

	//If l==m+1, P(l, m)==P(m+1, m)
	if(l==m+1)
		return pmp1m;

	//Otherwise, l>m+1.
	//Iterate rule 1 to get the result
	double plm=0.0;

	for(int i=m+2; i<=l; ++i)
	{
		plm=((2.0*i-1.0)*x*pmp1m-(i+m-1.0)*pmm)/(i-m);
		pmm=pmp1m;
		pmp1m=plm;
	}

	return plm;
}



//Calculate the normalisation constant for an SH function
//No need to use |m| since SH always passes positive m
double K(int l, int m)
{
	double temp=((2.0*l+1.0)*Factorial(l-m))/((4.0*M_PI)*Factorial(l+m));
	
	return sqrt(temp);
}



//Sample a spherical harmonic basis function Y(l, m) at a point on the unit sphere
double SH(int l, int m, double theta, double phi)
{
	const double sqrt2=sqrt(2.0);

	if(m==0)
		return K(l, 0)*P(l, m, cos(theta));

	if(m>0)
		return sqrt2*K(l, m)*cos(m*phi)*P(l, m, cos(theta));

	//m<0
	return sqrt2*K(l,-m)*sin(-m*phi)*P(l, -m, cos(theta));
}



//Calculate n! (n>=0)
int Factorial(int n)
{
	if(n<=1)
		return 1;

	int result=n;

	while(--n > 1)
		result*=n;

	return result;
}

void SHRotationMatrix(Eigen::MatrixXf& mat_out, float sin_alph, float cos_alph)
{
  mat_out = Eigen::MatrixXf::Zero(9, 9);
  mat_out(0, 0) = mat_out(2, 2) = mat_out(6, 6) = 1;
  mat_out(1, 1) = mat_out(3, 3) = mat_out(5, 5) = mat_out(7, 7) = cos_alph;
  mat_out(1, 3) = mat_out(5, 7) = sin_alph;
  mat_out(3, 1) = mat_out(7, 5) = -sin_alph;
  mat_out(4, 4) = mat_out(8, 8) = 2 * cos_alph * cos_alph - 1;
  mat_out(4, 8) = 2 * sin_alph * cos_alph;
  mat_out(8, 4) = -2 * sin_alph * cos_alph;
}

void rotateSH(Eigen::Matrix3f& rot_mat, Eigen::VectorXf& coef_in, Eigen::VectorXf& coef_out)
{
  // only deal with 2nd order (3 band) coefficients now

  // first compute alpha, beta and gamma from rot_mat
  double cos_beta = rot_mat(2, 2);
  double sin_beta = sqrt(1 - cos_beta * cos_beta);
  double cos_alph = fabs(sin_beta) < 1e-4 ? rot_mat(1, 1) : (rot_mat(2, 0) / sin_beta);
  double sin_alph = fabs(sin_beta) < 1e-4 ? -rot_mat(1, 0) : (rot_mat(2, 1) / sin_beta);
  double cos_gamm = fabs(sin_beta) < 1e-4 ? 1 : (-rot_mat(0, 2) / sin_beta);
  double sin_gamm = fabs(sin_beta) < 1e-4 ? 0 : (rot_mat(1, 2) / sin_beta);

  Eigen::MatrixXf X_pos_90 = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf X_neg_90 = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf Z_alph = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf Z_beta = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf Z_gamm = Eigen::MatrixXf::Zero(9, 9);
  Eigen::MatrixXf Z_neg_90 = Eigen::MatrixXf::Zero(9, 9);

  X_neg_90(0, 0) = X_neg_90(1, 2) = X_neg_90(3, 3) = X_neg_90(4, 7) = 1;
  X_neg_90(2, 1) = X_neg_90(5, 5) = X_neg_90(7, 4) = -1;
  X_neg_90(6, 6) = -0.5; X_neg_90(8, 8) = 0.5;
  X_neg_90(6, 8) = X_neg_90(8, 6) = -sqrt(3) / 2;

  X_pos_90 = X_neg_90.transpose();
  SHRotationMatrix(Z_alph, sin_alph, cos_alph);
  SHRotationMatrix(Z_beta, sin_beta, cos_beta);
  SHRotationMatrix(Z_gamm, sin_gamm, cos_gamm);
  SHRotationMatrix(Z_neg_90, 1, 0);
  coef_out = Z_gamm * X_neg_90 * Z_beta * X_pos_90 * Z_alph * coef_in;
}

