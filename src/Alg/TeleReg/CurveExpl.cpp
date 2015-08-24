/*
	Tele2d: This software implements the 2d tele-registration method in the paper,

	ARTICLE{Telereg2013
	title = {¡°Mind the Gap¡±: Tele-Registration for Structure-Driven Image Completion},
	author = {H. Huang and K.Yin and M. Gong and D. Lischinski and D. Cohen-Or and U. Ascher and B. Chen},
	journal = {ACM Transactions on Graphics (Proceedings of SIGGRAPH ASIA 2013)},
	volume = {32},
	issue = {6},
	pages = {174:1--174:10},
	year = {2013},
	}

	Copyright (C) <2014>  <Kangxue Yin - yinkangxue@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include<vector>  
#include<iostream>
#include "tele_basicType.h"

#include "tele2d.h"

void getBernsteinCoefficient( std::vector<double> &bc, int n   ){

	bc.resize(n);
	
	n=n-1;

	int j,k;
	for (k=0;k<=n;k++)
	{ //compute n! / (k!*(n-k)!)
		bc[k] = 1;
		for (j = n;j>=k+1;j--)
			bc[k] *=j;
		for (j = n-k;j>=2;j--)
			bc[k] /= j;
	}



}


#define DIRIVATIVE_POSITION  2

void tele2d::computeOsculatingCircle(  ){
	
	std::vector<std::vector<double2>> bezierCurves ; 

	bezierCurves.clear();
	bezierCurves.resize(curves.size());
	osculatingCircles.clear() ;
	for( int i=0; i<curves.size(); ++i ){

			// Coefficients of Bernstein Polynomials
			std::vector<double> bc ;

			//// reduce curves[i] to 5 or even less samples
			//std::vector<double2> points;
			//int step = curves[i].size() / 5 ;
			
			const int n = curves[i].size() ;

			if( n< 5 )
				continue ;

			getBernsteinCoefficient(bc, n ) ;

			// store the Bezier curve
			bezierCurves[i].clear();
			for( double u=0; u<=1.0; u+=0.01 ){
				double2 p ;
				for( int k = 0; k<n; ++k ){
					p.x += bc[k] * pow(u, k) * pow( 1-u, n-1-k) * curves[i][k].x ;
					p.y += bc[k] * pow(u, k) * pow( 1-u, n-1-k) * curves[i][k].y ;
				}	

				bezierCurves[i].push_back( p ) ;
			}

			//  p(u) = SIGMA{ p[k] * bc[k] * u^k * (1-u)^(d-k) } ,  d = n-1
			//  p'(u) = SIGMA{ p[k] * bc[k] * ( k * u^(k-1) * (1-u)^(d-k) + (d-k) * (1-u)^(d-k-1) * u^k },  d = n-1
			//  p''(u) =  SIGMA{ p[k] * bc[k] * (   k * (k-1) * u^(k-2) * (1-u)^(d-k) + k * (d-k) * u^(k-1) * (1-u)^(d-k-1) 
			//									  + (d-k) * (d-k-1) * (1-u)^(d-k-2) * u^k + (d-k) * k * (1-u)^(d-k-1) * u^(k-1) ) } ,  d = n-1

			
			
			// compute the derivative at the the point u = 2/(double)n
			double x, y, dx, dy, ddx, ddy ;
			x = y = dx = dy = ddx = ddy = 0.0 ;
			
			 double u = DIRIVATIVE_POSITION/(double)n;
			//std::cout << "u=" << u <<std::endl ;
			for( int k = 0; k< n; ++k){
				const int d = n - 1;
				

				x += curves[i][k].x * bc[k] * pow( u, k) * pow(1-u, d-k ) ;
				y += curves[i][k].y * bc[k] * pow( u, k) * pow(1-u, d-k ) ;
				
				//dx += curves[i][k].x * bc[k] * ( k * pow( u, k-1) * pow( 1-u, d-k) + (d-k) * pow( 1-u, d-k-1) * pow( u, k ) );
				//dy += curves[i][k].y * bc[k] * ( k * pow( u, k-1) * pow( 1-u, d-k) + (d-k) * pow( 1-u, d-k-1) * pow( u, k ) );
				//

				//ddx += curves[i][k].x * bc[k] * ( k * (k-1) * pow( u, k-2) * pow( 1-u, d-k) + k * (d-k) * pow( u, k-1) * pow( 1-u, d-k-1 ) 
				//		+  (d-k) * (d-k-1) * pow(1-u, d-k-2) * pow(u,k) + (d-k) * k * pow( 1-u, d-k-1) * pow( u, k-1) );

				//ddy += curves[i][k].y * bc[k] * ( k * (k-1) * pow( u, k-2) * pow( 1-u, d-k) + k * (d-k) * pow( u, k-1) * pow( 1-u, d-k-1 ) 
				//		+  (d-k) * (d-k-1) * pow(1-u, d-k-2) * pow(u,k) + (d-k) * k * pow( 1-u, d-k-1) * pow( u, k-1) );


			}
			

			// compute osculating circle of the tail
			std::vector<double> bc1 ;
			getBernsteinCoefficient(bc1, n-1 ) ;
			for( int k=0; k<n-1; ++k ){
				const int d = n - 2;
				dx += bc1[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( curves[i][k+1].x - curves[i][k].x  ) ; 
				dy += bc1[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( curves[i][k+1].y - curves[i][k].y  ) ; 

			}
			std::vector<double> bc2 ;
			getBernsteinCoefficient(bc2, n-2 ) ;
			for( int k=0; k<n-2; ++k ){
				const int d = n - 3;
				ddx += bc2[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( d+2) * ( curves[i][k+2].x + curves[i][k].x - 2 * curves[i][k+1].x  ) ; 
				ddy += bc2[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( d+2) * ( curves[i][k+2].y + curves[i][k].y - 2 * curves[i][k+1].y  ) ; 

			}


			double curvature = ( dx * ddy - dy * ddx ) / pow( dx * dx + dy * dy , 1.5 ) ;
			double radius = 1.0 / curvature ;
			double2  center ;
			center.x = x - radius * dy / pow( dx*dx + dy*dy, 0.5 ) ;
			center.y = y + radius * dx / pow( dx*dx + dy*dy, 0.5 ) ;


			x = curves[i][0].x ;
			y = curves[i][0].y ;


			Circle cir ;
			cir.first = center ;
			cir.second = radius ;
			cir.curvature = curvature ;
			double2 tp = cir.touchpoint = double2( x, y ) ;

			double degree = 3.1415926 * 90/180 ;
			if( curvature>0 ) degree = -degree ;
			cir.tangent =  double2( (center-tp).x * cos(degree) +(center-tp).y * sin(degree), - (center-tp).x * sin(degree) +(center-tp).y * cos(degree) ) ;

			osculatingCircles.push_back(cir ) ;
	

			// compute the derivative at the the point u = (n-2)/n

			x = y = dx = dy = ddx = ddy = 0.0 ;
			u = (n-DIRIVATIVE_POSITION)/(double)n;
			for( int k = 0; k< n; ++k){
				const int d = n - 1;

				x += curves[i][k].x * bc[k] * pow( u, k) * pow(1-u, d-k ) ;
				y += curves[i][k].y * bc[k] * pow( u, k) * pow(1-u, d-k ) ;

			}


			getBernsteinCoefficient(bc1, n-1 ) ;
			for( int k=0; k<n-1; ++k ){
				const int d = n - 2;
				dx += bc1[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( curves[i][k+1].x - curves[i][k].x  ) ; 
				dy += bc1[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( curves[i][k+1].y - curves[i][k].y  ) ; 

			}

			getBernsteinCoefficient(bc2, n-2 ) ;
			for( int k=0; k<n-2; ++k ){
				const int d = n - 3;
				ddx += bc2[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( d+2) * ( curves[i][k+2].x + curves[i][k].x - 2 * curves[i][k+1].x  ) ; 
				ddy += bc2[k] * pow(u, k ) * pow( 1-u, d - k ) * (d+1) * ( d+2) * ( curves[i][k+2].y + curves[i][k].y - 2 * curves[i][k+1].y  ) ; 

			}

			// compute osculating circle of the head
			curvature = ( dx * ddy - dy * ddx ) / pow( dx * dx + dy * dy , 1.5 ) ;
			radius = 1.0 / curvature ;

			x = curves[i].back().x ;
			y = curves[i].back().y ;


			center.x = x - radius * dy / pow( dx*dx + dy*dy, 0.5 ) ;
			center.y = y + radius * dx / pow( dx*dx + dy*dy, 0.5 ) ;

			cir.first = center ;
			cir.second = radius ;
			cir.curvature = curvature ;
			 tp = cir.touchpoint = double2( x, y ) ;

			 degree = 3.1415926 * 90/180 ;
			if( curvature<0 )
				degree = -degree ;
			 cir.tangent =  double2( (center-tp).x * cos(degree) +(center-tp).y * sin(degree), - (center-tp).x * sin(degree) +(center-tp).y * cos(degree) ) ;

			osculatingCircles.push_back(cir ) ;
	}

			      

}