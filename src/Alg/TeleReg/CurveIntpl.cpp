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

int first_time_interpolate = 1 ;
double2 get_hermite_value( double2 p0, double2 p1, double2 tan0, double2 tan1, double t ){

	tan0.normalize() ;
	tan1.normalize() ;

	double dis = (p0-p1).norm();

	tan0 = tan0 * dis;
	tan1 = tan1 * dis ;



	double m[4][4] = { {2,-2,1,1},{-3,3,-2,-1},{0,0,1,0},{1,0,0,0} } ;

	double T[4] ;
	double temp[4] ;
	double2 result ;

	T[0] = t*t*t ;
	T[1] = t*t;
	T[2] = t ;
	T[3] = 1 ;
	for( int x = 0; x<4; ++x ){
		temp[x]  = T[0] * m[0][x] ;
		temp[x] += T[1] * m[1][x] ;
		temp[x] += T[2] * m[2][x] ;
		temp[x] += T[3] * m[3][x] ;
	}

	result.x = temp[0] * p0.x + temp[1] * p1.x + temp[2] * tan0.x + temp[3] * tan1.x ;
	result.y = temp[0] * p0.y + temp[1] * p1.y + temp[2] * tan0.y + temp[3] * tan1.y ;

	return result ;

}


std::vector<std::vector<int>> get_all_sequences ( std::vector<int> array_ ) {
	
	std::vector<std::vector<int>>  ss ;

	if( array_.size() <2 ){
		return ss ;
	
	}
	if( array_.size()==2 || array_.size() == 3 ){
		std::vector<int> x ;
		x.push_back(array_[0]);
		x.push_back(array_[1]);
		ss.push_back(x);
		return ss ;
	}

	for( int i= 1; i<array_.size(); i++ ){

		int a = array_[0] ;
		int b = array_[i] ;

		std::vector<int> array2 = array_ ;
		array2.erase(array2.begin()+i) ;
		array2.erase(array2.begin()) ;

		std::vector<std::vector<int>>  subsequence = get_all_sequences( array2 ) ;
		
		for( int j =0; j<subsequence.size(); ++j ){
			subsequence[j].push_back( a ) ;
			subsequence[j].push_back( b ) ;
			ss.push_back( subsequence[j] ) ;
		}

	}
		

	return ss ;

}





double tele2d::interpolateBetweenPairedCurves( ){

	//std::cout << "interpolateBetweenPairedCurves( ){";

	computeOsculatingCircle( ) ;

	// generate interpolated curves
	bridging_curves.clear() ;
	bridging_curves_endpoints.clear() ;

	for( int i =0; i<best_sequence.size()/2; ++ i){


		int endpid1 = best_sequence[i*2] ;
		int endpid2 = best_sequence[i*2+1] ;

		int curve1 = endpid1/2 ;
		int curve2 = endpid2/2 ;



		// endpoints
		const int endpid1_id = (endpid1%2 == 0) ? 0 : curves[curve1].size() -1;
		const int endpid2_id = (endpid2%2 == 0) ? 0 : curves[curve2].size() -1;


		double2 p0 = curves[curve1][endpid1_id] ;
		double2 p1 = curves[curve2][endpid2_id] ;
		double2 tan0, tan1 ;

		// slope of tangent
		if( endpid1_id == 0 )
			tan0 = curves[curve1][0] - curves[curve1][1] ;
		else
			tan0 = curves[curve1].back() - curves[curve1][curves[curve1].size()-2] ;

		if( endpid2_id == 0 )
			tan1 = curves[curve2][1] - curves[curve2][0] ; 
		else
			tan1 = curves[curve2][curves[curve2].size()-2] - curves[curve2].back() ;

		tan0.normalize();
		tan1.normalize();


		std::vector<double2>  nullcurve;
		bridging_curves.push_back(nullcurve);
		bridging_curves_endpoints.push_back( int2(endpid1, endpid2) );
		for( double t=0.0; t<=1.0; t+=0.01 )
			bridging_curves.back().push_back( get_hermite_value( p0, p1, tan0, tan1, t ) );

	}

	return 0 ;
}
