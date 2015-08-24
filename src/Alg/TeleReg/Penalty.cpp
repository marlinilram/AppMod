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
#include <assert.h>
#include "tele2d.h"

double tele2d::get_penalty(std::vector<double2> samples, int endp1, int endp2 ){

	int i = endp1 ;
	int j = endp2;
	
	// reduce samples to make the sampling step equal to 1.0/resolution
	std::vector<double2> reduced_samples ;
	reduced_samples.push_back( samples[0] ) ;
	double length = 0;
	for( int id=1; id<samples.size(); ++id){
		length += ( samples[id] - samples[id-1] ).norm() ;
		if( length > 0.25 / resolution ){
			reduced_samples.push_back( samples[id] ) ;
			length = 0;
		}

	}


	double penalty  =  0;
	for( int id=1; id<reduced_samples.size(); ++id){
		// get x index of nearest vertex
		double x = reduced_samples[id].x * resolution - 0.5 ;
		int ix ;
		if( x-floor(x) < 0.5 ) ix = floor(x) ;
		else	ix = ceil( x ) ;
		// get y index of nearest vertex
		double y = reduced_samples[id].y * resolution - 0.5 ;
		int iy ;
		if( y-floor(y) < 0.5 ) iy = floor(y) ;
		else	iy = ceil( y ) ;

		/*
		// if any points in a curve are out of boundaries, return a large penaty value 
		if( ix > resolution-1) return 2.0 ;
		if( iy > resolution-1) return 2.0;
		if( ix < 0 ) return 2.0 ;
		if( iy < 0 ) return 2.0 ;
		*/

		if( ix>resolution-1 || iy > resolution-1 || ix < 0 || iy < 0){
			penalty+=2 ;
			continue;
		}

		// if the nearest vertex is a constrained vertex, then the penelty is Overlap_Penalty
		if( constrained_vertices_mark[ix][iy]  && 0){
			penalty += Overlap_Penalty ;
		}
		else{
			// get the direction vector of the nearest vertex
			double2 vec = vector_field[ ix + iy * resolution] ;

			// compute the direction vector of current sample
			double2 dir = reduced_samples[id] - reduced_samples[id-1] ;

			// compute penalty of this sample
			double costheta = (dir * vec) *(   1/(dir.norm()*vec.norm())   ) ;
			
			//std::cout << "costheta = " << costheta <<std::endl;

			if( costheta > 1 ){

				;
			}
			assert(1 - costheta*costheta >= 0 && 1 - costheta*costheta <= 1) ;

			//penalty += sqrt( 1 - costheta*costheta) ;
			penalty += 1 - costheta;



		}

		double2 c1 = osculatingCircles[ i ].first ;
		double2 c2 = osculatingCircles[ j ].first ;

		double r1 = osculatingCircles[ i ].second ;
		double r2 = osculatingCircles[ j ].second ;


		penalty += (1 - getScalarValue( osculatingCircles , correspondence,  reduced_samples[id] ))  * scalar_weight;

	}
	penalty /= reduced_samples.size() ;




	return penalty ;
}