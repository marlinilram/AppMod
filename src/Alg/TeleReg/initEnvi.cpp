
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

#include "tele2d.h"


void tele2d::init( CURVES curves_, std::vector<std::vector<int>> group, std::vector<int2> endps  ){


	// normalize curves
	curves = curves_ ;
	double2 minCorner ( 1e10, 1e10);
	double2 maxCorner (-1e10, -1e10);
	for( int i=0; i<curves.size(); ++i )
		for( int j=0; j<curves[i].size(); ++j ){
			if( curves[i][j].x < minCorner.x  ) minCorner.x =  curves[i][j].x ;
			if( curves[i][j].y < minCorner.y  ) minCorner.y =  curves[i][j].y ;
			if( curves[i][j].x > maxCorner.x  ) maxCorner.x =  curves[i][j].x ;
			if( curves[i][j].y > maxCorner.y  ) maxCorner.y =  curves[i][j].y ;
		}

	double2 cter = (minCorner+maxCorner) / 2 ;
	normalize_translate =  double2(0.5, 0.5 ) - cter ;
	normalize_scale = 0.6 / std::max( (maxCorner-minCorner).x, (maxCorner-minCorner).y  ) ;


	for( int i=0; i<curves.size(); ++i )
		for( int j=0; j<curves[i].size(); ++j ){
			curves[i][j] = ( curves[i][j] + normalize_translate - double2(0.5, 0.5 ) ) * normalize_scale + double2(0.5, 0.5 );
		}

	initialCurves = curves ;


	curves_group = group ;
	endpoints = endps ;

	scalar_field_4cc = false ;
	findCorres_byScalarField() ;

	correctDirs() ;

}

void tele2d::init( CURVES curves_, std::vector<std::vector<int>> group, std::vector<int2> endps, std::vector<int2> corres  ){


	curves = curves_ ;

	// normalize curves
	double2 minCorner ( 1e10, 1e10);
	double2 maxCorner (-1e10, -1e10);
	for( int i=0; i<curves.size(); ++i )
		for( int j=0; j<curves[i].size(); ++j ){
			if( curves[i][j].x < minCorner.x  ) minCorner.x =  curves[i][j].x ;
			if( curves[i][j].y < minCorner.y  ) minCorner.y =  curves[i][j].y ;
			if( curves[i][j].x > maxCorner.x  ) maxCorner.x =  curves[i][j].x ;
			if( curves[i][j].y > maxCorner.y  ) maxCorner.y =  curves[i][j].y ;
		}

	double2 cter = (minCorner+maxCorner) / 2 ;
	normalize_translate =  double2(0.5, 0.5 ) - cter ;
	normalize_scale = 0.6 / std::max( (maxCorner-minCorner).x, (maxCorner-minCorner).y  ) ;


	for( int i=0; i<curves.size(); ++i )
		for( int j=0; j<curves[i].size(); ++j ){
			curves[i][j] = ( curves[i][j] + normalize_translate - double2(0.5, 0.5 ) ) * normalize_scale + double2(0.5, 0.5 );
		}

	initialCurves = curves ;



	curves_group = group ;
	endpoints = endps ;

	correspondence = corres ;
	best_sequence.clear() ;
	for( int i=0; i<correspondence.size(); ++i ){
		best_sequence.push_back( correspondence[i].x ) ;
		best_sequence.push_back( correspondence[i].y ) ;
	}


	correctDirs() ;
}