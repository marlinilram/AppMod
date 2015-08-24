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




#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "tele_basicType.h"
#include <assert.h>

#include "tele2d.h"


double tele2d::energy_function( std::vector<double> X ){


	std::vector<std::vector<double2> > curves_backup = curves ;

	// transform the curve(s) specified by curve_group[]
	for( int gid =0; gid<curves_group.size(); ++gid ){

		std::vector<int> curves_id = curves_group[gid] ;

		// translation 
		for( int i = 0; i<curves_id.size(); ++i  ){
			int cvid = curves_id[i] ;
			for (int j=0; j<curves[cvid].size(); ++j){
				curves[cvid][j].x = curves_backup[cvid][j].x + X[gid*3] ;
				curves[cvid][j].y = curves_backup[cvid][j].y + X[gid*3+1] ;
			}
		}
		// rotation
		double2 rotation_center = curves[curves_id[0]].back() ;
		for( int i = 0; i<curves_id.size(); ++i  ){
			int cvid = curves_id[i] ;
			for( int j=0; j<curves[cvid].size(); ++j){

				curves[cvid][j].x -= rotation_center.x ;
				curves[cvid][j].y -= rotation_center.y ;
				double2 temp = curves[cvid][j] ;
				curves[cvid][j].x  = cos(X[gid*3+2]) * temp.x + sin( X[gid*3+2]) * temp.y ;
				curves[cvid][j].y  = -sin(X[gid*3+2]) * temp.x + cos( X[gid*3+2]) * temp.y ;
				curves[cvid][j].x += rotation_center.x ;
				curves[cvid][j].y += rotation_center.y ;

			}
		}
	}
	// compute penalties for each bridging curve

	if( updateVectorFieldWhenComputingEnegergy )	
		computeVectorField() ;
	computeOsculatingCircle() ;
	interpolateBetweenPairedCurves() ;

	double penalty = 0;
	for( int fcid = 0; fcid<bridging_curves.size(); ++fcid)
		penalty += get_penalty( bridging_curves[fcid], bridging_curves_endpoints[fcid].first, bridging_curves_endpoints[fcid].second ) ;

	curves = curves_backup ;

	return penalty ;
}