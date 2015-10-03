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

#include <ctime>
#include "tele2d.h"
#include "nlopt.hpp"


double efunc(const std::vector<double> &x, std::vector<double> &grad, void* regInst){

	std::vector<double> X0 = x ;
	tele2d *tele = (tele2d*)regInst ;

	tele->updateVectorFieldWhenComputingEnegergy = true ;
	double fx = tele->energy_function(X0) ;
	tele->updateVectorFieldWhenComputingEnegergy = false ;

	grad.resize(x.size()) ;
	std::vector<double> cppX = X0 ;

  int resolution = std::min(((tele2d*)regInst)->resolution.x, ((tele2d*)regInst)->resolution.y);

	for( int i=0; i<x.size(); ++i ){
		std::vector<double> cppX2 = cppX;

		double step ;
		if( i%3 == 2 )
			step = energyGraAngStep*3.14/180.0 ;
		else
			step = energyGraTranStep / resolution ;


		cppX2[i] += step ;

		grad[i] = (((tele2d*)regInst)->energy_function( cppX2 ) - fx ) / step ;

	}




	//std::cout<< "f(";
	//for( int i=0; i<x.size()-1; ++i )
	//	std::cout<<x[i]<<"," ;
	//std::cout<<x.back()<<") = "<<fx <<std::endl;

	std::cout <<"f(x) = " <<fx <<std::endl;

	return  fx;
}



void tele2d::runRegister() {

	
	std::cout<<"begin registration"<<std::endl;
	unsigned time1 = clock() ;

	std::vector<double> x0(curves_group.size() *3, 0);

	int xn = x0.size() ;
	nlopt::opt opt(nlopt::LD_LBFGS, xn);

	std::vector<double> lb(xn);
	std::vector<double> ub(xn);
	for( int i=0; i<xn; ++i ){
		int innerId = i % 3 ;
		if( innerId <2 ){
			lb[i] = -0.5 ;
			ub[i] =  0.5;
		}else{
			lb[i] = -1.0 ;
			ub[i] = 1.0 ;
		}
	}

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective( efunc, this );


	opt.set_ftol_rel(0.001);

	double minf ;
	nlopt::result result ;

	try{
		result = opt.optimize(x0, minf);
	}catch (std::exception& e){  std::cerr << "exception caught: " << e.what() << '\n'; }

	std::cout << "result code: " << result <<std::endl;

	std::vector<double> X = x0;

	// transform the curves
	resCurves = curves ;
	for( int gid =0; gid<curves_group.size(); ++gid ){

		std::vector<int> curves_id = curves_group[gid] ;

		// translation 
		for( int i = 0; i<curves_id.size(); ++i  ){
			int cvid = curves_id[i] ;
			for (int j=0; j<resCurves[cvid].size(); ++j){
				resCurves[cvid][j].x = resCurves[cvid][j].x +  X[gid*3] ;
				resCurves[cvid][j].y = resCurves[cvid][j].y + X[gid*3+1] ;
			}
		}

		// rotation
		double2 rotation_center = double2( 0, 0 ) ;
		int count = 0;
		for( int i = 0; i<curves_id.size(); ++i  ){
			int cvid = curves_id[i] ;
			for( int j=0; j<resCurves[cvid].size(); ++j){
				rotation_center = rotation_center + resCurves[cvid][j] ;
				count ++ ;
			}
		}
		rotation_center = rotation_center * (1.0/count ) ;


		//std::cout<<"rotation_center = ( "<<rotation_center.x<<","<<rotation_center.y <<")"<<std::endl ; 

		//rotation_center = resCurves[curves_id[0]].back() ;

		for( int i = 0; i<curves_id.size(); ++i  ){
			int cvid = curves_id[i] ;
			for( int j=0; j<resCurves[cvid].size(); ++j){

				resCurves[cvid][j].x -= rotation_center.x ;
				resCurves[cvid][j].y -= rotation_center.y ;
				double2 temp = resCurves[cvid][j] ;
				resCurves[cvid][j].x  = cos(X[gid*3+2]) * temp.x + sin( X[gid*3+2]) * temp.y ;
				resCurves[cvid][j].y  = -sin(X[gid*3+2]) * temp.x + cos( X[gid*3+2]) * temp.y ;
				resCurves[cvid][j].x += rotation_center.x ;
				resCurves[cvid][j].y += rotation_center.y ;

			}
		}
	}


	std::cout << "registration time = " << (double)(clock()-time1)/CLOCKS_PER_SEC <<" s" <<std::endl ;
	std::cout<<"end registration"<<std::endl;


}