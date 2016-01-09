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

#ifndef _tele2d_h_
#define _tele2d_h_


#include "tele_basicType.h"

#include <fstream>

typedef std::vector<std::vector<double2>> CURVES ;
typedef             std::vector<double2>  CURVE ;


//#define  _use_openmp_


#ifdef TELEREGLIB_EXPORTS
class  __declspec(dllexport) tele2d{
#else
class tele2d{
#endif


public:
	
	tele2d ( ) {
		resolution = 100 ;
		gussian_h = 0.02 ;
		scalar_weight = 1 ;
	}
	tele2d( int res, double h, double w ) {
		resolution = res ;
		gussian_h = h ;
		scalar_weight = w ;
	}

 
	void init( CURVES curves, std::vector<std::vector<int>> group, std::vector<int2> endps  );
	void init( CURVES curves, std::vector<std::vector<int>> group, std::vector<int2> endps, std::vector<int2> corres  );
	void runRegister() ;
	void outputResCurves( std::string fname, bool link = false ) ;

  bool load_Curves( std::string fname, 	CURVES &curves , std::vector<std::vector<int>> &curves_group, std::vector<int2> &endpoints);


	/* -------------------------------------------------------------------------------------------------------------------------

	If you are just a user rather than a developer of the lib, the interfaces above are enough for you.

	-------------------------------------------------------------------------------------------------------------------------*/



	void setResultField(){  // only for visualization
		std::vector<std::vector<double2>> curves_backup = curves ;
		curves = resCurves ;
		computeVectorField() ;
		interpolateBetweenPairedCurves() ;
		computeOsculatingCircle(  ) ;
		getDis() ;
		curves = curves_backup ;
	}

	void setInputField(){ // only for visualization
		std::vector<std::vector<double2>> curves_backup = curves ;
		curves = initialCurves ;
		//computeVectorField() ;
		//interpolateBetweenPairedCurves() ;
		//computeOsculatingCircle(  ) ;
		//getDis() ;
		curves = curves_backup ;
	}




	void findCorres_byScalarField( ) ;
	void correctDirs() ;
	void computeVectorField();
	void computeOsculatingCircle( ) ;
	double interpolateBetweenPairedCurves( ) ;
	double energy_function( std::vector<double> X );
	bool updateVectorFieldWhenComputingEnegergy ;
	double get_penalty(std::vector<double2> samples, int endp1, int endp2 ) ;

  static double2 get_hermite_value( double2 p0, double2 p1, double2 tan0, double2 tan1, double t );
	
	double getScalarValue_4cc( std::vector<Circle> oscircle , std::vector<int2> endpoints,  double2 p , bool consider_all = false) ; 
	double distance_to_arc(Circle cir, double2 p, int head_or_tail ) ;
	double getScalarValue( std::vector<Circle> &oscircle , std::vector<int2> &corres, double2 p ) ;
	double disInField( std::vector<Circle> &osculatingCircles, std::vector<int2> &endpoints, int pid1, int pid2) ;
	void getDis();           // only for visualization


	int resolution; // resolution of vector field
	std::vector<std::vector<double2>> initialCurves ;	// the input curves
	std::vector<std::vector<double2>> curves ;			 // the input curves for operating
	std::vector<std::vector<double2>> resCurves ;	    // curves after registration
	std::vector<std::vector<double2>> bridging_curves ;	// curves that bridge 2 curves in "curves" 
	std::vector<int2> bridging_curves_endpoints ;		// index of endpoints of curves in "curves" that bridging_curves bridge
	std::vector<std::vector<int>> curves_group;			// each group if curves is a set of curves that lie on the same image piece
	std::vector<double2>  vector_field;					// normalized ambient vector field start from left bottom stored as (x,y) not (row,col)
	std::vector<std::vector<int>> constrained_vertices_mark ;  // where in the field is constrained by the input curves
	std::vector<int2> endpoints ;			// the endpoints selected to be bridged
	std::vector<Circle> osculatingCircles ;			// osculating circles at ends of curves
	double dis[800][800] ;						// scalar field. Only for visulization
	double gussian_h ;
	double scalar_weight ;
	std::vector<int2> correspondence ;
	std::vector<int> best_sequence ;

	bool scalar_field_4cc ;                          // true if scalar field is the field that used for computing correspondence


	double2 normalize_translate ;
	double normalize_scale ;


};



#endif