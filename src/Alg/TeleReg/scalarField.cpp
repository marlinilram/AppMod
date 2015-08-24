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

#include "tele_basicType.h"
#include <stdlib.h>

#include "tele2d.h"
const double h1 = 0.02 ;
const double h2 = 0.03 ;


double tele2d::distance_to_arc(Circle cir, double2 p, int head_or_tail ) {

	//return fabs((p-cir.first).norm() - fabs(cir.second));

	double2 c = cir.first ;
	double  r = cir.second ;
	double2 tp = cir.touchpoint ;
	double2 dest ;

	if( head_or_tail != 0 ) {  //head

		if( r < 0 ){	// the circle is on the right
			// rotate the endpoint 90 degree clockwise about the center of circle
			dest = c + double2( ( tp-c).y,  -( tp-c).x ) ;
		} else{// the circle is on the left
			// rotate the endpoint 90 degree anticlockwise about the center of circle
			dest = c + double2( -( tp-c).y,  ( tp-c).x ) ;
		}

	}else{ // tail

		if( r < 0 ){	// the circle is on the right
			// rotate the endpoint 90 degree clockwise about the center of circle
			dest = c + double2( -( tp-c).y,  ( tp-c).x ) ;
		} else{// the circle is on the left
			// rotate the endpoint 90 degree anticlockwise about the center of circle
			dest = c + double2( ( tp-c).y,  -( tp-c).x ) ;
		}

	}

	// test if p is in the first quadrant
	if( (tp-c) * (p -c) > 0 && (dest-c)*(p-c) > 0 )
		return abs( (p-c).norm() - abs(r) )  ;

	else

		return std::min( (p-tp).norm(), (p-dest).norm() )  ;


}


double tele2d::getScalarValue_4cc( std::vector<Circle> oscircle , 	
						std::vector<int2> endpoints, // this vector determines whether an endpoint will be consider
						double2 p , bool consider_all){
/* get scalar value for calculating correspondences */



	std::vector<bool> consider( oscircle.size()) ;
	for( int i=0; i<consider.size(); ++i) 
		consider[i] = false ;
	for( int i=0; i<endpoints.size();  ++i){
		if( endpoints[i][0] ) 
			consider[i*2] = true ;
		if( endpoints[i][1] )
			consider[i*2+1] = true ;
	}

	double scaValue = 0;
	for( int i=0; i<consider.size(); ++i ){
		if( !consider[i])
			continue ;

		double dis = distance_to_arc( oscircle[i], p, i%2 ) ;
		double dis_2p = ( p-oscircle[i].touchpoint ).norm() ;

			scaValue += pow( 2.71828, -dis*dis/(h1*h1) ) * pow( 2.71828, -dis_2p*dis_2p/(h2*h2) ) ;
	}
	
	return scaValue ;

}

double tele2d::getScalarValue( std::vector<Circle> &oscircle , std::vector<int2> &corres, double2 p ){


	double scalar = 0.0 ;
	for( int i=0; i<corres.size(); ++i ){

		double dis1 = distance_to_arc( oscircle[ corres[i].x ], p, corres[i].x%2 ) ;
		double dis2 = distance_to_arc( oscircle[ corres[i].y ], p, corres[i].y%2 ) ;

		scalar += pow( 2.71828, -dis1*dis1/(gussian_h*gussian_h) ) * pow( 2.71828, -dis2*dis2/(gussian_h*gussian_h) )  ;
		//scalar += pow( 2.71828, -dis1/(h3) ) * pow( 2.71828, -dis2/(h3) )  ;

	}

	return scalar ;

}



void tele2d::getDis(){


	//std::cout<<"getDis: scalar_field_4cc = "<<scalar_field_4cc<<std::endl;

	computeOsculatingCircle() ;

	for( int i =0; i<800 ; ++i )
		for( int j=0; j<800; ++j )
			dis[i][j] = 1.0 ;


	for( int i =0; i<800 ; ++i ){
		for( int j=0; j<800; ++j ){

			double2 p( i/800.0, j/800.0 ) ;
			if( scalar_field_4cc )
				dis[i][j] = getScalarValue_4cc( osculatingCircles, endpoints, p) ;
			else
				dis[i][j] = getScalarValue( osculatingCircles, correspondence, p) ;


		}

	}

	// scale dismap to [0,1]
	double maxDis = 0;
	for( int i=0;i<800;++i) for( int j=0; j<800; ++j )
		if( dis[i][j] > maxDis) maxDis = dis[i][j] ;

	double minDis = 1000;
	for( int i=0;i<800;++i) for( int j=0; j<800; ++j )
		if( dis[i][j] < minDis) minDis = dis[i][j] ;


	for( int i=0;i<800;++i) for( int j=0; j<800; ++j ){
	
		//if(scalar_field_4cc)
		//	dis[i][j] = (1 - (dis[i][j]/ 2) )/2 ;
		//else
		if( 0 ) 
				dis[i][j] = 1.0 ;
		else
			dis[i][j] = 1- ( dis[i][j] - minDis) / (maxDis-minDis) ;
	}




}


