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
#include <iostream>

#include "tele2d.h"





double2 get_hermite_value( double2 p0, double2 p1, double2 tan0, double2 tan1, double t ) ;

const double openDegree = 3.14 * 45/180; ;




double tele2d::disInField( std::vector<Circle> &osculatingCircles, std::vector<int2> &endpoints, int pid1, int pid2){

	if(pid1 == pid2 )
		return 0.0;

	double2 Pi = osculatingCircles[pid1].touchpoint ;
	double2 Pj = osculatingCircles[pid2].touchpoint ;
	double2 Ti = osculatingCircles[pid1].tangent  ;
	double2 Tj = osculatingCircles[pid2].tangent ;
	double2 PiPj = Pj - Pi ;
	Ti.normalize() ;
	Tj.normalize() ;
	Ti = Ti * PiPj.norm() ;
	Tj = Tj * PiPj.norm() ;
	Tj = -Tj ;

	double step = 0.01 ;

	double length =0;
	std::vector<double2> curve ;
	for( double t=0; t<=1.0; t=t+0.01 ){
		double2 p = get_hermite_value(Pi, Pj, Ti, Tj, t ) ;
		curve.push_back(p ) ;
	}
	double2 lastPoint = curve[0] ;
	for( int i=1; i<curve.size(); ++i ){
		if( ( lastPoint - curve[i]).norm() >step   ){
			length += 1000- getScalarValue_4cc(osculatingCircles, endpoints, curve[i], true) ;
			lastPoint =  curve[i] ;
		}
	}

	return length ;

}


void tele2d::findCorres_byScalarField()  {
													 
		
		computeOsculatingCircle() ;



		int N = curves.size() ;

		std::vector<int> groups ;
		for( int i=0;i<N; ++i ){
			for( int j=0; j<curves_group.size(); ++j )
				for( int k=0; k<curves_group[j].size(); ++k)
					if( curves_group[j][k] == i)
						groups.push_back(j);
		}

		if( groups.size() != N){
			std::cerr << "groups.size() != N " << std::endl;
			exit(1);
		}


		//std::vector<int2> corres ;

		std::vector< std::vector<int> > neighss(N*2) ;
		std::vector< std::vector<double> > neighss_dis(N*2) ;

		// find all edges
		for( int i=0; i<N*2; ++i ){
			
			//double maxscalar = 0.0 ;
			//double maxscalar_j = 0;

			double minlength = 1.0e10 ;
			double min_j = 0 ;
			
			// donot consider invalid endpoints
			if(  ! endpoints[i/2][i%2]   )
				continue ;

			for( int j=0; j<N*2; ++j ){

				// ignore endpoints in the same group
				if( groups[i/2] == groups[j/2] )
					continue ;
				// ignore invalid endpoints
				if( ! endpoints[i/2][i%2]  || ! endpoints[j/2][j%2] )
					continue ;


				double length = disInField( osculatingCircles, endpoints, i, j);

				if( length < minlength  ){
					minlength = length ;
					min_j = j ;
				}

				std::cout<<"<"<<i<<","<<j<<"> = "<< length <<std::endl;			
			}
			//corres.push_back( int2(i, min_j) ) ;
			neighss[min_j].push_back( i) ;
			neighss[i].push_back( min_j) ;

			neighss_dis[min_j].push_back( minlength) ;
			neighss_dis[i].push_back( minlength) ;

			std::cout<<std::endl;

		}

		// delete redundency
		for(  int i=0; i<N*2; ++i){
			if( neighss[i].size() < 2)
				continue ;
			for( int j=0; j<neighss[i].size(); ++j  ){
				for( int k=j+1; k<neighss[i].size(); ++k  ){
					if( neighss[i][j] == neighss[i][k] ){
						neighss[i].erase(neighss[i].begin() + k) ;
						neighss_dis[i].erase(neighss_dis[i].begin() + k) ;
						k-- ;
					}
				}
			}

		}





		//---------------------------------------------------------------------------




		// eliminate ambiguity
		std::vector<int> neigh ;
		for(  int i=0; i<N*2; ++i){
			if( neighss[i].size() == 0 ){
				neigh.push_back(-1) ;
				continue ;
			}
			if( neighss[i].size() == 1 ){
				neigh.push_back( neighss[i][0]) ;
				continue ;
			}

			std::vector<double> diff ;
			for( int j=0; j< neighss[i].size(); ++j ){
				//diff.push_back( 	colorDif(fragments, curves_intcor, i, neighss[i][j] )  );

				// This version do not assume there is input image, so we use the distance of endpoint instead instead
				diff.push_back( neighss_dis[i][j] );
			}

			// delete all but the one with most similar color
			double min_diff = 1.0e10 ;
			int minId = 0;
			for( int j=0; j<neighss[i].size(); ++j ){
				if( diff[j] < min_diff ){ min_diff = diff[j];  minId = j; }
			}
			neigh.push_back(neighss[i][minId] ) ;

		}


		// construct correspondence
		std::vector<int2> corres ;
		for( int i=0; i<2*N; ++i )
			if(neigh[i]!=-1)
				corres.push_back( int2(i, neigh[i])) ;

		// delete non-double
		for( int i=0; i<corres.size(); ++i) {

			bool isDouble = false ;

			for( int j=0; j<corres.size(); ++j){
				if( i==j) 
					continue ;
				if(	 corres[i][0] == corres[j][1] &&  corres[i][1] == corres[j][0] ) 
					isDouble = true ;
			}

			if( !isDouble ){
				corres.erase( corres.begin()+i) ;
				i--;
			}
		}

		// delete redundancy
		for( int i=0; i<corres.size(); ++i) for( int j=i+1; j<corres.size(); ++j){
			if(		( corres[i][0] == corres[j][0] &&  corres[i][1] == corres[j][1] )
				||  ( corres[i][0] == corres[j][1] &&  corres[i][1] == corres[j][0] ) 
			) 
			{
				corres.erase(corres.begin()+j) ;
				j-- ;
			}
		}


		// update correspondence
		best_sequence.clear() ;
		for( int i=0; i<corres.size(); ++i ){
			best_sequence.push_back( corres[i][0] ) ;
			best_sequence.push_back( corres[i][1] ) ;
		}



		// update validity of endpoints 
		for( int i=0; i<N; ++i ){
			bool valid0 = false ;
			bool valid1 = false ;
			for( int id =0; id<best_sequence.size(); ++id ){
				if( best_sequence[id] == i*2 ){
					valid0 = true ;
				}else if( best_sequence[id] == i*2 + 1 )
					valid1 = true ;
			}

			int2 a = int2(0,0) ;
			if( valid0) a.first = 1 ;
			if( valid1) a.second = 1 ;

			endpoints[i] = a ;
		}

		// delete curves

		std::vector<int> newBestSequece = best_sequence ;
		for( int i=0; i<endpoints.size(); ++i  ){
			if( endpoints[i].x == 0 && endpoints[i].y == 0){

				curves[i].clear() ;

				for( int j=0; j<best_sequence.size(); ++j)
					if( best_sequence[j] > i*2 ) newBestSequece[j] -= 2 ; 
			
			}
		}
		best_sequence = newBestSequece ;

		// delete curves id in group
		for( int cid =0; cid<curves.size(); ++cid ){
			if( curves[cid].size() == 0) for( int j=0; j<curves_group.size(); ++j)for( int k=0; k<curves_group[j].size(); ++k){
					if( curves_group[j][k] == cid )
						curves_group[j].erase( curves_group[j].begin() + k);
				}
		}



		// update endpoints  and curves
		std::vector<int> curveDstID(curves.size()) ;
		int count = 0;
		for( int i=0; i<curves.size(); ++i ){
			if( curves[i].size() == 0){
				curves.erase( curves.begin() + i ) ;
				endpoints.erase( endpoints.begin() + i ) ;

				curveDstID[count] = -1 ;
				count++;
				
				i-- ;
				continue;
			}

			curveDstID[count] = i ;
			count++;
		}

		// update group 
		for( int j=0; j<curves_group.size(); ++j)for( int k=0; k<curves_group[j].size(); ++k){
			curves_group[j][k] = curveDstID[ curves_group[j][k] ] ;
		}



}

