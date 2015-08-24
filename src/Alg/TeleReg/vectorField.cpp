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
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "cholmod_matrix.h"

#include "tele_basicType.h"
#include <string>


#include "tele2d.h"


typedef Cholmod_matrix<double>			CMatrix;


void tele2d::computeVectorField(){

	unsigned time1, time2, time3 ;
	time1 = clock() ;

	std::vector<std::vector<double2>>  allcurves = curves ;
	vector_field.clear() ;
	vector_field.resize(resolution*resolution) ;

	// delete too short curves
	for( int i=0; i<allcurves.size(); ++i ){
		if( allcurves[i].size() < 5 )
			allcurves.erase( allcurves.begin() + i ) ;
	}

	if( allcurves.size() == 0 ){
		std::cout<<"no valid curves!" ;
		exit(1) ;
	}

	// mark constrained vertices
	constrained_vertices_mark.clear() ;
	for( int i=0; i<resolution; ++i ) {
		std::vector<int> a ;
		for( int j=0;j<resolution; ++j )
			a.push_back(0) ;
		constrained_vertices_mark.push_back(a) ;
	}
	for( int i=0; i<allcurves.size(); ++ i){
		for( int j =0; j<allcurves[i].size(); ++ j){
			// get x index of closest vertices
			float x = allcurves[i][j].x * resolution - 0.5 ;
			int ix ;
			if( x-floor(x) < 0.5 ) ix = floor(x) ;
			else	ix = ceil( x ) ;
			// get y index of closest vertices
			float y = allcurves[i][j].y * resolution - 0.5 ;
			int iy ;
			if( y-floor(y) < 0.5 ) iy = floor(y) ;
			else	iy = ceil( y ) ;

			if( ix < 0 ) ix = 0;
			if( ix > resolution-1) ix = resolution -1;
			if( iy < 0 ) iy = 0;
			if( iy > resolution-1) iy = resolution -1;

			constrained_vertices_mark[ix][iy] = 1 ;

		}
	}

	// compute b
	std::vector<double2> b ;
	b.resize(resolution*resolution) ;
	for( int i=0; i<resolution; ++i ){
		for( int j=0; j<resolution; ++j){
			
			if(constrained_vertices_mark[i][j] == 0 ){
				b[i+j*resolution].x = 0; 
				b[i+j*resolution].y = 0; 
				continue ;
			}

			// otherwise, the vertex indexed by (i,j) is constrained
			double vx = ((double)i+0.5)/(double)resolution ; 
			double vy = ((double)j+0.5)/(double)resolution ; 
			
			// search for the closest points
			int curveid_record = 0;
			int pointid_record = 0;
			double mindis = 1000.0f ;
			for( int curveid=0; curveid<allcurves.size(); ++curveid ){				
				for( int pointid=0; pointid<allcurves[curveid].size(); ++pointid  ){
					double quadratic_dis = ( allcurves[curveid][pointid].x - vx )*( allcurves[curveid][pointid].x - vx ) + ( allcurves[curveid][pointid].y - vy )*( allcurves[curveid][pointid].y - vy ) ;
					if( quadratic_dis < mindis ){
							mindis = quadratic_dis ;
							curveid_record = curveid ;
							pointid_record = pointid ;
					}
				}
			}

			// compute the vector of the vertex indexed by (i,j)
			int pid1 = pointid_record-1 > 0 ? pointid_record-1 : 0 ;
			int pid2 = pointid_record+1 <  allcurves[curveid_record].size()-1 ? pointid_record+1 : allcurves[curveid_record].size()-1;

			double2 vector_of_vertex ;
			vector_of_vertex.x = allcurves[curveid_record][pid2].x - allcurves[curveid_record][pid1].x ;
			vector_of_vertex.y = allcurves[curveid_record][pid2].y - allcurves[curveid_record][pid1].y ;
			double norm = sqrt( vector_of_vertex.x * vector_of_vertex.x + vector_of_vertex.y * vector_of_vertex.y) ;
			vector_of_vertex.x /= norm ;
			vector_of_vertex.y /= norm ;


			assert( norm > 0 && norm < 1) ;

			//std::cout<<"norm "<<norm<<std::endl;
			
			b[i+j*resolution ] = vector_of_vertex ;


		}
	}


	// compute Pb
	std::vector<double2> Pb = b ;
	for( int i=0; i<Pb.size(); ++i ){
		Pb[i].x *= 1.0e8 ;
		Pb[i].y *= 1.0e8 ;
	}

	// compute L+P
	int vnum =  resolution*resolution  ;
	sparse_matrix L_add_P(vnum) ;  // create a sparse matrix of vnum rows

	// L_add_P <- D - W
	for( int id_x =0; id_x<resolution; ++id_x ){
		for( int id_y =0; id_y<resolution; ++id_y ){
			int vid = id_x + id_y * resolution ;
			if( id_x != 0 && id_x != resolution-1 && id_y != 0 && id_y != resolution-1 ){ // inner area
				//L_add_P[ vid + vid*vnum] += 6.8284 ;
				L_add_P.pluse(vid,vid,6.8284 ) ;
				int neibour_id_1 =  id_x + id_y * resolution - 1 ;
				int neibour_id_2 =  id_x + id_y * resolution + 1 ;
				int neibour_id_3 =  id_x + (id_y-1) * resolution ;
				int neibour_id_4 =  id_x + (id_y+1) * resolution ;
				int neibour_id_5 =  id_x + (id_y+1) * resolution - 1 ;
				int neibour_id_6 =  id_x + (id_y+1) * resolution + 1 ;
				int neibour_id_7 =  id_x + (id_y-1) * resolution - 1 ;
				int neibour_id_8 =  id_x + (id_y-1) * resolution + 1 ;
				//L_add_P[neibour_id_1+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_1, -1 ) ;
				//L_add_P[neibour_id_2+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_2, -1 ) ;
				//L_add_P[neibour_id_3+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_3, -1 ) ;
				//L_add_P[neibour_id_4+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_4, -1 ) ;
				//L_add_P[neibour_id_5+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_5, -0.7071 ) ;
				//L_add_P[neibour_id_6+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_6, -0.7071 ) ;
				//L_add_P[neibour_id_7+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_7, -0.7071 ) ;
				//L_add_P[neibour_id_8+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_8, -0.7071 ) ;

			}
			else if((id_x == 0 || id_x==resolution-1) && (id_y == 0 || id_y==resolution-1)  ){  // coners
				//L_add_P[ vid + vid*vnum] += 2.7071 ;
				L_add_P.pluse(vid,vid, 2.7071 ) ;
				int neibour_id_1 =  ( id_x == 0 ? ( id_x+id_y * resolution+1) :  ( id_x+id_y * resolution - 1) );
				int neibour_id_2 =  ( id_y == 0 ? ( id_x+ (id_y+1) * resolution) : ( id_x+ (id_y-1) * resolution )) ;
				int neibour_id_3 =  ( id_x == 0 ? 1 : (resolution-2) ) + ( id_y == 0 ? 1 : (resolution - 2)) * resolution ;

				//L_add_P[neibour_id_1+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_1, -1 ) ;
				//L_add_P[neibour_id_2+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_2, -1 ) ;
				//L_add_P[neibour_id_3+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_3, -0.7071 ) ;

			}
			else {																		// boundaries
				//L_add_P[ vid + vid*vnum] += 4.4142 ;
				L_add_P.pluse(vid,vid, 4.4142 ) ;

				int neibour_id_1, neibour_id_2, neibour_id_3, neibour_id_4, neibour_id_5 ;
				if( id_x == 0){
					neibour_id_1 =  id_x + id_y * resolution + 1 ;
					neibour_id_2 =  id_x + (id_y+1) * resolution ;
					neibour_id_3 =  id_x + (id_y-1) * resolution ;
					neibour_id_4 =  id_x + (id_y+1) * resolution + 1;
					neibour_id_5 =  id_x + (id_y-1) * resolution + 1 ;
				}
				else if( id_x == resolution-1 ){
					neibour_id_1 =  id_x + id_y * resolution - 1 ;
					neibour_id_2 =  id_x + (id_y+1) * resolution ;
					neibour_id_3 =  id_x + (id_y-1) * resolution ;
					neibour_id_4 =  id_x + (id_y+1) * resolution - 1;
					neibour_id_5 =  id_x + (id_y-1) * resolution - 1 ;
				}
				else if( id_y == resolution-1 ){
					neibour_id_1 =  id_x + id_y * resolution + 1 ;
					neibour_id_2 =  id_x + id_y * resolution - 1 ;
					neibour_id_3 =  id_x + (id_y-1) * resolution ;
					neibour_id_4 =  id_x + (id_y-1) * resolution + 1;
					neibour_id_5 =  id_x + (id_y-1) * resolution - 1 ;
				}
				else {
					neibour_id_1 =  id_x + id_y * resolution + 1 ;
					neibour_id_2 =  id_x + id_y * resolution - 1 ;
					neibour_id_3 =  id_x + (id_y+1) * resolution ;
					neibour_id_4 =  id_x + (id_y+1) * resolution + 1;
					neibour_id_5 =  id_x + (id_y+1) * resolution - 1 ;
				}
				//L_add_P[neibour_id_1+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_1, -1 ) ;
				//L_add_P[neibour_id_2+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_2, -1 ) ;
				//L_add_P[neibour_id_3+vid*vnum] -= 1 ;
				L_add_P.pluse(vid,neibour_id_3, -1 ) ;
				//L_add_P[neibour_id_4+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_4, -0.7071) ;
				//L_add_P[neibour_id_5+vid*vnum] -=  0.7071;
				L_add_P.pluse(vid,neibour_id_5, -0.7071 ) ;

			}

		}
	}
	// L_add_P <- D - W + P
	for( int i=0; i<resolution; ++i ){
		for( int j=0; j<resolution; ++j){
			if(constrained_vertices_mark[i][j] == 1  ){
				int vid = i + j*resolution ;
				//L_add_P[vid+vid*vnum]+=1e8 ; 
				L_add_P.pluse(vid,vid, 1.0e8  ) ;

			}
		}
	}




	// solve the linear system with cholmod
	cholmod_sparse *A ;
	cholmod_dense *x, *y, *b1 ;
	double one [2] = {1,0}, m1 [2] = {-1,0} ; /* basic scalars */
	cholmod_factor *L ;
	cholmod_common c ;
	cholmod_start (&c) ;; /* start CHOLMOD */
	
	//A = cholmod_read_sparse (pFile, &c) ; /* read in a matrix */
	CMatrix  *SM = new CMatrix( vnum, true, &c) ;
	//for( int i=0; i<vnum; ++i ){
	//	for( int j=0; j<vnum; ++j ){
	//		if( L_add_P.getValue(j,i)!=0 )
	//			SM->set_coef(i, j,L_add_P.getValue(j,i) ) ;
	//	}
	//}
	for( int i=0; i<L_add_P.data.size(); ++i){
		for( int j=0; j<L_add_P.data[i].size(); ++j)
			SM->set_coef(L_add_P.data[i][j].row, i,  L_add_P.data[i][j].val ) ;
	}

	A =(cholmod_sparse *) SM->get_cholmod_sparse();


	time2 = clock() ;

	//cholmod_print_sparse (A, "A", &c) ; /* print the matrix */

	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		std::cout << "fail to load the matrix or it's not symmeric!"<<std::endl;
		exit(1) ;
	}

	b1 = cholmod_zeros(vnum, 1, CHOLMOD_REAL, &c);


	// --------------------- x demension -----------------------
	for( int i =0 ;i<Pb.size(); ++i ){
		((double*)(b1->x))[i] = Pb[i].x ;
	}
	L = cholmod_analyze (A, &c) ; /* analyze */
	cholmod_factorize (A, L, &c) ; /* factorize */
	x = cholmod_solve (CHOLMOD_A, L, b1, &c) ; /* solve Ax=b */


	// write x-values
	for( int i=0; i<vector_field.size(); ++i)
		vector_field[i].x = ((double*)(x->x))[i] ;
	// --------------------- y demension -----------------------
	for( int i =0 ;i<Pb.size(); ++i ){
		((double*)(b1->x))[i] = Pb[i].y ;
	}
	y = cholmod_solve (CHOLMOD_A, L, b1, &c) ; /* solve Ay=b */
	// write y-values
	for( int i=0; i<vector_field.size(); ++i)
		vector_field[i].y = ((double*)(y->x))[i] ;



	cholmod_free_factor (&L, &c) ; 
	cholmod_free_dense (&x, &c) ;
	cholmod_free_dense (&y, &c) ;
	cholmod_free_dense (&b1, &c) ;
	//delete L_add_P ;
	delete SM ;
	cholmod_finish (&c) ; /* finish CHOLMOD */



	double normx2, normy2 ;
	normx2 = normy2 = 0.0 ;

	for( int i=0; i<vnum; ++i ){
		normx2 += vector_field[i].x * vector_field[i].x ; 
		normy2 += vector_field[i].y * vector_field[i].y ; 
	}

	//std::cout<<"|x| = "<<sqrt(normx2) <<"\n|y| = "<<sqrt(normy2) <<std::endl;


	// normalize vector field
	for( int i=0; i<vector_field.size(); ++i){
		double norm = sqrt( vector_field[i].x * vector_field[i].x + vector_field[i].y * vector_field[i].y) ;
		vector_field[i].x /= norm ;
		vector_field[i].y /= norm ;

	}

	time3 = clock() ;


	//std::cout<<"time consumed by computing A and b: " << (double)(time2-time1)/CLOCKS_PER_SEC <<" s" <<std::endl ;
	//std::cout<<"time consumed by solving the system: " << (double)(time3-time2)/CLOCKS_PER_SEC <<" s" <<std::endl ;

	//std::cout<<"vector field computing completed."<<std::endl; ;
	int count = 0;
	for( int i=0; i<L_add_P.data.size(); ++i)
		count += L_add_P.data[i].size() ;

	//std::cout << "nonzero number: " << count <<std::endl; 

}