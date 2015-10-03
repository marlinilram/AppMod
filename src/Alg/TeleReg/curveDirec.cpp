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

#include "tele2d.h"


inline void inverseCurve( std::vector<double2> &cur ){  //r, int2 &corres_id, int2 &endpoint  
	std::vector<double2> cur_bk = cur ;
	cur.clear() ;
	for( int i=cur_bk.size()-1; i>=0; --i)
		cur.push_back(cur_bk[i]) ;

}


// optimize directions of  curves by maximize its consistence with vector field
void tele2d::correctDirs() {


	std::vector<std::vector<double2>> curves_bk0 = curves ;

	const int N = best_sequence.size()/2 ;
	
	std::vector<int2> corres ;
	for( int i=0; i<N; ++i )
		corres.push_back( int2(best_sequence[i*2], best_sequence[i*2+1] ) ) ;


	std::vector<int2> corres_bk0 = corres  ;
	
	// link curve with correspondence, id of correspondence was represented by curvesNum + corrID

	std::vector< std::vector<int> > curveIndexLists( corres.size() ) ;
	for( int i=0; i<corres.size(); ++i ){

		curveIndexLists[i].push_back( corres[i].x/2 ) ;
		curveIndexLists[i].push_back( i + curves.size() ) ;
		curveIndexLists[i].push_back( corres[i].y/2  ) ;

	}

	// merge curveIndexLists
	bool finished = false ;
	while( !finished ){
		finished = true ;
		for( int i=0; i<curveIndexLists.size(); ++i)
			for( int j=0; j<curveIndexLists.size(); ++j){
				int Li =curveIndexLists[i].size(); 
				int Lj =curveIndexLists[j].size(); 

				if( Li  == 0 || Lj == 0 || i == j)
					continue ;

				if( curveIndexLists[i][0] == curveIndexLists[j][0] ){
					for( int id =1; id<Lj; ++id )
						curveIndexLists[i].insert( curveIndexLists[i].begin(), curveIndexLists[j][id] ) ;
					curveIndexLists[j].clear() ;
					finished == false ; 
				}else if(curveIndexLists[i][0] == curveIndexLists[j][Lj-1] ){
					for( int id =1; id<Li; ++id )
						curveIndexLists[j].push_back( curveIndexLists[i][id]) ;
					curveIndexLists[i].clear();
					finished == false ; 
				}else if( curveIndexLists[i][Li-1] == curveIndexLists[j][0] ){
					for( int id =1; id<Lj; ++id )
						curveIndexLists[i].push_back( curveIndexLists[j][id]) ;
					curveIndexLists[j].clear() ;
					finished == false ; 
				}else if(curveIndexLists[i][Li-1] == curveIndexLists[j][Lj-1] ){
					for( int id = Lj-2; id >=0; --id )
						curveIndexLists[i].push_back(curveIndexLists[j][id]);
					curveIndexLists[j].clear();
					finished == false ; 
				}
			}
	}

	// delete empty list and delete the last element if the index list is a loop
	for( int i=0; i<curveIndexLists.size(); ++i){
		if( curveIndexLists[i].size() == 0 ){
			curveIndexLists.erase( curveIndexLists.begin() + i ) ;
			--i ;
		}else {
			if(  curveIndexLists[i].size()>1 && curveIndexLists[i][0] == *(curveIndexLists[i].end()-1) )
				curveIndexLists[i].erase(  curveIndexLists[i].end()-1) ;
		}

	}

	// swap corres according to keep consistent with curveIndexLists
	for( int i=0; i<curveIndexLists.size(); ++i){
		std::vector<int> list = curveIndexLists[i];
		for( int id =1; id<list.size(); id+=2 ){
			int corresID = list[id] - curves.size();
			if( list[id-1] != corres[corresID].x /2 ){
				int temp = corres[corresID].x ;
				corres[corresID].x = corres[corresID].y ;
				corres[corresID].y = temp ;
			}
		}
	}


	// decide the directions according to curveIndexLists

	for( int ILid =0; ILid<curveIndexLists.size() ; ++ILid ){
		
		for( int id = 1; id<curveIndexLists[ILid].size(); id+=2 ){
			int corresID = curveIndexLists[ILid][id] - curves.size();

			int e1= corres[corresID].x ;
			int e2= corres[corresID].y ;

			if( e1%2 == 0 && e2%2 == 0){
				inverseCurve( curves[e2/2]) ;
				corres[corresID].y = e2+1 ;
				if( id+2 < curveIndexLists[ILid].size() ){
					corres[curveIndexLists[ILid][id+2] - curves.size()].x = e2 ;
				}
			}
			if( e1%2 == 1 && e2%2 == 1){
				inverseCurve( curves[e2/2]) ;
				corres[corresID].y = e2-1 ;
				if( id+2 < curveIndexLists[ILid].size() ){
					corres[curveIndexLists[ILid][id+2] - curves.size()].x = e2 ;
				}
			}

		}

	}


	std::vector<std::vector<double2>> curves_bk = curves ;
	curves.clear() ;
	// find the  longest one
	double maxlength = 0;
	int maxId ;
	for( int ILid = 0; ILid<curveIndexLists.size(); ++ILid ){
		std::vector<int> list = curveIndexLists[ILid] ;
		double length = 0.0;
		for( int cid = 0; cid<list.size(); cid+=2 ){
			int CID = list[cid] ;
			for( int j=0; j<curves_bk[CID].size()-1; ++j ){
				length += (curves_bk[CID][j] - curves_bk[CID][j+1]).norm() ;
			}
		}
		if( length > maxlength ){
			maxId = ILid ;
			maxlength = length ;
		}
	}



	for( int i=0; i<curveIndexLists[maxId].size(); i+=2 ){
		curves.push_back( curves_bk[curveIndexLists[maxId][i]] ) ;
	}

	std::vector<int> listMsk ;  // 0-not fixed, 1-fixed
	for( int i=0; i<curveIndexLists.size(); ++i )
		listMsk.push_back(0) ;
	listMsk[maxId] = 1 ;


	// add the curves list by list
	for( int count=1; count<curveIndexLists.size(); ++count ){

		computeVectorField() ;

		double maxScore = -1.0e10 ;
		double maxScore_id ;
		bool inverseDir = false ;

		for( int id = 0; id<curveIndexLists.size(); ++id ){

			if( listMsk[id] )
				continue ;

			double score = 0.0 ;
			double score_inverse = 0.0 ;

			for( int cid=0; cid<curveIndexLists[id].size(); cid+=2 ){

				std::vector<double2> c1 = curves_bk[curveIndexLists[id][cid]] ;

				for( int pid=0; pid<c1.size()-1; ++pid ){
					double2 dir = c1[pid+1] - c1[pid] ;

					double x = c1[pid+1].x * resolution - 0.5 ;
					int ix ;

					if( x-floor(x) < 0.5 ) ix = floor(x) ;
					else	ix = ceil( x ) ;

					double y = c1[pid+1].y * resolution - 0.5 ;

					int iy ;
					if( y-floor(y) < 0.5 ) iy = floor(y) ;
					else	iy = ceil( y ) ;


					double2 vec = vector_field[ ix + iy * resolution] ;


					// compute penalty of this segment
					double costheta = (dir * vec) *(   1/(dir.norm()*vec.norm())   ) ;

					score += 1 + costheta ;
					score_inverse += 2 - costheta ;
				}

			}


			if( score > maxScore){
				maxScore = score ;
				maxScore_id = id ;
				inverseDir = false ;

			}
			if( score_inverse > maxScore){
				maxScore = score_inverse ;
				maxScore_id = id ;
				inverseDir = true ;
			}


		}

		if( inverseDir ){

			// inverse curves
			for( int i = 0; i<curveIndexLists[maxScore_id].size(); i+=2 ){

				inverseCurve( curves_bk[curveIndexLists[maxScore_id][i]]  ) ;	
			}

		}

		for( int i = 0; i<curveIndexLists[maxScore_id].size(); i+=2 ){

			curves.push_back( curves_bk[curveIndexLists[maxScore_id][i] ]  ) ;
		}

		listMsk[maxScore_id] = 1;
	}


	// resort curves
	std::vector<std::vector<double2>> newCurves;  
	for( int i=0; i<curves_bk0.size(); ++i ){
		for( int j = 0;j<curves.size(); ++j){
			if( curves[j][0] == curves_bk0[i][0] ||  curves[j][0] == curves_bk0[i].back()  ){
				newCurves.push_back( curves[j]) ;
				break;
			}
		}
	}

	curves = newCurves ;

	// update correspondeces
	corres = corres_bk0 ;
	for( int i=0; i<corres.size(); ++i)for( int j=0; j<2; ++j){
		int epid = corres[i][j];
		if( !( curves_bk0[epid/2][0] == curves[epid/2][0] )){
			
			epid = (epid/2) * 2 + ( 1 - epid%2) ;
		}

		corres[i][j] = epid ;
	}

	// update  endpoints
	for( int i=0; i<curves.size(); ++i){
		if(!( curves[i][0] == curves_bk0[i][0]) ){
			int temp = endpoints[i][0] ;
			endpoints[i][0]= endpoints[i][1] ;
			endpoints[i][1] = temp;
		}
	}




	// make sure that the direction of bridging curves are right
	for( int i=0; i<corres.size();  ++i ){
		if(corres[i].x % 2 == 0 ){

			int temp = corres[i].x ;
			corres[i].x = corres[i].y ;
			corres[i].y = temp;

		}
	}

	// write corres back
	best_sequence.clear();
	for( int i=0; i<corres.size(); ++i){
		best_sequence.push_back( corres[i].x );
		best_sequence.push_back( corres[i].y );
	}

	correspondence = corres ;
	initialCurves = curves ;

}