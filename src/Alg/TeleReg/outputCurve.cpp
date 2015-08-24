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


void inverse( std::vector<double2> &curve ){
	std::vector<double2> temp = curve ;
	for( int i=0; i<curve.size(); ++i ){
		curve[i] = temp[ curve.size()-1-i] ;
	}

}

void tele2d::outputResCurves( std::string fname, bool link ){


	if( !link ){

		// de-normalize resCurves

		CURVES outcurves ;
		outcurves.resize( resCurves.size() ) ;
		for( int i=0; i<resCurves.size(); ++i ){
			for( int j=0; j< resCurves[i].size(); ++j ){
				double2 p = (resCurves[i][j] - double2(0.5, 0.5 ) )/ normalize_scale + double2(0.5, 0.5 )  - normalize_translate ;
				outcurves[i].push_back ( p )  ;
			}
		}

		// output registered curves	
		std::ofstream   regcurOut ( fname) ;
		regcurOut << outcurves.size() <<std::endl;
		for( int i=0; i<outcurves.size(); ++i ){
			regcurOut << outcurves[i].size() <<std::endl;
			for( int j=0; j< outcurves[i].size(); ++j )
				regcurOut << outcurves[i][j].x << " " <<  outcurves[i][j].y <<std::endl;
		}
		regcurOut.close() ;

		std::cout << "curves saved!" ;
		return ;

	}


	// --------------------------- merge curves --------------------------
	// build a index list. the index of curves is its index in the "curves" vector
	// the index of interpolated curves is curves.size() + itsIndex
	std::vector< std::vector<int> > curveIndexLists(bridging_curves_endpoints.size()) ;

	for( int i=0; i<bridging_curves_endpoints.size(); ++i ){

		curveIndexLists[i].push_back( bridging_curves_endpoints[i].first /2 ) ;
		curveIndexLists[i].push_back( i + curves.size() ) ;
		curveIndexLists[i].push_back( bridging_curves_endpoints[i].second /2 ) ;
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

	// 

	// print index List
	for( int i=0; i<curveIndexLists.size(); ++i){
		std::cout<<std::endl;
		for( int j=0; j<curveIndexLists[i].size(); ++j)
			std::cout<<" "<<curveIndexLists[i][j] ;
	}

	// convert the merged index list to merged curves
	curves.insert(curves.end(), bridging_curves.begin(), bridging_curves.end() ) ;
	std::vector<std::vector<double2> > mergedCuves (curveIndexLists.size() );
	for( int ILid =0; ILid<curveIndexLists.size() ; ++ILid ){

		std::vector<int> indList = curveIndexLists[ILid] ;
		std::vector<double2>  curve;

		if(    std::min( (curves[indList[0]][0] - curves[indList[1]][0]).norm() , ( curves[indList[0]][0] - *(curves[indList[1]].end()-1)).norm() )
			<  std::min( ( *(curves[indList[0]].end()-1) - curves[indList[1]][0]).norm(), ( *(curves[indList[0]].end()-1) - *(curves[indList[1]].end()-1)).norm() ) ){

				for(int i =  curves[indList[0]].size()-1; i>=0; --i)
					curve.push_back(curves[indList[0]][i] ) ;
		}
		else
			curve.insert(curve.end(), curves[indList[0]].begin(),curves[indList[0]].end()  ) ;


		for( int i=1; i<indList.size(); i++ ){
			if(  (curves[indList[i]][0] - *(curve.end()-1)).norm() < (*(curves[indList[i]].end()-1) - *(curve.end()-1)).norm() )

				curve.insert(curve.end(), curves[indList[i]].begin(),curves[indList[i]].end()  ) ;

			else{
				inverse(curves[indList[i]] ) ;
				curve.insert(curve.end(), curves[indList[i]].begin(),curves[indList[i]].end()  ) ;
			}
		}

		mergedCuves[ILid] = curve;

	}




	// de-normalize merged curves
	CURVES outcurves ;
	outcurves.resize( mergedCuves.size() ) ;
	for( int i=0; i<mergedCuves.size(); ++i ){
		for( int j=0; j< mergedCuves[i].size(); ++j ){
			double2 p = (mergedCuves[i][j] - double2(0.5, 0.5 ) )/ normalize_scale + double2(0.5, 0.5 )  - normalize_translate ;
			outcurves[i].push_back ( p )  ;
		}
	}
	// output merged curves
	std::ofstream   megcurOut ( fname ) ;
	megcurOut << outcurves.size() <<std::endl;
	for( int i=0; i<outcurves.size(); ++i ){
		megcurOut << outcurves[i].size() <<std::endl;
		for( int j=0; j< outcurves[i].size(); ++j )
			megcurOut << outcurves[i][j].x << " " <<  outcurves[i][j].y <<std::endl;
	}
	megcurOut.close() ;

	std::cout << "curves saved!" ;
}