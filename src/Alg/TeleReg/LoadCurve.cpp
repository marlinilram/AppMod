#include "tele_basicType.h"
#include "tele2d.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#define safe_load_curve(X) if(!(X) ){ \
  std::cout << "cannot load curve file: "<< fname <<std::endl;  \
  return false ;\
} ;

bool tele2d::load_Curves(std::string fname, 
  CURVES &curves , std::vector<std::vector<int>> &curves_group,
  std::vector<int2> &endpoints)
{
  curves.clear() ;
  curves_group.clear() ;
  endpoints.clear() ;

  std::ifstream ifs( fname ) ;

  safe_load_curve(ifs) ;

  // load curves
  int curve_num ;
  safe_load_curve( ifs >> curve_num ) ; 

  for( int i=0; i<curve_num; ++i){
    int len ;
    curves.resize(curves.size() + 1 ) ;
    safe_load_curve( ifs >> len ) ; 
    for( int i=0; i<len; ++i ){
      double x, y ;
      safe_load_curve( ifs>>x>>y );
      curves.back().push_back( double2(x, y) ) ;
    }

  }

  //  interpolate short curves
  for(int i=0; i<curves.size(); ++i ){
    std::vector<double2> cur = curves[i];
    while( cur.size() < 10 ){
      std::vector<double2> cur1 ;
      cur1.push_back( cur[0] ) ;

      for( int j=1; j<cur.size(); ++j ){
        cur1.push_back( (cur[j-1] + cur[j]) * 0.5 ) ;
        cur1.push_back( cur[j] ) ;
      }
      cur = cur1 ;
    }
    curves[i] = cur ;
  }


  // load groups
  int gnum ;
  safe_load_curve( ifs >> gnum ) ; 
  for( int i=0; i<gnum; ++i){
    int len ;
    curves_group.resize(curves_group.size() + 1 ) ;
    safe_load_curve( ifs >> len ) ; 
    for( int i=0; i<len; ++i ){
      int cid ;
      safe_load_curve(ifs>>cid)  ;
      curves_group.back().push_back( cid ) ;
    }

  }


  // load endpoints
  endpoints.resize(curves.size());
  int tmp ;
  while( ifs>>tmp ){
    //howtoconnect.push_back(tmp);

    if( tmp%2 )
      endpoints[tmp/2].y = 1 ;
    else
      endpoints[tmp/2].x = 1 ;
  }

  return true;
}