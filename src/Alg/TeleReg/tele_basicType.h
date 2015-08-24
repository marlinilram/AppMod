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
#ifndef __declaration_h
#define  __declaration_h
#include <cmath>
#include <vector>
#include <iostream>
class double2{
public:
	double x ;
	double y ;
	
	double norm(){
		return sqrt( x*x+y*y) ;
	}
	double2 normalize(){
		double n = norm() ;
		x /= n;
		y /= n ;

		return *this ;
	}
	double2(){ x = y = 0.0;}
	double2(double _x, double _y ): x(_x), y(_y) {}

	double2 operator+(double2 &a){
		return double2(x+a.x, y+a.y) ;
	}
	double operator*(double2 &a){
		return x*a.x + y*a.y ;
	}
	double2 operator*(double a){
		return double2(a*x, a*y) ;
	}
	double2 operator/(double a){
		return double2(x/a, y/a) ;
	}
	double2 operator-(double2 &a){
		return double2(x-a.x, y-a.y) ;
	}

	double2 operator-(){
		return double2(-x, -y) ;
	}

	bool operator==(double2 a){
		return x==a.x&&y==a.y ;
	}
};


class int2{
public:
	int2():first(0),second(0){}
	int2(int a, int b):first(a), second(b) {}
	
	union{
		int first ;
		int x ;
	} ;

	union{

		int second ;
		int y ;
	};


	int &operator[](int a){
		if( a == 0)
			return x ;
		else if( a==1)
			return y;
		else{
			std::cerr<<"invalid index of int2" ;
			exit(1) ;
		}
	}
};

class int3{
public:
	int3():x(0),y(0),z(0){}
	int3(int a, int b, int c):x(a), y(b), z(c) {}
	int x ,y,z ;


	int3 operator+(int3 &a){
		return int3(x+a.x, y+a.y, z+a.z) ;
	}

	int3 operator-(int3 &a){
		return int3(x-a.x, y-a.y, z-a.z) ;
	}


	double norm(){
		return sqrt( (double)(x*x+y*y + z*z) );
	}

};

class double3{
public:
	double x, y, z;
	double3(double x0, double y0, double z0):x(x0), y(y0), z(z0){}
	double3(){}

	double3 operator/(double a){
		return double3(x/a, y/a, z/a) ;
	}
	double3 operator*(double a){
		return double3(x*a, y*a, z*a) ;
	}
	double3 operator-(double3 &a){
		return double3(x-a.x, y-a.y, z-a.z) ;
	}
	double3 operator+(double3 &a){
		return double3(x+a.x, y+a.y, z+a.z) ;
	}
	double norm(){
		return sqrt( x*x+y*y+z*z) ;
	}

};

struct smnode{
	int row ;
	double val ; 
};
class sparse_matrix{
public:
	sparse_matrix( int cols){
		data.resize(cols) ;
	}

	std::vector<std::vector<smnode>> data ; 

	inline double getValue( int col, int row ) {

		for( int i=0; i<data[col].size(); ++i)
			if( data[col][i].row == row)
				return  data[col][i].val ;

		return 0;
	}

	inline void pluse(int col, int row, double val ) {
		
		for( int i=0; i<data[col].size(); ++i){
			if( data[col][i].row == row){
				data[col][i].val += val ;
				return ;
			}
		}

		smnode node ;
		node.row = row ;
		node.val = val ;

		data[col].push_back(node) ;
			
	}


};

enum State{
	BEFORE_LOAD_CURVES,
	AFTER_LOAD_CURVES,
	SHOW_RESULT
};

//void drawCurves( IplImage * img , 	std::vector<std::vector<CvPoint>> curves, CvScalar corlor , int thickness ) ;


struct Circle{
	double2 first ;   // center
	double second ;		// radius
	double curvature ;
	double2  touchpoint ;
	double2 tangent ;
};

//
//#define DISTANCE_PEN_WEIGHT 0.5
#define DISPLACE_PEN_WEIGHT 1.0
#define ROTATE_PEN_WEIGHT 0.0
#define Overlap_Penalty 1.0

//#define min( x, y)  (std::min)(x, y)
//#define max( x, y)  (std::max)(x, y)


#define DIS2COLOR_R_(d)  (((d)<0.5)?((0.5-(d))/0.5):0 )
#define DIS2COLOR_G_(d)  (((d)<0.5)?(d)/0.5:(1-(d))/0.5 )
#define DIS2COLOR_B_(d)  (((d)>0.5)?((d)-0.5)/0.5:0 )

#define DIS2COLOR_R(d) ((GLubyte)(std::min)( std::max( DIS2COLOR_R_(d) * 255, 0.0)+100, 255.0)) 
#define DIS2COLOR_G(d) ((GLubyte)(std::min)( std::max( DIS2COLOR_G_(d) * 255, 0.0)+75, 255.0)) 
#define DIS2COLOR_B(d) ((GLubyte)(std::min)( std::max( DIS2COLOR_B_(d) * 255, 0.0)+50, 255.0)) 

#define DIS2COLOR_R_f(d) ((std::min)( std::max( DIS2COLOR_R_(d) * 255, 0.0)+100, 255.0)/255.0) 
#define DIS2COLOR_G_f(d) ((std::min)( std::max( DIS2COLOR_G_(d) * 255, 0.0)+75, 255.0)/255.0) 
#define DIS2COLOR_B_f(d) ((std::min)( std::max( DIS2COLOR_B_(d) * 255, 0.0)+50, 255.0)/255.0) 


#define EPID2DOUB2(i) ((i==0)?curves[i/2][0]:curves[i/2].back()) 



#define PixelIsZero3C( imagedata, ws, x, y)  (imagedata[ws*(y)+(x)*3+0]==0&&imagedata[ws*(y)+(x)*3+1]==0&&imagedata[ws*(y)+(x)*3+2]==0)
#define PixelIsZero1C( imagedata, ws, x, y)  (imagedata[ws*(y)+(x)+0]==0)



#define GUSSIAN_H 0.02
#define SCALAR_WEIGHT 0.1

#define energyGraAngStep  10.0 
#define energyGraTranStep 1.0 
#define energyScaleValue  0.01

#define show_briging_direction 0

#define __fix 0   // fix one of the piece
#endif
