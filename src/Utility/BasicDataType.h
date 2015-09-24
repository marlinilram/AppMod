#ifndef BasicDataType_H
#define BasicDataType_H

#include <cmath>

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
  operator double2() const {
    return double2(x, y);
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

#endif // !BasicDataType_H
