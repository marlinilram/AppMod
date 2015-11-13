#ifndef Bound_H
#define Bound_H
#include <cv.h>

class Bound
{
public:
    Bound();
    ~Bound();
	  double getRadius();
    void setRadius();
    void getCenter(float center[3]);
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
    double radius; 
    CvPoint3D32f centroid;
};

#endif