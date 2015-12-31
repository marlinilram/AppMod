// Based on jMonkeyEngine, ported from java code (New BSD License)
#pragma once

#include "PolygonMesh.h"
#include "BasicHeader.h"
using namespace LG;

/**
* BoundingBox defines an axis-aligned cube that defines a
* container for a group of vertices of a particular piece of geometry. This box
* defines a center and extents from that center along the x, y and z axis.
*
* @author Joshua Slack
*/
class BoundingBox
{

public:
	Vector3f center;
	Vector3f vmax, vmin;
	double xExtent, yExtent, zExtent;

	BoundingBox();
	BoundingBox(const Vector3f& c, double x, double y, double z);
	BoundingBox(const Vector3f& fromMin, const Vector3f& toMax);
	BoundingBox& operator= (const BoundingBox& other);

    /*void computeFromTris(const std::vector<BaseTriangle*>& tris);*/
    /*bool intersects(const Ray& ray) const;*/

	bool contains(const Vector3f& point) const;

	bool containsTriangle(const Vector3f& tv1, const Vector3f& tv2, const Vector3f& tv3) const;
	bool intersectsBoundingBox(const BoundingBox& bb) const;
	bool intersectsSphere(const Vector3f& sphere_center, double radius);

    std::vector<Vector3f> getCorners();

	Vector3f Center();
	void Offset( double s );
	void Offset( Vector3f delta );
	double Diag();
};

/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-M�ller                              */
#define X 0
#define Y 1
#define Z 2

#define FINDMINMAX(x0,x1,x2,min,max) \
	min = max = x0;   \
	if(x1<min) min=x1;\
	if(x1>max) max=x1;\
	if(x2<min) min=x2;\
	if(x2>max) max=x2;

static inline int planeBoxOverlap(const Vector3f& normal, const Vector3f& vert, const Vector3f& maxbox)
{
	Vector3f vmin,vmax;

	for(int q=X; q<=Z; q++)
	{
		double v = vert[q];					
		if(normal[q]>0.0)
		{
			vmin[q]=-maxbox[q] - v;
			vmax[q]= maxbox[q] - v;
		}
		else
		{
			vmin[q]= maxbox[q] - v;
			vmax[q]=-maxbox[q] - v;
		}
	}
	if(normal.dot(vmin) > 0.0) return 0;
	if(normal.dot(vmax) >= 0.0) return 1;

	return 0;
}
/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)			   \
	p0 = a*v0[Y] - b*v0[Z];			       	   \
	p2 = a*v2[Y] - b*v2[Z];			       	   \
	if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb)			   \
	p0 = a*v0[Y] - b*v0[Z];			           \
	p1 = a*v1[Y] - b*v1[Z];			       	   \
	if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;
/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)			   \
	p0 = -a*v0[X] + b*v0[Z];		      	   \
	p2 = -a*v2[X] + b*v2[Z];	       	       	   \
	if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb)			   \
	p0 = -a*v0[X] + b*v0[Z];		      	   \
	p1 = -a*v1[X] + b*v1[Z];	     	       	   \
	if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
	if(min>rad || max<-rad) return 0;
/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb)			   \
	p1 = a*v1[X] - b*v1[Y];			           \
	p2 = a*v2[X] - b*v2[Y];			       	   \
	if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
	if(min>rad || max<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb)			   \
	p0 = a*v0[X] - b*v0[Y];				   \
	p1 = a*v1[X] - b*v1[Y];			           \
	if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
	if(min>rad || max<-rad) return 0;

#undef X
#undef Y
#undef Z
