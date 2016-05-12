
#ifndef _GEOMETRY_TYPES_H_
#define _GEOMETRY_TYPES_H_

/*********************************************************************/
#include <CGAL/Cartesian.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Linear_algebraCd.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

#include <glew-1.11.0/include/gl/glew.h>
#include "color.h"
typedef float                FT;
typedef CGAL::Cartesian<FT>  KK;

typedef KK::Point_2           Point2f;
typedef KK::Vector_2          Vector2_f;
typedef KK::Segment_2		 Segment2f;
typedef KK::Line_2			 Line2f;
typedef KK::Circle_2			 Circle_2f;

typedef KK::Iso_rectangle_2   IsoRectangle2f;
typedef CGAL::Bbox_2		 Bbox2f;

typedef CGAL::Polygon_2<KK, std::vector<Point2f>>   Polygon2f;

//////////////////////////////////////////////////////////////////////////

typedef KK::Point_3           Point3f;
typedef KK::Vector_3          Vector3_f;

typedef KK::Ray_3			 Ray3f;
typedef KK::Line_3            Line3f;
typedef KK::Segment_3         Segment3f;
typedef KK::Triangle_3		 Triangle3f;
typedef KK::Circle_3          Circle3f;
typedef KK::Plane_3           Plane3f;
typedef KK::Sphere_3			 Sphere_3f;
typedef KK::Circle_3			 Circle_3f;

typedef KK::Iso_cuboid_3		 IsoCuboid3f;
typedef CGAL::Bbox_3		 Bbox3f;

typedef KK::Sphere_3          Sphere3f;
typedef KK::Tetrahedron_3     Tetrahedron3f;
typedef CGAL::Exact_predicates_inexact_constructions_kernel  CH_Kernel;
typedef	CGAL::Polyhedron_3<CH_Kernel>	Polyhedron_3;


typedef KK::Aff_transformation_3   AffTransformation3f;

typedef CGAL::Linear_Algebra::Matrix_<FT , CGAL_ALLOCATOR(FT)>  Matrixf;
typedef CGAL::Linear_Algebra::Vector_<FT, CGAL_ALLOCATOR(FT)>  Vectorf;
typedef CGAL::Linear_algebraCd<FT, CGAL_ALLOCATOR(FT) > Linear_algebraf;
class ApproxObb {
public:
	float		ext[3];
	Point3f		center;
	Vector3_f	axis[3];
};
//////////////////////////////////////////////////////////////////////////





struct Vertices_VBO
{
	int num_points_;
	int num_faces_;
	GLuint tagOfIndices_;
	GLuint tagOfPositions_;
	GLuint tagOfNormals_;
	GLuint tagOfColors_;
	GLuint tagTriMeshes_;

	Vertices_VBO()
	{
		this->num_points_ = 0;
		this->num_faces_ = 0;
	}

};

struct Faces_VBO
{

};

struct vertex_2d
{
	vertex_2d()
	{

	};

	vertex_2d(const Point2f& p)
	{
		m_pos_ = p;
	};
	vertex_2d(const Point2f& p, Colorf c)
	{
		m_pos_ = p;
		m_color_ = c;
	};
	Point2f m_pos_;
	Colorf  m_color_;
};

#endif


