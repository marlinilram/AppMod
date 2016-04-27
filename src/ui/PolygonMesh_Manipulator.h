#pragma once
#include "PolygonMesh.h"
#include "Shape.h"

struct shape_Manipulator
{

};


class PolygonMesh_Manipulator
{
public:
	PolygonMesh_Manipulator();
	~PolygonMesh_Manipulator();

public:
	static void translate(LG::PolygonMesh* poly, const LG::Vec3& translation);
	static void scale(LG::PolygonMesh* poly, LG::Vec3 center, float s);
	static void scale_along_axis(LG::PolygonMesh* poly, const LG::Vec3& origin, float s, LG::Vec3 axis);
	static void set_color(LG::PolygonMesh* poly, LG::Vec3 color);
	static void rotate(LG::PolygonMesh* poly, LG::Vec3 point_on_line, LG::Vec3 line_vector, float angle_radian);
public:

};

