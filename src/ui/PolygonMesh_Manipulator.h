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
	static void rotate(std::vector<LG::Vec3>& point_input, LG::Vec3 point_on_line, LG::Vec3 line_vector, float angle_radian);

	static void boundary_find(LG::PolygonMesh* poly, const std::vector<bool>& selected_faces, std::vector<std::vector<int>>& ordered_boundaries);
	static void faces_has_common_vertex(LG::PolygonMesh* poly, int f_id, std::vector<int>& faces_boundaries);
	static void faces_has_common_edge(LG::PolygonMesh* poly, int f_id, std::vector<int>& faces_boundaries);
public:

};

