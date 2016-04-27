#include "PolygonMesh_Manipulator.h"

PolygonMesh_Manipulator::PolygonMesh_Manipulator()
{
}


PolygonMesh_Manipulator::~PolygonMesh_Manipulator()
{
}

void PolygonMesh_Manipulator::set_color(LG::PolygonMesh* poly, LG::Vec3 color)
{
	LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_colors = poly->vertex_attribute<LG::Vec3>("v:colors");
	for (auto vit : poly->vertices())
	{
		v_colors[vit] = color;
	}
		
};
void PolygonMesh_Manipulator::translate(LG::PolygonMesh* poly, const LG::Vec3& translation)
{
	for (auto vit : poly->vertices())
	{
		LG::Vec3& pt = poly->position(vit);
		pt = pt + translation;
	}
};

void PolygonMesh_Manipulator::scale(LG::PolygonMesh* poly, LG::Vec3 center, float s)
{
	for (auto vit : poly->vertices())
	{
		LG::Vec3& pt = poly->position(vit);
		LG::Vec3 pt_tmp = pt;
		pt_tmp = (pt_tmp - center) * s;
		pt = center + pt_tmp;
	}
};

void PolygonMesh_Manipulator::scale_along_axis(LG::PolygonMesh* poly, const LG::Vec3& origin, float s, LG::Vec3 axis)
{

	axis.normalize();
	for (auto vit : poly->vertices())
	{
		LG::Vec3 pt = poly->position(vit);
		pt = pt - origin;
		float length = pt[0] * axis[0] + pt[1] * axis[1] + pt[2] * axis[2];
		length = length * (s - 1);

		pt = poly->position(vit);

		pt = pt + axis * length;
		poly->position(vit) = pt;
	}
};

void PolygonMesh_Manipulator::rotate(LG::PolygonMesh* poly, LG::Vec3 point_on_line, LG::Vec3 line_vector, float angle_radian)
{
	Point3f pppp(point_on_line[0], point_on_line[1], point_on_line[2]);
	Vector3_f v(line_vector[0], line_vector[1], line_vector[2]);
	Line3f axis_line(pppp, v);

	Vector3_f u = axis_line.direction().vector();
	double cos_angle = std::cos(angle_radian);
	double sin_angle = std::sin(angle_radian);

	double rot[3][3];
	rot[0][0] = u.x() * u.x() * (1.0f - cos_angle) + cos_angle;
	rot[0][1] = u.x() * u.y() * (1.0f - cos_angle) - u.z() * sin_angle;
	rot[0][2] = u.x() * u.z() * (1.0f - cos_angle) + u.y() * sin_angle;
	rot[1][0] = u.y() * u.x() * (1.0f - cos_angle) + u.z() * sin_angle;
	rot[1][1] = u.y() * u.y() * (1.0f - cos_angle) + cos_angle;
	rot[1][2] = u.y() * u.z() * (1.0f - cos_angle) - u.x() * sin_angle;
	rot[2][0] = u.z() * u.x() * (1.0f - cos_angle) - u.y() * sin_angle;
	rot[2][1] = u.z() * u.y() * (1.0f - cos_angle) + u.x() * sin_angle;
	rot[2][2] = u.z() * u.z() * (1.0f - cos_angle) + cos_angle;

	AffTransformation3f mat2(
		rot[0][0], rot[0][1], rot[0][2],
		rot[1][0], rot[1][1], rot[1][2],
		rot[2][0], rot[2][1], rot[2][2]);

	for (auto vit : poly->vertices())
	{
		LG::Vec3& pt = poly->position(vit);
		Point3f p(pt[0], pt[1], pt[2]);
		Point3f p1 = axis_line.projection(p);
		AffTransformation3f mat1(CGAL::Translation(), Point3f(0.0f, 0.0f, 0.0f) - p1);
		AffTransformation3f mat3(CGAL::Translation(), p1 - Point3f(0.0f, 0.0f, 0.0f));
		AffTransformation3f mat = mat3 * mat2 * mat1;
		p = mat.transform(p);
		pt = LG::Vec3(p.x(), p.y(), p.z());
	}

};