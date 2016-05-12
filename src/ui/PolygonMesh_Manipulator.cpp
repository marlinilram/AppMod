#include "PolygonMesh_Manipulator.h"
#include <set>
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
void PolygonMesh_Manipulator::boundary_find(LG::PolygonMesh* poly_mesh, const std::vector<bool>& selected_faces, std::vector<std::vector<int>>& ordered_boundaries)
{
	std::vector<bool> boundary_faces_tmp(selected_faces.size(), false);

	

	for (unsigned int i = 0; i < selected_faces.size(); i++)
	{
		if (!selected_faces[i])
		{
			continue;;
		}
		std::vector<int> faces_around;

		PolygonMesh_Manipulator::faces_has_common_edge(poly_mesh, i, faces_around);
		for (unsigned int j = 0; j < faces_around.size(); j++)
		{
			if (!selected_faces[faces_around[j]])
			{
				boundary_faces_tmp[i] = true;
				break;;
			}
		}
	}

	
	//std::vector<std::vector<int>> ordered_boundaries_tmp;
	std::vector<bool> checked(boundary_faces_tmp.size(), false);
	ordered_boundaries.clear();
	for (unsigned int i = 0; i < boundary_faces_tmp.size(); i++)
	{
		if (!boundary_faces_tmp[i])
		{
			continue;
		}
		if (checked[i])
		{
			continue;
		}
		std::vector<int> boudary;
		boudary.push_back(i);
		checked[i] = true;


		for (unsigned int k = 0; k < boudary.size(); k++)
		{
			std::vector<int> faces_around;
			PolygonMesh_Manipulator::faces_has_common_vertex(poly_mesh, boudary[k], faces_around);

			for (unsigned int j = 0; j < faces_around.size(); j++)
			{
				if (!boundary_faces_tmp[faces_around[j]])
				{
					continue;
				}
				if (checked[faces_around[j]])
				{
					continue;
				}
				boudary.push_back(faces_around[j]);
				checked[faces_around[j]] = true;
				break;;
			}
		
		}

		/************************************************************************/

		std::vector<int> faces_around_tmp;
		PolygonMesh_Manipulator::faces_has_common_edge(poly_mesh, boudary[0], faces_around_tmp);
		std::vector<int> boudary_tmp;
		for (unsigned int k = 0; k < faces_around_tmp.size(); k++)
		{
			if (!boundary_faces_tmp[faces_around_tmp[k]] && !selected_faces[faces_around_tmp[k]])
			{
				boudary_tmp.push_back(faces_around_tmp[k]);
				break;
			}
		}

		for (unsigned int k = 0; k < boudary_tmp.size(); k++)
		{
			std::vector<int> faces_around;
			PolygonMesh_Manipulator::faces_has_common_vertex(poly_mesh, boudary_tmp[k], faces_around);

			for (unsigned int j = 0; j < faces_around.size(); j++)
			{
				if (!boundary_faces_tmp[faces_around[j]])
				{
					continue;
				}
				if (checked[faces_around[j]])
				{
					continue;
				}
				boudary_tmp.push_back(faces_around[j]);
				checked[faces_around[j]] = true;
				break;;
			}
		}

		for (unsigned int k = 0; k < boudary_tmp.size(); k++)
		{
			boudary.push_back(boudary_tmp[boudary_tmp.size() - 1 -k]);
		}

		/************************************************************************/

		if (boudary.size() > 20)
		{
			ordered_boundaries.push_back(boudary);
		}
	}

	for (unsigned int k = 0; k < ordered_boundaries.size(); k++)
	{
		for (unsigned int j = k + 1; j < ordered_boundaries.size(); j++)
		{
			if (ordered_boundaries[k].size() < ordered_boundaries[j].size())
			{
				std::vector<int> tmp = ordered_boundaries[k];
				ordered_boundaries[k] = ordered_boundaries[j];
				ordered_boundaries[j] = tmp;
			}
		}
	}

};
void PolygonMesh_Manipulator::faces_has_common_edge(LG::PolygonMesh* poly_mesh, int f_id, std::vector<int>& faces_around)
{
	std::vector<int> face_has_common_vertex;
	PolygonMesh_Manipulator::faces_has_common_vertex(poly_mesh, f_id, face_has_common_vertex);


	std::set<int> ids_around;
	faces_around.clear();
	for (auto efc : poly_mesh->halfedges(LG::PolygonMesh::Face(f_id)))
	{
		for (unsigned int i = 0; i < face_has_common_vertex.size(); i++)
		{
			for (auto efc1 : poly_mesh->halfedges(LG::PolygonMesh::Face(face_has_common_vertex[i])))
			{
				if (poly_mesh->opposite_halfedge(efc).idx() == efc1.idx())
				{
					faces_around.push_back(face_has_common_vertex[i]);
					break;
				}
				
			}
		}
	}

};
void PolygonMesh_Manipulator::faces_has_common_vertex(LG::PolygonMesh* poly_mesh, int f_id, std::vector<int>& faces_around)
{
	if (poly_mesh == NULL)
	{
		return;
	}
	int f_num = poly_mesh->n_faces();

	if (f_id <0 || f_id >= f_num)
	{
		return;
	}
	
	std::set<int> ids_around;
	for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
	{
		LG::PolygonMesh::Face_around_vertex_circulator faces = poly_mesh->faces(vfc);
		for (auto fc : faces)
		{
			int id = fc.idx();
			ids_around.insert(id);
		}
	}

	faces_around.clear();
	std::set<int>::iterator itr = ids_around.begin();
	for (int i = 0; itr != ids_around.end(); ++itr) {
		int id = *itr;
		if (id != f_id)
		{
			faces_around.push_back(id);
		}
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

void PolygonMesh_Manipulator::rotate(std::vector<LG::Vec3>& point_input, LG::Vec3 point_on_line, LG::Vec3 line_vector, float angle_radian)
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

	for (unsigned int i = 0; i < point_input.size(); i++)
	{
		LG::Vec3& pt = point_input[i];
		Point3f p(point_input[i][0], point_input[i][1], point_input[i][2]);
		Point3f p1 = axis_line.projection(p);
		AffTransformation3f mat1(CGAL::Translation(), Point3f(0.0f, 0.0f, 0.0f) - p1);
		AffTransformation3f mat3(CGAL::Translation(), p1 - Point3f(0.0f, 0.0f, 0.0f));
		AffTransformation3f mat = mat3 * mat2 * mat1;
		p = mat.transform(p);
		pt = LG::Vec3(p.x(), p.y(), p.z());
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