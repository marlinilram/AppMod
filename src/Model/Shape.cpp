#include "Shape.h"
#include "Bound.h"
#include "KDTreeWrapper.h"
#include "PolygonMesh.h"
#include <gl/GLUT.H>
#include <gl/gl.h>
#include "color.h"
#include "Ray.h"
#include "SAMPLE.h"
#include "GenerateSamples.h"
#include "geometry_types.h"
#include <set>
#include <fstream>
#include <QGLViewer/qglviewer.h>
#include <QtCore/QPoint>
#include "shape_manipulator.h"
#include "PolygonMesh_Manipulator.h"
#include "Model.h"
#include "../Viewer/DispObject.h"
using namespace LG;



Shape::Shape()
	: bound(new Bound()),
	m_is_selected_(false),
	m_show_manipulator_(false),
	m_sm_(NULL),
	m_model_(NULL)
{

	m_viewer_ = NULL;
}

Shape::~Shape()
{
	if (this->m_sm_)
	{
		delete (this->m_sm_);
	}
	std::cout << "Deleted a Shape.\n";
}

void Shape::init(VertexList& vertexList, FaceList& faceList, FaceList& UVIdList, STLVectorf& UVList)
{
  poly_mesh.reset(new PolygonMesh());

  setVertexList(vertexList);
  setFaceList(faceList);
  setUVCoord(UVIdList, UVList, faceList);

  ori_vertex_list  = vertex_list;

  color_list.resize(vertex_list.size(), 0.5);
  PolygonMesh::Vertex_attribute<Vec3> colors = poly_mesh->vertex_attribute<Vec3>("v:colors");
  for (auto vit : poly_mesh->vertices())
  {
    colors[vit] = Vec3(0.5, 0.5, 0.5);
  }
  face_color_list.resize(face_list.size(), 0.5);


  std::cout<<"Building face adjacent list...\n";
  buildFaceAdj();

  std::cout<<"Building 1-ring neighbors list...\n";
  buildVertexShareFaces();

  std::cout<<"Building vertex adjacent list...\n";
  buildVertexAdj();

  std::cout << "Building edge connectivity...\n";
  computeEdgeConnectivity();

  std::cout<<"Computing bounding box...\n";
  computeBounds();

  std::cout<<"Computing face normals...\n";
  computeFaceNormal();
  computeVertexNormal();

  std::cout << "Computing laplacian cotangent weight...\n";
  poly_mesh->update_laplacian_cot();

  buildKDTree();
}

const void Shape::draw_manipulator()
{
	if ( !(this->m_show_manipulator_ ) )
	{
		return;
	}
	if (!(this->m_is_selected_))
	{
		return;
	}
	if (this->m_viewer_ == NULL)
	{
		return;
	}

	this->m_sm_->draw();
};

bool Shape::show_mani()
{
	return this->m_show_manipulator_;
};
void Shape::set_show_mani(bool b)
{
	this->m_show_manipulator_ = b;
};

QGLViewer* Shape::glviewer()
{
	return this->m_viewer_;
};
void Shape::set_glviewer(QGLViewer* g)
{
	this->m_viewer_ = g;
};
bool Shape::double_click(QMouseEvent* e, int& activated)
{
	if (this->m_show_manipulator_ == false)
	{
		this->m_is_selected_ = false;
		activated = -1;
		return false;
	}
	else
	{
		this->m_is_selected_ = true;
		this->compute_mainipulator();
		return m_sm_->double_click(e, activated);
	}
};
int Shape::mouse_press(QMouseEvent* e)
{
	if (this->m_show_manipulator_ == false)
	{
		this->m_is_selected_ = false;
		return false;
	}
	else if (this->m_is_selected_ == true)
	{
		this->compute_mainipulator();
		
		return m_sm_->mouse_press(e);
	}
	return false;
};
int Shape::mouse_move(QMouseEvent* e, Vector3_f& vt)
{
	if (this->m_show_manipulator_ == false)
	{
		this->m_is_selected_ = false;
		return false;
	}
	else if (this->m_is_selected_ == true)
	{
		this->compute_mainipulator();
		Vector3_f v_t;
		int m = this->get_manipulator()->mouse_move(e, v_t);
		if (m >= 0)
		{
			this->translate(v_t);
			this->get_model()->get_dis_obj()->updateModelBuffer();
		}
		
		return m;
	}
	return false;
};
int	Shape::release(QMouseEvent *e)
{
	if (this->m_show_manipulator_ == false)
	{
		this->m_is_selected_ = false;
		return false;
	}
	else if(this->m_is_selected_ == true)
	{
		this->compute_mainipulator();

		return m_sm_->release(e);
	}
	return false;
};
Shape_Manipulator* Shape::get_manipulator()
{
	return this->m_sm_;
};
void Shape::compute_mainipulator()
{
	if (this->m_sm_ == NULL)
	{
		this->m_sm_ = new Shape_Manipulator();
		this->m_sm_->set_shape(this);
	}
};
bool Shape::is_selected()
{
	return this->m_is_selected_;
};
void Shape::set_selected(bool b)
{
	this->m_is_selected_ = b;
};
void Shape::setVertexList(VertexList& vertexList)
{
  //vertex_list = vertexList;
  //std::cout << "test loading speed: add_vertex().\n";
  for (size_t i = 0; i < vertexList.size() / 3; ++i)
  {
    poly_mesh->add_vertex(Vec3(vertexList[3 * i + 0], vertexList[3 * i + 1], vertexList[3 * i + 2]));
  }

  vertex_list.clear();
  for (auto vit : poly_mesh->vertices())
  {
    const Vec3& pt = poly_mesh->position(vit);
    vertex_list.push_back(pt[0]);
    vertex_list.push_back(pt[1]);
    vertex_list.push_back(pt[2]);
  } // this will update the internal variable vertex_list from poly_mesh
}

void Shape::setFaceList(FaceList& faceList)
{
  //face_list = faceList;
  //std::cout << "test loading speed: add_face().\n";
  std::vector<PolygonMesh::Vertex> vertices;
  for (size_t i = 0; i < faceList.size() / 3; ++i)
  {
    vertices.clear();
    vertices.push_back(PolygonMesh::Vertex(faceList[3 * i + 0]));
    vertices.push_back(PolygonMesh::Vertex(faceList[3 * i + 1]));
    vertices.push_back(PolygonMesh::Vertex(faceList[3 * i + 2]));
    poly_mesh->add_face(vertices);
  }

  face_list.clear();
  for (auto fit : poly_mesh->faces())
  {
    for (auto vfc_it : poly_mesh->vertices(fit))
    {
      face_list.push_back(vfc_it.idx());
    }
  } // this will update the internal variable face_list from poly_mesh
}

void Shape::setColorList(STLVectorf& colorList)
{
  //color_list = colorList;
  PolygonMesh::Vertex_attribute<Vec3> colors = poly_mesh->vertex_attribute<Vec3>("v:colors");
  for (auto vit : poly_mesh->vertices())
  {
    colors[vit] = Vec3(colorList[3 * vit.idx() + 0], colorList[3 * vit.idx() + 1], colorList[3 * vit.idx() + 2]);
  }

  color_list = colorList;
  //color_list.resize(3 * poly_mesh->n_vertices());
  ////PolygonMesh::Vertex_attribute<Vec3> colors = poly_mesh->vertex_attribute<Vec3>("v:colors");
  //for (auto vit : poly_mesh->vertices())
  //{
  //  const Vec3& color = colors[vit];
  //  color_list[3 * vit.idx() + 0] = color[0];
  //  color_list[3 * vit.idx() + 1] = color[1];
  //  color_list[3 * vit.idx() + 2] = color[2];
  //} // this will update the internal variable color_list from poly_mesh
}

void Shape::setFaceColorList(STLVectorf& facecolorList)
{
  face_color_list = facecolorList;
}

void Shape::setUVCoord(FaceList& UVIdList, STLVectorf& UVCoord, FaceList& ori_face_list)
{
  // uv coord are stored as halfedge attributes
  // we need to original face list to keep coherence between vt id and v id
  if (!UVIdList.empty() && !UVCoord.empty())
  {
    std::vector<Vec2>& he_texcoord = poly_mesh->add_attribute<std::vector<Vec2> >("he:texcoord");
    for (size_t i = 0; i < UVCoord.size() / 2; ++i)
    {
      he_texcoord.push_back(Vec2(UVCoord[2 * i + 0], UVCoord[2 * i + 1]));
    }

    face_varying_UV_list = UVCoord;
    UV_id_list = UVIdList;
    PolygonMesh::Halfedge_attribute<Vec2> f_uv_coord = poly_mesh->halfedge_attribute<Vec2>("he:face_uv");
    PolygonMesh::Halfedge_attribute<int> f_uv_id = poly_mesh->halfedge_attribute<int>("he:uv_id");
    for (size_t i = 0; i < ori_face_list.size() / 3; ++i)
    {
      // i is the face id, we need to iterate the halfedges around this face
      PolygonMesh::Halfedge he_start_v;
      PolygonMesh::Halfedge he_iter_v;
      for (auto hefc : poly_mesh->halfedges(PolygonMesh::Face(int(i))))
      {
        if (poly_mesh->to_vertex(hefc).idx() == ori_face_list[3 * i + 0])
        {
          he_start_v = hefc;
          break;
        }
      }
      for (int j = 0; j < 3; ++j)
      {
        f_uv_coord[he_start_v] = Vec2(UVCoord[2 * UVIdList[3 * i + j] + 0], UVCoord[2 * UVIdList[3 * i + j] + 1]);
        f_uv_id[he_start_v] = UVIdList[3 * i + j];
        he_start_v = poly_mesh->next_halfedge(he_start_v);
      }
    }

    PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
    for (auto fit : poly_mesh->faces())
    {
      for (auto hefc : poly_mesh->halfedges(fit))
      {
        tex_coords[poly_mesh->to_vertex(hefc)] = f_uv_coord[hefc];
      }
    }

    UV_list.resize(2 * poly_mesh->n_vertices());
    //PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
    for (auto vit : poly_mesh->vertices())
    {
      const Vec2& uv_coord = tex_coords[vit];
      UV_list[2 * vit.idx() + 0] = uv_coord[0];
      UV_list[2 * vit.idx() + 1] = uv_coord[1];
    } // this will update the internal variable UV_list from poly_mesh
  }
  else
  {
    UV_list.resize(2 * poly_mesh->n_vertices(), 0);
  }


  ////UV_list = UVCoord;
  ////std::cout << "test loading speed: add vertex attribute texture coord.\n";
  //PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
  //if (UVCoord.size() == 2 * poly_mesh->n_vertices())
  //{
  //  for (auto vit : poly_mesh->vertices())
  //  {
  //    tex_coords[vit] = Vec2(UVCoord[2 * vit.idx() + 0], UVCoord[2 * vit.idx() + 1]);
  //  }
  //}
  //else
  //{
  //  for (auto vit : poly_mesh->vertices())
  //  {
  //    tex_coords[vit] = Vec2(vit.idx(), vit.idx());
  //  }
  //}

  //UV_list.resize(2 * poly_mesh->n_vertices());
  ////PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
  //for (auto vit : poly_mesh->vertices())
  //{
  //  const Vec2& uv_coord = tex_coords[vit];
  //  UV_list[2 * vit.idx() + 0] = uv_coord[0];
  //  UV_list[2 * vit.idx() + 1] = uv_coord[1];
  //} // this will update the internal variable UV_list from poly_mesh
}

void Shape::setUVCoord(STLVectorf& UVCoord)
{
  //UV_list = UVCoord;
  //std::cout << "test loading speed: add vertex attribute texture coord.\n";
  PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
  if (UVCoord.size() == 2 * poly_mesh->n_vertices())
  {
    for (auto vit : poly_mesh->vertices())
    {
      tex_coords[vit] = Vec2(UVCoord[2 * vit.idx() + 0], UVCoord[2 * vit.idx() + 1]);
    }
  }
  else
  {
    for (auto vit : poly_mesh->vertices())
    {
      tex_coords[vit] = Vec2(vit.idx(), vit.idx());
    }
  }

  UV_list.resize(2 * poly_mesh->n_vertices());
  //PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
  for (auto vit : poly_mesh->vertices())
  {
    const Vec2& uv_coord = tex_coords[vit];
    UV_list[2 * vit.idx() + 0] = uv_coord[0];
    UV_list[2 * vit.idx() + 1] = uv_coord[1];
  } // this will update the internal variable UV_list from poly_mesh
}

const VertexList& Shape::getVertexList()
{
  return vertex_list;
}

const VertexList& Shape::getOriVertexList()
{
 return ori_vertex_list;
}

const FaceList& Shape::getFaceList()
{
  return face_list;
}

const STLVectorf& Shape::getUVCoord()
{
  return UV_list;
}

const STLVectorf& Shape::getFaceVaringUVCoord()
{
  return face_varying_UV_list;
}

const FaceList& Shape::getFaceVaringUVId()
{
  return UV_id_list;
}

const NormalList& Shape::getNormalList()
{
  return vertex_normal;
}

const NormalList& Shape::getFaceNormal()
{
  return face_normal;
}

const STLVectorf& Shape::getColorList()
{
  return color_list;
}

const STLVectorf& Shape::getFaceColorList()
{
  return face_color_list;
}

const AdjList& Shape::getVertexShareFaces()
{
  return vertex_adj_faces;
}

const AdjList& Shape::getVertexAdjList()
{
  return vertex_adjlist;
}

const AdjList& Shape::getFaceAdjList()
{
  return face_adjlist;
}

const STLVectori& Shape::getEdgeConnectivity()
{
  return edge_connectivity;
}

void Shape::buildFaceAdj()
{
  face_adjlist.clear();
  
  //for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
  //{
  //    std::vector<int> cur_face_adj;
  //    // traverse the face list to find adjacent faces
  //    for (decltype(model_faces.size()) j = 0; j < model_faces.size() / 3; ++j)
  //    {
  //        // if i == j its the same face, ignore
  //        if (i != j)
  //        {
  //            if (shareEdge((int)i, (int)j))
  //                cur_face_adj.push_back((int)j);
  //        }
  //    }
  //    model_faces_adj.push_back(cur_face_adj);
  //}


  //std::map<Edge, int> edge_face_map;
  //std::map<Edge, int>::iterator EFMap_iter;
  //face_adjlist.resize(face_list.size() / 3);

  //for (decltype(face_list.size()) face_id = 0; face_id < face_list.size() / 3; ++face_id)
  //{
  //  int pt_1 = face_list[3*face_id + 0];
  //  int pt_2 = face_list[3*face_id + 1];
  //  int pt_3 = face_list[3*face_id + 2];

  //  Edge edge_1 = pt_1 < pt_2 ? Edge(pt_1, pt_2) : Edge(pt_2, pt_1);
  //  Edge edge_2 = pt_2 < pt_3 ? Edge(pt_2, pt_3) : Edge(pt_3, pt_2);
  //  Edge edge_3 = pt_3 < pt_1 ? Edge(pt_3, pt_1) : Edge(pt_1, pt_3);

  //  EFMap_iter = edge_face_map.find(edge_1);
  //  if (EFMap_iter == edge_face_map.end())
  //    edge_face_map.insert(std::pair<Edge, int>(edge_1, face_id));
  //  else
  //  {
  //    int share_edge_face_id = edge_face_map[edge_1];
  //    face_adjlist[face_id].push_back(share_edge_face_id);
  //    face_adjlist[share_edge_face_id].push_back(face_id);
  //  }

  //  EFMap_iter = edge_face_map.find(edge_2);
  //  if (EFMap_iter == edge_face_map.end())
  //    edge_face_map.insert(std::pair<Edge, int>(edge_2, face_id));
  //  else
  //  {
  //    int share_edge_face_id = edge_face_map[edge_2];
  //    face_adjlist[face_id].push_back(share_edge_face_id);
  //    face_adjlist[share_edge_face_id].push_back(face_id);
  //  }

  //  EFMap_iter = edge_face_map.find(edge_3);
  //  if (EFMap_iter == edge_face_map.end())
  //    edge_face_map.insert(std::pair<Edge, int>(edge_3, face_id));
  //  else
  //  {
  //    int share_edge_face_id = edge_face_map[edge_3];
  //    face_adjlist[face_id].push_back(share_edge_face_id);
  //    face_adjlist[share_edge_face_id].push_back(face_id);
  //  }
  //}

  // test LgMesh
  for (auto fit : poly_mesh->faces())
  {
    std::vector<int> cur_adj;
    for (auto hffc_it : poly_mesh->halfedges(fit))
    {
      int f_id = poly_mesh->face(poly_mesh->opposite_halfedge(hffc_it)).idx();
      cur_adj.push_back(f_id);
    }
    face_adjlist.push_back(cur_adj);
  }

 /* LG::PolygonMesh::Halfedge_around_face_circulator hfc, hfce;
  LG::PolygonMesh::Face_iterator fit;
  hfc = hfce = poly_mesh->halfedges(LG::PolygonMesh::Face(1));
  do
  {
    LG::PolygonMesh::Vertex v = poly_mesh->to_vertex(*hfc);
  } while (++hfc != hfce);*/

  //std::ofstream f_debug("face_adj.txt");
  //if (f_debug)
  //{
  //    for (size_t i = 0; i < face_adjlist.size(); ++i)
  //    {
  //        for (auto &j : face_adjlist[i])
  //        {
  //            f_debug << j << "\t";
  //        }
  //        f_debug << "\n";
  //    }
  //    f_debug.close();
  //}

  //f_debug.open("face_adj_temp.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < temp_adjlist.size(); ++i)
  //  {
  //    for (auto &j : temp_adjlist[i])
  //    {
  //      f_debug << j << "\t";
  //    }
  //    f_debug << "\n";
  //  }
  //  f_debug.close();
  //}
}

void Shape::buildVertexShareFaces()
{
  vertex_adj_faces.clear();
  //vertex_adj_faces.resize(vertex_list.size() / 3);
  //for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  //{
  //  vertex_adj_faces[face_list[3 * i + 0]].push_back(i);
  //  vertex_adj_faces[face_list[3 * i + 1]].push_back(i);
  //  vertex_adj_faces[face_list[3 * i + 2]].push_back(i);
  //}

  AdjList temp_fvc_list;
  for (auto vit : poly_mesh->vertices())
  {
    std::vector<int> cur_adj;
    for (auto fit : poly_mesh->faces(vit))
    {
      cur_adj.push_back(fit.idx());
    }
    vertex_adj_faces.push_back(cur_adj);
  }

  //std::ofstream f_vert_share_face("vert_share_face.txt");
  //if (f_vert_share_face)
  //{
  //	for (size_t i = 0; i < vertex_adj_faces.size(); ++i)
  //	{
  //		for (size_t j = 0; j < vertex_adj_faces[i].size(); ++j)
  //		{
  //			f_vert_share_face << vertex_adj_faces[i][j]<<"\t";
  //		}
  //		f_vert_share_face <<"\n";
  //	}
  //	f_vert_share_face.close();
  //}

  //f_vert_share_face.open("vert_share_face_temp.txt");
  //if (f_vert_share_face)
  //{
  //  for (size_t i = 0; i < temp_fvc_list.size(); ++i)
  //  {
  //    for (size_t j = 0; j < temp_fvc_list[i].size(); ++j)
  //    {
  //      f_vert_share_face << temp_fvc_list[i][j]<<"\t";
  //    }
  //    f_vert_share_face <<"\n";
  //  }
  //  f_vert_share_face.close();
  //}
}

void Shape::buildVertexAdj()
{
  vertex_adjlist.clear();
  //vertex_adjlist.resize(vertex_list.size() / 3);
  //for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  //{
  //  vertex_adjlist[face_list[3 * i + 0]].push_back(face_list[3 * i + 1]);
  //  vertex_adjlist[face_list[3 * i + 0]].push_back(face_list[3 * i + 2]);
  //  vertex_adjlist[face_list[3 * i + 1]].push_back(face_list[3 * i + 2]);
  //  vertex_adjlist[face_list[3 * i + 1]].push_back(face_list[3 * i + 0]);
  //  vertex_adjlist[face_list[3 * i + 2]].push_back(face_list[3 * i + 0]);
  //  vertex_adjlist[face_list[3 * i + 2]].push_back(face_list[3 * i + 1]);
  //}

  //// adj list is redundant, we need to sort it and delete duplicated element
  //for (auto &i : vertex_adjlist)
  //{
  //  std::sort(i.begin(), i.end());
  //  std::vector<int>::iterator iter = std::unique(i.begin(), i.end());
  //  i.erase(iter, i.end());
  //  i.shrink_to_fit();
  //}

  AdjList temp_v_adjlist;
  for (auto vit : poly_mesh->vertices())
  {
    std::vector<int> cur_adj;
    for (auto vvc_it : poly_mesh->vertices(vit))
    {
      cur_adj.push_back(vvc_it.idx());
    }
    vertex_adjlist.push_back(cur_adj);
  }

  //std::ofstream f_debug("vertex_adj.txt");
  //if (f_debug)
  //{
  //    for (size_t i = 0; i < vertex_adjlist.size(); ++i)
  //    {
  //        for (auto &j : vertex_adjlist[i])
  //        {
  //            f_debug << j << "\t";
  //        }
  //        f_debug << "\n";
  //    }
  //    f_debug.close();
  //}

  //f_debug.open("vertex_adj_temp.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < temp_v_adjlist.size(); ++i)
  //  {
  //    for (auto &j : temp_v_adjlist[i])
  //    {
  //      f_debug << j << "\t";
  //    }
  //    f_debug << "\n";
  //  }
  //  f_debug.close();
  //}
}

void Shape::computeBounds()
{
  bound->minX = std::numeric_limits<float>::max();
  bound->maxX = std::numeric_limits<float>::min();
  bound->minY = std::numeric_limits<float>::max();
  bound->maxY = std::numeric_limits<float>::min();
  bound->minZ = std::numeric_limits<float>::max();
  bound->maxZ = std::numeric_limits<float>::min();
  float sum_x = 0,sum_y = 0,sum_z = 0;

  for (auto vit : this->poly_mesh->vertices())
  {
	  LG::Vec3& pt = this->poly_mesh->position(vit);
	  float x = pt[0];
	  float y = pt[1];
	  float z = pt[2];
	  sum_x += x;
	  sum_y += y;
	  sum_z += z;
	  if (x < bound->minX) bound->minX = x;
	  if (x > bound->maxX) bound->maxX = x;
	  if (y < bound->minY) bound->minY = y;
	  if (y > bound->maxY) bound->maxY = y;
	  if (z < bound->minZ) bound->minZ = z;
	  if (z > bound->maxZ) bound->maxZ = z;
  }
  bound->centroid.x = sum_x / (vertex_list.size() / 3);
	bound->centroid.y = sum_y / (vertex_list.size() / 3);
	bound->centroid.z = sum_z / (vertex_list.size() / 3);
  bound->setRadius();
}

void Shape::computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3])
{
  // v presents triangle vertex, pt locate inside triangle
  // pt[0]->x pt[1]->y pt[2]->z
  // v0[0]->x1 v0[1]->y1 v0[2]->z1
  // v1[0]->x2 v1[1]->y2 v1[2]->z2
  // v2[0]->x3 v2[1]->y3 v2[2]->z3
  Eigen::Vector3f e0(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
  Eigen::Vector3f e1(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
  Eigen::Vector3f e2(pt[0] - v0[0], pt[1] - v0[1], pt[2] - v0[2]);

  float d00 = e0.dot(e0);
  float d01 = e0.dot(e1);
  float d11 = e1.dot(e1);
  float d20 = e2.dot(e0);
  float d21 = e2.dot(e1);
  float denom = d00*d11 - d01*d01;

  lambd[1] = (d11*d20 - d01*d21) / denom;

  lambd[2] = (d00*d21 - d01*d20) / denom;

  lambd[0] = 1.0f - lambd[1] - lambd[2];

}

void Shape::computeFaceNormal()
{
  //face_normal.clear();

  //for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  //{
  //  Eigen::Vector3f v0;
  //  v0 << vertex_list[3 * face_list[3 * i + 0] + 0],
  //    vertex_list[3 * face_list[3 * i + 0] + 1],
  //    vertex_list[3 * face_list[3 * i + 0] + 2];

  //  Eigen::Vector3f v1;
  //  v1 << vertex_list[3 * face_list[3 * i + 1] + 0],
  //    vertex_list[3 * face_list[3 * i + 1] + 1],
  //    vertex_list[3 * face_list[3 * i + 1] + 2];

  //  Eigen::Vector3f v2;
  //  v2 << vertex_list[3 * face_list[3 * i + 2] + 0],
  //    vertex_list[3 * face_list[3 * i + 2] + 1],
  //    vertex_list[3 * face_list[3 * i + 2] + 2];

  //  Eigen::Vector3f edge_0 = v1 - v0;
  //  Eigen::Vector3f edge_1 = v2 - v1;

  //  Eigen::Vector3f cur_face_normal = edge_0.cross(edge_1);
  //  cur_face_normal.normalize();
  //  face_normal.push_back(cur_face_normal(0));
  //  face_normal.push_back(cur_face_normal(1));
  //  face_normal.push_back(cur_face_normal(2));
  //}


  //std::cout << "test loading speed: face normals.\n";
  poly_mesh->update_face_normals();

  face_normal.resize(3 * poly_mesh->n_faces());
  PolygonMesh::Face_attribute<Vec3> f_normals = poly_mesh->face_attribute<Vec3>("f:normal");
  for (auto fit : poly_mesh->faces())
  {
    const Vec3& cur_normal = f_normals[fit];
    face_normal[3 * fit.idx() + 0] = cur_normal[0];
    face_normal[3 * fit.idx() + 1] = cur_normal[1];
    face_normal[3 * fit.idx() + 2] = cur_normal[2];
  } // this will update the internal variable face_normal from poly_mesh
}

void Shape::computeVertexNormal()
{
  //vertex_normal.clear();

  //for (size_t i = 0; i < vertex_list.size()/3; ++i)
  //{
  //  std::vector<int> &cur_1_ring_face = vertex_adj_faces[i];

  //  Eigen::Vector3f cur_v_normal(0.0f,0.0f,0.0f);
  //  for (size_t j = 0; j < cur_1_ring_face.size(); ++j)
  //  {
  //    cur_v_normal(0) += face_normal[3*cur_1_ring_face[j] + 0];
  //    cur_v_normal(1) += face_normal[3*cur_1_ring_face[j] + 1];
  //    cur_v_normal(2) += face_normal[3*cur_1_ring_face[j] + 2];
  //  }
  //  cur_v_normal = cur_v_normal / cur_1_ring_face.size();
  //  cur_v_normal.normalize();

  //  vertex_normal.push_back(cur_v_normal(0));
  //  vertex_normal.push_back(cur_v_normal(1));
  //  vertex_normal.push_back(cur_v_normal(2));
  //}

  //std::cout << "test loading speed: vertex normals.\n";
  poly_mesh->update_vertex_normals();
  
  vertex_normal.resize(3 * poly_mesh->n_vertices());
  PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
  for (auto vit : poly_mesh->vertices())
  {
    const Vec3& cur_normal = v_normals[vit];
    vertex_normal[3 * vit.idx() + 0] = cur_normal[0];
    vertex_normal[3 * vit.idx() + 1] = cur_normal[1];
    vertex_normal[3 * vit.idx() + 2] = cur_normal[2];
  } // this will update the internal variable vertex_normal from poly_mesh
  //std::cout << "finished.\n";
}

void Shape::computeEdgeConnectivity()
{
  edge_connectivity.clear();
  edge_connectivity.resize(face_list.size(), -1);

  // for each edge in each face
  int start;
  int end;
  int edge_id = 0;
  int inner_index[6] = {0, 1, 1, 2, 2, 0};
  // the edge map stores edges occurred before in the "key" and the edgeIdx in the value
  std::map<std::pair<int, int>, int> edge_map;
  std::map<std::pair<int, int>, int>::iterator iter;
  for (size_t i = 0; i < face_list.size() / 3; ++i)
  {
    // iterate 3 edges in a face
    for (int j = 0; j < 3; ++j)
    {
      start = face_list[3 * i + inner_index[2 * j + 0]];
      end = face_list[3 * i + inner_index[2 * j + 1]];
      iter = edge_map.find(std::pair<int, int>(end, start));
      if (iter == edge_map.end())
      {
        // if not occurred before, store it
        edge_map[std::pair<int, int>(start, end)] = edge_id;
      }
      else
      {
        // found an opposite edge
        // set the two into the connectivityOut
        edge_connectivity[iter->second] = edge_id;
        edge_connectivity[edge_id] = iter->second;
      }
      ++edge_id;
    }
  }
}

Bound* Shape::getBoundbox()
{
  return bound.get();
}

float Shape::avgEdgeLength()
{
  float total_length = 0.0;
  for (int i = 0; i < face_list.size() / 3; ++i)
  {
    int v[3] = { face_list[3 * i + 0], face_list[3 * i + 1], face_list[3 * i + 2] };
    Vector3f v_0(vertex_list[3 * v[0] + 0], vertex_list[3 * v[0] + 1], vertex_list[3 * v[0] + 2]);
    Vector3f v_1(vertex_list[3 * v[1] + 0], vertex_list[3 * v[1] + 1], vertex_list[3 * v[1] + 2]);
    Vector3f v_2(vertex_list[3 * v[2] + 0], vertex_list[3 * v[2] + 1], vertex_list[3 * v[2] + 2]);
    total_length += (v_0 - v_1).norm() + (v_1 - v_2).norm() + (v_2 - v_0).norm();
  }
  return total_length / face_list.size();
}

std::shared_ptr<KDTreeWrapper> Shape::getKDTree()
{
  return kdTree;
}

void Shape::buildKDTree()
{
  kdTree.reset(new KDTreeWrapper);
  kdTree->initKDTree(vertex_list, vertex_list.size() / 3, 3);
}

void Shape::updateShape(VertexList& new_vertex_list)
{
  vertex_list = new_vertex_list;

  for (auto vit : poly_mesh->vertices())
  {
    poly_mesh->position(vit) = Vec3(vertex_list[3 * vit.idx() + 0], vertex_list[3 * vit.idx() + 1], vertex_list[3 * vit.idx() + 2]);
  } // this will update the internal variable poly_mesh from vertex_list

  computeFaceNormal();

  computeVertexNormal();

  computeBounds();

  buildKDTree();

  computeShadowSHCoeffs();
}

void Shape::getFaceCenter(int f_id, float p[3])
{
  size_t v0 = face_list[3 * f_id + 0];
  size_t v1 = face_list[3 * f_id + 1];
  size_t v2 = face_list[3 * f_id + 2];

  p[0] = vertex_list[3 * v0 + 0] / 3 + vertex_list[3 * v1 + 0] / 3 + vertex_list[3 * v2 + 0] / 3;
  p[1] = vertex_list[3 * v0 + 1] / 3 + vertex_list[3 * v1 + 1] / 3 + vertex_list[3 * v2 + 1] / 3;
  p[2] = vertex_list[3 * v0 + 2] / 3 + vertex_list[3 * v1 + 2] / 3 + vertex_list[3 * v2 + 2] / 3;
}

void Shape::computeShadowSHCoeffs(int num_band)
{
  // only called when the shape is changed

  // 1. initialize BSPTree
  std::cout << "Initialize BSPTree.\n";
  std::shared_ptr<Ray> ray(new Ray);
  ray->passModel(vertex_list, face_list);

  // 2. generate direction samples
  std::cout << "Generate samples.\n";
  int sqrtNumSamples = 50;
  int numSamples = sqrtNumSamples * sqrtNumSamples;
  std::vector<SAMPLE> samples(numSamples);
  GenerateSamples(sqrtNumSamples, num_band, &samples[0]);

  // 3. compute coeffs
  std::cout << "Compute SH Coefficients.\n";
  int numBand = num_band;
  int numFunctions = numBand * numBand;
  PolygonMesh::Vertex_attribute<STLVectorf> shadowCoeff = poly_mesh->vertex_attribute<STLVectorf>("v:SHShadowCoeffs");
  PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
  float perc = 0;
  for (auto i : poly_mesh->vertices())
  {
    shadowCoeff[i].clear();
    shadowCoeff[i].resize(numFunctions, 0.0f);
    for (int k = 0; k < numSamples; ++k)
    {
      double dot = (double)samples[k].direction.dot(v_normals[i]);

      if (dot > 0.0)
      {
        //Eigen::Vector3d ray_start = (poly_mesh->position(i) + 2 * 0.01 * bound->getRadius() * v_normals[i]).cast<double>();
        //Eigen::Vector3d ray_end   = ray_start + (5 * bound->getRadius() * samples[k].direction).cast<double>();
        //if (ray->intersectModel(ray_start, ray_end))
        {
          for (int l = 0; l < numFunctions; ++l)
          {
            shadowCoeff[i][l] += dot * samples[k].shValues[l];
          }
        }
      }
    }

    // rescale
    for (int l = 0; l < numFunctions; ++l)
    {
      shadowCoeff[i][l] *= 4.0 * M_PI / numSamples;
    }

    float cur_perc = (float)i.idx() / poly_mesh->n_vertices();
    if (cur_perc - perc >= 0.05)
    {
      perc = cur_perc;
      std::cout << perc << "...";
    }
  }
  std::cout << "Compute SH coefficients finished.\n";
}
bool Shape::wheel(QWheelEvent *e)
{
	if (this->m_show_manipulator_ == false)
	{
		return false;
	}
	else if (this->get_manipulator() == NULL)
	{
		return false;
	}

	float angle;
	Vector3_f v_line;
	Point3f center;
	float scale;
	int	rotate_scale;
	bool edited = this->get_manipulator()->wheel(e, v_line, angle, center, scale, rotate_scale);

	if (edited)
	{
		if ((e->modifiers() == Qt::ControlModifier) && rotate_scale  == 1)
		{
			CvPoint3D32f c = this->getBoundbox()->centroid;
			Point3f center_scale = Point3f(c.x, c.y, c.z);
			this->scale(center_scale, scale);
			this->computeBounds();
			this->get_model()->get_dis_obj()->updateModelBuffer();
			return true;
		}
		else if (e->modifiers() == Qt::AltModifier && rotate_scale == 2)
		{
			CvPoint3D32f c = this->getBoundbox()->centroid;
			Point3f center_scale = Point3f(c.x, c.y, c.z);
			this->scale_along_line(center_scale, v_line, scale);
			this->computeBounds();
			this->get_model()->get_dis_obj()->updateModelBuffer();

			return true;
		}
		else if (rotate_scale == 0)
		{
			CvPoint3D32f c = this->getBoundbox()->centroid;
			Point3f center_scale = Point3f(c.x, c.y, c.z);
			this->rotate(center_scale, v_line, angle);
			this->computeBounds();
			this->get_model()->get_dis_obj()->updateModelBuffer();
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool Shape::translate(Vector3_f v_t)
{
	LG::Vec3 vv_t(v_t.x(), v_t.y(), v_t.z());
	PolygonMesh_Manipulator::translate(this->getPolygonMesh(), vv_t);
	this->computeBounds();
	return true;
};
bool Shape::rotate(const Point3f& p_on_line, const Vector3_f& vline, const float& angle)
{
	LG::Vec3 p_t(p_on_line.x(), p_on_line.y(), p_on_line.z());
	LG::Vec3 v_t(vline.x(), vline.y(), vline.z());
	PolygonMesh_Manipulator::rotate(this->getPolygonMesh(), p_t, v_t, angle);
	this->computeBounds();
	return true;
};
bool Shape::scale(const Point3f& standard, const float& scale)
{
	CvPoint3D32f c = this->getBoundbox()->centroid;
	LG::Vec3 center(c.x, c.y, c.z);
	PolygonMesh_Manipulator::scale(this->getPolygonMesh(), center, scale);
	return true;
};
bool Shape::scale_along_line(const Point3f& standard, Vector3_f v_line, const float& scale)
{
	CvPoint3D32f c = this->getBoundbox()->centroid;
	LG::Vec3 center(c.x, c.y, c.z);
	LG::Vec3 axis(v_line.x(), v_line.y(), v_line.z());
	PolygonMesh_Manipulator::scale_along_axis(this->getPolygonMesh(), center, scale, axis);
	return true;
};
Model* Shape::get_model()
{
	return this->m_model_;
};
void Shape::set_model(Model* m)
{
	this->m_model_ = m;
};