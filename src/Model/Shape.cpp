#include "Shape.h"
#include "Bound.h"
#include "KDTreeWrapper.h"
#include "PolygonMesh.h"

#include <set>
#include <fstream>

using namespace LG;

Shape::Shape()
  : bound(new Bound())
{
}

Shape::~Shape()
{
  std::cout << "Deleted a Shape.\n";
}

void Shape::init(VertexList& vertexList, FaceList& faceList, STLVectorf& UVList)
{
  poly_mesh.reset(new PolygonMesh());

  setVertexList(vertexList);
  setFaceList(faceList);
  setUVCoord(UVList);

  color_list.resize(vertex_list.size(), 0.5);
  PolygonMesh::Vertex_attribute<Vec3> colors = poly_mesh->vertex_attribute<Vec3>("v:colors");
  for (auto vit : poly_mesh->vertices())
  {
    colors[vit] = Vec3(0.5, 0.5, 0.5);
  }


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

  color_list.resize(3 * poly_mesh->n_vertices());
  //PolygonMesh::Vertex_attribute<Vec3> colors = poly_mesh->vertex_attribute<Vec3>("v:colors");
  for (auto vit : poly_mesh->vertices())
  {
    const Vec3& color = colors[vit];
    color_list[3 * vit.idx() + 0] = color[0];
    color_list[3 * vit.idx() + 1] = color[1];
    color_list[3 * vit.idx() + 2] = color[2];
  } // this will update the internal variable color_list from poly_mesh
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

const FaceList& Shape::getFaceList()
{
  return face_list;
}

const STLVectorf& Shape::getUVCoord()
{
  return UV_list;
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
  for (decltype(vertex_list.size()) i = 0; i < vertex_list.size() / 3; ++i)
  {
    float x = vertex_list[3 * i + 0];
    float y = vertex_list[3 * i + 1];
    float z = vertex_list[3 * i + 2];
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