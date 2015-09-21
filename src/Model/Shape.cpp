#include "Shape.h"
#include "Bound.h"

#include <set>

Shape::Shape()
  : bound(new Bound())
{
}

Shape::~Shape()
{
}

void Shape::init(VertexList& vertexList, FaceList& faceList, STLVectorf& UVList)
{
  setVertexList(vertexList);
  setFaceList(faceList);
  setUVCoord(UVList);

  color_list.resize(vertex_list.size(), 0.5);

  std::cout<<"Building face adjacent list...\n";
  buildFaceAdj();

  std::cout<<"Building 1-ring neighbors list...\n";
  buildVertexShareFaces();

  std::cout<<"Building vertex adjacent list...\n";
  buildVertexAdj();

  std::cout<<"Computing bounding box...\n";
  computeBounds();

  std::cout<<"Computing face normals...\n";
  computeFaceNormal();
  computeVertexNormal();

}

void Shape::setVertexList(VertexList& vertexList)
{
  vertex_list = vertexList;
}

void Shape::setFaceList(FaceList& faceList)
{
  face_list = faceList;
}

void Shape::setUVCoord(STLVectorf& UVCoord)
{
  UV_list = UVCoord;
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

const STLVectorf& Shape::getColorList()
{
  return color_list;
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


  std::map<Edge, int> edge_face_map;
  std::map<Edge, int>::iterator EFMap_iter;
  face_adjlist.resize(face_list.size() / 3);

  for (decltype(face_list.size()) face_id = 0; face_id < face_list.size() / 3; ++face_id)
  {
    int pt_1 = face_list[3*face_id + 0];
    int pt_2 = face_list[3*face_id + 1];
    int pt_3 = face_list[3*face_id + 2];

    Edge edge_1 = pt_1 < pt_2 ? Edge(pt_1, pt_2) : Edge(pt_2, pt_1);
    Edge edge_2 = pt_2 < pt_3 ? Edge(pt_2, pt_3) : Edge(pt_3, pt_2);
    Edge edge_3 = pt_3 < pt_1 ? Edge(pt_3, pt_1) : Edge(pt_1, pt_3);

    EFMap_iter = edge_face_map.find(edge_1);
    if (EFMap_iter == edge_face_map.end())
      edge_face_map.insert(std::pair<Edge, int>(edge_1, face_id));
    else
    {
      int share_edge_face_id = edge_face_map[edge_1];
      face_adjlist[face_id].push_back(share_edge_face_id);
      face_adjlist[share_edge_face_id].push_back(face_id);
    }

    EFMap_iter = edge_face_map.find(edge_2);
    if (EFMap_iter == edge_face_map.end())
      edge_face_map.insert(std::pair<Edge, int>(edge_2, face_id));
    else
    {
      int share_edge_face_id = edge_face_map[edge_2];
      face_adjlist[face_id].push_back(share_edge_face_id);
      face_adjlist[share_edge_face_id].push_back(face_id);
    }

    EFMap_iter = edge_face_map.find(edge_3);
    if (EFMap_iter == edge_face_map.end())
      edge_face_map.insert(std::pair<Edge, int>(edge_3, face_id));
    else
    {
      int share_edge_face_id = edge_face_map[edge_3];
      face_adjlist[face_id].push_back(share_edge_face_id);
      face_adjlist[share_edge_face_id].push_back(face_id);
    }
  }

  //    std::ofstream f_debug(getDataPath()+"/face_adj.txt");
  //if (f_debug)
  //{
  //    for (size_t i = 0; i < model_faces_adj.size(); ++i)
  //    {
  //        for (auto &j : model_faces_adj[i])
  //        {
  //            f_debug << j << "\t";
  //        }
  //        f_debug << "\n";
  //    }
  //    f_debug.close();
  //}

}

void Shape::buildVertexShareFaces()
{
  vertex_adj_faces.clear();
  vertex_adj_faces.resize(vertex_list.size() / 3);
  for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  {
    vertex_adj_faces[face_list[3 * i + 0]].push_back(i);
    vertex_adj_faces[face_list[3 * i + 1]].push_back(i);
    vertex_adj_faces[face_list[3 * i + 2]].push_back(i);
  }

  //std::ofstream f_vert_share_face(getDataPath() + "/vert_share_face.mat");
  //if (f_vert_share_face)
  //{
  //	for (size_t i = 0; i < model_vertices_share_faces.size(); ++i)
  //	{
  //		for (size_t j = 0; j < model_vertices_share_faces[i].size(); ++j)
  //		{
  //			f_vert_share_face << model_vertices_share_faces[i][j]<<"\t";
  //		}
  //		f_vert_share_face <<"\n";
  //	}
  //	f_vert_share_face.close();
  //}
}

void Shape::buildVertexAdj()
{
  vertex_adjlist.clear();
  vertex_adjlist.resize(vertex_list.size() / 3);
  for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  {
    vertex_adjlist[face_list[3 * i + 0]].push_back(face_list[3 * i + 1]);
    vertex_adjlist[face_list[3 * i + 0]].push_back(face_list[3 * i + 2]);
    vertex_adjlist[face_list[3 * i + 1]].push_back(face_list[3 * i + 2]);
    vertex_adjlist[face_list[3 * i + 1]].push_back(face_list[3 * i + 0]);
    vertex_adjlist[face_list[3 * i + 2]].push_back(face_list[3 * i + 0]);
    vertex_adjlist[face_list[3 * i + 2]].push_back(face_list[3 * i + 1]);
  }

  // adj list is redundant, we need to sort it and delete duplicated element
  for (auto &i : vertex_adjlist)
  {
    std::sort(i.begin(), i.end());
    std::vector<int>::iterator iter = std::unique(i.begin(), i.end());
    i.erase(iter, i.end());
    i.shrink_to_fit();
  }
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
  face_normal.clear();

  for (decltype(face_list.size()) i = 0; i < face_list.size() / 3; ++i)
  {
    Eigen::Vector3f v0;
    v0 << vertex_list[3 * face_list[3 * i + 0] + 0],
      vertex_list[3 * face_list[3 * i + 0] + 1],
      vertex_list[3 * face_list[3 * i + 0] + 2];

    Eigen::Vector3f v1;
    v1 << vertex_list[3 * face_list[3 * i + 1] + 0],
      vertex_list[3 * face_list[3 * i + 1] + 1],
      vertex_list[3 * face_list[3 * i + 1] + 2];

    Eigen::Vector3f v2;
    v2 << vertex_list[3 * face_list[3 * i + 2] + 0],
      vertex_list[3 * face_list[3 * i + 2] + 1],
      vertex_list[3 * face_list[3 * i + 2] + 2];

    Eigen::Vector3f edge_0 = v1 - v0;
    Eigen::Vector3f edge_1 = v2 - v1;

    Eigen::Vector3f cur_face_normal = edge_0.cross(edge_1);
    cur_face_normal.normalize();
    face_normal.push_back(cur_face_normal(0));
    face_normal.push_back(cur_face_normal(1));
    face_normal.push_back(cur_face_normal(2));
  }
}

void Shape::computeVertexNormal()
{
  vertex_normal.clear();

  for (size_t i = 0; i < vertex_list.size()/3; ++i)
  {
    std::vector<int> &cur_1_ring_face = vertex_adj_faces[i];

    Eigen::Vector3f cur_v_normal(0.0f,0.0f,0.0f);
    for (size_t j = 0; j < cur_1_ring_face.size(); ++j)
    {
      cur_v_normal(0) += face_normal[3*cur_1_ring_face[j] + 0];
      cur_v_normal(1) += face_normal[3*cur_1_ring_face[j] + 1];
      cur_v_normal(2) += face_normal[3*cur_1_ring_face[j] + 2];
    }
    cur_v_normal = cur_v_normal / cur_1_ring_face.size();
    cur_v_normal.normalized();

    vertex_normal.push_back(cur_v_normal(0));
    vertex_normal.push_back(cur_v_normal(1));
    vertex_normal.push_back(cur_v_normal(2));
  }
}

Bound* Shape::getBoundbox()
{
  return bound.get();
}