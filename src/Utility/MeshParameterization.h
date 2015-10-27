#ifndef MeshParameterization_H
#define MeshParameterization_H

#include <memory>
#include <map>
#include<set>
#include <string>
#include "BasicHeader.h"

class Model;
class Shape;
class KDTreeWrapper;

class MeshParameterization
{
public:
  MeshParameterization();
  ~MeshParameterization();

  void init();
  void doMeshParameterization(std::shared_ptr<Model> model);
  void saveParameterization(std::string file_path, std::shared_ptr<Shape> shape, bool is_hidden);

private:
  // prepare cut_shape
  void cutMesh(std::shared_ptr<Model> model);
  void prepareCutShape(std::shared_ptr<Model> model, FaceList& f_list, STLVectori& v_set, std::shared_ptr<Shape>& shape);
  void findBoundary(std::shared_ptr<Shape> shape, STLVectori& b_loop);
  void fillHoles(std::set<int>& visible_faces, const AdjList& f_adjList);
  void findConnectedFaces(int f_id, std::set<int>& connected_faces, const std::set<int>& visible_faces, const AdjList& adj_list);
  void connectedComponents(std::vector<std::set<int> >& components, const std::set<int>& visible_faces, const AdjList& adj_list);
  int findLargestComponent(const std::vector<std::set<int> >& components);

  // barycentric parameterization
  void computeBaryCentericPara(std::shared_ptr<Shape>& shape, STLVectori& b_loop);
  void mapBoundary(STLVectorf& UV_list, const STLVectori& boundary_loop, const VertexList& vertex_list, int b_type = 0);
  void computeLaplacianWeight(int v_id, std::map<int, float>& weight, std::shared_ptr<Shape> shape);
  void findShareVertex(int pi, int pj, STLVectori& share_vertex, std::shared_ptr<Shape> shape);
  float computeWij(const float *p1, const float *p2, const float *p3, const float *p4 = nullptr);

  // build kd-tree for uv_list
  void buildKDTree_UV();

  // get shape of original mesh
  void getNormalOfOriginalMesh(std::shared_ptr<Model> model);
  void getVertexOfOriginalMesh(std::shared_ptr<Model> model);

public:
  FaceList cut_face_list; // triplets which store the old vertex id of the faces
  STLVectori vertex_set; // vertex id mapping from new id to old id
  STLVectori boundary_loop;
  std::shared_ptr<Shape> cut_shape;
  std::shared_ptr<KDTreeWrapper> kdTree_UV;
  NormalList normal_original_mesh;
  VertexList vertex_original_mesh;

  FaceList cut_face_list_hidden;
  STLVectori vertex_set_hidden;
  STLVectori boundary_loop_hidden;
  std::shared_ptr<Shape> cut_shape_hidden;

private:
  MeshParameterization(const MeshParameterization&);
  void operator = (const MeshParameterization&);
};

#endif // !MeshParameterization_H
