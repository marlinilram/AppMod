#ifndef MeshParameterization_H
#define MeshParameterization_H

#include <memory>
#include <map>
#include<set>
#include <string>
#include <cv.h>
#include "BasicHeader.h"

class Model;
class Shape;
class KDTreeWrapper;
class ParaShape;
class DetailSynthesis;

class MeshParameterization
{
public:
  MeshParameterization();
  ~MeshParameterization();

  void init();
  void doMeshParameterization(std::shared_ptr<Model> model);
  //void saveParameterization(std::string file_path, std::shared_ptr<Shape> shape, std::string fname);
  void doMeshParamterizationPatch(std::shared_ptr<Model> model, int plane_id, ParaShape* one_patch); // the API is not fixed yet here
  void doMeshParamterizationPatch(std::shared_ptr<Model> model, const std::set<int>& f_ids, ParaShape* one_patch, int start_v_id = -1); // start_v_id here is in original shape

private:
  // prepare cut_shape
  void cutMesh(std::shared_ptr<Model> model);
  void prepareCutShape(std::shared_ptr<Model> model, FaceList& f_list, STLVectori& v_set, std::shared_ptr<Shape>& shape);
  void findBoundary(std::shared_ptr<Shape> shape, STLVectori& b_loop, int start_v_id = -1); // start_v_id here is the v_id in para shape
  void fillHoles(std::set<int>& visible_faces, const AdjList& f_adjList);
  void findConnectedFaces(int f_id, std::set<int>& connected_faces, const std::set<int>& visible_faces, const AdjList& adj_list);
  void connectedComponents(std::vector<std::set<int> >& components, const std::set<int>& visible_faces, const AdjList& adj_list);
  int findLargestComponent(const std::vector<std::set<int> >& components);
  void expandCutShape(std::shared_ptr<Model> model, std::set<int>& f_id_set);
  bool eliminateSingleFace(std::shared_ptr<Model> model, std::set<int>& f_id_set);
  void eliminateSingleFaceAll(std::shared_ptr<Model> model, std::set<int>& f_id_set);

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

private:
  friend DetailSynthesis;

public:
  std::unique_ptr<ParaShape> seen_part;
  std::unique_ptr<ParaShape> unseen_part;
  std::vector<ParaShape> shape_patches;

  NormalList normal_original_mesh;
  VertexList vertex_original_mesh;

private:
  MeshParameterization(const MeshParameterization&);
  void operator = (const MeshParameterization&);
};

#endif // !MeshParameterization_H
