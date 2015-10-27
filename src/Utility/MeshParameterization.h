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

class MeshParameterization
{
public:
  MeshParameterization();
  ~MeshParameterization();

  void init();
  void doMeshParameterization(std::shared_ptr<Model> model);
  void saveParameterization(std::string file_path);

private:
  // prepare cut_shape
  void cutMesh(std::shared_ptr<Model> model);
  void prepareCutShape(std::shared_ptr<Model> model);
  void findBoundary();
  void fillHoles(std::set<int>& visible_faces, const AdjList& f_adjList);
  void findConnectedFaces(int f_id, std::set<int>& connected_faces, const std::set<int>& visible_faces, const AdjList& adj_list);
  void connectedComponents(std::vector<std::set<int> >& components, const std::set<int>& visible_faces, const AdjList& adj_list);
  int findLargestComponent(const std::vector<std::set<int> >& components);

  // barycentric parameterization
  void computeBaryCentericPara();
  void mapBoundary(STLVectorf& UV_list, const STLVectori& boundary_loop, const VertexList& vertex_list, int b_type = 0);
  void computeLaplacianWeight(int v_id, std::map<int, float>& weight);
  void findShareVertex(int pi, int pj, STLVectori& share_vertex);
  float computeWij(const float *p1, const float *p2, const float *p3, const float *p4 = nullptr);

public:
  FaceList cut_face_list; // triplets which store the old vertex id of the faces
  STLVectori vertex_set; // vertex id mapping from new id to old id
  STLVectori boundary_loop;
  std::shared_ptr<Shape> cut_shape;

private:
  MeshParameterization(const MeshParameterization&);
  void operator = (const MeshParameterization&);
};

#endif // !MeshParameterization_H
