#ifndef Shape_H
#define Shape_H

#include "BasicHeader.h"

#include <memory>

class Bound;
class KDTreeWrapper;

// a triangle mesh class
// if we need a more abstract shape base class in the future
// we can move the implementation into TriMesh
class Shape
{
public:
  Shape();
  virtual ~Shape();

  void init(VertexList& vertexList, FaceList& faceList, STLVectorf& UVList);
  void setVertexList(VertexList& vertexList);
  void setFaceList(FaceList& faceList);
  void setUVCoord(STLVectorf& UVCoord);
  const VertexList& getVertexList();
  const FaceList& getFaceList();
  const STLVectorf& getUVCoord();
  const NormalList& getNormalList();
  const NormalList& getFaceNormal();
  const STLVectorf& getColorList();
  const AdjList& getVertexShareFaces();
  const AdjList& getVertexAdjList();
  const AdjList& getFaceAdjList();
  const STLVectori& getEdgeConnectivity();
  void getFaceCenter(int f_id, float p[3]);
  Bound* getBoundbox();
  float avgEdgeLength();
  void buildKDTree();
  std::shared_ptr<KDTreeWrapper> getKDTree();
  void updateShape(VertexList& new_vertex_list);

private:
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);
  void buildFaceAdj();
  void buildVertexShareFaces();
  void buildVertexAdj();
  void computeBounds();
  void computeFaceNormal();
  void computeVertexNormal();
  void computeEdgeConnectivity();

private:
  // geometry information
  VertexList vertex_list;
  FaceList   face_list;

  // connectivity information
  AdjList    face_adjlist;
  AdjList    vertex_adjlist;
  AdjList    vertex_adj_faces; // vertex one-ring faces
  STLVectori edge_connectivity; // edge id is stored implicitly in the array index, the value stores edge id of the other half edge to it

  // attribute
  NormalList vertex_normal;
  NormalList face_normal;
  STLVectorf UV_list;
  STLVectorf color_list;

  std::unique_ptr<Bound> bound; // Shape is the owner of bound
  std::shared_ptr<KDTreeWrapper> kdTree;

private:
  Shape(const Shape&);
  void operator = (const Shape&);
};

#endif