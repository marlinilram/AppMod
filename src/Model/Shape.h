#ifndef Shape_H
#define Shape_H

#include "BasicHeader.h"

#include <memory>

class Bound;

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
  const STLVectorf& getColorList();
  Bound* getBoundbox();
  float avgEdgeLength();

private:
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);
  void buildFaceAdj();
  void buildVertexShareFaces();
  void buildVertexAdj();
  void computeBounds();
  void computeFaceNormal();
  void computeVertexNormal();

private:
  VertexList vertex_list;
  FaceList   face_list;

  NormalList vertex_normal;
  NormalList face_normal;

  AdjList    face_adjlist;
  AdjList    vertex_adjlist;
  AdjList    vertex_adj_faces; // vertex one-ring faces

  // attribute
  STLVectorf UV_list;
  STLVectorf color_list;

  std::unique_ptr<Bound> bound; // Shape is the owner of bound

private:
  Shape(const Shape&);
  void operator = (const Shape&);
};

#endif