#ifndef Shape_H
#define Shape_H

#include "BasicHeader.h"
#include "geometry_types.h"
#include <memory>

class Bound;
class KDTreeWrapper;
class QGLViewer;
class Shape_Manipulator;
class Model;
#include <QtGui/QMouseEvent>
namespace LG {
class PolygonMesh;
}

// a triangle mesh class
// if we need a more abstract shape base class in the future
// we can move the implementation into TriMesh
class Shape
{
public:
  Shape();
  virtual ~Shape();

  void init(VertexList& vertexList, FaceList& faceList, FaceList& UVIdList, STLVectorf& UVList);
  void setVertexList(VertexList& vertexList);
  void setFaceList(FaceList& faceList);
  void setColorList(STLVectorf& colorList);
  void setFaceColorList(STLVectorf& facecolorList);
  void setUVCoord(FaceList& UVIdList, STLVectorf& UVCoord, FaceList& ori_face_list);
  void setUVCoord(STLVectorf& UVCoord);
  const VertexList& getVertexList();
  const VertexList& getOriVertexList();
  const FaceList& getFaceList();
  const STLVectorf& getUVCoord();
  const STLVectorf& getFaceVaringUVCoord();
  const FaceList& getFaceVaringUVId();
  const NormalList& getNormalList();
  const NormalList& getFaceNormal();
  const STLVectorf& getColorList();
  const STLVectorf& getFaceColorList();
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
  void getBaryCentreCoord(float pt[3],int face_id,float lambda[3]);
  LG::PolygonMesh* getPolygonMesh() { return poly_mesh.get(); };
  const void draw_manipulator();

  bool show_mani();
  void set_show_mani(bool b);

  QGLViewer* glviewer();
  void set_glviewer(QGLViewer* g);

  bool double_click(QMouseEvent* e, int& activated);
  int mouse_press(QMouseEvent* e);
  int mouse_move(QMouseEvent* e, Vector3_f& vt);
  int release(QMouseEvent *e);
  bool wheel(QWheelEvent *e);

  bool is_selected();
  void set_selected(bool b);

  bool translate(Vector3_f v_t);
  bool rotate(const Point3f& p_on_line, const Vector3_f& vline, const float& angle);
  bool scale(const Point3f& standard, const float& scale);
  bool scale_along_line(const Point3f& standard, Vector3_f v_line, const float& scale);

  Model* get_model();
  void	 set_model(Model* m);
private:
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);
  void buildFaceAdj();
  void buildVertexShareFaces();
  void buildVertexAdj();
  void computeBounds();
  void computeFaceNormal();
  void computeVertexNormal();
  void computeEdgeConnectivity();
  void computeShadowSHCoeffs(int num_band = 3);
  Shape_Manipulator* get_manipulator();
  void compute_mainipulator();
private:
  // PolygonMesh
  std::shared_ptr<LG::PolygonMesh> poly_mesh;
  bool m_show_manipulator_;
  QGLViewer* m_viewer_;
  bool m_is_selected_;
  Shape_Manipulator* m_sm_;
  // geometry information
  VertexList vertex_list;
  FaceList   face_list;
  VertexList ori_vertex_list;

  // connectivity information
  AdjList    face_adjlist;
  AdjList    vertex_adjlist;
  AdjList    vertex_adj_faces; // vertex one-ring faces
  STLVectori edge_connectivity; // edge id is stored implicitly in the array index, the value stores edge id of the other half edge to it

  // attribute
  NormalList vertex_normal;
  NormalList face_normal;
  FaceList UV_id_list;
  STLVectorf UV_list;
  STLVectorf face_varying_UV_list;
  STLVectorf color_list;
  STLVectorf face_color_list;

  std::unique_ptr<Bound> bound; // Shape is the owner of bound
  std::shared_ptr<KDTreeWrapper> kdTree;

private:

  Shape(const Shape&);
  void operator = (const Shape&);
  Model* m_model_;
};

typedef Shape  Render_Shape;

#endif