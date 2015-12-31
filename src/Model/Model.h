#ifndef Model_H
#define Model_H

#include <iostream>
#include <memory>
#include <string>
#include <cv.h>
#include <highgui.h>

#include "BasicHeader.h"

class Shape;
class Sphere;
class ShapeCrest;
class ShapePlane;
class Bound;
namespace LG {
class PolygonMesh;
}

class Model
{
public:
  Model();
  Model(const std::string path, const std::string name);
  ~Model();

  bool loadOBJ(const std::string name, const std::string path);
  void exportOBJ(int cur_iter);

  Bound* getBoundBox();
  //std::shared_ptr<Shape> getShape() { return shape; };
  //std::shared_ptr<ShapeCrest> getShapeCrest();
  LG::PolygonMesh* getPolygonMesh();
  LG::PolygonMesh* getLightPolygonMesh();
  std::string getDataPath();
  std::string getOutputPath();

  //
  void updateShape(VertexList& new_vertex_list);
  void updateColor();
  void updateColorList(STLVectorf& colorList);
  void updateSHColor();
  void updateUVCoord(STLVectorf& new_uv_list);

  // get information from renderer
  inline cv::Mat &getRImg(){ return r_img; };
  inline cv::Mat &getRBGRAImg() { return rBGRA_img; };
  inline cv::Mat &getPrimitiveIDImg(){ return primitive_ID; };
  inline cv::Mat &getZImg() { return z_img; };
  inline cv::Mat &getRMask() { return mask_rimg; };
  inline cv::Mat &getEdgeImg() { return edge_image; };
  inline cv::Mat &getNImg() { return n_img; };
  inline cv::Mat &getOriRImg() { return ori_reflectance_img; };
  inline cv::Mat &getSynRImg() { return synthesis_reflectance_img; };
  inline double getZScale() { return z_scale; };
  inline void setZScale(double z_scale_) { z_scale = z_scale_; };

  // get information from Shape
  const VertexList& getShapeVertexList();
  const VertexList& getShapeOriVertexList();
  const FaceList& getShapeFaceList();
  const STLVectorf& getShapeUVCoord();
  const NormalList& getShapeNormalList();
  const NormalList& getShapeFaceNormal();
  const STLVectorf& getShapeColorList();
  const STLVectorf& getShapeFaceColorList();
  const AdjList& getShapeVertexShareFaces();
  const AdjList& getShapeVertexAdjList();
  const AdjList& getShapeFaceAdjList();
  const STLVectori& getShapeEdgeConnectivity();
  void getShapeFaceCenter(int f_id, float p[3]);
  std::map<int, int>& getVisbleGlobalMapper();
  std::map<int, std::vector<int>>& getGlobalVisibleMapper();

  // get information from ShapeCrest
  const std::vector<Edge>& getShapeCrestEdge();
  const std::vector<STLVectori>& getShapeCrestLine();
  const std::vector<STLVectori>& getShapeVisbleCrestLine();
  void computeShapeCrestVisible(std::set<int>& vis_faces);
  void updateShapeCrest();

  // some computations utility of Model
  float getModelAvgEdgeLength();
  void passCameraPara(float c_modelview[16], float c_projection[16], int c_viewport[4]);
  bool getWorldCoord(Vector3f rimg_coord, Vector3f &w_coord);
  bool getUnprojectPt(float winx, float winy, float winz, float object_coord[3]);
  int getClosestVertexId(float world_pos[3], int x, int y);
  void getCameraOri(float camera_ori[3]);
  void getProjRay(float proj_ray[3], int x, int y);
  bool getProjectPt(float object_coord[3], float &winx, float &winy);
  bool getProjectPt(const int vid, float& winx, float& winy);
  void getUnprojectVec(Vector3f& vec);
  void getProjectionMatrix(Matrix4f& proj_mat_out);

  // get info from ShapePlane
  void addTaggedPlane(int x, int y);
  void clearTaggedPlanes();
  void getTaggedPlaneVertices(std::vector<STLVectori>& vertices);
  void getPlaneVertices(std::vector<STLVectori>& vertices);
  const std::vector<std::set<int> >& getPlaneFaces();
  std::vector<std::pair<Vector3f, Vector3f>>& getOriginalPlaneCenter();
  std::vector<std::pair<Vector3f, Vector3f>>& getPlaneCenter();
  std::vector<std::set<int>>& getFlatSurfaces();
  void findCrspPatch(int input_id, int& output_id, std::vector<int>& candidate);

private:
  std::shared_ptr<Shape> shape; // Model is the owner of Shape
  std::shared_ptr<Sphere> lighting_ball;
  std::shared_ptr<ShapeCrest> shape_crest;
  std::shared_ptr<ShapePlane> shape_plane;

  // file system data
  std::string data_path;
  std::string file_name;
  std::string output_path;

  // render info
  cv::Mat z_img;
  cv::Mat n_img; // normal image
  cv::Mat primitive_ID;
  cv::Mat rBGRA_img;
  cv::Mat r_img;
  cv::Mat mask_rimg;
  cv::Mat edge_image;
  cv::Mat photo;
  cv::Mat ori_reflectance_img;
  cv::Mat synthesis_reflectance_img;
  double z_scale;

  // camera info
  Matrix4f m_modelview;
  Matrix4f m_projection;
  Matrix4f m_inv_modelview_projection;
  Vector4i m_viewport;

private:
  Model(const Model&);
  void operator = (const Model&);
};

#endif