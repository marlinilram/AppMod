#ifndef Model_H
#define Model_H

#include <iostream>
#include <memory>
#include <string>
#include <cv.h>
#include <highgui.h>

#include "BasicHeader.h"

class Shape;
class Bound;

class Model
{
public:
  Model();
  Model(const std::string path, const std::string name);
  ~Model();

  bool loadOBJ(const std::string name, const std::string path);
  void exportOBJ(int cur_iter);

  Bound* getBoundBox();
  std::shared_ptr<Shape> getShape();
  std::string getDataPath();
  std::string getOutputPath();

  inline cv::Mat &getRImg(){ return r_img; };
  inline cv::Mat &getRBGRAImg() { return rBGRA_img; };
  inline cv::Mat &getPrimitiveIDImg(){ return primitive_ID; };
  inline cv::Mat &getZImg() { return z_img; };
  inline cv::Mat &getRMask() { return mask_rimg; };
  inline cv::Mat &getEdgeImg() { return edge_image; };

  float getModelAvgEdgeLength();
  void passCameraPara(float c_modelview[16], float c_projection[16], int c_viewport[4]);
  bool getWorldCoord(Vector3f rimg_coord, Vector3f &w_coord);
  bool getUnprojectPt(float winx, float winy, float winz, float object_coord[3]);
  int getClosestVertexId(float world_pos[3], int x, int y);
  void getCameraOri(float camera_ori[3]);
  void getProjRay(float proj_ray[3], int x, int y);

private:
  std::shared_ptr<Shape> shape; // Model is the owner of Shape

  // file system data
  std::string data_path;
  std::string file_name;
  std::string output_path;

  // render info
  cv::Mat z_img;
  cv::Mat primitive_ID;
  cv::Mat rBGRA_img;
  cv::Mat r_img;
  cv::Mat mask_rimg;
  cv::Mat edge_image;

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