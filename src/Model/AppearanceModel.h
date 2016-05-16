#ifndef AppearanceModel_H
#define AppearanceModel_H

#include <string>
#include <vector>
#include <cv.h>
#include <memory>
#include "BasicHeader.h"
#include "PolygonMesh.h"
namespace LG {
  class PolygonMesh;
}

class AppearanceModel
{
  /* class to store Appearance Model
  ** 1. resolution
  ** 2. feature map
  ** 3. detail map
  ** 4. cca matrix
  */
public:
  AppearanceModel() : mesh_file_name("base_mesh.obj"), resolution(0) {};
  ~AppearanceModel() {};

  void importAppMod(std::string file_name_, std::string file_path_);
  void exportAppMod(std::string file_name_, std::string file_path_);


  void get_mask_from_origin_image_to_uv(const cv::Mat& mask_origin_image, cv::Mat& mask_uv);


  void setResolution(int resolution_) { resolution = resolution_; };
  float getResolution() { return resolution; };

  void setPhoto(cv::Mat& photo_);
  void getPhoto(cv::Mat& photo_);

  void setD0Features(std::vector<cv::Mat>& feature_maps);
  void setD0Details(std::vector<cv::Mat>& detail_maps);
  void setD1Features(std::vector<cv::Mat>& feature_maps);
  void setD1Details(std::vector<cv::Mat>& detail_maps);

  void getD0Features(std::vector<cv::Mat>& feature_maps);
  void getD0Details(std::vector<cv::Mat>& detail_maps);
  void getD1Features(std::vector<cv::Mat>& feature_maps);
  void getD1Details(std::vector<cv::Mat>& detail_maps);
  void getD1Reflectance(cv::Mat& reflectance, bool diliate = false);

  void setBaseMesh(LG::PolygonMesh* mesh);
  void getBaseMesh(LG::PolygonMesh* mesh);
  LG::PolygonMesh* getBaseMesh(); // return pointer

  void setCCAMat(cv::Mat& mat);
  void geteCCAMat(cv::Mat& mat);

  void setCCAMin(std::vector<float>& vec);
  void setCCAMax(std::vector<float>& vec);
  void getCCAMin(std::vector<float>& vec);
  void getCCAMax(std::vector<float>& vec);

  void setCameraInfo(Matrix4f& modelview, Matrix4f& projection, Vector4i& viewport);
  void setZImg(cv::Mat& z_img_);
  void setPrimitiveID(cv::Mat& primitive_ID_);
  const cv::Mat& getPrimitiveID(){ return this->primitive_ID; };

  void coordImgToUV(std::vector<CvPoint>& coords);
  bool coordImgToUV(const CvPoint& coord_in, CvPoint& coord_out);
  void coordFaceToUV(std::vector<CvPoint>& coords, std::vector<int>& f_ids);

private:
  void writeMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name);
  void readMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name);

  void writeMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh);
  void readMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh);

  void deepCopyCvMatVector(std::vector<cv::Mat>& vector_in, std::vector<cv::Mat>& vector_out);

  template<typename T> 
  void writeVector(cv::FileStorage& fs, std::vector<T>& vec, std::string vec_name);
  template<typename T>
  void readVector(cv::FileStorage& fs, std::vector<T>& vec, std::string vec_name);

  void writeCameraInfo(cv::FileStorage& fs);
  void readCameraInfo(cv::FileStorage& fs);

private:
  std::string file_name;
  std::string file_path;
  std::string mesh_file_name;

  int resolution;
  std::vector<cv::Mat> d0_feature_maps;
  std::vector<cv::Mat> d0_detail_maps;
  std::vector<cv::Mat> d1_feature_maps;
  std::vector<cv::Mat> d1_detail_maps;
  cv::Mat photo;
  cv::Mat cca_mat;
  std::vector<float> cca_min;
  std::vector<float> cca_max;

  // base mesh
  std::unique_ptr<LG::PolygonMesh> base_mesh;
  
  // camera info
  cv::Mat z_img;
  cv::Mat primitive_ID;
  Matrix4f m_modelview;
  Matrix4f m_projection;
  Matrix4f m_inv_modelview_projection;
  Vector4i m_viewport;

private:
  AppearanceModel(const AppearanceModel&);
  void operator = (const AppearanceModel&);
};

#endif // AppearanceModel_H