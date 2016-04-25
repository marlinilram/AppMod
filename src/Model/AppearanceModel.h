#ifndef AppearanceModel_H
#define AppearanceModel_H

#include <string>
#include <vector>
#include <cv.h>
#include <memory>

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
  AppearanceModel() : mesh_file_name("base_mesh.obj") {};
  ~AppearanceModel() {};

  void importAppMod(std::string file_name_, std::string file_path_);
  void exportAppMod(std::string file_name_, std::string file_path_);

  void setD0Features(std::vector<cv::Mat>& feature_maps);
  void setD0Details(std::vector<cv::Mat>& detail_maps);
  void setD1Features(std::vector<cv::Mat>& feature_maps);
  void setD1Details(std::vector<cv::Mat>& detail_maps);

  void getD0Features(std::vector<cv::Mat>& feature_maps);
  void getD0Details(std::vector<cv::Mat>& detail_maps);
  void getD1Features(std::vector<cv::Mat>& feature_maps);
  void getD1Details(std::vector<cv::Mat>& detail_maps);

  void setBaseMesh(LG::PolygonMesh* mesh);
  void getBaseMesh(LG::PolygonMesh* mesh);

private:
  void writeMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name);
  void readMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name);

  void writeMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh);
  void readMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh);

  void deepCopyCvMatVector(std::vector<cv::Mat>& vector_in, std::vector<cv::Mat>& vector_out);

private:
  std::string file_name;
  std::string file_path;
  std::string mesh_file_name;

  int resolution;
  std::vector<cv::Mat> d0_feature_maps;
  std::vector<cv::Mat> d0_detail_maps;
  std::vector<cv::Mat> d1_feature_maps;
  std::vector<cv::Mat> d1_detail_maps;

  // base mesh
  std::unique_ptr<LG::PolygonMesh> base_mesh;

private:
  AppearanceModel(const AppearanceModel&);
  void operator = (const AppearanceModel&);
};

#endif // AppearanceModel_H