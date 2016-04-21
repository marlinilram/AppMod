#ifndef AppearanceModel_H
#define AppearanceModel_H

#include <string>
#include <vector>
#include <cv.h>

class AppearanceModel
{
  /* class to store Appearance Model
  ** 1. resolution
  ** 2. feature map
  ** 3. detail map
  ** 4. cca matrix
  */
public:
  AppearanceModel() {};
  ~AppearanceModel() {};

private:
  std::string file_name;
  std::string file_path;

  int resolution;
  std::vector<cv::Mat> feature_maps;
  std::vector<cv::Mat> detail_maps;

private:
  AppearanceModel(const AppearanceModel&);
  void operator = (const AppearanceModel&);
};

#endif // AppearanceModel_H