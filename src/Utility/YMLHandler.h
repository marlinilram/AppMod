#ifndef YMLHandler_H
#define YMLHandler_H

#include <cv.h>
#include <highgui.h>

class YMLHandler
{
public:
  YMLHandler();
  ~YMLHandler();

  static void saveToFile(std::string fPath, std::string fName, cv::Mat &file);
  static void saveToMat(std::string fPath, std::string fName, cv::Mat &file);
  static void loadToCvMat(std::string fName, cv::Mat &file);
};

#endif