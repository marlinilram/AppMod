#include "YMLHandler.h"

#include <fstream>
#include "BasicHeader.h"

YMLHandler::YMLHandler()
{

}

YMLHandler::~YMLHandler()
{

}

void YMLHandler::saveToFile(std::string fPath, std::string fName, cv::Mat &file)
{
  cv::FileStorage fs(fPath + "/" + fName, cv::FileStorage::WRITE);
  fs << fName.substr(0, fName.find_last_of('.')) << file;
  fs.release();
}

void YMLHandler::loadToCvMat(std::string fName, cv::Mat &file)
{
  cv::FileStorage fs(fName, cv::FileStorage::READ);

  std::cout<<fName.substr(fName.find_last_of('/') + 1, fName.find_last_of('.') - fName.find_last_of('/') - 1)<<"\n";

  fs[fName.substr(fName.find_last_of('/') + 1, fName.find_last_of('.') - fName.find_last_of('/') - 1)] >> file;

  fs.release();

}

void YMLHandler::saveToMat(std::string fPath, std::string fName, cv::Mat &file)
{
  std::ofstream mat_output(fPath + "/" + fName);
  if (mat_output)
  {
    Eigen::Map<MatrixXf>temp((float*)file.data, file.cols, file.rows);
    mat_output << temp.transpose();
    mat_output.close();
  }
}