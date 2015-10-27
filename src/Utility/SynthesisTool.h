#ifndef SynthesisTool_H
#define SynthesisTool_H

#include <memory>
#include <cv.h>

class MeshParameterization;

class SynthesisTool
{
public:
  typedef std::vector<cv::Mat> ImagePyramid;  

public:
  SynthesisTool() {};
  ~SynthesisTool() {};

  void init(cv::Mat& src_feature, cv::Mat& tar_feature, cv::Mat& src_detail);
  void doSynthesis();

private:
  void generatePyramid(ImagePyramid& pyr, int level);
  void findBestMatch(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int pointX, int pointY, int& findX, int& findY);
  double distNeighbor(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);

private:
  int levels;
  std::vector<cv::Size> NeighborRange;

  ImagePyramid gpsrc_feature;
  ImagePyramid gptar_feature;
  ImagePyramid gpsrc_detail;
  ImagePyramid gptar_detail;

private:
  SynthesisTool(const SynthesisTool&);
  void operator = (const SynthesisTool&);
};

#endif // !SynthesisTool_H
