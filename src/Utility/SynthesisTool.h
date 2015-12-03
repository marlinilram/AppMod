#ifndef SynthesisTool_H
#define SynthesisTool_H

#include <memory>
#include <cv.h>

struct distance_position
{
  double d;
  std::pair<int, int> pos;
  bool operator < (const distance_position& a) const
  {
    return a.d > d;
  }
};

class MeshParameterization;

class SynthesisTool
{
public:
  typedef std::vector<cv::Mat> ImagePyramid;  

public:
  SynthesisTool() {};
  ~SynthesisTool() {};

  void init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, cv::Mat& src_detail);
  void doSynthesis();
  void doImageSynthesis(cv::Mat& src_detail);
  inline ImagePyramid& getTargetDetail(){ return gptar_detail; };

private:
  void generatePyramid(ImagePyramid& pyr, int level);
  void findCandidates(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  double distNeighborOnFeature(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);
  void findBestMatch(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int pointX, int pointY, std::set<distance_position> candidates, int& findX, int& findY);
  double distNeighborOnDetail(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);
  void findBest(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int pointX, int pointY, int& findX, int& findY);

private:
  int levels;
  std::vector<cv::Size> NeighborRange;

  std::vector<ImagePyramid> gpsrc_feature;
  std::vector<ImagePyramid> gptar_feature;
  ImagePyramid gpsrc_detail;
  ImagePyramid gptar_detail;

private:
  SynthesisTool(const SynthesisTool&);
  void operator = (const SynthesisTool&);
};

#endif // !SynthesisTool_H
