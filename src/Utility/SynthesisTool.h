#ifndef SynthesisTool_H
#define SynthesisTool_H

#include <memory>
#include <cv.h>
#include "BasicHeader.h"

struct distance_position
{
  double d;
  std::pair<int, int> pos;
  bool operator < (const distance_position& a) const
  {
    return a.d > d;
  }
  distance_position(double _d = 0, std::pair<int, int> _pos = std::pair<int, int>(0, 0)) : d(_d), pos(_pos) {};
};

class MeshParameterization;

class SynthesisTool
{
public:
  typedef std::vector<cv::Mat> ImagePyramid;  
  typedef std::vector<ImagePyramid> ImagePyramidVec;
  typedef std::set<distance_position> FCandidates;
  typedef std::vector<FCandidates> ImageFCandidates;
  typedef std::pair<int, int> Point2D;
  typedef std::vector<std::set<Point2D> > FBucket;
  typedef std::vector<FBucket> FBucketPryamid;

public:
  SynthesisTool() {};
  ~SynthesisTool() {};

  void init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail);
  void doSynthesis();
  void doImageSynthesis(std::vector<cv::Mat>& src_detail);
  inline std::vector<ImagePyramid>& getTargetDetail(){ return gptar_detail; };

private:
  void generatePyramid(ImagePyramid& pyr, int level);
  void findCandidates(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  double distNeighborOnFeature(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);
  void findBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position> candidates, int& findX, int& findY);
  double distNeighborOnDetail(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);
  void findBest(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, int& findX, int& findY);
  void generateFeatureCandidateForLowestLevel(std::vector<std::set<distance_position> >& all_pixel_candidates, std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar);
  void getFeatureCandidateFromLowestLevel(std::set<distance_position>& candidates, std::vector<std::set<distance_position> >& all_pixel_candidates, int l, int pointX, int pointY);
  void generateFeatureCandidateFromLastLevel(ImageFCandidates& new_image_candidates, ImageFCandidates& last_image_candidates, int l, ImagePyramidVec& gpsrc, ImagePyramidVec gptar);
  void findCandidatesFromLastLevel(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int tarpointX, int tarpointY, std::set<distance_position>& candidates);
  void buildAllFeatureButkects(std::vector<ImagePyramid>& gpsrc, std::vector<FBucketPryamid>& gpsrc_buckets); // build feature buckets for all features' pyramid
  void buildPryFeatureBuckets(ImagePyramid& gpsrc, FBucketPryamid&  gpsrc_buckets); // build feature buckets for one feature's pyramid
  void buildImgFeatureBuckets(cv::Mat& img, FBucket& buket);
  void findCandidatesInBuckets(std::vector<FBucketPryamid>& gpsrc_buckets, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  void getElementsFromBuckets(FBucket& bucket, std::set<Point2D>& elements, float val);

private:
  int levels;
  std::vector<cv::Size> NeighborRange;

  std::vector<ImagePyramid> gpsrc_feature;
  std::vector<ImagePyramid> gptar_feature;
  std::vector<ImagePyramid> gpsrc_detail;
  std::vector<ImagePyramid> gptar_detail;

  std::vector<FBucketPryamid> gpsrc_feature_buckets;

private:
  SynthesisTool(const SynthesisTool&);
  void operator = (const SynthesisTool&);
};

#endif // !SynthesisTool_H
