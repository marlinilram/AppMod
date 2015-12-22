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
  typedef std::vector<Point2D> NNF;

public:
  SynthesisTool() {};
  ~SynthesisTool() {};

  void init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail);
  void doSynthesis();
  void doSynthesisNew();
  void doImageSynthesis(std::vector<cv::Mat>& src_detail);
  inline std::vector<ImagePyramid>& getTargetDetail(){ return gptar_detail; };

private:
  void generatePyramid(ImagePyramid& pyr, int level);
  void findCandidates(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  double distNeighborOnFeature(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY);
  void findBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates, int& findX, int& findY);
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
  void findCandidatesWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  void findCandidatesFromLastLevelWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& candidates);
  void getLastLevelCandidate(std::vector<ImagePyramid>& gptar, ImageFCandidates& last_image_candidates, int level, int pointX, int pointY, std::set<distance_position>& candidates);
  void findBestMatchWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int pointX, int pointY, std::set<distance_position>& candidates, std::set<distance_position>& best_match);
  void getValFromBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match);
  void findCombineCandidates(std::vector<ImagePyramid>& gpsrc_f, std::vector<ImagePyramid>& gptar_f, std::vector<ImagePyramid>& gpsrc_d, std::vector<ImagePyramid>& gptar_d, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match);
  void findCombineCandidatesFromLastLevel(std::vector<ImagePyramid>& gpsrc_f, std::vector<ImagePyramid>& gptar_f, std::vector<ImagePyramid>& gpsrc_d, std::vector<ImagePyramid>& gptar_d, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match);

  // patch match based method
  void getRandomPosition(std::vector<Point2D>& random_set, int n_set, int max_height, int max_width);
  void initializeNNF(ImagePyramid& gptar_d, NNF& nnf, int level);
  void initializeNNFFromLastLevel(ImagePyramid& gptar_d, NNF& nnf_last, int level, NNF& nnf_new);
  void initializeTarDetail(ImagePyramidVec& gptar_d, int level);
  void voteImage(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level);
  void votePixel(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level, Point2D& tarPos);
  void updateNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                 ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                 NNF& nnf, std::vector<float>& ref_cnt, int level);
  void updateNNFReverse(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
    ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
    NNF& nnf, std::vector<float>& ref_cnt, int level);
  double distPatch(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                   ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                   std::vector<float>& ref_cnt, int level, Point2D& srcPatch, Point2D& tarPatch);
  void bestPatchInSet(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                      ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                      std::vector<float>& ref_cnt, int level, Point2D& tarPatch, std::vector<Point2D>& srcPatches, Point2D& best_patch);

   
private:
  int levels;
  int candidate_size;
  int best_random_size;
  int patch_size;
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
