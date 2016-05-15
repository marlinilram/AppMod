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
class DetailSynthesis;

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
  SynthesisTool();
  ~SynthesisTool() {};

  void init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail);
  void doSynthesis();
  void doSynthesisNew(bool is_doComplete = false);
  void doFilling(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& src_detail);
  void doSynthesisWithMask(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail, std::vector<cv::Mat>& tar_detail);
  void doImageSynthesis(std::vector<cv::Mat>& src_detail);
  inline std::vector<ImagePyramid>& getTargetDetail(){ return gptar_detail; };

  void doNNFOptimization(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature);

  void setExportPath(std::string& path_in) { outputPath = path_in; };
  void exportFeature(cv::Mat& f_mat, std::string fname);
  void exportSrcFeature(ImagePyramidVec& gpsrc, int level);
  void exportTarFeature(ImagePyramidVec& gptar, int level);
  void exportRelfectance(cv::Mat& r, cv::Mat& g, cv::Mat& b, std::string fname);
  void exportDisplacement(cv::Mat& d_mat, std::string fname);
  void exportSrcDetail(ImagePyramidVec& gpsrc, int level, int iter);
  void exportTarDetail(ImagePyramidVec& gptar, int level, int iter);
  void exportNNF(NNF& nnf, ImagePyramidVec& gpsrc, ImagePyramidVec& gptar, int level, int iter);
  void exportMask(std::vector<int> mask, int mask_height, int mask_width, std::string fname);
  void exportSrcMask();
  void exportTarMask();

  void findSrcCrsp(Point2D& tar_id, std::vector<Point2D>& src_id);

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
  void getRandomPosition(int l, std::vector<Point2D>& random_set, int n_set, int max_height, int max_width, int min_height = 0, int min_width = 0);
  void initializeNNF(ImagePyramid& gptar_d, NNF& nnf, int level, bool is_doComplete = false);
  void initializeNNFFromLastLevel(ImagePyramid& gpsrc_d, ImagePyramid& gptar_d, NNF& nnf_last, int level, NNF& nnf_new, bool is_doComplete = false);
  void initializeTarDetail(ImagePyramidVec& gptar_d, int level, bool is_doComplete = false);
  void voteImage(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level);
  void votePixel(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level, Point2D& tarPos);
  double updateNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                 ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                 NNF& nnf, std::vector<float>& ref_cnt, int level, int iter = 0);
  void updateNNFReverse(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
    ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
    NNF& nnf, std::vector<float>& ref_cnt, int level);
  double distPatch(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                   ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                   std::vector<float>& ref_cnt, int level, Point2D& srcPatch, Point2D& tarPatch);
  double bestPatchInSet(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                      ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                      std::vector<float>& ref_cnt, int level, Point2D& tarPatch, std::vector<Point2D>& srcPatches, Point2D& best_patch);

  void buildMask(cv::Mat& tar_feature, std::vector<int>& pixel_mask, std::vector<int>& patch_mask, int level, bool is_doComplete = false);//0 -> valid; 1 -> invalid
  void initializeFillingNNF(ImagePyramid& gptar_d, NNF& nnf, std::vector<int>& patch_mask, int level);
  void initializeFillingTarDetail(ImagePyramidVec& gptar_d, std::vector<int>& pixel_mask, int level);
  void updateFillingNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                        ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                        NNF& nnf, std::vector<float>& ref_cnt, std::vector<int>& patch_mask, int level);
  void updateFillingNNFReverse(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                               ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                               NNF& nnf, std::vector<float>& ref_cnt, std::vector<int>& patch_mask, int level);
  void getRandomPositionWithMask(std::vector<Point2D>& random_set, std::vector<int>& patch_mask,int nnf_width, int nnf_height, int n_set, int max_height, int max_width, int min_height = 0, int min_width = 0);
  void voteFillingImage(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, std::vector<int>& pixel_mask, int level);
  void initializeFillingUpTarDetail(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, std::vector<int>& pixel_mask, int level);
  bool validPatchWithMask(Point2D& patch_pos, std::vector<int>& patch_mask, int nnf_height, int nnf_width);//0 -> valid; 1 -> invalid
  void updateRefCount(STLVectorf& ref_cnt, Point2D& best_patch, ImagePyramidVec& gpsrc_d, int level);
  void updateNNFWithMask(std::vector<int>& source_patch_mask, ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                 ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                 NNF& nnf, std::vector<float>& ref_cnt, int level, int iter = 0);
  void buildSourcePatchMask(cv::Mat& src_detail, std::vector<int>& source_patch_mask);
  void buildTargetMask(cv::Mat& tar_detail, std::vector<int>& target_pixel_mask, std::vector<int>& target_patch_mask);

  double updateNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f, NNF& nnf, std::vector<float>& ref_cnt, int level, int iter = 0);
  double distPatch(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
    std::vector<float>& ref_cnt, int level, Point2D& srcPatch, Point2D& tarPatch);
  double bestPatchInSet(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
    std::vector<float>& ref_cnt, int level, Point2D& tarPatch, std::vector<Point2D>& srcPatches, Point2D& best_patch);

private:
  friend class DetailSynthesis;

private:
  int levels;
  int candidate_size;
  int best_random_size;
  int patch_size;
  double bias_rate;
  double lamd_occ;
  double lamd_gradient;
  int max_iter;
  float py_scale;
  float feature_threshold;
  float beta_func_center;
  float beta_func_mult;
  std::vector<cv::Size> NeighborRange;

  std::vector<ImagePyramid> gpsrc_feature;
  std::vector<ImagePyramid> gptar_feature;
  std::vector<ImagePyramid> gpsrc_detail;
  std::vector<ImagePyramid> gptar_detail;

  std::vector<cv::Mat> src_detail_gradient;
  std::vector<cv::Mat> tar_detail_gradient;

  NNF tar_feature_NNF;

  std::vector<FBucketPryamid> gpsrc_feature_buckets;

  std::string outputPath;
  std::vector<std::vector<int> > src_patch_mask;
  std::vector<std::vector<int> > tar_patch_mask;
  std::vector<std::vector<int> > tar_pixel_mask;

private:
  SynthesisTool(const SynthesisTool&);
  void operator = (const SynthesisTool&);
};

#endif // !SynthesisTool_H
