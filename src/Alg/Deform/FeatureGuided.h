#ifndef FeatureGuided_H
#define FeatureGuided_H

#include <cv.h>
#include <highgui.h>

#include "tele_basicType.h"
#include "kdtree.h" // to make life easier...

class BasicViewer;
class FeatureGuidedVis;
class tele2d;
typedef std::vector<std::vector<double2> > CURVES;
typedef             std::vector<double2>  CURVE;

class FeatureGuided
{
public:
  FeatureGuided(std::string sourceFile, std::string targetFile);
  FeatureGuided();
  virtual ~FeatureGuided();

  void initAllPtr();
  void initImages(const cv::Mat& source, const cv::Mat& target);
  void initImages(std::string sourceFile, std::string targetFile);
  void initRegister();
  void initDispObj();
  void initVisualization(BasicViewer* renderer);
  kdtree::KDTree* getSourceKDTree();

  inline tele2d* GetTeleRegister() { return tele_register; };
  void NormalizedTargetCurves(CURVES& curves);
  void NormalizedSourceCurves(CURVES& curves);
  void OptimizeConnection();
  double MatchScoreToVectorField(std::vector<double2>& curve);
  void BuildDispMap(const cv::Mat& source, kdtree::KDTreeArray& KDTree_data);
  void GetSourceNormalizePara(double2& translate, double& scale);
  void GetFittedCurves(CURVES& curves);

  static void ExtractCurves(const cv::Mat& source, CURVES& curves);
  static std::vector<double2> SearchCurve(
    const cv::Mat& source, int r, int c, 
    std::vector<std::vector<bool>>& visited_table);
  static void SearchCurve(const cv::Mat& source,
    int cur_row, int cur_col,
    std::vector<std::vector<bool>>& visited_table,
    std::vector<double2>& curve);
  static CURVES ReorganizeCurves(CURVES& curves);
  static CURVES SplitCurve(std::vector<double2> curve);
  static std::vector<double2> ConnectCurves(
    std::vector<double2> curve0, std::vector<double2> curve1,
    int endtag0, int endtag1);
  static double CurveLength(std::vector<double2>& curve);
  static void EliminateRedundancy(CURVES& curves);
  static void NormalizedCurves(CURVES& curves);
  static void NormalizedCurves(CURVES& curves, double2 translate, double scale);
  static void NormalizedCurve(std::vector<double2>& curve, double2 translate, double scale);
  static void NormalizePara(CURVES& curves, double2& translate, double& scale);

private:
  cv::Mat source_img;
  cv::Mat target_img;

  CURVES source_curves;
  CURVES target_curves;

  kdtree::KDTree* source_KDTree;
  kdtree::KDTreeArray source_KDTree_data;
  kdtree::KDTree* target_KDTree;
  kdtree::KDTreeArray target_KDTree_data;

  tele2d* tele_register;

  BasicViewer* renderer;
  FeatureGuidedVis* disp_obj;
private:
  FeatureGuided(const FeatureGuided&);
  void operator = (const FeatureGuided&);
};

#endif