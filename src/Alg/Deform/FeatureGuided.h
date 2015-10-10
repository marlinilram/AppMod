#ifndef FeatureGuided_H
#define FeatureGuided_H

#include <cv.h>
#include <highgui.h>
#include <memory>

#include "BasicDataType.h"
#include "kdtree.h" // to make life easier...
#include "BasicHeader.h"

class Model;
class FeatureLine;
class ScalarField;
class KDTreeWrapper;
class tele2d;
typedef std::vector<std::vector<double2> > CURVES;
typedef             std::vector<double2>   CURVE;
// suppose the HIST contains 8 + 1 + 2 elements
// 8 for directions, 1 for scalar and 2 for pos (x, y)
typedef             std::vector<double>    HIST;
typedef std::vector<std::vector<double> >  HISTS;

class FeatureGuided
{
public:
  FeatureGuided(std::shared_ptr<Model> source_model, std::string targetFile);
  FeatureGuided();
  virtual ~FeatureGuided();

  void initTargetImage(std::string targetFile);
  void initRegister();
  void updateSourceVectorField();
  void updateScalarField();
  std::shared_ptr<KDTreeWrapper> getSourceKDTree();

  inline std::shared_ptr<tele2d> GetTeleRegister() { return source_tele_register; };
  inline std::shared_ptr<tele2d> GetTargetTeleRegister() { return target_tele_register; };
  inline std::shared_ptr<ScalarField> getSourceScalarField() { return source_scalar_field; };
  inline std::shared_ptr<ScalarField> getTargetScalarField() { return target_scalar_field; };
  void NormalizedTargetCurves(CURVES& curves);
  void NormalizedSourceCurves(CURVES& curves);
  void setNormalizePara();
  void OptimizeConnection();
  double MatchScoreToVectorField(std::vector<double2>& curve);
  void BuildDispMap(const cv::Mat& source, kdtree::KDTreeArray& KDTree_data);
  void BuildSourceEdgeKDTree();
  void BuildTargetEdgeKDTree();
  void BuildEdgeKDTree(CURVES& curves, std::shared_ptr<KDTreeWrapper> kdTree);
  void GetSourceNormalizePara(double2& translate, double& scale);
  void GetFittedCurves(CURVES& curves);
  void CalculateHists(
    HISTS& hists,
    CURVES& curves, double radius, tele2d* tele);
  void SearchRadius(
    std::vector<double>& hist,
    double2 center, double r, std::vector<int2>& area,
    std::vector<double2>& vector_field, int resolution);
  void FindHistMatchCrsp(CURVES &curves);
  void GetCrspPair(CURVES& curves);
  void GetUserCrspPair(CURVES& curves, float sample_density);
  void BuildClosestPtPair(std::vector<std::pair<int, double2> >& crsp_list);

  void ExtractCurves(const cv::Mat& source, CURVES& curves);
  void ExtractSrcCurves(const cv::Mat& source, CURVES& curves);
  void SearchCurve(const cv::Mat& source,
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
  static void NormalizedCurve(CURVE& curve, double2 translate, double scale);
  static void DenormalizedCurve(CURVE& curve, double2 translate, double scale);
  static void NormalizePara(CURVES& curves, double2& translate, double& scale);

public:
  // user defined feature line
  std::shared_ptr<FeatureLine> source_vector_field_lines;
  std::shared_ptr<FeatureLine> target_vector_field_lines;

  std::vector<std::pair<int, int> > src_crsp_list;
  std::vector<std::pair<int, int> > tar_crsp_list;

private:
  std::shared_ptr<Model> source_model;
  cv::Mat target_img;

  CURVES source_curves;
  CURVES target_curves;
  double2 curve_translate;
  double curve_scale;
  float edge_threshold; // threshold for edge detection

  std::shared_ptr<KDTreeWrapper> source_KDTree;
  std::shared_ptr<KDTreeWrapper> target_KDTree;

  std::shared_ptr<tele2d> source_tele_register;
  std::shared_ptr<tele2d> target_tele_register;

  std::shared_ptr<ScalarField> source_scalar_field;
  std::shared_ptr<ScalarField> target_scalar_field;
private:
  FeatureGuided(const FeatureGuided&);
  void operator = (const FeatureGuided&);
};

#endif