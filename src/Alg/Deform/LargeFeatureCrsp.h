#ifndef LargeFeatureCrsp_H
#define LargeFeatureCrsp_h

#include <vector>
#include <map>
#include <deque>
#include "BasicDataType.h"

class FeatureGuided;

class LargeFeatureCrsp
{
public:
  typedef std::pair<int, int> CurvePt;
  typedef std::pair<std::pair<int, int>, double> CrspCurvePt;

public:
  LargeFeatureCrsp() : feature_model(nullptr) {};
  LargeFeatureCrsp(FeatureGuided* model) : feature_model(model) {};
  ~LargeFeatureCrsp() {};

  void buildCrsp(std::map<CurvePt, CurvePt>& crsp);
  void refineCrsp(std::map<CurvePt, CrspCurvePt>& crsp_map_out, std::deque<bool>& tar_curve_mark, CURVES& n_src_curves, CURVES& n_tar_curves);
  void buildCrsp(std::map<CurvePt, CurvePt>& crsp, CURVES& curves_in);
  void solveHMM(std::vector<std::pair<int, int>>& observations, std::vector<std::pair<int, int>>& hidden, std::vector<std::pair<int, int>>& path, bool is_source_hidden);

private:
  FeatureGuided* feature_model;

private:
  LargeFeatureCrsp(const LargeFeatureCrsp&);
  void operator=(const LargeFeatureCrsp&);
};

#endif // !LargeFeatureCrsp_H
