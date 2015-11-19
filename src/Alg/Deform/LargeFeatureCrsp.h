#ifndef LargeFeatureCrsp_H
#define LargeFeatureCrsp_h

#include <vector>
#include <map>

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

private:
  FeatureGuided* feature_model;

private:
  LargeFeatureCrsp(const LargeFeatureCrsp&);
  void operator=(const LargeFeatureCrsp&);
};

#endif // !LargeFeatureCrsp_H
