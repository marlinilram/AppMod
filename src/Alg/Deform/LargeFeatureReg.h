#ifndef LARGE_FEATURE_REG_H
#define LARGE_FEATURE_REG_H

#include <vector>

class MainCanvasViewer;
class FeatureGuided;

class LargeFeatureReg
{
public:
  LargeFeatureReg()
  : feature_model(nullptr), main_viewer(nullptr) {};
  ~LargeFeatureReg() {};

  double energyFunc(const std::vector<double>& X);
  void runReg(int method_id = 0);
  void testNlopt();
  double modelRadius();
  void setFeatureModel(FeatureGuided* model) { feature_model = model; };

private:
  MainCanvasViewer* main_viewer;
  FeatureGuided*    feature_model;


private:
  LargeFeatureReg(const LargeFeatureReg&);
  void operator=(const LargeFeatureReg&);
};


#endif // !LARGE_FEATURE_REG_H
