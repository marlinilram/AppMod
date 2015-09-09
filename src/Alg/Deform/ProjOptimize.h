#ifndef ProjOptimize_H
#define ProjOptimize_H

#include <cv.h>
#include <Eigen\Eigen>

class FeatureGuided;
class Model;

class ProjOptimize
{
public:
  ProjOptimize();
  ~ProjOptimize();

  void updateShape(FeatureGuided* feature_guided, Model* model);
  bool isBoundary(cv::Mat& primitive_img, int x, int y);
  void updateScreenShape(Model* model, Eigen::VectorXf& P_Opt);

private:
  ProjOptimize(const ProjOptimize&);
  void operator = (const ProjOptimize&);
};

#endif