#ifndef ProjOptimize_H
#define ProjOptimize_H

#include <cv.h>
#include <Eigen\Eigen>
#include <memory>
#include <vector>
#include "GLActor.h"

class FeatureGuided;
class Model;

class ProjOptimize
{
public:
  ProjOptimize();
  ~ProjOptimize();

  void updateShape(std::shared_ptr<FeatureGuided> feature_guided, std::shared_ptr<Model> model);
  bool isBoundary(cv::Mat& primitive_img, int x, int y);
  void updateScreenShape(std::shared_ptr<Model> model, Eigen::VectorXf& P_Opt);
  void getDrawableActors(std::vector<GLActor>& actors);

private:
  std::vector<GLActor> actors;

private:
  ProjOptimize(const ProjOptimize&);
  void operator = (const ProjOptimize&);
};

#endif