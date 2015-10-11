#ifndef ProjOptimize_H
#define ProjOptimize_H

#include <cv.h>
#include <Eigen\Eigen>
#include <memory>
#include <vector>
#include "GLActor.h"

class FeatureGuided;
class Model;
class Solver;
class ProjConstraint;
class ARAP;

class ProjOptimize
{
public:
  ProjOptimize();
  ~ProjOptimize();

  void updateShape(std::shared_ptr<FeatureGuided> feature_guided, std::shared_ptr<Model> model);
  void updateShapeFromInteraction(std::shared_ptr<FeatureGuided> feature_guided, std::shared_ptr<Model> model);

  void updateScreenShape(std::shared_ptr<Model> model, Eigen::VectorXf& P_Opt);
  void getDrawableActors(std::vector<GLActor>& actors);

private:
  std::vector<GLActor> actors;

  std::shared_ptr<Solver> solver;
  std::shared_ptr<ProjConstraint> proj_constraint;
  std::shared_ptr<ARAP> arap;

  std::vector<int> constrained_vertex_id;
  std::vector<float> constrained_ray;

private:
  ProjOptimize(const ProjOptimize&);
  void operator = (const ProjOptimize&);
};

#endif