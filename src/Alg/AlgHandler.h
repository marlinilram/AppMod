#ifndef AlgHandler_H
#define AlgHandler_H

#include <memory>

class ProjOptimize;
class FeatureGuided;
class Model;

class AlgHandler
{
public:
  AlgHandler();
  ~AlgHandler();

  void init();
  void setFeatureModel(std::shared_ptr<FeatureGuided> model);
  void setShapeModel(std::shared_ptr<Model> model);

  void doProjOptimize();

private:
  std::shared_ptr<ProjOptimize> proj_optimize;

  std::shared_ptr<FeatureGuided> feature_model;
  std::shared_ptr<Model>         shape_model;

  // parameters for algorithms
  //float sample_density;


private:
  AlgHandler(const AlgHandler&);
  void operator = (const AlgHandler&);
};
#endif // !AlgHandler_H
