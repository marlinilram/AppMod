#ifndef AlgHandler_H
#define AlgHandler_H

#include <memory>
#include <vector>
#include "GLActor.h"

class ProjOptimize;
class NormalTransfer;
class DecompImg;
class DetailSynthesis;
class ProjICP;
class LargeFeatureReg;
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
  void setSynthesisModel(std::shared_ptr<Model> model);
  std::vector<GLActor>& getGLActors() { return actors; };
  bool workable();

  void doProjOptimize();
  void doInteractiveProjOptimize();

  void doNormalTransfer();
  void doNormalCompute();
  std::shared_ptr<DecompImg> getDecompImg() { return decomp_img; };

  void doDetailSynthesis();
  void doProjICP();
  void doLargeFeatureReg(int reg_type = 0);
  void testApplyDisplacement();
  void runApplyDisplacement();

private:
  std::shared_ptr<ProjOptimize> proj_optimize;
  std::shared_ptr<NormalTransfer> normal_transfer;
  std::shared_ptr<DetailSynthesis> detail_synthesis;
  std::shared_ptr<ProjICP> proj_icp;
  std::shared_ptr<LargeFeatureReg> lf_reg;

  std::shared_ptr<FeatureGuided> feature_model;
  std::shared_ptr<Model>         shape_model;
  std::shared_ptr<DecompImg>     decomp_img;
  std::shared_ptr<Model>         synthesis_model;

  // parameters for algorithms
  //float sample_density;
  std::vector<GLActor> actors;

private:
  AlgHandler(const AlgHandler&);
  void operator = (const AlgHandler&);
};
#endif // !AlgHandler_H
