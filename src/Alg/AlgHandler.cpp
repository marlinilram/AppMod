#include "AlgHandler.h"
#include "ProjOptimize.h"
#include "NormalTransfer.h"
#include "DetailSynthesis.h"
#include "ProjICP.h"
#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "Model.h"
#include "DecompImg.h"

#include "ParameterMgr.h"

AlgHandler::AlgHandler()
{
  feature_model = nullptr;
  shape_model = nullptr;
  init();
}

AlgHandler::~AlgHandler()
{

}

void AlgHandler::init()
{
  proj_optimize.reset(new ProjOptimize);
  normal_transfer.reset(new NormalTransfer);
  detail_synthesis.reset(new DetailSynthesis);
  proj_icp.reset(new ProjICP);
  lf_reg.reset(new LargeFeatureReg);
  decomp_img.reset(new DecompImg);
}

void AlgHandler::setFeatureModel(std::shared_ptr<FeatureGuided> model)
{
  feature_model = model;
}

void AlgHandler::setShapeModel(std::shared_ptr<Model> model)
{
  shape_model = model;
  init();
}

void AlgHandler::setSynthesisModel(std::shared_ptr<Model> model)
{
  synthesis_model = model;
}

bool AlgHandler::workable()
{
  if (!feature_model || !shape_model)
  {
    std::cout << "Early return: feature model or shape model is not built correctly.\n";
    return false;
  }

  return true;
}

void AlgHandler::doProjOptimize()
{
  if (!workable())
  {
   return;
  }

  actors.clear();
  proj_optimize->updateShape(feature_model, shape_model);
  // Warning: cannot put updateSourceField() here. Because it need to
  // recompute source curve from visible crest line and
  // visible crest line is updated only if the main canvas has been
  // redrawn and primitive_id image has been updated.
  //feature_model->updateSourceField();
  std::vector<GLActor> temp_actors;
  proj_optimize->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::doInteractiveProjOptimize()
{
  if (!workable())
  {
    return;
  }

  actors.clear();
  proj_optimize->updateShapeFromInteraction(feature_model, shape_model);
  //feature_model->updateSourceField();
  std::vector<GLActor> temp_actors;
  proj_optimize->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::doNormalTransfer()
{
  if (!shape_model)//!shape_model
  {
    return;
  }

  //shape_model->updateSHColor();
  normal_transfer->prepareNewNormal(shape_model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  normal_transfer->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::doNormalCompute()
{
  /*decomp_img->setModel(shape_model);*/
  decomp_img->computeNormal(shape_model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  decomp_img->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::doDetailSynthesis()
{
  //if (!workable())
  //{
  //  return;
  //}
  detail_synthesis->testMeshPara(shape_model);
  //detail_synthesis->testShapePlane(shape_model);
  /*shape_model->exportOBJ(0);
  doNormalTransfer();
  shape_model->exportOBJ(0);*/

  //detail_synthesis->doTransfer(shape_model, synthesis_model);
  detail_synthesis->startDetailSynthesis(shape_model);


  //detail_synthesis->computeVectorField(shape_model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  detail_synthesis->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::doProjICP()
{
  if (!workable())
  {
    return;
  }

  proj_icp->buildCrsp(feature_model);
}

void AlgHandler::doLargeFeatureReg(int reg_type)
{
  if (!workable())
  {
    return;
  }

  lf_reg->setFeatureModel(feature_model.get());
  if (reg_type == 0)
  {
    lf_reg->runReg(LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:registerMethod"));
  }
  else if (reg_type == 1)
  {
    lf_reg->runRegNonRigid(LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:registerMethod"));
  }
  //lf_reg->testNlopt();
}