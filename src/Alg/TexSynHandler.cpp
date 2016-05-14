#include "TexSynHandler.h"
#include "GLActor.h"
#include "Model.h"
#include "PolygonMesh.h"
#include "AppearanceModel.h"
#include "DetailSynthesis.h"
#include "ParameterMgr.h"

TexSynHandler::TexSynHandler()
{
  this->synthesis_model = nullptr;
  this->syn_app_mod = nullptr;
  this->detail_synthesis.reset(new DetailSynthesis);
}

TexSynHandler::~TexSynHandler()
{}

void TexSynHandler::setSynthesisModel(std::shared_ptr<Model> model)
{
  this->synthesis_model = model;

  // prepare the appearance model for synthesis model
  this->syn_app_mod.reset(new AppearanceModel());
  int resolution = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution");
  std::cout << "Target AppMod resolution initialized as: " << resolution << std::endl;
  this->syn_app_mod->setResolution(resolution);
  this->syn_app_mod->setBaseMesh(model->getPolygonMesh());
  this->detail_synthesis->setCurResolution(resolution);
  this->detail_synthesis->generateD0Feature(this->syn_app_mod.get(), this->synthesis_model);
  this->detail_synthesis->generateD1Feature(this->syn_app_mod.get(), this->synthesis_model, true);

  actors.clear();
  this->detail_synthesis->getDrawableActors(actors);
}

void TexSynHandler::runD1Synthesis(std::string app_mod_file)
{
  if (!synthesis_model || !syn_app_mod)
  {
    std::cout << "target data has not been initialized yet." << std::endl;
  }

  this->detail_synthesis->runSynthesisD1(app_mod_file, syn_app_mod.get(), synthesis_model);
}

std::string TexSynHandler::runD0Synthesis(std::string app_mod_file)
{
  if (!synthesis_model || !syn_app_mod)
  {
    std::cout << "target data has not been initialized yet." << std::endl;
  }

 return this->detail_synthesis->runSynthesisD0(app_mod_file, syn_app_mod.get(), synthesis_model);
}

std::string TexSynHandler::applyD1Displacement(cv::Mat& mask)
{
  if (!synthesis_model || !syn_app_mod)
  {
    std::cout << "target data has not been initialized yet." << std::endl;
  }

  return this->detail_synthesis->applyD1Displacement(synthesis_model, mask);
}