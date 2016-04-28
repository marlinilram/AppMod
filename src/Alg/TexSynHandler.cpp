#include "TexSynHandler.h"

#include "DetailSynthesis.h"

TexSynHandler::TexSynHandler()
{
  this->synthesis_model = nullptr;
  this->detail_synthesis.reset(new DetailSynthesis);
}

TexSynHandler::~TexSynHandler()
{}

void TexSynHandler::setSynthesisModel(std::shared_ptr<Model> model)
{
  this->synthesis_model = model;
}

void TexSynHandler::runD1Synthesis(std::string app_mod_file)
{
  this->detail_synthesis->debugSynthesisD1(app_mod_file, synthesis_model);
}

void TexSynHandler::runD0Synthesis(std::string app_mod_file)
{
  this->detail_synthesis->debugSynthesisD0(app_mod_file, synthesis_model);
}