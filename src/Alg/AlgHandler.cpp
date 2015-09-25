#include "AlgHandler.h"
#include "ProjOptimize.h"
#include "FeatureGuided.h"
#include "Model.h"

AlgHandler::AlgHandler()
{
  init();
}

AlgHandler::~AlgHandler()
{

}

void AlgHandler::init()
{
  proj_optimize.reset(new ProjOptimize);

  feature_model = nullptr;
  shape_model = nullptr;
}

void AlgHandler::setFeatureModel(std::shared_ptr<FeatureGuided> model)
{
  feature_model = model;
}

void AlgHandler::setShapeModel(std::shared_ptr<Model> model)
{
  shape_model = model;
}

void AlgHandler::doProjOptimize()
{
  if (!feature_model || !shape_model)
  {
    std::cout << "Early return: feature model or shape model is not built correctly.\n";
    return;
  }

  actors.clear();
  proj_optimize->updateShape(feature_model, shape_model);
  std::vector<GLActor> temp_actors;
  proj_optimize->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}