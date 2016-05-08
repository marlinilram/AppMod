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
  std::string normal_file_name = "final_normal";

  normal_transfer->prepareNewNormal(shape_model, normal_file_name);
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

void AlgHandler::doDetailSynthesis()
{
	//if (!workable())
	//{
	//  return;
	//}
	//detail_synthesis->testShapePlane(shape_model);
	/*shape_model->exportOBJ(0);
	doNormalTransfer();
	shape_model->exportOBJ(0);*/

	//detail_synthesis->doTransfer(shape_model, synthesis_model);
	//detail_synthesis->startDetailSynthesis(shape_model);
  detail_synthesis->debugSynthesisD1(shape_model->getDataPath() + "/appearance_model", synthesis_model);


	//detail_synthesis->computeVectorField(shape_model);
	actors.clear();
	std::vector<GLActor> temp_actors;
	detail_synthesis->getDrawableActors(temp_actors);
	for (size_t i = 0; i < temp_actors.size(); ++i)
	{
		actors.push_back(temp_actors[i]);
	}
}

void AlgHandler::testApplyDisplacement()
{
  //detail_synthesis->prepareParaPatches(shape_model, synthesis_model);return;

  //detail_synthesis->doGeometryTransfer(shape_model, synthesis_model);
  detail_synthesis->debugSynthesisD0(shape_model->getDataPath() + "/appearance_model", synthesis_model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  detail_synthesis->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
  syn_actors.clear();
  detail_synthesis->getDrawableActors(syn_actors, 1);
}

void AlgHandler::runApplyDisplacement()
{
  detail_synthesis->applyNewDisp(shape_model, synthesis_model);
}

void AlgHandler::loadDetailMap()
{
  // generate detail map
  //detail_synthesis->test(shape_model, synthesis_model);

  // test AppearanceModel
  detail_synthesis->generateAppearanceModel(shape_model, synthesis_model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  detail_synthesis->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void AlgHandler::debugSymmetry()
{
  if (!shape_model)
  {
    return;
  }

  VertexList v_list = shape_model->getShapeVertexList();
  std::set<STLPairii> sym_pairs;
  shape_model->getSymPairs(sym_pairs);

  actors.clear();
  actors.push_back(GLActor(ML_POINT, 3.0f));
  actors.push_back(GLActor(ML_LINE, 1.0f));

  for (auto i : sym_pairs)
  {
    Vector3f start, end;
    start[0] = v_list[3 * i.first + 0];
    start[1] = v_list[3 * i.first + 1];
    start[2] = v_list[3 * i.first + 2];
    end[0] = v_list[3 * i.second + 0];
    end[1] = v_list[3 * i.second + 1];
    end[2] = v_list[3 * i.second + 2];
    actors[0].addElement(start[0], start[1], start[2], 1, 0, 0);
    actors[0].addElement(end[0], end[1], end[2], 1, 0, 0);
    actors[1].addElement(start[0], start[1], start[2], 0, 0, 0);
    actors[1].addElement(end[0], end[1], end[2], 0, 0, 0);
  }

  //detail_synthesis->doGeometryTransfer(shape_model, synthesis_model);
  //actors.clear();
  //detail_synthesis->getDrawableActors(actors);
  //syn_actors.clear();
  //detail_synthesis->getDrawableActors(syn_actors, 1);
}