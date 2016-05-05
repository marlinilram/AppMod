// Use Appearance Model to synthesize new model

#include "DetailSynthesis.h"

#include "GeometryTransfer.h"

#include "Shape.h"
#include "ParaShape.h"

#include "SynthesisTool.h"
#include "AppearanceModel.h"
#include "KevinVectorField.h"
#include "ShapeUtility.h"
#include "ImageUtility.h"
#include "ParameterMgr.h"
#include "YMLHandler.h"
#include <fstream>
#include "global.h"
#include "BasicHeader.h"
std::string DetailSynthesis::synthesisD0(AppearanceModel* app_mod_src, AppearanceModel* app_mod_tar, std::shared_ptr<Model> tar_model)
{
	// 0. prepare the sample vertices on the target mesh
	std::shared_ptr<GeometryTransfer> geometry_transfer(new GeometryTransfer);
	std::vector<int> sampled_tar_model;
	std::shared_ptr<ParaShape> src_para_shape(new ParaShape);
	src_para_shape->initWithExtPolygonMesh(app_mod_src->getBaseMesh());
	std::shared_ptr<ParaShape> tar_para_shape(new ParaShape);
	tar_para_shape->initWithExtShape(tar_model);
	geometry_transfer->prepareSampleVertex(tar_model, sampled_tar_model);
	ShapeUtility::meshParaBoundaryFilter(sampled_tar_model, tar_para_shape->vertex_set, tar_para_shape->cut_shape->getPolygonMesh());
	std::vector<STLVectori> src_v_ids(sampled_tar_model.size(), STLVectori());

	// 1. build the mask for source and target
	std::vector<cv::Mat> src_feature_map;
	app_mod_src->getD0Features(src_feature_map);
	cv::Mat src_mask = GLOBAL::m_mat_source_mask0_;
	/*ImageUtility::generateMultiMask(src_feature_map[0].clone(), src_mask);*/

	// 1.1 generate mask from source image
	cv::Mat src_photo;
	app_mod_src->getPhoto(src_photo);
	// convert image stroke to para shape stroke

	std::vector<cv::Mat> tar_feature_map;
	app_mod_tar->getD0Features(tar_feature_map);


	std::vector<CvPoint> stroke;
	for (unsigned int i = 0; i < GLOBAL::m_selected_faces_.size(); i++)
	{
		Vector2f uv;
		ShapeUtility::getFaceUVCenter(tar_model->getPolygonMesh(), GLOBAL::m_selected_faces_[i], uv);
		CvPoint p = cvPoint(uv.x(), uv.y());
		stroke.push_back(p);
	}


	cv::Mat tar_mask(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1, 1);
	//ImageUtility::generateMultiMask(tar_feature_map[0].clone(), tar_mask);
	ImageUtility::generateMaskFromStroke(tar_feature_map[0].clone(), stroke,tar_mask);

	std::vector<int> cur_sampled_tar_models;
	ShapeUtility::vertexFilterFromParaMask(sampled_tar_model, cur_sampled_tar_models, tar_para_shape->vertex_set, tar_para_shape->cut_shape->getPolygonMesh(), tar_mask);

	// 2. make the masked source and target feature map
	// 2.1 masked source
	std::vector<cv::Mat> masked_src_feature_map;
	ImageUtility::generateMaskedMatVec(src_feature_map, masked_src_feature_map, src_mask);
	// 2.2 masked target
	std::vector<cv::Mat> masked_tar_feature_map;
	ImageUtility::generateMaskedMatVec(tar_feature_map, masked_tar_feature_map, tar_mask);

	// 3. correspondences from NNF
	// 3.1 run patch match
	syn_tool.reset(new SynthesisTool);
	syn_tool->levels = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
	syn_tool->patch_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
	syn_tool->max_iter = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
	syn_tool->best_random_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
	syn_tool->lamd_occ = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
	syn_tool->bias_rate = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
	syn_tool->setExportPath(tar_model->getOutputPath());
	syn_tool->doNNFOptimization(masked_src_feature_map, masked_tar_feature_map);

	// 3.2 find correspondences from NNF
	std::vector<STLVectori> cur_src_crsp;
	this->prepareLocalTransformCrsp(src_para_shape, tar_para_shape, syn_tool, cur_sampled_tar_models, cur_src_crsp);

	// 3.3 merge the current correspondences to sampled vertex set
	ShapeUtility::mergeSubVector(sampled_tar_model, src_v_ids, cur_sampled_tar_models, cur_src_crsp);

	STLVectorf new_v_list;
	double transform_scale = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale");
	ShapeUtility::prepareLocalTransform(app_mod_src->getBaseMesh(), tar_model->getPolygonMesh(), src_v_ids, sampled_tar_model, new_v_list, transform_scale);

	LG::PolygonMesh old_tar_mesh = (*tar_model->getPolygonMesh()); // copy the old one
	std::string obj_path = geometry_transfer->transferDeformation(tar_model, sampled_tar_model, new_v_list);

	ShapeUtility::computeLocalTransform(&old_tar_mesh, tar_model->getPolygonMesh());
	ShapeUtility::exportVisForLocalTransform(tar_model->getPolygonMesh(), tar_model->getOutputPath());

	return obj_path;
}



//   std::string DetailSynthesis::synthesisD0(AppearanceModel* app_mod_src, AppearanceModel* app_mod_tar, std::shared_ptr<Model> tar_model)
//   {
//     // 0. prepare the sample vertices on the target mesh
//     std::shared_ptr<GeometryTransfer> geometry_transfer(new GeometryTransfer);
//     std::vector<int> sampled_tar_model;
//     std::shared_ptr<ParaShape> src_para_shape(new ParaShape);
//     src_para_shape->initWithExtPolygonMesh(app_mod_src->getBaseMesh());
//     std::shared_ptr<ParaShape> tar_para_shape(new ParaShape);
//     tar_para_shape->initWithExtShape(tar_model);
//     geometry_transfer->prepareSampleVertex(tar_model, sampled_tar_model);
//     ShapeUtility::meshParaBoundaryFilter(sampled_tar_model, tar_para_shape->vertex_set, tar_para_shape->cut_shape->getPolygonMesh());
//     std::vector<STLVectori> src_v_ids(sampled_tar_model.size(), STLVectori());
//   
//     // 1. build the mask for source and target
//     std::vector<cv::Mat> src_feature_map;
//     app_mod_src->getD0Features(src_feature_map);
//     cv::Mat src_mask(src_feature_map[0].rows, src_feature_map[0].cols, CV_32FC1, 1);
//     /*ImageUtility::generateMultiMask(src_feature_map[0].clone(), src_mask);*/
//   
//     // 1.1 generate mask from source image
//     std::vector<std::vector<CvPoint> > mask_strokes;
//     cv::Mat src_photo;
//     app_mod_src->getPhoto(src_photo);
//     ImageUtility::generateMultiStrokes(src_photo, mask_strokes);
//     // convert image stroke to para shape stroke
//     for (size_t i = 0; i < mask_strokes.size(); ++i) app_mod_src->coordImgToUV(mask_strokes[i]);
//     ImageUtility::generateMaskFromStrokes(src_feature_map[0].clone(), mask_strokes, src_mask);
//   
//     std::vector<cv::Mat> tar_feature_map;
//     app_mod_tar->getD0Features(tar_feature_map);
//     cv::Mat tar_mask(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1, 1);
//     ImageUtility::generateMultiMask(tar_feature_map[0].clone(), tar_mask);
//     std::vector<int> cur_sampled_tar_models;
//     ShapeUtility::vertexFilterFromParaMask(sampled_tar_model, cur_sampled_tar_models, tar_para_shape->vertex_set, tar_para_shape->cut_shape->getPolygonMesh(), tar_mask);
//   
//     // 2. make the masked source and target feature map
//     // 2.1 masked source
//     std::vector<cv::Mat> masked_src_feature_map;
//     ImageUtility::generateMaskedMatVec(src_feature_map, masked_src_feature_map, src_mask);
//     // 2.2 masked target
//     std::vector<cv::Mat> masked_tar_feature_map;
//     ImageUtility::generateMaskedMatVec(tar_feature_map, masked_tar_feature_map, tar_mask);
//   
//     // 3. correspondences from NNF
//     // 3.1 run patch match
//     syn_tool.reset(new SynthesisTool);
//     syn_tool->levels = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
//     syn_tool->patch_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
//     syn_tool->max_iter = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
//     syn_tool->best_random_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
//     syn_tool->lamd_occ = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
//     syn_tool->bias_rate = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
//     syn_tool->setExportPath(tar_model->getOutputPath());
//     syn_tool->doNNFOptimization(masked_src_feature_map, masked_tar_feature_map);
//   
//     // 3.2 find correspondences from NNF
//     std::vector<STLVectori> cur_src_crsp;
//     this->prepareLocalTransformCrsp(src_para_shape, tar_para_shape, syn_tool, cur_sampled_tar_models, cur_src_crsp);
//   
//     // 3.3 merge the current correspondences to sampled vertex set
//     ShapeUtility::mergeSubVector(sampled_tar_model, src_v_ids, cur_sampled_tar_models, cur_src_crsp);
//   
//     STLVectorf new_v_list;
//     double transform_scale = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale");
//     ShapeUtility::prepareLocalTransform(app_mod_src->getBaseMesh(), tar_model->getPolygonMesh(), src_v_ids, sampled_tar_model, new_v_list, transform_scale);
//   
//     LG::PolygonMesh old_tar_mesh = (*tar_model->getPolygonMesh()); // copy the old one
//     std::string obj_path = geometry_transfer->transferDeformation(tar_model, sampled_tar_model, new_v_list);
//   
//     ShapeUtility::computeLocalTransform(&old_tar_mesh, tar_model->getPolygonMesh());
//     ShapeUtility::exportVisForLocalTransform(tar_model->getPolygonMesh(), tar_model->getOutputPath());
//   
//     return obj_path;
//   }

void DetailSynthesis::debugSynthesisD0(std::string app_mod_path, std::shared_ptr<Model> tar_model)
{
  // load source appearance model
  std::shared_ptr<AppearanceModel> src_app_mod(new AppearanceModel());
  src_app_mod->importAppMod("app_model.xml", app_mod_path);

  // init target appearance model for the D0 features
  std::shared_ptr<AppearanceModel> tar_app_mod(new AppearanceModel());
  resolution = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution");
  tar_app_mod->setResolution(resolution);
  this->generateD0Feature(tar_app_mod.get(), tar_model);

  // run synthesis
  this->synthesisD0(src_app_mod.get(), tar_app_mod.get(), tar_model);
}

void DetailSynthesis::synthesisD1(AppearanceModel* app_mod_src, AppearanceModel* app_mod_tar, std::shared_ptr<Model> tar_model)
{
  std::vector<cv::Mat> src_feature_map;
  app_mod_src->getD1Features(src_feature_map);

  std::vector<cv::Mat> src_detail_map;
  app_mod_src->getD1Details(src_detail_map);

  std::vector<cv::Mat> tar_feature_map;
  app_mod_tar->getD1Features(tar_feature_map);

  cv::Mat cca_mat;
  app_mod_src->geteCCAMat(cca_mat);

  int count = 0;
  for (int i = 0; i < resolution; i++)
  {
    for (int j = 0; j < resolution; j++)
    {
      if (tar_feature_map[0].at<float>(i, j) > -1)
      {
        ++count;
      }
    }
  }

  std::cout << "n_filled = " << count << std::endl;

  cv::Mat cca_X_mat(count, tar_feature_map.size(), CV_32FC1);
  count = 0;
  std::vector<std::pair<int, int> > tar_pos;
  for (int i = 0; i < resolution; i++)
  {
    for (int j = 0; j < resolution; j++)
    {
      if (tar_feature_map[0].at<float>(i, j) > -1)
      {
        for (int k = 0; k < tar_feature_map.size(); k++)
        {
          cca_X_mat.at<float>(count, k) = tar_feature_map[k].at<float>(i, j);
        }
        count++;
        tar_pos.push_back(std::pair<int, int>(i, j));
      }
    }
  }

  cv::Mat new_X = cca_X_mat * cca_mat;
  std::vector<float> cca_min;
  std::vector<float> cca_max;
  app_mod_src->getCCAMin(cca_min);
  app_mod_src->getCCAMax(cca_max);
  ImageUtility::centralizeMat(new_X, 0, cca_min, cca_max, true);
  YMLHandler::saveToMat(tar_model->getOutputPath(), "test.mat", new_X);
  cv::Mat feature_map_backup = tar_feature_map[0].clone();
  tar_feature_map.clear();
  tar_feature_map.resize(new_X.cols);
  for (int i = 0; i < tar_feature_map.size(); i++)
  {
    tar_feature_map[i] = cv::Mat(resolution, resolution, CV_32FC1, -1);
  }
  for (int i = 0; i < tar_feature_map.size(); i++)
  {
    for (int j = 0; j < tar_pos.size(); j++)
    {
      tar_feature_map[i].at<float>(tar_pos[j].first, tar_pos[j].second) = new_X.at<float>(j, i);
    }
  }

  std::vector<cv::Mat> target_detail_map;
  app_mod_tar->getD1Details(target_detail_map); // use the existed target detail map
  if (target_detail_map.size() != src_detail_map.size())
  {
    target_detail_map.clear();
    for (size_t i = 0; i < src_detail_map.size(); ++i)
    {
      target_detail_map.push_back(cv::Mat::zeros(src_detail_map[0].rows, src_detail_map[0].cols, CV_32FC1));
    }
  }

  // 1. build the mask for source and target
  cv::Mat src_mask(src_feature_map[0].rows, src_feature_map[0].cols, CV_32FC1, 1);
  ImageUtility::generateMultiMask(src_detail_map[0].clone(), src_mask);
  cv::Mat tar_mask(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1, 1);
  ImageUtility::generateMultiMask(feature_map_backup.clone(), tar_mask);

  // 2. make the masked source and target feature map
  // 2.1 masked source
  std::vector<cv::Mat> masked_src_feature_map;
  ImageUtility::generateMaskedMatVec(src_feature_map, masked_src_feature_map, src_mask);
  std::vector<cv::Mat> masked_src_detail_map;
  ImageUtility::generateMaskedMatVec(src_detail_map, masked_src_detail_map, src_mask);
  // 2.2 masked target
  std::vector<cv::Mat> masked_tar_feature_map;
  ImageUtility::generateMaskedMatVec(tar_feature_map, masked_tar_feature_map, tar_mask);

  // 3. do synthesis
  syn_tool.reset(new SynthesisTool);
  syn_tool->setExportPath(tar_model->getOutputPath());
  syn_tool->lamd_gradient = 0.1;
  syn_tool->levels = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
  syn_tool->patch_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
  syn_tool->max_iter = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
  syn_tool->best_random_size = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
  syn_tool->lamd_occ = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
  syn_tool->bias_rate = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
  syn_tool->beta_func_center = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_center");
  syn_tool->beta_func_mult = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_mult");

  std::ofstream paraOutput(tar_model->getOutputPath() + "/synpara.txt");
  if (paraOutput)
  {
    paraOutput << "levels: " << syn_tool->levels << std::endl;
    paraOutput << "patch size: " << syn_tool->patch_size << std::endl;
    paraOutput << "max iter: " << syn_tool->max_iter << std::endl;
    paraOutput << "random size: " << syn_tool->best_random_size << std::endl;
    paraOutput << "lamd occ: " << syn_tool->lamd_occ << std::endl;
    paraOutput << "bias rate: " << syn_tool->bias_rate << std::endl;
    paraOutput << "beta center: " << syn_tool->beta_func_center << std::endl;
    paraOutput << "beta mult: " << syn_tool->beta_func_mult << std::endl;
    paraOutput.close();
  }

  syn_tool->init(masked_src_feature_map, masked_tar_feature_map, masked_src_detail_map);
  syn_tool->doSynthesisNew();

  // 4. merge the detail map together
  std::vector<cv::Mat> cur_result_detail;
  cur_result_detail.push_back(syn_tool->getTargetDetail()[0][0].clone()); // b
  cur_result_detail.push_back(syn_tool->getTargetDetail()[1][0].clone()); // g
  cur_result_detail.push_back(syn_tool->getTargetDetail()[2][0].clone()); // r
  cur_result_detail.push_back(syn_tool->getTargetDetail()[3][0].clone()); // disp
  ImageUtility::mergeMatVecFromMask(cur_result_detail, target_detail_map, tar_mask);

  std::vector<cv::Mat> result_detail;
  result_detail.push_back(target_detail_map[2].clone()); // r
  result_detail.push_back(target_detail_map[1].clone()); // g
  result_detail.push_back(target_detail_map[0].clone()); // b
  cv::Mat result_reflectance;
  cv::merge(result_detail, result_reflectance);
  double min, max;
  cv::minMaxLoc(result_reflectance, &min, &max);
  if (normalize_max < 0) result_reflectance = result_reflectance / max;
  else result_reflectance = result_reflectance / normalize_max;
  cv::Mat result_displacement = target_detail_map[3].clone();

  cv::imwrite(tar_model->getOutputPath() + "/reflectance.png", result_reflectance * 255);
  cv::imwrite(tar_model->getOutputPath() + "/tar_displacement.png", result_displacement * 255);
  YMLHandler::saveToFile(tar_model->getOutputPath(), "new_d2_displacement.yml", result_displacement);

  app_mod_tar->setD1Details(target_detail_map);
}

void DetailSynthesis::debugSynthesisD1(std::string app_mod_path, std::shared_ptr<Model> tar_model)
{
  // load source appearance model
  std::cout << std::endl << "Load Appearance Model" << std::endl;
  std::shared_ptr<AppearanceModel> src_app_mod(new AppearanceModel());
  src_app_mod->importAppMod("app_model.xml", app_mod_path);

  // init target appearance model for the D0 features
  std::cout << std::endl << "Generate Target D1 Features" << std::endl;
  std::shared_ptr<AppearanceModel> tar_app_mod(new AppearanceModel());
  resolution = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution");
  tar_app_mod->setResolution(resolution);
  this->generateD1Feature(tar_app_mod.get(), tar_model, true);

  // run synthesis
  this->synthesisD1(src_app_mod.get(), tar_app_mod.get(), tar_model);
}

std::string DetailSynthesis::runSynthesisD0(std::string app_mod_path, AppearanceModel* app_mod_tar, std::shared_ptr<Model> tar_model)
{
  // load source appearance model
  std::shared_ptr<AppearanceModel> src_app_mod(new AppearanceModel());
  src_app_mod->importAppMod("app_model.xml", app_mod_path);

  return this->synthesisD0(src_app_mod.get(), app_mod_tar, tar_model);
}

void DetailSynthesis::runSynthesisD1(std::string app_mod_path, AppearanceModel* app_mod_tar, std::shared_ptr<Model> tar_model)
{
  // load source appearance model
  std::cout << std::endl << "Load Appearance Model" << std::endl;
  std::shared_ptr<AppearanceModel> src_app_mod(new AppearanceModel());
  src_app_mod->importAppMod("app_model.xml", app_mod_path);

  // run synthesis
  this->synthesisD1(src_app_mod.get(), app_mod_tar, tar_model);
}