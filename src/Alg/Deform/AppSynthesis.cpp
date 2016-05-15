// Use Appearance Model to synthesize new model

#include "DetailSynthesis.h"

#include "GeometryTransfer.h"

#include "Shape.h"
#include "ParaShape.h"
#include "Ray.h"
#include "Bound.h"

#include "SynthesisTool.h"
#include "AppearanceModel.h"
#include "KevinVectorField.h"
#include "ShapeUtility.h"
#include "ImageUtility.h"
#include "ParameterMgr.h"
#include "YMLHandler.h"
#include <fstream>
#include "BasicHeader.h"

using namespace LG;

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
   // cv::Mat src_mask = GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:SrcAppMask").clone();// GLOBAL::m_mat_source_mask0_.clone();
	cv::Mat src_mask;
	app_mod_src->get_mask_from_origin_image_to_uv(
		GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:SrcAppOriginImageMask").clone(),
		src_mask
		);
	/*ImageUtility::generateMultiMask(src_feature_map[0].clone(), src_mask);*/

	// 1.1 generate mask from source image
	cv::Mat src_photo;
	app_mod_src->getPhoto(src_photo);
	// convert image stroke to para shape stroke

	std::vector<cv::Mat> tar_feature_map;
	app_mod_tar->getD0Features(tar_feature_map);


// 	std::vector<CvPoint> stroke;
//     app_mod_tar->coordFaceToUV(stroke, GlobalParameterMgr::GetInstance()->get_parameter<std::vector<int> >("Synthesis:TarAppMaskStroke"));
// 
// 	cv::Mat tar_mask(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1, 1);
// 	ImageUtility::generateMaskFromStroke(tar_feature_map[0].clone(), stroke, tar_mask);

	cv::Mat tar_mask = GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:TarAppMask").clone();

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
	syn_tool->levels = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
	syn_tool->patch_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
	syn_tool->max_iter = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
	syn_tool->best_random_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
	syn_tool->lamd_occ = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
	syn_tool->bias_rate = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
	syn_tool->setExportPath(tar_model->getOutputPath());
	syn_tool->doNNFOptimization(masked_src_feature_map, masked_tar_feature_map);

	// 3.2 find correspondences from NNF
	std::vector<STLVectori> cur_src_crsp;
	this->prepareLocalTransformCrsp(src_para_shape, tar_para_shape, syn_tool, cur_sampled_tar_models, cur_src_crsp);

	// 3.3 merge the current correspondences to sampled vertex set
	ShapeUtility::mergeSubVector(sampled_tar_model, src_v_ids, cur_sampled_tar_models, cur_src_crsp);

	STLVectorf new_v_list;
  double transform_scale = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale");
	ShapeUtility::prepareLocalTransform(app_mod_src->getBaseMesh(), tar_model->getPolygonMesh(), src_v_ids, sampled_tar_model, new_v_list, transform_scale);

	PolygonMesh old_tar_mesh = (*tar_model->getPolygonMesh()); // copy the old one
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
//     syn_tool->levels = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
//     syn_tool->patch_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
//     syn_tool->max_iter = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
//     syn_tool->best_random_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
//     syn_tool->lamd_occ = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
//     syn_tool->bias_rate = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
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
//     double transform_scale = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale");
//     ShapeUtility::prepareLocalTransform(app_mod_src->getBaseMesh(), tar_model->getPolygonMesh(), src_v_ids, sampled_tar_model, new_v_list, transform_scale);
//   
//     PolygonMesh old_tar_mesh = (*tar_model->getPolygonMesh()); // copy the old one
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
  resolution = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution");
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
      target_detail_map.push_back(cv::Mat::zeros(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1));
    }
  }

  // 1. build the mask for source and target
  //cv::Mat src_mask(src_feature_map[0].rows, src_feature_map[0].cols, CV_32FC1, 1);
  //ImageUtility::generateMultiMask(src_detail_map[0].clone(), src_mask);
  //cv::Mat tar_mask(tar_feature_map[0].rows, tar_feature_map[0].cols, CV_32FC1, 1);
  //ImageUtility::generateMultiMask(feature_map_backup.clone(), tar_mask);
  cv::Mat src_mask;
  app_mod_src->get_mask_from_origin_image_to_uv(
    GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:SrcAppOriginImageMask").clone(),
    src_mask
    );
  cv::Mat tar_mask = GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:TarAppMask").clone();


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
  syn_tool->levels = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels");
  syn_tool->patch_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size");
  syn_tool->max_iter = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter");
  syn_tool->best_random_size = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size");
  syn_tool->lamd_occ = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ");
  syn_tool->bias_rate = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate");
  syn_tool->beta_func_center = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_center");
  syn_tool->beta_func_mult = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_mult");

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
  resolution = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution");
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

std::string DetailSynthesis::applyD1Displacement(std::shared_ptr<Model> tar_model, cv::Mat& mask)
{
  cv::FileStorage fs2(tar_model->getOutputPath() + "/new_d2_displacement.yml", cv::FileStorage::READ);
  cv::Mat disp_map;
  fs2["new_d2_displacement"] >> disp_map;

  // test if mask and displacement have same resolution
  int m_resolution = std::min(mask.rows, mask.cols);
  int d_resolution = std::min(disp_map.rows, disp_map.cols);
  if (m_resolution != d_resolution)
  {
    std::cout << "mask and displacmenet map don't have same resolution!" << std::endl;
    return "";
  }
  {
    // centerize
    float value = 0;
    int value_cnt = 0;
    for (int i = 0; i < disp_map.rows; i++)
    {
      for (int j = 0; j < disp_map.cols; j++)
      {
        if (mask.at<float>(i, j) > 0.5)
        {
          value += disp_map.at<float>(i, j);
          ++value_cnt;
        }
      }
    }
    value = value / value_cnt;
    std::cout << "value: " << value << std::endl;
    for (int i = 0; i < disp_map.rows; i++)
    {
      for (int j = 0; j < disp_map.cols; j++)
      {
        disp_map.at<float>(i, j) = disp_map.at<float>(i, j) - value;
      }
    }
  }

  int n_ring = GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:n_ring");
  std::shared_ptr<ParaShape> src_para_shape(new ParaShape);
  src_para_shape->initWithExtPolygonMesh(tar_model->getPolygonMesh()); // target model provide smooth normal

  std::shared_ptr<ParaShape> tar_para_shape(new ParaShape);
  PolygonMesh subdiv_target;
  ShapeUtility::loadPolyMesh(&subdiv_target, tar_model->getDataPath() + "/subdiv_" + tar_model->getFileName());
  tar_para_shape->initWithExtPolygonMesh(&subdiv_target);

  std::set<int> crest_lines_points;
  for (size_t i = 0; i < tar_model->getShapeCrestLine().size(); i++)
  {
    for (size_t j = 0; j < tar_model->getShapeCrestLine()[i].size(); j++)
    {
      crest_lines_points.insert(tar_model->getShapeCrestLine()[i][j]);
    }
  }

  double scale = GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale");

  PolygonMesh* tar_mesh = &subdiv_target;
  PolygonMesh* src_mesh = tar_model->getPolygonMesh();

  std::vector<bool> visited_tag(tar_mesh->n_vertices(), false);

  PolygonMesh new_tar_mesh = subdiv_target;
  std::vector<int> displaced_vertex;
  std::vector<float> displaced_positions;

  // for debug
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::min();
  std::vector<Vec3> cache_normal;

  const STLVectori& vertex_set = tar_para_shape->vertex_set;
  std::shared_ptr<Shape> cut_shape = tar_para_shape->cut_shape;

  for (int i = 0; i < vertex_set.size(); i++)
  {
    if (visited_tag[vertex_set[i]]) continue;
    visited_tag[vertex_set[i]] = true;

    std::set<int> test_boundary_vertices;
    ShapeUtility::nRingVertices(cut_shape->getPolygonMesh(), i, test_boundary_vertices, n_ring);
    bool is_para_boundary = false;
    for (auto i_bound : test_boundary_vertices)
    {
      if (cut_shape->getPolygonMesh()->is_boundary(PolygonMesh::Vertex(i_bound)))
      {
        is_para_boundary = true;
        break;
      }
    }
    if (is_para_boundary) continue;

    float U, V;
    U = (cut_shape->getUVCoord())[2 * i + 0];
    V = (cut_shape->getUVCoord())[2 * i + 1];
    std::vector<float> pt(2, 0);
    pt[0] = U; pt[1] = V;
    int face_id;
    std::vector<int> id;
    std::vector<float> lambda;

    bool in_uv_mesh = ShapeUtility::findClosestUVFace(pt, src_para_shape.get(), lambda, face_id, id);
    int closest_v_id = ShapeUtility::closestVertex(src_para_shape->cut_shape->getPolygonMesh(), id, cut_shape->getPolygonMesh(), i);
    //bool in_crest_line = crest_lines_points.find(src_para_shape->vertex_set[closest_v_id]) == crest_lines_points.end() ? false : true;
    std::set<int> near_vertices;
    ShapeUtility::nRingVertices(src_mesh, src_para_shape->vertex_set[closest_v_id], near_vertices, 0); // near_vertices stores the vertex id in source mesh not para shape
    bool in_crest_line = false;
    for (auto i_near : near_vertices)
    {
      if (crest_lines_points.find(i_near) != crest_lines_points.end())
      {
        in_crest_line = true;
        break;
      }
    }
    //bool in_crest_line = false; // test if using outlier filter can allow not use feature line detect now

    Vec3 normal_check(0, 0, 0);
    if (in_uv_mesh && !in_crest_line)
    {
      // use face normal here
      //normal_check = src_mesh->face_attribute<Vec3>("f:normal")[PolygonMesh::Face(src_para_shape->face_set[face_id])];
      // use source mesh normal
      normal_check = lambda[0] * src_mesh->vertex_attribute<Vec3>("v:normal")[PolygonMesh::Vertex(src_para_shape->vertex_set[id[0]])]
        + lambda[1] * src_mesh->vertex_attribute<Vec3>("v:normal")[PolygonMesh::Vertex(src_para_shape->vertex_set[id[1]])]
        + lambda[2] * src_mesh->vertex_attribute<Vec3>("v:normal")[PolygonMesh::Vertex(src_para_shape->vertex_set[id[2]])];
      normal_check.normalize();
      if (_isnan(normal_check[0])) std::cout << "nan in normal computing!!!" << std::endl;
      /*ShapeUtility::getAverageNormalAroundVertex(tar_mesh, vertex_set[i], normal_check, 2);*/
    }
    else
    {
      // use the target average normal around this vertex
      ShapeUtility::getAverageNormalAroundVertex(tar_mesh, vertex_set[i], normal_check, 2);
    }

    // get vertex position and its displacement
    Vec3 pt_check = tar_mesh->position(PolygonMesh::Vertex(vertex_set[i]));
    int img_x, img_y;
    img_x = std::max(0, std::min(int(U * (float)resolution), resolution - 1));
    img_y = std::max(0, std::min(int(V * (float)resolution), resolution - 1));
    float cur_disp = 0;
    if (mask.at<float>(resolution - img_y - 1, img_x) > 0.5)
    {
      cur_disp = disp_map.at<float>(resolution - img_y - 1, img_x);
      //bool outlier = cur_disp < -1 ? true : (cur_disp > 1 ? true : false);
      bool outlier = false; // outlier has been put outside of the uv_mask now, determined by z-score
      if (in_uv_mesh && !in_crest_line && !outlier)
      {
        pt_check = pt_check + scale * cur_disp * normal_check;
      }
      else if (in_crest_line && in_uv_mesh)
      {
        pt_check = pt_check + scale * cur_disp * normal_check;
      }

      new_tar_mesh.position(PolygonMesh::Vertex(vertex_set[i])) = pt_check;

      if (in_uv_mesh && !in_crest_line && !outlier)
      {
        displaced_vertex.push_back(vertex_set[i]);
        displaced_positions.push_back(pt_check[0]);
        displaced_positions.push_back(pt_check[1]);
        displaced_positions.push_back(pt_check[2]);

        if (cur_disp > max) max = cur_disp;
        if (cur_disp < min) min = cur_disp;
        cache_normal.push_back(normal_check);
      }
    }
  }

  std::cout << "min: " << min << "\tmax: " << max << std::endl;
  //std::ofstream fdebug(tar_model->getOutputPath() + "/displace_info.txt");
  //if (fdebug)
  //{
  //  for (size_t i = 0; i < displaced_vertex.size(); ++i)
  //  {
  //    fdebug << displaced_vertex[i] << " " << displaced_positions[3 * i + 0] << " " << displaced_positions[3 * i + 1] << " " << displaced_positions[3 * i + 2] << " " << cache_normal[i].transpose() << std::endl;
  //  }
  //  fdebug.close();
  //}

  std::shared_ptr<GeometryTransfer> geometry_transfer(new GeometryTransfer);
  geometry_transfer->transferDeformation(tar_mesh, displaced_vertex, displaced_positions, 10.0f, false);

  char time_postfix[50];
  time_t current_time = time(NULL);
  strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
  std::string file_time_postfix = time_postfix;
  std::string output_name = tar_model->getOutputPath() + "/by_deform_detail_synthesis" + file_time_postfix + ".obj";
  ShapeUtility::savePolyMesh(&new_tar_mesh, tar_model->getOutputPath() + "/detail_synthesis" + file_time_postfix + ".obj");
  ShapeUtility::savePolyMesh(tar_mesh, output_name);
  return output_name;
}

void DetailSynthesis::generateD1FromAligned(std::shared_ptr<Model> tar_model)
{
  PolygonMesh height_mesh;
  if (!read_poly(height_mesh, tar_model->getDataPath() + "/height_mesh.obj"))
  {
    std::cout << "cannot load height_mesh.obj" << std::endl;
    cv::FileStorage fs2(tar_model->getDataPath() + "/final_height.xml", cv::FileStorage::READ);
    cv::Mat final_height_mat;
    fs2["final_height"] >> final_height_mat;
    PolygonMesh new_mesh;
    ShapeUtility::heightToMesh(final_height_mat, new_mesh, tar_model);
    ShapeUtility::savePolyMesh(&new_mesh, tar_model->getDataPath() + "/height_mesh.obj", false);

    return;
  }


  std::shared_ptr<Ray> ray_instance(new Ray);
  ShapeUtility::initBSPTreeRayFromPolyMesh(ray_instance.get(), &height_mesh);

  std::shared_ptr<ParaShape> src_para_shape(new ParaShape);
  src_para_shape->initWithExtShape(tar_model);
  std::set<int> visible_faces;
  ShapeUtility::visibleFacesInModel(tar_model, visible_faces);
  PolygonMesh* mesh = tar_model->getPolygonMesh();
  PolygonMesh::Vertex_attribute<Vec3> v_normals = mesh->vertex_attribute<Vec3>("v:normal");
  float perc = 0;

  std::cout << "bound radius: " << tar_model->getBoundBox()->getRadius() << std::endl;
  std::vector<float> dis;

  for (auto vit : mesh->vertices())
  {
    Vector3f pos = mesh->position(vit);
    Vector3f dir = v_normals[vit];
    dir.normalize();

    Vector3f end = pos + tar_model->getBoundBox()->getRadius() * dir;
    Vector3f neg_end = pos - tar_model->getBoundBox()->getRadius() * dir;
    float epslon = 1e-5;
    Eigen::Vector3d intersect_point, neg_intersect_point;
    bool is_intersected, is_neg_intersected;
    is_intersected = ray_instance->intersectModel((pos + epslon * dir).cast<double>(), end.cast<double>(), intersect_point.data());
    is_neg_intersected = ray_instance->intersectModel((pos - epslon * dir).cast<double>(), neg_end.cast<double>(), neg_intersect_point.data());
    if (is_intersected == false && is_neg_intersected == false)
    {
      // if no intersection, we don't store this point and make it and outlier
    }
    else
    {
      float distance = (intersect_point.cast<float>() - pos).norm();;
      float neg_distance = (neg_intersect_point.cast<float>() - pos).norm();;
      float actual_distance = distance > neg_distance ? -neg_distance : distance;
      if (fabs(actual_distance) < (tar_model->getBoundBox()->getRadius() * 0.03))
      {
        mesh->position(vit) = pos + actual_distance * dir;
        dis.push_back(actual_distance);
      }
    }

    float cur_perc = (float)vit.idx() / mesh->n_vertices();
    if (cur_perc - perc >= 0.05)
    {
      perc = cur_perc;
      std::cout << perc << "...";
    }
  }

  ShapeUtility::savePolyMesh(mesh, tar_model->getOutputPath() + "/D1FromAligned.obj");

  VertexList old_v_list = tar_model->getShapeOriVertexList();
  tar_model->updateShape(old_v_list);

  std::ofstream f_debug(tar_model->getOutputPath() + "/dis.txt");
  if (f_debug)
  {
    for (auto i : dis)
    {
      f_debug << i << std::endl;
    }
    f_debug.close();
  }
}