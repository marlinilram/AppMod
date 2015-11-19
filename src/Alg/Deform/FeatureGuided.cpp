#include "FeatureGuided.h"
#include "Model.h"
#include "FeatureLine.h"
#include "ScalarField.h"
#include "LargeFeatureCrsp.h"
#include "KDTreeWrapper.h"
#include "tele2d.h"
#include "CurvesUtility.h"
#include "ParameterMgr.h"
#include "YMLHandler.h"

FeatureGuided::FeatureGuided()
{
}

FeatureGuided::~FeatureGuided()
{
  std::cout << "Deleted a FeatureGuided.\n";
}

FeatureGuided::FeatureGuided(std::shared_ptr<Model> source_model, std::string targetFile)
{
  this->initTargetImage(targetFile);
  this->source_model = source_model;
  this->initRegister();
}

void FeatureGuided::initTargetImage(std::string targetFile)
{
  cv::imread(targetFile + "/featurePPnms.png", CV_LOAD_IMAGE_GRAYSCALE).convertTo(this->target_img, CV_32FC1);
  this->target_img = this->target_img / 255.0;

  cv::imread(targetFile + "/featurePP.png", CV_LOAD_IMAGE_GRAYSCALE).convertTo(this->target_edge_saliency, CV_32FC1);
  this->target_edge_saliency = this->target_edge_saliency / 255.0;
}

void FeatureGuided::initRegister()
{
  // extract edges from souce image and target image
  this->source_curves.clear();
  this->target_curves.clear();
  this->edge_threshold = 0.9;
  this->ExtractSrcCurves(source_model->getEdgeImg(), this->source_curves);
  this->edge_threshold = 0.01;
  this->ExtractCurves(this->target_img, this->target_curves);

  // output curves
  //std::ofstream f_debug(source_model->getDataPath() + "/source_curve.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < source_curves.size(); ++i)
  //  {
  //    for (size_t j = 0; j < source_curves[i].size(); ++j)
  //    {
  //      f_debug << source_curves[i][j].x << " " << source_curves[i][j].y << "\n";
  //    }
  //  }
  //  f_debug.close();
  //}

  //f_debug.open(source_model->getDataPath() + "/target_curve.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < target_curves.size(); ++i)
  //  {
  //    for (size_t j = 0; j < target_curves[i].size(); ++j)
  //    {
  //      f_debug << target_curves[i][j].x << " " << target_curves[i][j].y << "\n";
  //    }
  //  }
  //  f_debug.close();
  //}

  this->setNormalizePara();
  CURVES temp_source_curves;
  CURVES temp_target_curves;
  this->NormalizedSourceCurves(temp_source_curves);
  this->NormalizedTargetCurves(temp_target_curves);

  // init source tele2d
  std::vector<std::vector<int>> group(1);
  std::vector<int2> endps;
  for (int i = 0; i < this->source_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  this->source_tele_register.reset(new tele2d(100, 0.02, 1));
  this->source_tele_register->init(temp_source_curves, group, endps);
  this->source_tele_register->setInputField();

  // init target tele2d
  group[0].clear();
  endps.clear();
  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  this->target_tele_register.reset(new tele2d(100, 0.02, 1));
  this->target_tele_register->init(temp_target_curves, group, endps);
  this->target_tele_register->setInputField();

  // init source distance map
  //this->BuildDispMap(source_img, source_KDTree_data);
  //this->source_KDTree.reset(new kdtree::KDTree(source_KDTree_data));

  // Optimize Connection
  //this->OptimizeConnection();

  // Search correspondences
  source_vector_field_lines.reset(new FeatureLine);
  target_vector_field_lines.reset(new FeatureLine);
  source_KDTree.reset(new KDTreeWrapper);
  target_KDTree.reset(new KDTreeWrapper);
  BuildTargetEdgeKDTree();

  source_scalar_field.reset(new ScalarField(100, 2));
  target_scalar_field.reset(new ScalarField(300, 50));
  source_scalar_field->setTeleRegister(source_tele_register);
  target_scalar_field->setTeleRegister(target_tele_register);
  //source_scalar_field->computeVariationMap();
  //target_scalar_field->computeDistanceMap(this);
  //target_scalar_field->computeVariationMap();
  //target_scalar_field->computeMatchingMap(source_tele_register->vector_field);
  this->updateDistSField();

  this->BuildClosestPtPair();
}

void FeatureGuided::updateSourceVectorField()
{
  this->source_curves.clear();
  this->edge_threshold = 0.9;
  this->ExtractSrcCurves(source_model->getEdgeImg(), this->source_curves);
  CURVES temp_source_curves;
  this->NormalizedSourceCurves(temp_source_curves);

  std::vector<std::vector<int>> group(1);
  std::vector<int2> endps;
  for (int i = 0; i < this->source_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  this->source_tele_register->init(temp_source_curves, group, endps);
  this->source_tele_register->setInputField();

  std::cout << "curve integrate: " << this->target_scalar_field->curveIntegrate(this->source_curves, this) << std::endl;
}

void FeatureGuided::updateScalarField()
{
  //this->source_scalar_field->computeVariationMap();
  //this->target_scalar_field->computeMatchingMap(this->source_tele_register->vector_field);
}

void FeatureGuided::updateSourceField(int update_type)
{
  if (update_type == 0)
  {
    this->updateSourceVectorField();
    this->updateScalarField();

    this->BuildClosestPtPair();
  }
  else if (update_type == 1)
  {
    this->updateDistSField();
    this->BuildClosestPtPair();
  }
  else if (update_type == 2)
  {
    this->updateSourceVectorField();
  }
  else if (update_type == 3)
  {
    this->target_scalar_field->win_center = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinCenter");
    this->target_scalar_field->win_width = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinWidth");
  }
}

void FeatureGuided::updateDistSField()
{
  this->target_scalar_field->dist_attenuation = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:rad");
  this->target_scalar_field->para_a = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  this->target_scalar_field->para_b = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  this->target_scalar_field->win_center = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinCenter");
  this->target_scalar_field->win_width = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinWidth");
  this->target_scalar_field->computeDistanceMap(this);
}

void FeatureGuided::ExtractSrcCurves(const cv::Mat& source, CURVES& curves)
{
  // source curves just use the vis_crest_lines in ShapeCrest
  const std::vector<STLVectori>& crest_lines = source_model->getShapeVisbleCrestLine();
  const VertexList& vertex_list = source_model->getShapeVertexList();
  Matrix4f model_transform = Matrix4f::Identity();
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:renderWithTransform") != 0)
  {
    model_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  }
  std::vector<double2> curve;
  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    curve.clear();
    for (size_t j = 0; j < crest_lines[i].size(); ++j)
    {
      Vector4f v;
      v[0] = vertex_list[3 * crest_lines[i][j] + 0];
      v[1] = vertex_list[3 * crest_lines[i][j] + 1];
      v[2] = vertex_list[3 * crest_lines[i][j] + 2];
      v[3] = 1.0f;
      v = model_transform * v;
      v = v / v[3];
      float winx, winy;
      source_model->getProjectPt(v.data(), winx, winy);
      curve.push_back(double2(winx, source.rows - winy));
    }
    curves.push_back(curve);
  }

  // we also need to store the boundary curve
  CURVES boundary_curve;
  //this->ExtractCurves(source_model->getEdgeImg(), boundary_curve);
  //curves.insert(curves.end(), boundary_curve.begin(), boundary_curve.end());
}

void FeatureGuided::ExtractCurves(const cv::Mat& source, CURVES& curves)
{
  // find curves
  std::vector<std::vector<bool>> visited_table(source.rows, std::vector<bool>(source.cols, false));
  std::vector<double2> curve;
  for (int i = 0; i < source.rows; ++i)
  {
    for (int j = 0; j < source.cols; ++j)
    {
      if (source.at<float>(i, j) > edge_threshold && visited_table[i][j] == false)
      {
        curve.clear();
        this->SearchCurve(source, i, j, visited_table, curve);
        CURVES cur_curves = CurvesUtility::SplitCurve(curve);
        curves.insert(curves.end(), cur_curves.begin(), cur_curves.end());
      }
    }
  }

  // reorganize curves and detect break points
  int bk_sp_rate = 3;
  curves = CurvesUtility::ReorganizeCurves(curves, 1);
  std::vector<std::vector<int> > bk_points = CurvesUtility::DetectBreakPointAll(CurvesUtility::SmoothCurves(CurvesUtility::ReorganizeCurves(curves, bk_sp_rate), 3), 11, 0.87); // cos(30') = 0.8660
  // break the curves according to the break points
  for (size_t i = 0; i < bk_points.size(); +i)
  {
    for (size_t j = 0; j < bk_points[i].size(); ++j)
    {
      bk_points[i][j] *= bk_sp_rate;
    }
  }
  curves = CurvesUtility::BreakCurves(curves, bk_points);


  // save edges lenth from each sample to two ends of the curve
  target_edges_sp_len.clear();
  CurvesUtility::CurveSpTwoSideDist(target_edges_sp_len, curves);

  target_edges_sp_sl.clear();
  CurvesUtility::CurveSpSaliency(target_edges_sp_sl, curves, target_edge_saliency);

#define DEBUG
#ifdef DEBUG
  std::vector<std::vector<cv::Point>> contours;
  //cv::findContours(source, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // draw curves
  cv::Mat contour = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
  for (int i = 0; i < curves.size(); ++i)
  {
    for (int j = 0; j < curves[i].size(); ++j)
    {
      contour.at<uchar>(source.rows - 1 - curves[i][j].y, curves[i][j].x) = 255;
    }
    cv::imwrite(source_model->getDataPath() + "/curve_imgs/" + std::to_string(i) + ".png", contour);
  }

#endif
}

void FeatureGuided::SearchCurve(const cv::Mat& source,
  int cur_row, int cur_col,
  std::vector<std::vector<bool>>& visited_table,
  std::vector<double2>& curve)
{
  // if the current pos isn't a edge or is visited before or out of boundary, return
  if (cur_row < 0 || cur_row >= source.rows || cur_col < 0 || cur_col >= source.cols
    || source.at<float>(cur_row, cur_col) < edge_threshold || visited_table[cur_row][cur_col] == true)
  {
    return;
  }

  curve.push_back(double2(cur_col, source.rows - 1 - cur_row));
  visited_table[cur_row][cur_col] = true;


  int new_row = cur_row;
  int new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  //for (int i = 0; i < 3; ++i)
  //{
  //  for (int j = 0; j < 3; ++j)
  //  {
  //    int new_row = cur_row + i - 1;
  //    int new_col = cur_col + j - 1;
  //    if (!((new_row == cur_row && new_col == cur_row)
  //      || (new_row == last_row && new_col == last_col)))
  //    {
  //      FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  //    }
  //  }
  //}
}


void FeatureGuided::NormalizedTargetCurves(CURVES& curves)
{
  curves = this->target_curves;
  CurvesUtility::NormalizedCurves(curves, curve_translate, curve_scale);
}

void FeatureGuided::NormalizedSourceCurves(CURVES& curves)
{
  curves = this->source_curves;
  CurvesUtility::NormalizedCurves(curves, curve_translate, curve_scale);
}

void FeatureGuided::NormalizedPts(double2& pt)
{
  pt = ( pt + curve_translate - double2(0.5, 0.5 ) ) * curve_scale + double2(0.5, 0.5 );
}

void FeatureGuided::GetSourceNormalizePara(double2& translate, double& scale)
{
  translate = curve_translate;
  scale = curve_scale;
  //FeatureGuided::NormalizePara(this->source_curves, translate, scale);
}

std::shared_ptr<KDTreeWrapper> FeatureGuided::getSourceKDTree()
{
  return this->source_KDTree;
}

void FeatureGuided::BuildEdgeKDTree(CURVES& curves, std::shared_ptr<KDTreeWrapper> kdTree)
{
  std::vector<float> data;
  kdtree_id_mapper.clear();
  size_t n_pts = 0;
  for (size_t i = 0; i < curves.size(); ++i)
  {
    for (size_t j = 0; j < curves[i].size(); ++j)
    {
      data.push_back((float)curves[i][j].x);
      data.push_back((float)curves[i][j].y);
      kdtree_id_mapper[n_pts] = std::pair<int, int>(i, j);
      ++ n_pts;
    }
  }


  kdTree->initKDTree(data, n_pts, 2);
}

void FeatureGuided::BuildSourceEdgeKDTree()
{
  BuildEdgeKDTree(source_curves, source_KDTree);
}

void FeatureGuided::BuildTargetEdgeKDTree()
{
  BuildEdgeKDTree(target_curves, target_KDTree);
}

void FeatureGuided::setNormalizePara()
{
  CurvesUtility::NormalizePara(target_curves, curve_translate, curve_scale);
}

void FeatureGuided::BuildClosestPtPair()
{
  typedef std::pair<int, int> CurvePt;
  std::map<CurvePt, CurvePt> crsp_map;
  std::map<CurvePt, CurvePt>::iterator it;
  std::shared_ptr<LargeFeatureCrsp> lf_crsp(new LargeFeatureCrsp(this));
  lf_crsp->buildCrsp(crsp_map);

  // this function should be called if and only if the source curve is updated
  // when rebuild the correspondence pair, the src_crsp_list and tar_crsp_list
  // need to be updated and user_correct_crsp_map need to be clear up
  src_crsp_list.clear();
  tar_crsp_list.clear();
  user_correct_crsp_map.clear();
  for (it = crsp_map.begin(); it != crsp_map.end(); ++it)
  {
    src_crsp_list.push_back(CurvePt(it->first));
    tar_crsp_list.push_back(CurvePt(it->second));
  }

  //std::ofstream f_debug(source_model->getDataPath() + "/source_curve.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < src_crsp_list.size(); ++i)
  //  {
  //    double2 pt = source_curves[src_crsp_list[i].first][src_crsp_list[i].second];
  //    f_debug << pt.x << " " << pt.y << std::endl;
  //  }
  //  f_debug.close();
  //}
  //f_debug.open(source_model->getDataPath() + "/target_curve.txt");
  //if (f_debug)
  //{
  //  for (size_t i = 0; i < tar_crsp_list.size(); ++i)
  //  {
  //    double2 pt = target_curves[tar_crsp_list[i].first][tar_crsp_list[i].second];
  //    f_debug << pt.x << " " << pt.y << std::endl;
  //  }
  //  f_debug.close();
  //}
}

void FeatureGuided::setUserCrspPair(double start[2], double end[2])
{
  // input are normalized coordinate
  double2 src_p(start[0], start[1]);
  src_p = (src_p - double2(0.5, 0.5)) / curve_scale + double2(0.5, 0.5) - curve_translate;

  int src_i = -1;
  int src_j = -1;
  double dis = 0.0;
  if (CurvesUtility::closestPtInCurves(src_p, source_curves, src_i, src_j, dis))
  {
    const std::vector<STLVectori>& crest_lines = source_model->getShapeVisbleCrestLine();
    user_constrained_src_v = crest_lines[src_i][src_j];

    user_constrained_tar_p = double2(end[0], end[1]);
    user_constrained_tar_p = (user_constrained_tar_p - double2(0.5, 0.5)) / curve_scale + double2(0.5, 0.5) - curve_translate;

    user_correct_crsp_map[STLPairii(src_i, src_j)] = user_constrained_tar_p;
  }
}

void FeatureGuided::GetCurrentCrspList(std::vector<std::pair<int, double2> >& crsp_list)
{
  const std::vector<STLVectori>& crest_lines = source_model->getShapeVisbleCrestLine();

  if (src_crsp_list.size() != tar_crsp_list.size())
  {
    std::cerr << "Size of source correspondence list doesn't match that of target.\n";
    return;
  }

  std::map<STLPairii, double2>::iterator map_iter;
  for (size_t i = 0; i < src_crsp_list.size(); ++i)
  {
    int v_id = crest_lines[src_crsp_list[i].first][src_crsp_list[i].second];
    double2 tar_pt;
    
    map_iter = user_correct_crsp_map.find(src_crsp_list[i]);
    if (map_iter == user_correct_crsp_map.end())
    {
      tar_pt = target_curves[tar_crsp_list[i].first][tar_crsp_list[i].second];
    }
    else
    {
      tar_pt = map_iter->second;;
    }

    crsp_list.push_back(std::pair<int ,double2>(v_id, tar_pt));
  }
}