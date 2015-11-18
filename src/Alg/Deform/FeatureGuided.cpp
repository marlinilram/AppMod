#include "FeatureGuided.h"
#include "Model.h"
#include "FeatureLine.h"
#include "ScalarField.h"
#include "KDTreeWrapper.h"
#include "tele2d.h"
#include "CurvesUtility.h"
#include "ParameterMgr.h"
#include "YMLHandler.h"
#include "RandSample.h"

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

  curves = CurvesUtility::ReorganizeCurves(curves, 1);

  // save edges lenth from each sample to two ends of the curve
  target_edges_sp_len.clear();
  std::vector<double2> temp_edge_sp_len;
  for (size_t i = 0; i < curves.size(); ++i)
  {
    temp_edge_sp_len.clear();
    double cur_length = 0;
    temp_edge_sp_len.push_back(double2(cur_length, 0));
    for (int j = 1; j < curves[i].size(); ++j)
    {
      cur_length += 
        sqrt(pow(curves[i][j].x - curves[i][j - 1].x, 2)
        + pow(curves[i][j].y - curves[i][j - 1].y, 2));

      temp_edge_sp_len.push_back(double2(cur_length, 0));
    }
    
    for (size_t j = 0; j < temp_edge_sp_len.size(); ++j)
    {
      temp_edge_sp_len[j].y = cur_length - temp_edge_sp_len[j].x;
    }
    target_edges_sp_len.push_back(temp_edge_sp_len);
  }

  target_edges_sp_sl.clear();
  for (size_t i = 0; i < curves.size(); ++i)
  {
    std::vector<double> temp_edge_sp_sl;
    for (size_t j = 0; j < curves[i].size(); ++j)
    {
      double saliency = 0.0;
      for (int k = -10; k <= 10; ++k)
      {
        int cur_idx = int(j) + k;
        if (cur_idx >= 0 && cur_idx < curves[i].size())
        {
          // get the saliency
          int img_i = target_edge_saliency.rows - (curves[i][cur_idx].y + 0.5);
          int img_j = curves[i][cur_idx].x + 0.5;
          img_i = img_i < 0 ? 0 : (img_i < target_edge_saliency.rows ? img_i : target_edge_saliency.rows);
          img_j = img_j < 0 ? 0 : (img_j < target_edge_saliency.cols ? img_j : target_edge_saliency.cols);
          saliency += target_edge_saliency.at<float>(img_i, img_j);
        }
      }
      temp_edge_sp_sl.push_back(saliency / 21);
    }
    target_edges_sp_sl.push_back(temp_edge_sp_sl);
  }

std::ofstream f_debug(source_model->getDataPath() + "/saliency.mat");
if (f_debug)
{
  for (size_t i = 0; i < target_edges_sp_sl.size(); ++i)
  {
    for (size_t j = 0; j < target_edges_sp_sl[i].size(); ++j)
    {
      f_debug << target_edges_sp_sl[i][j] << "\n";
    }
  }
  f_debug.close();
}

//#define DEBUG
//#ifdef DEBUG
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
//#endif
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


void FeatureGuided::GetUserCrspPair(CURVES& curves, float sample_density)
{
  double2 target_translate = curve_translate;
  double target_scale = curve_scale;
  double2 source_translate = curve_translate;
  double source_scale = curve_scale;

  CURVES source_user_lines = source_vector_field_lines->lines;
  CURVES target_user_lines = target_vector_field_lines->lines;

  size_t num_line_pair = source_user_lines.size();
  if (source_user_lines.size() != target_user_lines.size())
  {
    std::cout << "Number of User defined line-pair doesn't match.\n";
    num_line_pair = source_user_lines.size() < target_user_lines.size() ? source_user_lines.size() : target_user_lines.size();
  }

  for (size_t i = 0; i < num_line_pair; ++i)
  {
    CurvesUtility::DenormalizedCurve(source_user_lines[i], source_translate, source_scale);
    CurvesUtility::DenormalizedCurve(target_user_lines[i], target_translate, target_scale);

    // resample source curve based on sample density
    std::cout << "Number of points in user defined source curve: " << source_user_lines[i].size() << "\n";
    std::cout << "Number of points in user defined target curve: " << target_user_lines[i].size() << "\n";

    CURVE sampled_curve;
    sampled_curve.push_back(source_user_lines[i][0]);
    double accum_dist = 0.0;
    for (size_t j = 1; j < source_user_lines[i].size(); ++j)
    {
      double2 previousPoint = source_user_lines[i][j - 1];
      double2 currentPoint = source_user_lines[i][j];
      accum_dist += sqrt(pow(currentPoint.x - previousPoint.x,2) + pow(currentPoint.y - previousPoint.y,2));
      if (accum_dist > sample_density)
      {
        sampled_curve.push_back(currentPoint);
        accum_dist = 0.0;
      }
    }
    source_user_lines[i] = sampled_curve;

    // resample target curve based on the number of sampled source curve
    // in this way, correspondences are built automatically
    double target_sample_density = (CurvesUtility::CurveLength(target_user_lines[i]) / (sampled_curve.size()) - 1);
    size_t step = target_user_lines[i].size() / (sampled_curve.size()) + 1;
    if (step * (sampled_curve.size()) != target_user_lines[i].size())
    {
      int interp_num = step * (sampled_curve.size()) - target_user_lines[i].size();
      std::vector<int> interp_v;
      RandSample(1, target_user_lines[i].size() - 1, interp_num, interp_v);

      CURVE interp_curve;
      for (size_t j = 0; j < interp_v.size(); ++j)
      {
        double2 previousPoint = target_user_lines[i][interp_v[j] - 1 + j];
        double2 currentPoint = target_user_lines[i][interp_v[j] + j];
        double2 midPoint(0.5 * (previousPoint.x + currentPoint.x), 0.5 * (previousPoint.y + currentPoint.y));
        target_user_lines[i].insert(target_user_lines[i].begin() + interp_v[j] + j, midPoint);
      }
    }
    sampled_curve.clear();
    for (size_t j = 0; j < target_user_lines[i].size(); ++j)
    {
      if (j % step == 0)
      {
        sampled_curve.push_back(target_user_lines[i][j]);
      }
      //double2 previousPoint = target_user_lines[i][j - 1];
      //double2 currentPoint = target_user_lines[i][j];
      //accum_dist += sqrt(pow(currentPoint.x - previousPoint.x,2) + pow(currentPoint.y - previousPoint.y,2));
      //if (accum_dist > target_sample_density)
      //{
      //  double interp = (accum_dist - target_sample_density)
      //    / sqrt(pow(currentPoint.x - previousPoint.x,2) + pow(currentPoint.y - previousPoint.y,2));
      //  double2 midPoint((1 - interp) * previousPoint.x + interp * currentPoint.x,
      //    (1 - interp) * previousPoint.y + interp * currentPoint.y);
      //  sampled_curve.push_back(currentPoint);
      //  //accum_dist = sqrt(pow(currentPoint.x - midPoint.x,2) + pow(currentPoint.y - midPoint.y,2));
      //  accum_dist = 0.0;
      //}
    }
    target_user_lines[i] = sampled_curve;

    if (source_user_lines[i].size() != target_user_lines[i].size())
    {
      std::cout << "Error: number of points in sampled user defined source and target curves doesn't match.\n";
      std::cout << "Points in source: " << source_user_lines[i].size() << " Points in target: " << target_user_lines[i].size() << std::endl;
    }
    else
    {
      std::cout << "Number of points in sampled user defined curve " << i << " : " << sampled_curve.size() << "\n";
    }
  }

  std::vector<double2> crsp_pair(2);
  std::vector<float>   temp_pt(2, 0.0);
  for (size_t i = 0; i < num_line_pair; ++i)
  {
    size_t num_pts = std::min(source_user_lines[i].size(), target_user_lines[i].size());
    for (size_t j = 0; j < num_pts; ++j)
    {
      crsp_pair[0] = source_user_lines[i][j];
      temp_pt[0] = (float)crsp_pair[0].x;
      temp_pt[1] = (float)crsp_pair[0].y;
      source_KDTree->nearestPt(temp_pt);
      crsp_pair[0].x = temp_pt[0];
      crsp_pair[0].y = temp_pt[1];

      crsp_pair[1] = target_user_lines[i][j];
      temp_pt[0] = (float)crsp_pair[1].x;
      temp_pt[1] = (float)crsp_pair[1].y;
      target_KDTree->nearestPt(temp_pt);
      crsp_pair[1].x = temp_pt[0];
      crsp_pair[1].y = temp_pt[1];

      curves.push_back(crsp_pair); 
    }
  }

  // test if the line are correct
  //for (size_t i = 0; i < target_user_lines.size(); ++i)
  //{
  //  FeatureGuided::NormalizedCurve(target_user_lines[i], target_translate, target_scale);
  //  target_vector_field_lines->lines[i] = target_user_lines[i];
  //}
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
  // Build corresponding point pair between source curves and target curves
  //
  // Method routine:
  // Search from each curve points in target to find the closest points
  // in source. Since points in source are one-to-one mapped to vertex
  // in the agent, we get vertex to pixel correspondences naturally
  // (see ExtractSrcCurves() about how the source curve maps to 
  // each vertex in shape)
  //
  // More than one curve points in target will find same closest points
  // in source (depending on the sample density of target curves). In this
  // case we choose the closest one.
  //
  // Distance Evaluation: There are different ways to evaluate the distance
  // between a source points and a target points. For now, we compute it in
  // this way. Integrate the matching scalar map between the target and source

  typedef std::pair<int, int> CurvePt;
  typedef std::pair<std::pair<int, int>, double> CrspCurvePt;

  CURVES n_src_curves; // get normalized source curves
  CURVES n_tar_curves; // get normalized target curves
  this->NormalizedSourceCurves(n_src_curves);
  this->NormalizedTargetCurves(n_tar_curves);

  // find corresponding points in source curves
  std::vector<double> paras(3, 0);
  paras[1] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  paras[2] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  std::map<CurvePt, CrspCurvePt> crsp_map;
  std::map<CurvePt, CrspCurvePt>::iterator it;
  for (size_t i = 0; i < n_tar_curves.size(); ++i)
  {
    for (size_t j = 0; j < n_tar_curves[i].size(); ++j)
    {
      int src_i = -1;
      int src_j = -1;
      double dis = 0.0;
      paras[0] = target_edges_sp_sl[i][j];
      //if (CurvesUtility::closestPtInCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, target_scalar_field->matching_map, target_scalar_field->resolution, 0.3))
      if (CurvesUtility::closestPtInSaliencyCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, paras))
      {
        it = crsp_map.find(CurvePt(src_i, src_j));
        if (it != crsp_map.end())
        {
          if (dis > it->second.second)
          {
            it->second = CrspCurvePt(CurvePt(int(i), int(j)), dis);
          }
        }
        else
        {
          crsp_map[CurvePt(src_i, src_j)] = CrspCurvePt(CurvePt(int(i), int(j)), dis);
        }
      }
    }
  }

  // this function should be called if and only if the source curve is updated
  // when rebuild the correspondence pair, the src_crsp_list and tar_crsp_list
  // need to be updated and user_correct_crsp_map need to be clear up
  src_crsp_list.clear();
  tar_crsp_list.clear();
  user_correct_crsp_map.clear();
  for (it = crsp_map.begin(); it != crsp_map.end(); ++it)
  {
    src_crsp_list.push_back(CurvePt(it->first));
    tar_crsp_list.push_back(CurvePt(it->second.first));
  }
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