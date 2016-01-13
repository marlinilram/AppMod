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
#include <QDir>
#include <QString>

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
  cv::imread(targetFile + "/feature_nms_1.png", CV_LOAD_IMAGE_GRAYSCALE).convertTo(this->target_img, CV_32FC1);
  this->target_img = this->target_img / 255.0;

  cv::imread(targetFile + "/feature_nms_0.png", CV_LOAD_IMAGE_GRAYSCALE).convertTo(this->target_edge_saliency, CV_32FC1);
  this->target_edge_saliency = this->target_edge_saliency / 255.0;
}

void FeatureGuided::initRegister()
{
  // Important! need to call initTarRegister() first
  this->initTarRegister();
  this->initSrcRegister();

  // extract edges from souce image and target image

  


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


  

  //source_scalar_field->computeVariationMap();
  //target_scalar_field->computeDistanceMap(this);
  //target_scalar_field->computeVariationMap();
  //target_scalar_field->computeMatchingMap(source_tele_register->vector_field);
  this->updateDistSField();

  this->BuildClosestPtPair();
}

void FeatureGuided::initTarRegister(int type)
{
  if (type == 0)
  {
    this->target_curves.clear();
    this->edge_threshold = 0.01;
    this->ExtractCurves(this->target_img, this->target_curves);
    this->setNormalizePara();
  }
  else if (type == 1)
  {
    // do nothing update the rest of target related things
  }

  CURVES temp_target_curves;
  this->NormalizedTargetCurves(temp_target_curves);

  // init target tele2d
  std::vector<std::vector<int>> group(1);
  std::vector<int2> endps;
  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  this->target_tele_register.reset(new tele2d(100, 0.02, 1));
  this->target_tele_register->init(temp_target_curves, group, endps);
  this->target_tele_register->setInputField();

  target_vector_field_lines.reset(new FeatureLine);
  target_KDTree.reset(new KDTreeWrapper);
  BuildTargetEdgeKDTree();

  target_scalar_field.reset(new ScalarField(500, 50));
  target_scalar_field->setTeleRegister(target_tele_register);
}

void FeatureGuided::initSrcRegister(int type)
{
  if (type == 0)
  {
    this->source_curves.clear();
    this->edge_threshold = 0.9;
    this->ExtractSrcCurves(source_model->getEdgeImg(), this->source_curves);
  }
  else if (type == 1)
  {
    // do nothing
  }

  CURVES temp_source_curves;
  this->NormalizedSourceCurves(temp_source_curves);

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



  // init source distance map
  //this->BuildDispMap(source_img, source_KDTree_data);
  //this->source_KDTree.reset(new kdtree::KDTree(source_KDTree_data));

  // Optimize Connection
  //this->OptimizeConnection();

  // Search correspondences
  source_vector_field_lines.reset(new FeatureLine);
  source_KDTree.reset(new KDTreeWrapper);
  BuildSourceEdgeKDTree();
  


  source_scalar_field.reset(new ScalarField(100, 2));
  source_scalar_field->setTeleRegister(source_tele_register);
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

  //std::cout << "curve integrate: " << this->target_scalar_field->curveIntegrate(this->source_curves, this) << std::endl;
  //this->user_define_curve_crsp.clear();
  //this->user_marked_crsp.clear();
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
    this->BuildSourceEdgeKDTree();
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
    this->BuildSourceEdgeKDTree();
    this->BuildClosestPtPair();
  }
  else if (update_type == 3)
  {
    this->target_scalar_field->win_center = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinCenter");
    this->target_scalar_field->win_width = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinWidth");
  }
  else if (update_type == 4)
  {
    this->locateMarkedCurves();
    this->BuildClosestPtPair();
  }
  else if (update_type == 5)
  {
    this->updateSourceVectorField();
    this->updateScalarField();
  }
  else if (update_type == 6)
  {
    this->updateDistSField();
    this->BuildClosestPtPair();
  }
}

void FeatureGuided::updateDistSField()
{
  this->target_scalar_field->dist_attenuation = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:rad");
  this->target_scalar_field->para_a = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  this->target_scalar_field->para_b = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  this->target_scalar_field->para_w = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:w");
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
  src_vid_mapper.clear();
  src_rev_vid_mapp.clear();
  std::vector<double2> curve;
  double distance = 0.0;
  src_sample_rate.clear();
  src_sample_rate.resize(crest_lines.size());
  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    curve.clear();
    double tmp_distance = 0.0;
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
      if(j != 0)
      {
        tmp_distance += sqrt(pow(winx - curve[j - 1].x, 2) + pow(source.rows - winy - curve[j - 1].y, 2));
      }
      src_vid_mapper[std::pair<int, int>(i, j)] = crest_lines[i][j];
      src_rev_vid_mapp[crest_lines[i][j]] = std::pair<int, int>(i, j);
    }
    tmp_distance /= (crest_lines[i].size() - 1);
    src_sample_rate[i] = tmp_distance * curve_scale;
    distance += tmp_distance;
    curves.push_back(curve);
  }
  distance /= crest_lines.size();
  average_sourcePts_interval = distance;
  sample_rate = 1 * average_sourcePts_interval * curve_scale;

  src_avg_direction.clear();
  CurvesUtility::CurvesAvgDir(curves, src_avg_direction, 1);
  // we also need to store the boundary curve
  //CURVES boundary_curve;
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
  for (size_t i = 0; i < bk_points.size(); ++i)
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
  CurvesUtility::CurveSpSaliency(target_edges_sp_sl, curves, target_edge_saliency, target_edges_average_sp_sl);

  AnalyzeTargetRelationship();

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

void FeatureGuided::AnalyzeTargetRelationship()
{
  // compute avg direction of each target curve
  tar_avg_direction.clear();
  CurvesUtility::CurvesAvgDir(target_curves, tar_avg_direction, 5);

  // build target curve groups
  // TODO: there might be better way to do this for example clustering
  double dir_th = 0.9; // cosine
  double end_th = 7; // pixel length
  double end_th_extra = 11;
  tar_relationship.clear();
  tar_relationship.resize(tar_avg_direction.size(), std::set<int>());
  for (size_t i = 0; i < tar_relationship.size(); ++i)
  {
    // put the i itself into the relationship set
    tar_relationship[i].insert(int(i));
    std::set<int> new_relationship;
    bool find_good;
    do 
    {
      new_relationship.clear();
      find_good = false;
      for (size_t j = 0; j < target_curves.size(); ++j)
      {
        if (tar_relationship[i].find(int(j)) != tar_relationship[i].end())
        {
          continue;
        }
        for (auto i_set : tar_relationship[i])
        {
          double dir_diff = fabs(tar_avg_direction[i_set].dot(tar_avg_direction[j]));
          double2 end_diff_vec = CurvesUtility::ClosestConnection(target_curves[i_set][0], target_curves[i_set].back(), target_curves[j][0], target_curves[j].back());
          double end_diff = end_diff_vec.norm();
          if (end_diff < end_th && dir_diff > dir_th)
          {
            // j is a good one, can be inserted into the relationship set
            new_relationship.insert(int(j));
            find_good = true;
            break;
          }
          //else if (end_diff < end_th_extra && dir_diff > dir_th)
          //{
          //  // check if the connection line is good when the distance is not that close
          //  if (fabs(tar_avg_direction[i_set].dot(Vector2f(end_diff_vec.x, end_diff_vec.y))) > dir_th)
          //  {
          //    // j is a good one, can be inserted into the relationship set
          //    new_relationship.insert(int(j));
          //    find_good = true;
          //    break;
          //  }
          //}
        }
      }
      tar_relationship[i].insert(new_relationship.begin(), new_relationship.end());
      for (auto i_set : new_relationship)
      {
        // i is also good for every curve in the new_relationship
        tar_relationship[i_set].insert(int(i));
      }
    } while (find_good);
  }
#define DEBUG
#ifdef DEBUG
  std::vector<std::vector<cv::Point>> contours;
  //cv::findContours(source, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // draw curves
  QDir dir;
  dir.mkpath(QString((source_model->getDataPath() + "/curve_imgs/").c_str()));

  for (int i = 0; i < target_curves.size(); ++i)
  {  cv::Mat contour = cv::Mat::zeros(target_img.rows, target_img.cols, CV_8UC1);
    for (auto j : tar_relationship[i])
    {
      for (int k = 0; k < target_curves[j].size(); ++k)
      {
        int imgx = std::max(double(0), std::min(target_curves[j][k].x, double(target_img.cols - 1)));
        int imgy = std::max(double(0), std::min(target_img.rows - 1 - target_curves[j][k].y, double(target_img.rows - 1)));
        contour.at<uchar>(imgy, imgx) = 255;
      }
    }
    cv::imwrite(source_model->getDataPath() + "/curve_imgs/relation_" + std::to_string(i) + ".png", contour);
  }

#endif
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

std::shared_ptr<KDTreeWrapper> FeatureGuided::getTargetKDTree()
{
  return this->target_KDTree;
}

void FeatureGuided::BuildEdgeKDTree(CURVES& curves, std::map<int, std::pair<int, int> >& id_mapper, std::shared_ptr<KDTreeWrapper> kdTree)
{
  std::vector<float> data;
  id_mapper.clear();
  size_t n_pts = 0;
  for (size_t i = 0; i < curves.size(); ++i)
  {
    for (size_t j = 0; j < curves[i].size(); ++j)
    {
      /*if(i == 0)
        std::cout << "x: " << curves[i][j].x << " , " << "y: " << curves[i][j].y << std::endl;*/
      data.push_back((float)curves[i][j].x);
      data.push_back((float)curves[i][j].y);
      id_mapper[n_pts] = std::pair<int, int>(i, j);
      ++ n_pts;
    }
  }
  kdTree->initKDTree(data, n_pts, 2);
}

void FeatureGuided::BuildSourceEdgeKDTree()
{
  BuildEdgeKDTree(source_curves, kdtree_id_mapper_source, source_KDTree);
}

void FeatureGuided::BuildTargetEdgeKDTree()
{
  BuildEdgeKDTree(target_curves, kdtree_id_mapper, target_KDTree);
}

std::map<int, std::pair<int, int> >& FeatureGuided::getSourceKDTreeMapper()
{
  return this->kdtree_id_mapper_source;
}

std::map<int, std::pair<int, int> >& FeatureGuided::getTargetKDTreeMapper()
{
  return this->kdtree_id_mapper;
}

CURVES& FeatureGuided::getSourceCurves()
{
  return this->source_curves;
}

CURVES& FeatureGuided::getTargetCurves()
{
  return this->target_curves;
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
  //user_correct_crsp_map.clear();
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

void FeatureGuided::BuildClosestPtPair(CURVES& curves, std::map<int, std::pair<Vector2f, Vector2f> >& data_crsp, bool update_saliency)
{
  std::map<CurvePt, CurvePt> crsp_map;
  std::map<CurvePt, CurvePt>::iterator it;
  std::shared_ptr<LargeFeatureCrsp> lf_crsp(new LargeFeatureCrsp(this));
  lf_crsp->buildCrsp(crsp_map, curves);

  for (auto i : crsp_map)
  {
    double2& temp = target_curves[i.second.first][i.second.second];
    Vector2f& temp_dir = tar_avg_direction[i.second.first]; // use average curve direction as current line direction
    //if (_isnan(temp_dir[0])) std::cout << "nan happen for target curve" << i.second.first << std::endl;
    data_crsp[src_vid_mapper[i.first]] = std::pair<Vector2f, Vector2f>(Vector2f(temp.x, temp.y), temp_dir); // for detected correspondence we minimize point to line distance
  }

  for (auto i : user_correct_crsp_map)
  {
    //int v_id = src_vid_mapper[i.first];
    int v_id = i.first;
    data_crsp[v_id] = std::pair<Vector2f, Vector2f>(Vector2f(i.second.x, i.second.y), Vector2f(0, 0)); // for user correspondence we minimize absolute distance
  }

  if (update_saliency)
  {
    this->updateSliencyFromCrsp(crsp_map);
  }
}

void FeatureGuided::updateSliencyFromCrsp(std::map<CurvePt, CurvePt>& crsp_map)
{
  std::vector<float> crsp_cnt(target_curves.size(), 0);
  float max_cnt = std::numeric_limits<float>::min();
  for (auto i : crsp_map)
  {
    crsp_cnt[i.second.first] += 1;
    if (crsp_cnt[i.second.first] > max_cnt)
    {
      max_cnt = crsp_cnt[i.second.first];
    }
  }

  // update saliency map
  for (size_t i = 0; i < crsp_cnt.size(); ++i)
  {
    if (crsp_cnt[i] > 0.5)
    {
      for (size_t j = 0; j < target_curves[i].size(); ++j)
      {
        int img_x = std::max(0, std::min(target_edge_saliency.cols - 1, int(target_curves[i][j].x + 0.5)));
        int img_y = std::max(0, std::min(target_edge_saliency.rows - 1, target_edge_saliency.rows - 1 - int(target_curves[i][j].y + 0.5)));
        float cur_sl = target_edge_saliency.at<float>(img_y, img_x);
        target_edge_saliency.at<float>(img_y, img_x) = std::min(1.0, cur_sl * (1 + (crsp_cnt[i] / max_cnt) * 0.1));
      }
    }
    else
    {
      for (size_t j = 0; j < target_curves[i].size(); ++j)
      {
        int img_x = std::max(0, std::min(target_edge_saliency.cols - 1, int(target_curves[i][j].x + 0.5)));
        int img_y = std::max(0, std::min(target_edge_saliency.rows - 1, target_edge_saliency.rows - 1 - int(target_curves[i][j].y + 0.5)));
        float cur_sl = target_edge_saliency.at<float>(img_y, img_x);
        target_edge_saliency.at<float>(img_y, img_x) = std::max(0.0, cur_sl * (1 - 0.1));
      }
    }
  }

  // recompute target saliency
  target_edges_sp_sl.clear();
  target_edges_average_sp_sl.clear();
  CurvesUtility::CurveSpSaliency(target_edges_sp_sl, target_curves, target_edge_saliency, target_edges_average_sp_sl);

  cv::imshow("saliency map", target_edge_saliency);
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

    user_correct_crsp_map[user_constrained_src_v] = user_constrained_tar_p;
  }
}

void FeatureGuided::GetCurrentCrspList(std::vector<std::pair<int, double2> >& crsp_list)
{
  //const std::vector<STLVectori>& crest_lines = source_model->getShapeVisbleCrestLine();

  if (src_crsp_list.size() != tar_crsp_list.size())
  {
    std::cerr << "Size of source correspondence list doesn't match that of target.\n";
    return;
  }

  std::map<int, double2>::iterator map_iter;
  std::map<int, double2> temp_crsp_map;
  for (size_t i = 0; i < src_crsp_list.size(); ++i)
  {
    int v_id = src_vid_mapper[src_crsp_list[i]];
    double2 tar_pt;
    
    map_iter = user_correct_crsp_map.find(src_vid_mapper[src_crsp_list[i]]);
    if (map_iter == user_correct_crsp_map.end())
    {
      tar_pt = target_curves[tar_crsp_list[i].first][tar_crsp_list[i].second];
      temp_crsp_map[v_id] = tar_pt;
    }
  }
  for (auto i : user_correct_crsp_map)
  {
    //int v_id = src_vid_mapper[i.first];
    int v_id = i.first;
    temp_crsp_map[v_id] = i.second;
  }
  for (auto i : temp_crsp_map)
  {
    crsp_list.push_back(std::pair<int ,double2>(i.first, i.second));
  }
}

void FeatureGuided::updateUserMarkedCurves()
{
  global_user_marked_crsp.clear();
  if(!user_marked_crsp.empty())
  {
    if(user_marked_crsp.size() % 2 != 0)
    {
      for(size_t i = 0; i < user_marked_crsp.size() - 1; i = i + 2)
      {
        global_user_marked_crsp.insert(std::pair<int, int>(user_marked_crsp[i], user_marked_crsp[i + 1]));
      }
    }
    else
    {
      for(size_t i = 0; i < user_marked_crsp.size(); i = i + 2)
      {
        global_user_marked_crsp.insert(std::pair<int, int>(user_marked_crsp[i], user_marked_crsp[i + 1]));
      }
    }
  }
}

void FeatureGuided::locateMarkedCurves()
{
  this->updateUserMarkedCurves();
  marked_source_curves.clear();
  marked_target_curves.clear();
  for(std::set<std::pair<int, int>>::iterator it = global_user_marked_crsp.begin(); it != global_user_marked_crsp.end(); it ++)
  {
    std::vector<int> tmp = source_model->getGlobalVisibleMapper()[(*it).first];
    for(size_t i = 0; i < tmp.size(); i ++)
    {
      marked_source_curves.push_back(tmp[i]);
    }
    marked_target_curves.push_back((*it).second);
  }
}

std::map<int, int>& FeatureGuided::getVisibleGlobalMapper()
{
  return source_model->getVisbleGlobalMapper();
}

std::map<int, std::vector<int>>& FeatureGuided::getGlobalVisibleMapper()
{
  return source_model->getGlobalVisibleMapper();
}

std::vector<double>& FeatureGuided::getEdgesAverageSpSl()
{
  return this->target_edges_average_sp_sl;
}

void FeatureGuided::deleteTargetCurves(std::vector<int>& deleted_tags)
{
  // delete all tags curves
  CURVES new_tar_curves;
  for (size_t i = 0; i < deleted_tags.size(); ++i)
  {
     if (deleted_tags[i] == 0)
     {
        new_tar_curves.push_back(target_curves[i]);
     }
  }

  target_curves.swap(new_tar_curves);

  target_edges_sp_sl.clear();
  target_edges_average_sp_sl.clear();
  CurvesUtility::CurveSpSaliency(target_edges_sp_sl, target_curves, target_edge_saliency, target_edges_average_sp_sl);

  target_edges_sp_len.clear();
  CurvesUtility::CurveSpTwoSideDist(target_edges_sp_len, target_curves);

  AnalyzeTargetRelationship();

  this->setNormalizePara();
  this->initTarRegister(1);
}

void FeatureGuided::addTargetCurves(std::vector<double2>& add_curve)
{
  if (add_curve.size() > 0)
  {
    target_curves.push_back(add_curve);

    // find the current largest saliency
    float max_saliency = *(std::max_element(target_edges_average_sp_sl.begin(), target_edges_average_sp_sl.end()));

    target_edges_sp_sl.push_back(std::vector<double>(add_curve.size(), max_saliency));
    target_edges_average_sp_sl.push_back(max_saliency);

    for (size_t i = 0; i < add_curve.size(); ++i)
    {
      int img_x = std::max(0, std::min(target_edge_saliency.cols - 1, int(add_curve[i].x + 0.5)));
      int img_y = std::max(0, std::min(target_edge_saliency.rows - 1, target_edge_saliency.rows - 1 - int(add_curve[i].y + 0.5)));
      target_edge_saliency.at<float>(img_y, img_x) = max_saliency;
    }

  }

  target_edges_sp_len.clear();
  CurvesUtility::CurveSpTwoSideDist(target_edges_sp_len, target_curves);

  AnalyzeTargetRelationship();

  this->initTarRegister(1);
}

void FeatureGuided::computeAverageTargetCurvesDir()
{
  CURVES tar_curves;
  this->NormalizedTargetCurves(tar_curves);
  sampled_target_curves_average_dir.clear();
  sampled_target_curves_average_dir.resize(tar_curves.size());
  for(size_t i = 0; i < tar_curves.size(); i ++)
  {
    sampled_target_curves_average_dir[i].resize(tar_curves[i].size());
    int start;
    start = 0;
    std::vector<int> segment;
    for(size_t j = 0; j < tar_curves[i].size(); j ++)
    {
      double2 diff;
      diff = tar_curves[i][j] - tar_curves[i][start];
      double distance = sqrt(diff.x * diff.x + diff.y * diff.y);
      if(distance < sample_rate && j != (tar_curves[i].size() - 1))
      {
        segment.push_back(j);
      }
      else
      {
        segment.push_back(j);
        Vector2f dir;
        dir << diff.x , diff.y;
        dir.normalize();
        for(size_t k = 0; k < segment.size(); k ++)
        {
          sampled_target_curves_average_dir[i][segment[k]] = dir;
        }
        start = j + 1;
        segment.clear();
      }
    }
  }
}
