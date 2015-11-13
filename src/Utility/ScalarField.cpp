#include "ScalarField.h"
#include "FeatureGuided.h"
#include "KDTreeWrapper.h"
#include "VectorVariation.h"
#include "tele2d.h"
#include "Model.h"

#include "ParameterMgr.h"

#include <cv.h>
#include <highgui.h>

double sigmoid(double x, double center, double k)
{
  return 1 - (1 / (1 + k * exp(center - x)));
}

ScalarField::ScalarField(int res, float rad)
{
  this->init(res);
  this->setSearchRad(rad);
  this->initPara();
}

ScalarField::~ScalarField()
{
}

void ScalarField::initPara()
{
  dist_attenuation = LG::GlobalParameterMgr::GetInstance()->get_parameter<float>("SField:DistAttenuation");
  para_a = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  para_b = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");

  win_width = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinWidth");
  win_center = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinCenter");
}

void ScalarField::setTeleRegister(std::shared_ptr<tele2d> tele)
{
  tele_register = tele;
}

void ScalarField::init(int resolution)
{
  this->resolution = resolution;

  distance_map.resize(resolution * resolution, 0);
  variation_map.resize(resolution * resolution, 0);
  matching_map.resize(resolution * resolution, 0);
}

void ScalarField::setSearchRad(float radius)
{
  this->search_rad = radius;
}

void ScalarField::computeVariationMap()
{
  std::vector<double2>& vector_field = tele_register->vector_field;
  float max_var = std::numeric_limits<float>::min();
  float min_var = std::numeric_limits<float>::max();
  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {

      std::vector<double2> neighbour_area;
      double2 center = vector_field[i + j * resolution];
      for (int i_step = i - search_rad; i_step <= i + search_rad; ++i_step)
      {
        for (int j_step = j - search_rad; j_step <= j + search_rad; ++j_step)
        {
          if (i_step >= 0 && i_step < resolution
           && j_step >= 0 && j_step < resolution
           && i_step != i && j_step != j)
          {
            neighbour_area.push_back(vector_field[i_step + j_step * resolution]);
          }
        }
      }
      variation_map[i + j * resolution] = (float)computeVariation(neighbour_area, center);
      if (variation_map[i + j * resolution] > max_var)
      {
        max_var = variation_map[i + j * resolution];
      }
      if (variation_map[i + j * resolution] < min_var)
      {
        min_var = variation_map[i + j * resolution];
      }
    }
  }

  // normalize the variation map
  for (size_t i = 0; i < variation_map.size(); ++i)
  {
    variation_map[i] = (variation_map[i] - min_var) / (max_var - min_var);
  }

  //cv::imshow("variation map", cv::Mat(resolution, resolution, CV_32FC1, &variation_map[0]));
}

void ScalarField::computeMatchingMap(std::vector<double2>& ext_vector_field)
{
  std::vector<double2>& vector_field = tele_register->vector_field;
  float max_var = std::numeric_limits<float>::min();
  float min_var = std::numeric_limits<float>::max();

  if (vector_field.size() != ext_vector_field.size())
  {
    std::cerr << "Dimension of vector field doesn't match.\n";
    return;
  }

  for (size_t i = 0; i < vector_field.size(); ++i)
  {
    double cur_cos = vector_field[i].x * ext_vector_field[i].x + vector_field[i].y * ext_vector_field[i].y;
    cur_cos = fabs(cur_cos);
    matching_map[i] = (float)cur_cos;
    if (cur_cos > max_var)
    {
      max_var = cur_cos;
    }
    if (cur_cos < min_var)
    {
      min_var = cur_cos;
    }
  }

  // normalize the variation map
  for (size_t i = 0; i < matching_map.size(); ++i)
  {
    matching_map[i] = 1 - (matching_map[i] - min_var) / (max_var - min_var);
  }
}

void ScalarField::computeDistanceMap(FeatureGuided* feature_model)
{
  double scale = feature_model->curve_scale;
  double2 curve_translate = feature_model->curve_translate;
  float max_val = std::numeric_limits<float>::min();
  float min_val = std::numeric_limits<float>::max();


  double avg_edge_len = 0;
  for (size_t i = 0; i < feature_model->target_edges_sp_len.size(); ++i)
  {
    avg_edge_len += feature_model->target_edges_sp_len[i][0].x + feature_model->target_edges_sp_len[i][0].y;
  }
  avg_edge_len /= feature_model->target_edges_sp_len.size();
  double sigmoid_center = std::max(feature_model->target_edge_saliency.cols, feature_model->target_edge_saliency.rows);
  search_rad = dist_attenuation * 0.6 / scale;

  std::cout << "dist attenuation: " << dist_attenuation << "\tsearch radius: " << search_rad << "\n";

  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {
      std::vector<float> pos(2, 0.0);
      pos[0] = float(j) / resolution;
      pos[1] = float(i) / resolution;
      pos[0] = (pos[0] - 0.5) / scale + 0.5 - curve_translate.x;
      pos[1] = (pos[1] - 0.5) / scale + 0.5 - curve_translate.y;

      //float cur_dist = 0;
      //for (size_t k = 0; k < feature_model->target_curves.size(); ++k)
      //{
      //  for (size_t kk = 0; kk < feature_model->target_curves[k].size(); ++kk)
      //  {
      //      int img_i = feature_model->target_edge_saliency.rows - (feature_model->target_curves[k][kk].y + 0.5);
      //      int img_j = feature_model->target_curves[k][kk].x + 0.5;
      //      img_i = img_i < 0 ? 0 : (img_i < feature_model->target_edge_saliency.rows ? img_i : feature_model->target_edge_saliency.rows);
      //      img_j = img_j < 0 ? 0 : (img_j < feature_model->target_edge_saliency.cols ? img_j : feature_model->target_edge_saliency.cols);
      //      float saliency = feature_model->target_edge_saliency.at<float>(img_i, img_j);
      //      //std::pair<int, int> curve_id = feature_model->kdtree_id_mapper[nearest_sp_id[k]];
      //      double2 edge_lens = feature_model->target_edges_sp_len[k][kk];
      //      double new_edge_len = 1 - sigmoid(edge_lens.x + edge_lens.y, avg_edge_len, 1);
      //      edge_lens = edge_lens / (edge_lens.x + edge_lens.y) * new_edge_len;

      //      // saliency * sigmoid(d)
      //      double e_dist = sqrt(pow(feature_model->target_curves[k][kk].x - pos[0], 2) + pow(feature_model->target_curves[k][kk].y - pos[1], 2));
      //      cur_dist +=  edge_lens.x * edge_lens.y * saliency / e_dist; //* sigmoid(e_dist, sigmoid_center / 2, dist_attenuation);
      //  }
      //}

      // here the radius for rNearestPt is r^2 and returned dist is also square distance
      std::vector<float> nearest_sp;
      std::vector<float> nearest_sp_dist;
      std::vector<int>   nearest_sp_id;
      feature_model->target_KDTree->rNearestPt(search_rad * search_rad, pos, nearest_sp, nearest_sp_dist, nearest_sp_id);
      float cur_dist = std::numeric_limits<float>::min();
      for (size_t k = 0; k < nearest_sp_id.size(); ++k)
      {
        //int img_i = feature_model->target_edge_saliency.rows - (nearest_sp[2 * k + 1] + 0.5);
        //int img_j = nearest_sp[2 * k + 0];
        //img_i = img_i < 0 ? 0 : (img_i < feature_model->target_edge_saliency.rows ? img_i : feature_model->target_edge_saliency.rows);
        //img_j = img_j < 0 ? 0 : (img_j < feature_model->target_edge_saliency.cols ? img_j : feature_model->target_edge_saliency.cols);
        //float saliency = feature_model->target_edge_saliency.at<float>(img_i, img_j);
        std::pair<int, int> curve_id = feature_model->kdtree_id_mapper[nearest_sp_id[k]];
        //double2 edge_lens = feature_model->target_edges_sp_len[curve_id.first][curve_id.second];
        //double new_edge_len = 1 - sigmoid(edge_lens.x + edge_lens.y, avg_edge_len, 1);
        //edge_lens = edge_lens / (edge_lens.x + edge_lens.y) * new_edge_len;
        double saliency = feature_model->target_edges_sp_sl[curve_id.first][curve_id.second];

        // saliency * sigmoid(d)
        //cur_dist +=  edge_lens.x * edge_lens.y * saliency * sigmoid(sqrt(nearest_sp_dist[k]), search_rad / 2, dist_attenuation);// *
        //cur_dist += saliency * exp(-nearest_sp_dist[k] / search_rad / search_rad);
        double score = sqrt(nearest_sp_dist[k]);//pow(saliency, para_a) / pow((sqrt(nearest_sp_dist[k]) / search_rad + 0.0001), para_b);
        if (cur_dist < score)
        {
          cur_dist = score;
        }
      }
      if (nearest_sp_id.size() == 0)
      {
        distance_map[i * resolution + j] = -1;
      }
      else
      {
        if (cur_dist > max_val)
        {
          max_val = cur_dist;
        }
        if (cur_dist < min_val)
        {
          min_val = cur_dist;
        }
        distance_map[i * resolution + j] = cur_dist;
      }
      //int img_i = feature_model->target_edge_saliency.rows - pos[1] - 0.5;
      //int img_j = pos[0];
      //if (img_i < feature_model->target_edge_saliency.rows && img_i >= 0
      // && img_j < feature_model->target_edge_saliency.cols && img_j >= 0)
      //{
      //  distance_map[i * resolution + j] = feature_model->target_edge_saliency.at<float>(img_i, img_j);
      //}
    }
  }

  std::ofstream f_debug(feature_model->source_model->getDataPath() + "/SField.mat");

  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {
      if (distance_map[i * resolution + j] < 0)
      {
        distance_map[i * resolution + j] = 0.0;
      }
      else
      {
        distance_map[i * resolution + j] = (distance_map[i * resolution + j] - min_val) / (max_val - min_val);// = log(1 + distance_map[i * resolution + j]);// /= max_val;
        if (f_debug)
        {
          f_debug << distance_map[i * resolution + j] << "\n";
        }
      }
    }
  }

  f_debug.close();

  std::cout << "max val: " << max_val << "\tmin val: " << min_val << "\n";

  cv::Mat temp_img(resolution, resolution, CV_32FC1, &distance_map[0]);
  cv::Mat tempp_img;
  cv::flip(temp_img, tempp_img, 0);
  //cv::imshow("distance_map map", tempp_img);
}

double ScalarField::curveIntegrate(std::vector<std::vector<double2> >& curves, FeatureGuided* feature_model)
{
  double scale = feature_model->curve_scale;
  double2 curve_translate = feature_model->curve_translate;

  double integ = 0.0;
  for (size_t i = 0; i < curves.size(); ++i)
  {
    for (size_t j = 0; j < curves[i].size(); ++j)
    {
      double2 pos = (curves[i][j] + curve_translate - double2(0.5, 0.5)) * scale + double2(0.5, 0.5);
      int field_j = int(pos.x * resolution);
      int field_i = int(pos.y * resolution);
      field_j = (field_j < 0) ? 0 : ((field_j >= resolution) ? (resolution - 1) : field_j);
      field_i = (field_i < 0) ? 0 : ((field_i >= resolution) ? (resolution - 1) : field_i);
      integ += distance_map[field_i * resolution + field_j];
    }
  }
  //std::cout << "curve integrate: " << integ << std::endl;
  return integ;
}