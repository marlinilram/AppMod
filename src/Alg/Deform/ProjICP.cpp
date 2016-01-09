#include "ProjICP.h"
#include "FeatureGuided.h"
#include "Model.h"
#include "WunderSVD3x3.h"

ProjICP::ProjICP()
{

}

ProjICP::~ProjICP()
{

}

void ProjICP::buildCrsp(std::shared_ptr<FeatureGuided> feature_model)
{
  // test a different way to compute crsp
  this->testCrsp(feature_model);
  return;

  CURVES& src_curves = feature_model->source_curves;
  CURVES& tar_curves = feature_model->target_curves;

  double avg_edge_len = 0;
  for (size_t i = 0; i < feature_model->target_edges_sp_len.size(); ++i)
  {
    avg_edge_len += feature_model->target_edges_sp_len[i][0].x + feature_model->target_edges_sp_len[i][0].y;
  }
  avg_edge_len /= feature_model->target_edges_sp_len.size();

  // find 2D correspondences by tuned distance function
  typedef std::pair<int, int> CURVEIDX;
  std::map<CURVEIDX, CURVEIDX> crsp_map;
  for (size_t i = 0; i < src_curves.size(); ++i)
  {
    for (size_t j = 0; j < src_curves[i].size(); ++j)
    {
      double2 pos = src_curves[i][j];
      CURVEIDX curve_idx;

      double max_dist = std::numeric_limits<double>::min();
      for (size_t k = 0; k < feature_model->target_curves.size(); ++k)
      {
        for (size_t kk = 0; kk < feature_model->target_curves[k].size(); ++kk)
        {
            int img_i = feature_model->target_edge_saliency.rows - 1 - int(feature_model->target_curves[k][kk].y + 0.5);
            int img_j = feature_model->target_curves[k][kk].x + 0.5;
            img_i = img_i < 0 ? 0 : (img_i < feature_model->target_edge_saliency.rows ? img_i : feature_model->target_edge_saliency.rows);
            img_j = img_j < 0 ? 0 : (img_j < feature_model->target_edge_saliency.cols ? img_j : feature_model->target_edge_saliency.cols);
            float saliency = feature_model->target_edge_saliency.at<float>(img_i, img_j);
            //std::pair<int, int> curve_id = feature_model->kdtree_id_mapper[nearest_sp_id[k]];
            double2 edge_lens = feature_model->target_edges_sp_len[k][kk];
            double new_edge_len = (1 / (1 + 1 * exp(avg_edge_len - edge_lens.x - edge_lens.y)));
            edge_lens = edge_lens / (edge_lens.x + edge_lens.y) * new_edge_len;

            // saliency * sigmoid(d)
            double e_dist = sqrt(pow(feature_model->target_curves[k][kk].x - pos.x, 2) + pow(feature_model->target_curves[k][kk].y - pos.y, 2));
            double cur_dist = edge_lens.x * edge_lens.y * saliency / e_dist; //* sigmoid(e_dist, sigmoid_center / 2, dist_attenuation);
            if (cur_dist > max_dist)
            {
              max_dist = cur_dist;
              curve_idx.first = k;
              curve_idx.second = kk;
            }
        }
      }

      crsp_map[CURVEIDX(i, j)] = curve_idx;
    }
  }

  // test the crsp here
  std::vector<std::pair<int, int> >& src_crsp_list = feature_model->src_crsp_list;
  std::vector<std::pair<int, int> >& tar_crsp_list = feature_model->tar_crsp_list;
  src_crsp_list.clear();
  tar_crsp_list.clear();
  for (std::map<CURVEIDX, CURVEIDX>::iterator it = crsp_map.begin(); it != crsp_map.end(); ++it)
  {
    src_crsp_list.push_back(it->first);
    tar_crsp_list.push_back(it->second);
  }


  // from 3D vertex to its closest point in the ray
  Matrix3Xf icp_source(3, crsp_map.size());
  Matrix3Xf icp_target(3, crsp_map.size());
  size_t point_cnt = 0;
  // visible_crest_line should have same size of source curve
  const std::vector<STLVectori>& visible_crest_line = feature_model->source_model->getShapeVisbleCrestLine();
  Vector3f camera_ori;
  feature_model->source_model->getCameraOri(camera_ori.data());
  for (std::map<CURVEIDX, CURVEIDX>::iterator it = crsp_map.begin(); it != crsp_map.end(); ++it)
  {
    // compute the corresponding projection ray from corresponding curve sample point from image
    CURVEIDX curve_idx = it->second;
    int img_i = feature_model->target_edge_saliency.rows - 1 - int(feature_model->target_curves[curve_idx.first][curve_idx.second].y + 0.5);
    int img_j = feature_model->target_curves[curve_idx.first][curve_idx.second].x + 0.5;
    img_i = img_i < 0 ? 0 : (img_i < feature_model->target_edge_saliency.rows ? img_i : (feature_model->target_edge_saliency.rows - 1));
    img_j = img_j < 0 ? 0 : (img_j < feature_model->target_edge_saliency.cols ? img_j : (feature_model->target_edge_saliency.cols - 1));
    Vector3f proj_ray; // it's a direction
    feature_model->source_model->getProjRay(proj_ray.data(), img_j, img_i);

    // compute the closest point in the projection ray
    Vector3f shape_vertex;
    shape_vertex[0] = feature_model->source_model->getShapeVertexList()[3 * visible_crest_line[it->first.first][it->first.second] + 0];
    shape_vertex[1] = feature_model->source_model->getShapeVertexList()[3 * visible_crest_line[it->first.first][it->first.second] + 1];
    shape_vertex[2] = feature_model->source_model->getShapeVertexList()[3 * visible_crest_line[it->first.first][it->first.second] + 2];
    Vector3f crsp_pt = camera_ori + (proj_ray.dot(shape_vertex - camera_ori)) * proj_ray;

    // store closest point and the shape_vertex
    icp_source.col(point_cnt) = shape_vertex;
    icp_target.col(point_cnt) = crsp_pt;
    ++point_cnt;
  }

  // compute optimized Rotation, Scale and Translation
  Vector3f mu_t = icp_source * VectorXf::Ones(crsp_map.size()) / crsp_map.size();
  Vector3f mu_m = icp_target * VectorXf::Ones(crsp_map.size()) / crsp_map.size();
  MatrixXf q_t = ((MatrixXf::Identity(crsp_map.size(), crsp_map.size()) - MatrixXf::Ones(crsp_map.size(), crsp_map.size()) / crsp_map.size()) * icp_source.transpose());
  MatrixXf q_m = ((MatrixXf::Identity(crsp_map.size(), crsp_map.size()) - MatrixXf::Ones(crsp_map.size(), crsp_map.size()) / crsp_map.size()) * icp_target.transpose());
  Matrix3f H = q_t.transpose() * q_m;
  Matrix3f U;
  Vector3f W;
  Matrix3f V;
  wunderSVD3x3(H, U, W, V);
  Matrix3f R = V * U.transpose();
  Matrix3Xf sp = R * q_t.transpose();
  float Sp = 0;
  float D = 0;
  for (size_t i = 0; i < crsp_map.size(); ++i)
  {
    Sp += sp.col(i).squaredNorm();
    D += sp.col(i).dot(q_m.row(i));
  }
  float scale = D / Sp;

  R = scale * R;
  Vector3f t = mu_m - R * mu_t;

  VertexList new_v_list = feature_model->source_model->getShapeVertexList();
  Eigen::Map<Matrix3Xf>new_v_mat(&new_v_list[0], 3, new_v_list.size() / 3);
  for (size_t i = 0; i < new_v_list.size() / 3; ++i)
  {
    Vector3f new_pos = R * new_v_mat.col(i) + t;
    new_v_list[3 * i + 0] = new_pos[0];
    new_v_list[3 * i + 1] = new_pos[1];
    new_v_list[3 * i + 2] = new_pos[2];
  }
  feature_model->source_model->updateShape(new_v_list);
}

void ProjICP::testCrsp(std::shared_ptr<FeatureGuided> feature_model)
{
  CURVES& src_curves = feature_model->source_curves;
  CURVES& tar_curves = feature_model->target_curves;

  double avg_edge_len = 0;
  for (size_t i = 0; i < feature_model->target_edges_sp_len.size(); ++i)
  {
    avg_edge_len += feature_model->target_edges_sp_len[i][0].x + feature_model->target_edges_sp_len[i][0].y;
  }
  avg_edge_len /= feature_model->target_edges_sp_len.size();

  // 1. a vector for source point pool and a vector for target point pool
  typedef std::pair<int, int> CURVEIDX;
  std::vector<CURVEIDX> src_pool;
  std::vector<CURVEIDX> tar_pool;
  for (size_t i = 0; i < src_curves.size(); ++i)
  {
    for (size_t j = 0; j < src_curves[i].size(); ++j)
    {
      src_pool.push_back(CURVEIDX(i, j));
    }
  }
  for (size_t i = 0; i < tar_curves.size(); ++i)
  {
    for (size_t j = 0; j < tar_curves[i].size(); ++j)
    {
      tar_pool.push_back(CURVEIDX(i, j));
    }
  }

  typedef std::pair<CURVEIDX, double> CURVEIDXSCORE;
  std::map<CURVEIDX, CURVEIDX> crsp_map;
  std::map<CURVEIDX, CURVEIDXSCORE> crsp_score_map;
  std::map<CURVEIDX, CURVEIDXSCORE>::iterator crsp_score_it;
  std::vector<CURVEIDX> left_src_pool;

  do
  {
    left_src_pool.clear();
    while (!src_pool.empty())
    {
      CURVEIDX src_idx = src_pool.back();
      double2 pos = src_curves[src_idx.first][src_idx.second];
      CURVEIDX tar_idx(-1, -1);

      double max_dist = std::numeric_limits<double>::min();
      for (size_t i = 0; i < tar_pool.size(); ++i)
      {
        size_t k = tar_pool[i].first;
        size_t kk = tar_pool[i].second;

        int img_i = feature_model->target_edge_saliency.rows - 1 - int(feature_model->target_curves[k][kk].y + 0.5);
        int img_j = feature_model->target_curves[k][kk].x + 0.5;
        img_i = img_i < 0 ? 0 : (img_i < feature_model->target_edge_saliency.rows ? img_i : feature_model->target_edge_saliency.rows);
        img_j = img_j < 0 ? 0 : (img_j < feature_model->target_edge_saliency.cols ? img_j : feature_model->target_edge_saliency.cols);
        float saliency = feature_model->target_edge_saliency.at<float>(img_i, img_j);
        //std::pair<int, int> curve_id = feature_model->kdtree_id_mapper[nearest_sp_id[k]];
        double2 edge_lens = feature_model->target_edges_sp_len[k][kk];
        double new_edge_len = (1 / (1 + 1 * exp(avg_edge_len - edge_lens.x - edge_lens.y)));
        edge_lens = edge_lens / (edge_lens.x + edge_lens.y) * new_edge_len;

        // saliency * sigmoid(d)
        double e_dist = sqrt(pow(feature_model->target_curves[k][kk].x - pos.x, 2) + pow(feature_model->target_curves[k][kk].y - pos.y, 2));
        double cur_dist = edge_lens.x * edge_lens.y * saliency / e_dist; //* sigmoid(e_dist, sigmoid_center / 2, dist_attenuation);
        if (cur_dist > max_dist && e_dist < 25)
        {
          max_dist = cur_dist;
          tar_idx.first = k;
          tar_idx.second = kk;
        }
      }

      // if the target idx isn't any correspondence to a source idx
      // just store it
      crsp_score_it = crsp_score_map.find(tar_idx);
      if (crsp_score_it == crsp_score_map.end())
      {
        if (tar_idx.first != -1 && tar_idx.second != -1)
        {
          crsp_score_map[tar_idx] = CURVEIDXSCORE(src_idx, max_dist);
          crsp_map[src_idx] = tar_idx;
        }
        else
        {
          // if tar_idx is invalid, the source idx can not find
          // a suitable target idx
          // we don't add into crsp_map and also not add into left_src_pool
        }
      }
      else
      {
        // if the target idx is already corresponding to another source idx
        // if the new source idx is better than old
        if (crsp_score_it->second.second < max_dist)
        {
          // delete the old from crsp_map
          CURVEIDX old = crsp_score_it->second.first;
          crsp_map.erase(crsp_map.find(old));
          left_src_pool.push_back(old);
          // update the new source idx
          crsp_score_it->second = CURVEIDXSCORE(src_idx, max_dist);
          crsp_map[src_idx] = tar_idx;
          // pop back the current source idx from src_pool
        }
        else // if the old is better than new
        {
          left_src_pool.push_back(src_idx);
        }
      }
      src_pool.pop_back();
    }

    // now the crsp_map store the best correspondences
    // but we need to find best for the left_src_pool
    // update src_pool and tar_pool
    src_pool = left_src_pool;
    for (auto i : crsp_score_map)
    {
      tar_pool.erase(std::find(tar_pool.begin(), tar_pool.end(), i.first));
    }
    crsp_score_map.clear();
  } while (!left_src_pool.empty());

  std::vector<std::pair<int, int> >& src_crsp_list = feature_model->src_crsp_list;
  std::vector<std::pair<int, int> >& tar_crsp_list = feature_model->tar_crsp_list;
  src_crsp_list.clear();
  tar_crsp_list.clear();
  for (std::map<CURVEIDX, CURVEIDX>::iterator it = crsp_map.begin(); it != crsp_map.end(); ++it)
  {
    src_crsp_list.push_back(it->first);
    tar_crsp_list.push_back(it->second);
  }
}