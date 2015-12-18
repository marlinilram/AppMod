#include "LargeFeatureCrsp.h"
#include "FeatureGuided.h"

#include "CurvesUtility.h"
#include "ParameterMgr.h"

void LargeFeatureCrsp::buildCrsp(std::map<CurvePt, CurvePt>& crsp)
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

  CURVES n_src_curves; // get normalized source curves
  CURVES n_tar_curves; // get normalized target curves
  feature_model->NormalizedSourceCurves(n_src_curves);
  feature_model->NormalizedTargetCurves(n_tar_curves);
  std::map<CurvePt, CrspCurvePt> crsp_map_out;
  std::map<CurvePt, CrspCurvePt>::iterator it;
  std::deque<bool> tar_curve_mark(n_tar_curves.size(), true);
  for (int i = 0; i < 1; ++i)
  {
    refineCrsp(crsp_map_out, tar_curve_mark, n_src_curves, n_tar_curves);
  }

  for (it = crsp_map_out.begin(); it != crsp_map_out.end(); ++it)
  {
    crsp[it->first] = it->second.first;
  }
}

void LargeFeatureCrsp::buildCrsp(std::map<CurvePt, CurvePt>& crsp, CURVES& curves_in)
{
  CURVES n_src_curves = curves_in; // get normalized source curves
  CURVES n_tar_curves; // get normalized target curves
  CurvesUtility::NormalizedCurves(n_src_curves, feature_model->curve_translate, feature_model->curve_scale);
  feature_model->NormalizedTargetCurves(n_tar_curves);

  std::map<CurvePt, CrspCurvePt> crsp_map_out;
  std::map<CurvePt, CrspCurvePt>::iterator it;
  std::deque<bool> tar_curve_mark(n_tar_curves.size(), true);
  for (int i = 0; i < 1; ++i)
  {
    refineCrsp(crsp_map_out, tar_curve_mark, n_src_curves, n_tar_curves);
  }

  for (it = crsp_map_out.begin(); it != crsp_map_out.end(); ++it)
  {
    crsp[it->first] = it->second.first;
  }
}

/*
void LargeFeatureCrsp::refineCrsp(std::map<CurvePt, CrspCurvePt>& crsp_map_out, std::deque<bool>& tar_curve_mark, CURVES& n_src_curves, CURVES& n_tar_curves)
{
  // find corresponding points in source curves
  // guarantee that the found correspondences are two-sided closest
  // get the visible crest line (source curve) to global crest line mapper, for history user interaction information
  std::map<int, int>& vis_global_mapper = feature_model->getVisibleGlobalMapper(); 
  std::vector<double> paras(3, 0);
  paras[1] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  paras[2] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  std::map<CurvePt, CrspCurvePt> crsp_map; // key is source curve point as pair<int, int>, value is target curve point and their score
  std::map<CurvePt, CrspCurvePt>::iterator crsp_map_it;
  std::map<CurvePt, CurvePt> crsp;
  for (size_t i = 0; i < n_tar_curves.size(); ++i)
  {
    //if (tar_curve_mark[i])
    {
      for (size_t j = 0; j < n_tar_curves[i].size(); ++j)
      {
        int src_i = -1;
        int src_j = -1;
        double score = 0.0;
        paras[0] = feature_model->target_edges_sp_sl[i][j];
        //if (CurvesUtility::closestPtInCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, target_scalar_field->matching_map, target_scalar_field->resolution, 0.3))
        if (CurvesUtility::closestPtFromSaliencyCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, score, paras))
        {
          //if (feature_model->global_user_marked_crsp.find(std::pair<int, int>(vis_global_mapper[src_i], i)) == feature_model->global_user_marked_crsp.end()) continue;
          //if (crsp_map_out.find(CurvePt(src_i, src_j)) != crsp_map_out.end()) continue;

          score *= fabs(feature_model->src_avg_direction[src_i].dot(feature_model->tar_avg_direction[i])) ;

          crsp_map_it = crsp_map.find(CurvePt(src_i, src_j));
          if (crsp_map_it != crsp_map.end())
          {
            if (score > crsp_map_it->second.second)
            {
              crsp_map_it->second = CrspCurvePt(CurvePt(int(i), int(j)), score);
              crsp[CurvePt(src_i, src_j)] = CurvePt(int(i), int(j));
            }
          }
          else
          {
            crsp_map[CurvePt(src_i, src_j)] = CrspCurvePt(CurvePt(int(i), int(j)), score);
            crsp[CurvePt(src_i, src_j)] = CurvePt(int(i), int(j));
          }
        }
      }
    }
  }
  //for (size_t i = 0; i < n_src_curves.size(); ++i)
  //{
  //  //if (tar_curve_mark[i])
  //  {
  //    for (size_t j = 0; j < n_src_curves[i].size(); ++j)
  //    {
  //      int tar_i = -1;
  //      int tar_j = -1;
  //      double score = 0.0;
  //      //paras[0] = feature_model->target_edges_sp_sl[i][j];
  //      //if (CurvesUtility::closestPtInCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, target_scalar_field->matching_map, target_scalar_field->resolution, 0.3))
  //      if (CurvesUtility::closestPtInSaliencyCurves(n_src_curves[i][j], n_tar_curves, feature_model->target_edges_sp_sl, tar_i, tar_j, score, paras))
  //      {
  //        //if (fabs(feature_model->src_avg_direction[i].dot(feature_model->tar_avg_direction[tar_i])) < 0.9 
  //        //  && feature_model->global_user_marked_crsp.find(std::pair<int, int>(vis_global_mapper[i], tar_i)) == feature_model->global_user_marked_crsp.end()) continue;
  //        //if (crsp_map_out.find(CurvePt(src_i, src_j)) != crsp_map_out.end()) continue;

  //        crsp_map_it = crsp_map.find(CurvePt(i, j));
  //        if (crsp_map_it != crsp_map.end())
  //        {
  //          if (score > crsp_map_it->second.second)
  //          {
  //            crsp_map_it->second = CrspCurvePt(CurvePt(int(tar_i), int(tar_j)), score);
  //            crsp[CurvePt(i, j)] = CurvePt(int(tar_i), int(tar_j));
  //          }
  //        }
  //        else
  //        {
  //          crsp_map[CurvePt(i, j)] = CrspCurvePt(CurvePt(int(tar_i), int(tar_j)), score);
  //          crsp[CurvePt(i, j)] = CurvePt(int(tar_i), int(tar_j));
  //        }
  //      }
  //    }
  //  }
  //}
  //std::map<CurvePt, CrspCurvePt> t_s_crsp_map;
  //std::map<CurvePt, CrspCurvePt>::iterator it;
  //for(auto i : crsp_map)
  //{
  //  it = t_s_crsp_map.find(i.second.first);
  //  if(it != t_s_crsp_map.end())
  //  {
  //    if(i.second.second > it->second.second)
  //    {
  //      it->second = CrspCurvePt(i.first, i.second.second);
  //    }
  //  }
  //  else
  //  {
  //    t_s_crsp_map[i.second.first] = CrspCurvePt(i.first, i.second.second);
  //  }
  //}
  //crsp_map.clear();
  //for(auto i : t_s_crsp_map)
  //{
  //  crsp_map[i.second.first] = CrspCurvePt(i.first, i.second.second);
  //}
  
  //crsp_map_out = crsp_map;return;
  // refine the correspondences
  // 1. keep the most confidential target curve

  std::map<int, std::map<int, int > > s_t_cnt_map;
  std::map<int, std::map<int, int > >::iterator cnt_map_it;
  std::map<int, int>::iterator cnt_it;

  for (auto i : crsp_map)
  {
    // i.first.first is source curve id
    // i.second.first.first is target curve id
    cnt_map_it = s_t_cnt_map.find(i.first.first);
    if (cnt_map_it != s_t_cnt_map.end())
    {
      cnt_it = cnt_map_it->second.find(i.second.first.first);
      if (cnt_it != cnt_map_it->second.end())
      {
        cnt_it->second += 1;
      }
      else
      {
        cnt_map_it->second[i.second.first.first] = 1;
      }
    }
    else
    {
      s_t_cnt_map[i.first.first] = std::map<int, int>();
      s_t_cnt_map[i.first.first][i.second.first.first] = 1;
    }
  }

  //for (auto i : crsp)
  //{
  //  cnt_map_it = s_t_cnt_map.find(i.first.first);
  //  if (cnt_map_it != s_t_cnt_map.end())
  //  {
  //    cnt_it = cnt_map_it->second.find(i.second.first);
  //    if (cnt_it != cnt_map_it->second.end())
  //    {
  //      cnt_it->second.push_back(CurvePt(i.first.second, i.second.second));
  //    }
  //    else
  //    {
  //      cnt_map_it->second[i.second.first] = std::vector<CurvePt>(1, CurvePt(i.first.second, i.second.second));
  //    }
  //  }
  //  else
  //  {
  //    s_t_cnt_map[i.first.first] = std::map<int, std::vector<CurvePt> >();
  //    s_t_cnt_map[i.first.first][i.second.first] = std::vector<CurvePt>(1, CurvePt(i.first.second, i.second.second));
  //  }
  //}

  // cache best source curve to the target curve
  std::map<int, std::pair<int, int> > t_s_cnt_map;
  std::map<int, std::pair<int, int> >::iterator t_s_cnt_it;
  for (auto i : s_t_cnt_map)
  {
    // source curve id = i.first
    for (auto j : i.second)
    {
      // target curve id = j.first
      // corresponding curve point std::vector<int, int>& = j.second
      // a corresponding pair is (i.first, j.second[k].first) and (j.first, j.second[k].second)

      int cnt = j.second;
      t_s_cnt_it = t_s_cnt_map.find(j.first);
      if (t_s_cnt_it != t_s_cnt_map.end())
      {
        if (cnt > t_s_cnt_it->second.second)
        {
          t_s_cnt_it->second = std::pair<int, int>(i.first, cnt);
        }
      }
      else
      {
        t_s_cnt_map[j.first] = std::pair<int, int>(i.first, cnt);
      }
    }
  }

  // save result
  //typedef std::set<std::pair<int, int> > CrspCurvePtSet;
  //std::map<int, std::pair<int, CrspCurvePtSet > > src_core_crsp; // key is the source curve id, first of pair is target curve id, each pair in set is sample id in source curve and target curve
  //int cnt = 0;
  //crsp.clear();
  //for (auto i : s_t_cnt_map)
  //{
  //  // find the best target
  //  int best_target = -1;
  //  int best_target_cnt = std::numeric_limits<int>::min();
  //  for (auto j : i.second)
  //  {
  //    cnt = j.second.size();
  //    if (cnt > best_target_cnt)
  //    {
  //      best_target_cnt = j.second.size();
  //      best_target = j.first;
  //    }
  //  }

  //  // save the best crsp
  //  // if the best target curve also has its best source curve than we confirm it
  //  t_s_cnt_it = t_s_cnt_map.find(best_target);
  //  if (t_s_cnt_it != t_s_cnt_map.end())
  //  {
  //    if (i.first == t_s_cnt_it->second.first)
  //    {
  //      //std::cout << "best curve correspondence has " << i.second[best_target].size() << " pairs." << std::endl;
  //      src_core_crsp[i.first] = std::pair<int, CrspCurvePtSet>(best_target, CrspCurvePtSet());
  //      std::pair<int, CrspCurvePtSet>& best_pair_set = src_core_crsp[i.first];
  //      for (auto j : i.second[best_target])
  //      {
  //        // save info for crsp_map_out
  //        crsp[CurvePt(i.first, j.first)] = CurvePt(best_target, j.second);
  //        crsp_map_out[CurvePt(i.first, j.first)] = crsp_map[CurvePt(i.first, j.first)];
  //        tar_curve_mark[best_target] = false;

  //        // save info to src_core_crsp
  //        best_pair_set.second.insert(j);
  //      }
  //    }
  //  }
  //  else
  //  {
  //    std::cout << "The best target curve isn't found in the t_s_cnt_map." << std::endl;
  //  }
  //}

  // pruning based on Curve-to-Curve crsp
  std::map<int, int> curve_crsp;
  std::map<int, int>::iterator curve_crsp_it;
  for (auto i : s_t_cnt_map)
  {
    // find the best target
    int best_target = -1;
    int best_target_cnt = std::numeric_limits<int>::min();
    for (auto j : i.second)
    {
      if (j.second > best_target_cnt)
      {
        best_target_cnt = j.second;
        best_target = j.first;
      }
    }

    t_s_cnt_it = t_s_cnt_map.find(best_target);
    if (t_s_cnt_it != t_s_cnt_map.end() && t_s_cnt_it->second.first == i.first)
    {
      curve_crsp[i.first] = best_target;
    }
  }

  for (auto i : crsp_map)
  {
    int src_cur_id = i.first.first;
    int tar_cur_id = i.second.first.first;
    curve_crsp_it = curve_crsp.find(src_cur_id);

    if (curve_crsp_it == curve_crsp.end()) continue;
    int best_tar_cur_id = curve_crsp_it->second;

    if (tar_cur_id == best_tar_cur_id
      || feature_model->tar_relationship[best_tar_cur_id].find(tar_cur_id) != feature_model->tar_relationship[best_tar_cur_id].end()
      || feature_model->global_user_marked_crsp.find(std::pair<int, int>(vis_global_mapper[src_cur_id], tar_cur_id)) != feature_model->global_user_marked_crsp.end())
    {
      crsp_map_out[i.first] = i.second;
    }
  }

  std::map<int, std::vector<int>>& global_vis_mapper = feature_model->getGlobalVisibleMapper(); 
  for(auto i : feature_model->global_user_marked_crsp)
  {
    std::map<CurvePt, CrspCurvePt> s_t_map;
    std::map<CurvePt, CrspCurvePt>::iterator s_t_map_it;
    CURVES src_curves;
    std::vector<int> source_curves = global_vis_mapper[i.first];
    std::vector<int> id;
    for(size_t k = 0; k < source_curves.size(); k ++)
    {
      src_curves.push_back(n_src_curves[source_curves[k]]);
      id.push_back(source_curves[k]);
    }
    for(size_t j = 0; j < n_tar_curves[i.second].size(); ++j)
    {
      int src_i = -1;
      int src_j = -1;
      double score = 0.0;
      paras[0] = feature_model->target_edges_sp_sl[i.second][j];
      if (CurvesUtility::closestPtFromSaliencyCurves(n_tar_curves[i.second][j], src_curves, src_i, src_j, score, paras))
      {
        src_i = id[src_i];
        s_t_map_it = s_t_map.find(CurvePt(src_i, src_j));
        if (s_t_map_it != s_t_map.end())
        {
          if (score > s_t_map_it->second.second)
          {
            s_t_map_it->second = CrspCurvePt(CurvePt(int(i.second), int(j)), score);
          }
        }
        else
        {
          s_t_map[CurvePt(src_i, src_j)] = CrspCurvePt(CurvePt(int(i.second), int(j)), score);
        }
      }
    }
    for(auto k : s_t_map)
    {
      crsp_map_out[k.first] = k.second;
    }
  }

  // extend possible new correspondences from the core target curve for each source curve
  //std::map<int, std::pair<int, CrspCurvePtSet > >::iterator src_core_crsp_it;
  //std::map<CurvePt, CrspCurvePt> ext_crsp_map;
  //std::map<CurvePt, CrspCurvePt>::iterator ext_crsp_it;
  //paras[0] = paras[1]; paras[1] = paras[2];
  //for (auto i : src_core_crsp)
  //{
  //  // for source curve i.first, search possible new correspondences
  //  int src_curve_id = i.first;
  //  for (size_t j = 0; j < n_src_curves[src_curve_id].size(); ++j)
  //  {
  //    // only samples that are not in src_core_crsp need to be considered
  //    if (crsp.find(CurvePt(src_curve_id, int(j))) == crsp.end())
  //    {
  //      // search closest target sample for this source sample
  //      //int tar_i = -1;
  //      //int tar_j = -1;
  //      //double score = 0.0;
  //      //CurvesUtility::closestPtInSaliencyCurves(n_src_curves[src_curve_id][j], n_tar_curves, feature_model->target_edges_sp_sl, tar_i, tar_j, score, paras);
  //      // now check if this target sample should be accepted
  //      // criteria 1: the target curve it belongs to is compatible with the core target curve
  //      // criteria 2: this s_to_t pair is better than the current s_to_t pair, i.e. the dis is larger than the current one
  //      //if (feature_model->tar_relationship[i.second.first].find(tar_i) != feature_model->tar_relationship[i.second.first].end())
  //      //{
  //      //  ext_crsp_it = ext_crsp_map.find(CurvePt(src_curve_id, j));
  //      //  if (ext_crsp_it != ext_crsp_map.end())
  //      //  {
  //      //    if (score > ext_crsp_it->second.second)
  //      //    {
  //      //      ext_crsp_it->second = CrspCurvePt(CurvePt(tar_i, tar_j), score);
  //      //    }
  //      //  }
  //      //  else
  //      //  {
  //      //    ext_crsp_map[CurvePt(src_curve_id, int(j))] = CrspCurvePt(CurvePt(tar_i, tar_j), score);
  //      //  }
  //      //}

  //      crsp_map_it = crsp_map.find(CurvePt(src_curve_id, int(j)));
  //      if (crsp_map_it != crsp_map.end())
  //      {
  //        int tar_i = crsp_map_it->second.first.first;
  //        
  //        if (feature_model->tar_relationship[i.second.first].find(tar_i) != feature_model->tar_relationship[i.second.first].end()
  //          || feature_model->user_define_curve_crsp.find(std::pair<int, int>(src_curve_id, tar_i)) != feature_model->user_define_curve_crsp.end())
  //        {
  //          crsp_map_out[CurvePt(src_curve_id, int(j))] = crsp_map_it->second;
  //        }
  //        // TODO if tar_i not in the core curve's relationship
  //        // but user define it corresponding to the source curve
  //        // don't push them into final crsp_map_out
  //        // first cache them from crsp_map
  //        // then later we
  //      }
  //    }
  //  }
  //}

  // save the ext_crsp_map to crsp_map_out
  //for (auto i : ext_crsp_map)
  //{
  //  //crsp_map_out[i.first] = i.second;
  //}
}
*/

void LargeFeatureCrsp::refineCrsp(std::map<CurvePt, CrspCurvePt>& crsp_map_out, std::deque<bool>& tar_curve_mark, CURVES& n_src_curves, CURVES& n_tar_curves)
{
  std::map<int, int>& vis_global_mapper = feature_model->getVisibleGlobalMapper(); 
  std::vector<double> paras(3, 0);
  paras[1] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  paras[2] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  std::map<CurvePt, CrspCurvePt> crsp_map; // key is source curve point as pair<int, int>, value is target curve point and their score
  std::map<CurvePt, CrspCurvePt>::iterator crsp_map_it;
  feature_model->computeAverageTargetCurvesDir();
  for (size_t i = 0; i < n_tar_curves.size(); ++i)
  {
    for (size_t j = 0; j < n_tar_curves[i].size(); ++j)
    {
      int src_i = -1;
      int src_j = -1;
      double score = 0.0;
      paras[0] = feature_model->target_edges_sp_sl[i][j];
      if (CurvesUtility::closestPtFromSaliencyCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, score, paras))
      {
        //score *= fabs(feature_model->src_avg_direction[src_i].dot(feature_model->tar_avg_direction[i])) ;
        score *= fabs(feature_model->src_avg_direction[src_i].dot(feature_model->sampled_target_curves_average_dir[i][j])) ;
        crsp_map_it = crsp_map.find(CurvePt(src_i, src_j));
        if (crsp_map_it != crsp_map.end())
        {
          if (score > crsp_map_it->second.second)
          {
            crsp_map_it->second = CrspCurvePt(CurvePt(int(i), int(j)), score);
          }
        }
        else
        {
          crsp_map[CurvePt(src_i, src_j)] = CrspCurvePt(CurvePt(int(i), int(j)), score);
        }
      }
    }
  }

  /*std::map<CurvePt, CrspCurvePt> crsp_map2;
  for (size_t i = 0; i < n_src_curves.size(); ++i)
  {
    for (size_t j = 0; j < n_src_curves[i].size(); ++j)
    {
      int tar_i = -1;
      int tar_j = -1;
      double score = 0.0;
      if (CurvesUtility::closestPtInSaliencyCurves(n_src_curves[i][j], n_tar_curves, feature_model->target_edges_sp_sl, tar_i, tar_j, score, paras))
      {
        score *= fabs(feature_model->src_avg_direction[i].dot(feature_model->tar_avg_direction[tar_i])) ;
        crsp_map_it = crsp_map2.find(CurvePt(tar_i, tar_j));
        if (crsp_map_it != crsp_map2.end())
        {
          if (score > crsp_map_it->second.second)
          {
            crsp_map_it->second = CrspCurvePt(CurvePt(int(i), int(j)), score);
          }
        }
        else
        {
          crsp_map2[CurvePt(tar_i, tar_j)] = CrspCurvePt(CurvePt(int(i), int(j)), score);
        }
      }
    }
  }
  std::map<CurvePt, CrspCurvePt> tmp_crsp_map;
  for(auto i : crsp_map2)
  {
    crsp_map_it = tmp_crsp_map.find(i.second.first);
    if(crsp_map_it != tmp_crsp_map.end())
    {
      if(i.second.second > crsp_map_it->second.second)
      {
        crsp_map_it->second = CrspCurvePt(i.first, i.second.second);
      }
    }
    else
    {
      tmp_crsp_map[i.second.first] = CrspCurvePt(i.first, i.second.second);
    }
  }*/
  /*for (size_t i = 0; i < n_src_curves.size(); ++i)
  {
    for (size_t j = 0; j < n_src_curves[i].size(); ++j)
    {
      int tar_i = -1;
      int tar_j = -1;
      double score = 0.0;
      if (CurvesUtility::closestPtInSaliencyCurves(n_src_curves[i][j], n_tar_curves, feature_model->target_edges_sp_sl, tar_i, tar_j, score, paras))
      {
        score *= fabs(feature_model->src_avg_direction[i].dot(feature_model->tar_avg_direction[tar_i])) ;
        crsp_map_it = crsp_map.find(CurvePt(i, j));
        if (crsp_map_it != crsp_map.end())
        {
          if (score > crsp_map_it->second.second)
          {
            crsp_map_it->second = CrspCurvePt(CurvePt(int(tar_i), int(tar_j)), score);
          }
        }
        else
        {
          crsp_map[CurvePt(i, j)] = CrspCurvePt(CurvePt(int(tar_i), int(tar_j)), score);
        }
      }
    }
  }*/

  //crsp_map_out = crsp_map; return;

  crsp_map_out.clear();
  std::vector<std::pair<int, int>> test_tar, test_src, test_path;
  std::set<std::pair<double, double>> src_pts;
  std::set<std::pair<double, double>>::iterator src_pts_it;
  /*test_src.clear();
  test_tar.clear();
  std::set<int> tar_id_candidate;*/
  for(size_t i = 0; i < n_src_curves.size(); i ++)
  {
    test_src.clear();
    test_tar.clear();
    std::set<int> tar_id_candidate;
    for(size_t j = 0; j < n_src_curves[i].size(); j ++)
    {
      /*if(j != 0 && j != n_src_curves[i].size())
      {
        test_src.push_back(std::pair<int, int>(i, j));
      }*/
      src_pts_it = src_pts.find(std::pair<double, double>(n_src_curves[i][j].x, n_src_curves[i][j].y));
      if(src_pts_it == src_pts.end())
      {
        test_src.push_back(std::pair<int, int>(i, j));
        src_pts.insert(std::pair<double, double>(n_src_curves[i][j].x, n_src_curves[i][j].y));
      }
      
      crsp_map_it = crsp_map.find(CurvePt(i, j));
      if(crsp_map_it != crsp_map.end())
      {
        tar_id_candidate.insert(crsp_map_it->second.first.first);
      }
      /*crsp_map_it = tmp_crsp_map.find(CurvePt(i, j));
      if(crsp_map_it != tmp_crsp_map.end())
      {
        tar_id_candidate.insert(crsp_map_it->second.first.first);
      }*/
    }
    /*double d1 = 0.0, d2 = 0.0;
    for(size_t i = 0; i < n_src_curves.size(); i ++)
    {
      d2 = 0.0;
      for(size_t j = 0; j < n_src_curves[i].size(); j ++)
      {
        if(j != 0)
        {
          double2 diff = n_src_curves[i][j] - n_src_curves[i][j - 1];
          d2 += sqrt(diff.x * diff.x + diff.y * diff.y);
        }
      }
      d2 /= n_src_curves[i].size() - 1;
      d1 += d2;
    }
    d1 /= n_src_curves.size();*/
    //double sample_rate = 1 * feature_model->sample_rate;

    for(auto j : tar_id_candidate)
    {
      double2 previous_point;
      for(size_t k = 0; k < n_tar_curves[j].size(); k ++)
      {
        if(k == 0)
        {
          test_tar.push_back(std::pair<int, int>(j, k));
          previous_point = n_tar_curves[j][k];
        }
        else
        {
          double2 current_point;
          current_point = n_tar_curves[j][k];
          double2 diff = current_point - previous_point;
          if(sqrt(diff.x * diff.x + diff.y * diff.y) >= feature_model->src_sample_rate[i])
          {
            test_tar.push_back(std::pair<int, int>(j, k));
            previous_point = n_tar_curves[j][k];
          }
        }
      }
    }
    if(test_tar.size() != 0)
    {
      solveHMM(test_src, test_tar, test_path, false);
      for(size_t j = 0; j < test_path.size(); j ++)
      {
        double2 diff = n_src_curves[test_src[j].first][test_src[j].second] - n_tar_curves[test_path[j].first][test_path[j].second];
        double cur_score = sqrt(diff.x * diff.x + diff.y * diff.y);
        cur_score = pow(feature_model->target_edges_sp_sl[test_path[j].first][test_path[j].second], paras[1]) / pow(cur_score + 0.0001, paras[2]);
        cur_score *= fabs(feature_model->src_avg_direction[test_src[j].first].dot(feature_model->sampled_target_curves_average_dir[test_path[j].first][test_path[j].second])) ;
        crsp_map_out[CurvePt(test_src[j])] = CrspCurvePt(test_path[j], cur_score);
      }
    }
  }

  /*std::map<CurvePt, CrspCurvePt> t2s_map;
  for(auto i : crsp_map_out)
  {
    crsp_map_it = t2s_map.find(i.second.first);
    if(crsp_map_it != t2s_map.end())
    {
      if(i.second.second > crsp_map_it->second.second)
      {
        crsp_map_it->second = CrspCurvePt(i.first, i.second.second);
      }
    }
    else
    {
      t2s_map[i.second.first] = CrspCurvePt(i.first, i.second.second);
    }
  }
  crsp_map_out.clear();
  for(auto i : t2s_map)
  {
    crsp_map_out[i.second.first] = CrspCurvePt(i.first, i.second.second); 
  }*/
  /*for(auto j : tar_id_candidate)
  {
    for(size_t k = 0; k < n_tar_curves[j].size(); k ++)
    {
      test_tar.push_back(std::pair<int, int>(j, k));
    }
  }
  solveHMM(test_src, test_tar, test_path, false);
  for(size_t i = 0; i < test_path.size(); i ++)
  {
    crsp_map_out[CurvePt(test_src[i])] = CrspCurvePt(test_path[i], 10);
  }*/
  /*std::vector<std::pair<int, int>> test_tar, test_src, test_path;
  for(size_t i = 0; i < n_tar_curves[10].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(10, i));
  }
  for(size_t i = 0; i < n_tar_curves[9].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(9, i));
  }
  for(size_t i = 0; i < n_tar_curves[8].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(8, i));
  }
  for(size_t i = 0; i < n_tar_curves[7].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(7, i));
  }
  for(size_t i = 0; i < n_tar_curves[6].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(6, i));
  }
  for(size_t i = 0; i < n_tar_curves[25].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(25, i));
  }
  for(size_t i = 0; i < n_tar_curves[29].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(29, i));
  }
  for(size_t i = 0; i < n_tar_curves[30].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(30, i));
  }
  for(size_t i = 0; i < n_tar_curves[42].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(42, i));
  }
  for(size_t i = 0; i < n_tar_curves[44].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(44, i));
  }
  for(size_t i = 0; i < n_tar_curves[62].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(62, i));
  }
  for(size_t i = 0; i < n_tar_curves[109].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(109, i));
  }
  for(size_t i = 0; i < n_tar_curves[187].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(187, i));
  }
  for(size_t i = 0; i < n_tar_curves[33].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(33, i));
  }
  for(size_t i = 0; i < n_tar_curves[21].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(21, i));
  }
  for(size_t i = 0; i < n_tar_curves[27].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(27, i));
  }
  for(size_t i = 0; i < n_tar_curves[34].size(); i ++)
  {
    test_tar.push_back(std::pair<int, int>(34, i));
  }
  


  for(size_t i = 0; i < n_src_curves[6].size(); i ++)
  {
    test_src.push_back(std::pair<int, int>(6, i));
  }*/

  /*solveHMM(test_tar, test_src, test_path, true);

  crsp_map_out.clear();
  for(size_t i = 0; i < test_path.size(); i ++)
  {
    crsp_map_out[CurvePt(test_path[i])] = CrspCurvePt(test_tar[i], 10);
  }*/
}


void LargeFeatureCrsp::solveHMM(std::vector<std::pair<int, int>>& observations, std::vector<std::pair<int, int>>& hidden, std::vector<std::pair<int, int>>& path, bool is_source_hidden)
{
  path.clear();
  path.resize(observations.size());

  CURVES source_curves, target_curves;
  this->feature_model->NormalizedSourceCurves(source_curves);
  this->feature_model->NormalizedTargetCurves(target_curves);

  std::vector<double> paras(3, 0);
  paras[1] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  paras[2] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  double para_c = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:c");

  std::vector<std::vector<double>> probability_score;
  probability_score.resize(observations.size());
  double max = std::numeric_limits<double>::min();
  for(size_t i = 0; i < observations.size(); i ++)
  {
    for(size_t j = 0; j < hidden.size(); j ++)
    {
      double cur_score;
      double angle;
      double2 diff;
      if(is_source_hidden)
      {
        diff = target_curves[observations[i].first][observations[i].second] - source_curves[hidden[j].first][hidden[j].second];
        paras[0] = feature_model->target_edges_sp_sl[observations[i].first][observations[i].second];
        //angle = fabs(feature_model->src_avg_direction[hidden[j].first].dot(feature_model->tar_avg_direction[observations[i].first])) ;
        angle = fabs(feature_model->src_avg_direction[hidden[j].first].dot(feature_model->sampled_target_curves_average_dir[observations[i].first][observations[i].second])) ;
      }
      else
      {
        diff = target_curves[hidden[j].first][hidden[j].second] - source_curves[observations[i].first][observations[i].second];
        paras[0] = feature_model->target_edges_sp_sl[hidden[j].first][hidden[j].second];
        //angle = fabs(feature_model->src_avg_direction[observations[i].first].dot(feature_model->tar_avg_direction[hidden[j].first])) ;
        angle = fabs(feature_model->src_avg_direction[observations[i].first].dot(feature_model->sampled_target_curves_average_dir[hidden[j].first][hidden[j].second])) ;
      }
      cur_score = sqrt(diff.x * diff.x + diff.y * diff.y);
      cur_score = pow(paras[0], paras[1]) / pow(cur_score + 0.0001, paras[2]);
      cur_score *= angle;
      //probability_score[i].push_back(exp((-0.5) * pow((1 / cur_score), 2))); // the computation method can be modified
      //probability_score[i].push_back(cur_score);
      double cur_pow_score = (0.5) * pow((1 / cur_score), 2);
      probability_score[i].push_back(cur_pow_score);
      /*if (cur_pow_score > max)
      {
          max = cur_pow_score;
      }*/
    }
  }


  std::vector<std::vector<std::pair<int, int>>> candidate_path;
  std::vector<double> probability;
  candidate_path.resize(hidden.size());
  probability.resize(hidden.size());
  for(size_t i = 0; i < observations.size(); i ++)
  {
    if(i == 0)
    {
      for(size_t j = 0; j < hidden.size(); j ++)
      {
        probability[j] = probability_score[i][j];
        candidate_path[j].push_back(hidden[j]);
      }
    }
    else
    {
      std::vector<std::vector<std::pair<int, int>>> newest_candidate_path;
      newest_candidate_path.resize(hidden.size());
      std::vector<double> newest_probability;
      newest_probability.resize(hidden.size());
      for(size_t j = 0; j < hidden.size(); j ++)
      {
        size_t best_path = 0;
        //double best_path_probability = std::numeric_limits<double>::min();
        double best_path_probability = std::numeric_limits<double>::max();
        for(size_t k = 0; k < hidden.size(); k ++)
        {
          double probability_continuity_k2j;
          double2 hidden_diff, observation_diff;
          if(is_source_hidden)
          {
            observation_diff = target_curves[observations[i].first][observations[i].second] - target_curves[observations[i - 1].first][observations[i - 1].second];
            hidden_diff = source_curves[hidden[j].first][hidden[j].second] - source_curves[hidden[k].first][hidden[k].second];
            probability_continuity_k2j = (hidden_diff.x * hidden_diff.x + hidden_diff.y * hidden_diff.y) / (observation_diff.x * observation_diff.x + observation_diff.y * observation_diff.y);
            //probability_continuity_k2j = exp(-0.5 * pow((probability_continuity_k2j - 1) / 0.05, 2));
            //probability_continuity_k2j = 1;
            probability_continuity_k2j = 0.5 * pow((probability_continuity_k2j - 1) / para_c, 2);
          }
          else
          {
            hidden_diff = target_curves[hidden[j].first][hidden[j].second] - target_curves[hidden[k].first][hidden[k].second];
            observation_diff = source_curves[observations[i].first][observations[i].second] - source_curves[observations[i - 1].first][observations[i - 1].second];
            probability_continuity_k2j = (hidden_diff.x * hidden_diff.x + hidden_diff.y * hidden_diff.y) / (observation_diff.x * observation_diff.x + observation_diff.y * observation_diff.y);
            //probability_continuity_k2j = exp(-0.5 * pow((probability_continuity_k2j - 1) / 0.05, 2));
            //probability_continuity_k2j = 1;
            probability_continuity_k2j = 0.5 * pow((probability_continuity_k2j - 1) / para_c, 2);
          }
          if((probability[k] + probability_continuity_k2j + probability_score[i][j]) < best_path_probability)
          {
            best_path_probability = probability[k] + probability_continuity_k2j + probability_score[i][j];
            best_path = k;
          }
        }
        newest_candidate_path[j] = (candidate_path[best_path]);
        newest_candidate_path[j].push_back(hidden[j]);
        newest_probability[j] = best_path_probability;
      }
      for(size_t j = 0; j < candidate_path.size(); j ++)
      {
        candidate_path[j] = newest_candidate_path[j];
        probability[j] = newest_probability[j];
        /*if(probability[j] == std::numeric_limits<double>::max())
        {
          std::cout << "" << std::endl;
        }*/
      }
    }
  }

  //double max_probability = std::numeric_limits<double>::min();
  double max_probability = std::numeric_limits<double>::max();
  size_t path_id;
  for(size_t i = 0; i < probability.size(); i ++)
  {
    if(probability[i] < max_probability)
    {
      max_probability = probability[i];
      path_id = i;
    }
  }
  path = candidate_path[path_id];
}
