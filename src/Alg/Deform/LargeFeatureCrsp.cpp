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

void LargeFeatureCrsp::refineCrsp(std::map<CurvePt, CrspCurvePt>& crsp_map_out, std::deque<bool>& tar_curve_mark, CURVES& n_src_curves, CURVES& n_tar_curves)
{
  // find corresponding points in source curves
  // guarantee that the found correspondences are two-sided closest
  std::vector<double> paras(3, 0);
  paras[1] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a");
  paras[2] = LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b");
  std::map<CurvePt, CrspCurvePt> crsp_map;
  std::map<CurvePt, CrspCurvePt>::iterator crsp_map_it;
  std::map<CurvePt, CurvePt> crsp;
  for (size_t i = 0; i < n_tar_curves.size(); ++i)
  {
    if (tar_curve_mark[i])
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
          //if (fabs(feature_model->src_avg_direction[src_i].dot(feature_model->tar_avg_direction[i])) < 0.9 ) continue;
          if (crsp_map_out.find(CurvePt(src_i, src_j)) == crsp_map_out.end())
          {
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
  }

  // refine the correspondences
  // 1. keep the most confidential target curve

  std::map<int, std::map<int, std::vector<CurvePt> > > s_t_cnt_map;
  std::map<int, std::map<int, std::vector<CurvePt> > >::iterator cnt_map_it;
  std::map<int, std::vector<CurvePt> >::iterator cnt_it;
  for (auto i : crsp)
  {
    cnt_map_it = s_t_cnt_map.find(i.first.first);
    if (cnt_map_it != s_t_cnt_map.end())
    {
      cnt_it = cnt_map_it->second.find(i.second.first);
      if (cnt_it != cnt_map_it->second.end())
      {
        cnt_it->second.push_back(CurvePt(i.first.second, i.second.second));
      }
      else
      {
        cnt_map_it->second[i.second.first] = std::vector<CurvePt>(1, CurvePt(i.first.second, i.second.second));
      }
    }
    else
    {
      s_t_cnt_map[i.first.first] = std::map<int, std::vector<CurvePt> >();
      s_t_cnt_map[i.first.first][i.second.first] = std::vector<CurvePt>(1, CurvePt(i.first.second, i.second.second));
    }
  }

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

      int cnt = j.second.size();
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
  typedef std::set<std::pair<int, int> > CrspCurvePtSet;
  std::map<int, std::pair<int, CrspCurvePtSet > > src_core_crsp; // key is the source curve id, first of pair is target curve id, each pair in set is sample id in source curve and target curve
  int cnt = 0;
  crsp.clear();
  for (auto i : s_t_cnt_map)
  {
    // find the best target
    int best_target = -1;
    int best_target_cnt = std::numeric_limits<int>::min();
    for (auto j : i.second)
    {
      cnt = j.second.size();
      if (cnt > best_target_cnt)
      {
        best_target_cnt = j.second.size();
        best_target = j.first;
      }
    }

    // save the best crsp
    // if the best target curve also has its best source curve than we confirm it
    t_s_cnt_it = t_s_cnt_map.find(best_target);
    if (t_s_cnt_it != t_s_cnt_map.end())
    {
      if (i.first == t_s_cnt_it->second.first)
      {
        //std::cout << "best curve correspondence has " << i.second[best_target].size() << " pairs." << std::endl;
        src_core_crsp[i.first] = std::pair<int, CrspCurvePtSet>(best_target, CrspCurvePtSet());
        std::pair<int, CrspCurvePtSet>& best_pair_set = src_core_crsp[i.first];
        for (auto j : i.second[best_target])
        {
          // save info for crsp_map_out
          crsp[CurvePt(i.first, j.first)] = CurvePt(best_target, j.second);
          crsp_map_out[CurvePt(i.first, j.first)] = crsp_map[CurvePt(i.first, j.first)];
          tar_curve_mark[best_target] = false;

          // save info to src_core_crsp
          best_pair_set.second.insert(j);
        }
      }
    }
    else
    {
      std::cout << "The best target curve isn't found in the t_s_cnt_map." << std::endl;
    }
  }

  // extend possible new correspondences from the core target curve for each source curve
  std::map<int, std::pair<int, CrspCurvePtSet > >::iterator src_core_crsp_it;
  std::map<CurvePt, CrspCurvePt> ext_crsp_map;
  std::map<CurvePt, CrspCurvePt>::iterator ext_crsp_it;
  paras[0] = paras[1]; paras[1] = paras[2];
  for (auto i : src_core_crsp)
  {
    // for source curve i.first, search possible new correspondences
    int src_curve_id = i.first;
    for (size_t j = 0; j < n_src_curves[src_curve_id].size(); ++j)
    {
      // only samples that are not in src_core_crsp need to be considered
      if (crsp.find(CurvePt(src_curve_id, int(j))) == crsp.end())
      {
        // search closest target sample for this source sample
        //int tar_i = -1;
        //int tar_j = -1;
        //double score = 0.0;
        //CurvesUtility::closestPtInSaliencyCurves(n_src_curves[src_curve_id][j], n_tar_curves, feature_model->target_edges_sp_sl, tar_i, tar_j, score, paras);
        // now check if this target sample should be accepted
        // criteria 1: the target curve it belongs to is compatible with the core target curve
        // criteria 2: this s_to_t pair is better than the current s_to_t pair, i.e. the dis is larger than the current one
        //if (feature_model->tar_relationship[i.second.first].find(tar_i) != feature_model->tar_relationship[i.second.first].end())
        //{
        //  ext_crsp_it = ext_crsp_map.find(CurvePt(src_curve_id, j));
        //  if (ext_crsp_it != ext_crsp_map.end())
        //  {
        //    if (score > ext_crsp_it->second.second)
        //    {
        //      ext_crsp_it->second = CrspCurvePt(CurvePt(tar_i, tar_j), score);
        //    }
        //  }
        //  else
        //  {
        //    ext_crsp_map[CurvePt(src_curve_id, int(j))] = CrspCurvePt(CurvePt(tar_i, tar_j), score);
        //  }
        //}

        crsp_map_it = crsp_map.find(CurvePt(src_curve_id, int(j)));
        if (crsp_map_it != crsp_map.end())
        {
          int tar_i = crsp_map_it->second.first.first;
          if (feature_model->tar_relationship[i.second.first].find(tar_i) != feature_model->tar_relationship[i.second.first].end())
          {
            crsp_map_out[CurvePt(src_curve_id, int(j))] = crsp_map_it->second;
          }
        }
      }
    }
  }

  // save the ext_crsp_map to crsp_map_out
  //for (auto i : ext_crsp_map)
  //{
  //  //crsp_map_out[i.first] = i.second;
  //}
}