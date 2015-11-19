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

  // find corresponding points in source curves
  // guarantee that the found correspondences are two-sided closest
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
      paras[0] = feature_model->target_edges_sp_sl[i][j];
      //if (CurvesUtility::closestPtInCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, target_scalar_field->matching_map, target_scalar_field->resolution, 0.3))
      if (CurvesUtility::closestPtInSaliencyCurves(n_tar_curves[i][j], n_src_curves, src_i, src_j, dis, paras))
      {
        it = crsp_map.find(CurvePt(src_i, src_j));
        if (it != crsp_map.end())
        {
          if (dis > it->second.second)
          {
            it->second = CrspCurvePt(CurvePt(int(i), int(j)), dis);
            crsp[CurvePt(src_i, src_j)] = CurvePt(int(i), int(j));
          }
        }
        else
        {
          crsp_map[CurvePt(src_i, src_j)] = CrspCurvePt(CurvePt(int(i), int(j)), dis);
          crsp[CurvePt(src_i, src_j)] = CurvePt(int(i), int(j));
        }
      }
    }
  }

  return;
  // refine the correspondences
  // 1. 

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

  int cnt = 0;
  crsp.clear();
  for (auto i : s_t_cnt_map)
  {
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
    for (auto j : i.second[best_target])
    {
      crsp[CurvePt(i.first, j.first)] = CurvePt(best_target, j.second);
    }
    
  }
}