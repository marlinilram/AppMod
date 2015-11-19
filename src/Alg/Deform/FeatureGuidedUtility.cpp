#include "FeatureGuided.h"
#include "FeatureLine.h"
#include "tele2d.h"

#include "KDTreeWrapper.h"
#include "CurvesUtility.h"
#include "UtilityHeader.h"
#include "RandSample.h"

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

void FeatureGuided::OptimizeConnection()
{
  // delete curves highly incompatible with the vector field

  CURVES new_target_curves;
  CURVES normalized_target_curves;
  this->NormalizedTargetCurves(normalized_target_curves);

  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    if (this->MatchScoreToVectorField(normalized_target_curves[i]) >= 0.6)
    {
      new_target_curves.push_back(this->target_curves[i]);
    }
  }

  CURVES best_connections;
  normalized_target_curves = new_target_curves;
  CurvesUtility::NormalizedCurves(normalized_target_curves, curve_translate, curve_scale);

  for (int i = 0; i < normalized_target_curves.size(); ++i)
  {
    double cur_best_score = std::numeric_limits<double>::min();
    std::vector<double2> cur_best_connection;
    int2 cur_best_end;
    int cur_best_curve_id;
    for (int j = 0; j < normalized_target_curves.size(); ++j)
    {
      if (i != j) {
        for (int endi = 0; endi < 2; ++endi)
        {
          for (int endj = 0; endj < 2; ++endj)
          {
            double cur_score = this->MatchScoreToVectorField(
              CurvesUtility::ConnectCurves(
              normalized_target_curves[i], normalized_target_curves[j], endi, endj));
            double2 dis = 
              normalized_target_curves[i][endi * (normalized_target_curves[i].size() - 1)]
            - normalized_target_curves[j][endi * (normalized_target_curves[j].size() - 1)];
            //cur_score /= dis.norm();
            if (cur_score > cur_best_score)
            {
              cur_best_score = cur_score;
              cur_best_end.first = endi;
              cur_best_end.second = endj;
              cur_best_curve_id = j;
            }
          }
        }}
    }
    best_connections.push_back(CurvesUtility::ConnectCurves(
      new_target_curves[i], new_target_curves[cur_best_curve_id],
      cur_best_end.first, cur_best_end.second));
  }

  for (int i = 0; i < best_connections.size(); ++i)
  {
    if (CurvesUtility::CurveLength(best_connections[i]) < 150)
    {
      new_target_curves.push_back(best_connections[i]);
    }
  }
  this->target_curves = new_target_curves;

#ifdef DEBUG
  // draw curves
  cv::Mat contour = cv::Mat::zeros(this->target_img.rows, this->target_img.cols, CV_8UC1);
  for (int i = 0; i < new_target_curves.size(); ++i)
  {
    for (int j = 0; j < new_target_curves[i].size(); ++j)
    {
      contour.at<uchar>(this->target_img.rows - 1 - new_target_curves[i][j].y, new_target_curves[i][j].x) = 255;
    }
    cv::imwrite("optimized_" + std::to_string(i) + ".png", contour);
  }
#endif
}

double FeatureGuided::MatchScoreToVectorField(std::vector<double2>& curve)
{
  // integrate
  int resolution = this->source_tele_register->resolution;
  std::vector<double2>& vector_field = this->source_tele_register->vector_field;
  double angle = 0;
  double curve_length = 0;
  for (int i = 1; i < curve.size(); ++i)
  {
    double2 curve_angle = curve[i] - curve[i - 1];
    curve_angle.normalize();
    double2 vector_field_pos = (curve[i] + curve[i - 1]) / 2;
    int x = (vector_field_pos.x * resolution + 0.5);
    int y = (vector_field_pos.y * resolution + 0.5);
    x = (x >= resolution ? resolution : x);
    y = (y >= resolution ? resolution : y);
    double2 vector_field_dir = vector_field[x + y * resolution];
    vector_field_dir.normalize();
    angle += abs(curve_angle.x * vector_field_dir.x + curve_angle.y * vector_field_dir.y);
  }
  angle /= (curve.size() - 1);
  return angle;
}


void FeatureGuided::BuildDispMap(const cv::Mat& source, kdtree::KDTreeArray& KDTree_data)
{
  // find curves
  std::vector<std::vector<bool> > visited_table(source.rows, std::vector<bool>(source.cols, false));
  CURVES edge_points;
  std::vector<double2> edge;
  size_t n_all_edge_points = 0;
  for (int i = 0; i < source.rows; ++i)
  {
    for (int j = 0; j < source.cols; ++j)
    {
      if (source.at<uchar>(i, j) > 200 && visited_table[i][j] == false)
      {
        edge.clear();
        FeatureGuided::SearchCurve(source, i, j, visited_table, edge);
        edge_points.push_back(edge);
        n_all_edge_points += edge.size();
      }
    }
  }

  //FeatureGuided::NormalizedCurves(edge_points);
  KDTree_data.resize(boost::extents[n_all_edge_points][2]);
  int cnt_edge_points = 0;
  for (int i = 0; i < edge_points.size(); ++i)
  {
    for (int j = 0; j < edge_points[i].size(); ++j)
    {
      KDTree_data[cnt_edge_points][0] = float(edge_points[i][j].x);
      KDTree_data[cnt_edge_points][1] = float(edge_points[i][j].y);
      ++cnt_edge_points;
    }
  }
}

void FeatureGuided::GetFittedCurves(CURVES& curves)
{
  // find corresponding curves in source and target
  CURVES new_target_curves;
  CURVES normalized_target_curves;
  this->NormalizedTargetCurves(normalized_target_curves);

  std::vector<float> query(2, 0.0);
  kdtree::KDTreeResultVector result;
  std::vector<double2> crsp_target;
  std::vector<double2> crsp_source;

  double2 target_translate = curve_translate;
  double target_scale = curve_scale;
  double2 source_translate = curve_translate;
  double source_scale = curve_scale;
  //FeatureGuided::NormalizePara(this->target_curves, target_translate, target_scale);
  //FeatureGuided::NormalizePara(this->source_curves, source_translate, source_scale);

  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    if (this->MatchScoreToVectorField(normalized_target_curves[i]) >= 0.85)
    {
      //curves.push_back(this->target_curves[i]);

      for (int j = 0; j < this->target_curves[i].size(); ++j)
      {
        double2 target_pos_in_source =
          (this->target_curves[i][j] + target_translate - double2(0.5, 0.5))
          * target_scale / source_scale + double2(0.5, 0.5) - source_translate;
        query[0] = target_pos_in_source.x;
        query[1] = target_pos_in_source.y;
        this->source_KDTree->nearestPt(query);
        crsp_target.push_back(this->target_curves[i][j]);
        crsp_source.push_back(double2(query[0], query[1]));
      }
    }
  }

  CurvesUtility::NormalizedCurves(curves, target_translate, target_scale);
  CurvesUtility::NormalizedCurve(crsp_target, target_translate, target_scale);
  CurvesUtility::NormalizedCurve(crsp_source, source_translate, source_scale);
  std::vector<double2> crsp_pair(2);
  for (int i = 0; i < crsp_target.size(); ++i)
  {
    crsp_pair[0] = crsp_source[i];
    crsp_pair[1] = crsp_target[i];
    curves.push_back(crsp_pair);
  }
}

void FeatureGuided::CalculateHists(
  HISTS& hists,
  CURVES& curves, double radius, tele2d* tele)
{
  double2 translate = curve_translate;
  double scale = curve_scale;
  //FeatureGuided::NormalizePara(curves, translate, scale);
  int resolution = tele->resolution;
  std::vector<double2>& vector_field = tele->vector_field;

  // There are possible two ways to do hist match
  // 1. pixel based: search possible pixel pair
  // 2. curve points based: only search possible curve point pair

  // Here is implementation for curve points based

  for (int i = 0; i < curves.size(); ++i)
  {
    for (int j = 0; j < curves[i].size(); ++j)
    {
      // for each curve point, calculate its hist
      double2 curve_pos = 
        ((curves[i][j] + translate - double2(0.5, 0.5)) * scale
        + double2(0.5, 0.5)) * resolution;
      double search_rad = radius * resolution;
      std::vector<int2> area;
      area.push_back(
        int2(curve_pos.x - search_rad, curve_pos.y - search_rad));
      area.push_back(
        int2(curve_pos.x + search_rad + 1, curve_pos.y + search_rad + 1));
      std::vector<double> hist(8, 0.0);
      // search around the center and build the histogram
      this->SearchRadius(
        hist,
        curve_pos, search_rad, area,
        vector_field, resolution);
      // normalize the histogram
      double sum_bins = 0.0;
      for (int k = 0; k < hist.size(); ++k)
      {
        sum_bins += hist[k] * hist[k];
      }
      sum_bins = sqrt(sum_bins);
      for (int k = 0; k < hist.size(); ++k)
      {
        hist[k] = hist[k] / sum_bins;
      }
      // cache the center coordinate into the last two elements of the hist
      hist.push_back(curve_pos.x / resolution);
      hist.push_back(curve_pos.y / resolution);
      hists.push_back(hist);
    }
  }
}


void FeatureGuided::SearchRadius(
  std::vector<double>& hist,
  double2 center, double r, std::vector<int2>& area,
  std::vector<double2>& vector_field, int resolution)
{
  // first compute the center's vector
  int x = center.x + 0.5;
  x = (x >= resolution) ? resolution : x;
  int y = center.y + 0.5;
  y = (y >= resolution) ? resolution : y;
  double2 cter_vector = vector_field[x + y * resolution];
  // record the vector in the circular area around the center with radius r
  for (int xStep = area[0].x; xStep <= area[1].x; ++xStep)
  {
    for (int yStep = area[0].y; yStep <= area[1].y; ++yStep)
    {
      if (xStep >= 0 && xStep < resolution
        && yStep >= 0 && yStep < resolution
        && double2(center.x - xStep, center.y - yStep).norm() <= r)
      {
        // the histogram may store the relative variation but not the absolute angle
        //hist[ProjectDirToBin2D(vector_field[xStep + yStep * resolution])] += 1.0;
        hist[ProjectRelativeDirToBin2D(
          vector_field[xStep + yStep * resolution],
          cter_vector)] += 1.0;
      }
    }
  }
}

void FeatureGuided::FindHistMatchCrsp(CURVES &curves)
{
  // calculate hists for each edge points
  // 0.1 is the radius for neighborhood to build histogram
  HISTS source_hists;
  HISTS target_hists;
  this->CalculateHists(source_hists, this->source_curves, 0.1, source_tele_register.get());
  this->CalculateHists(target_hists, this->target_curves, 0.1, target_tele_register.get());

  // find best matched points from source to target
  // 0.05 is the radius for neighborhood to find correspondences
  std::map<int, int> source_to_target;
  std::map<int, int>::iterator map_iter;
  for (int i = 0; i < source_hists.size(); ++i)
  {
    double cur_min_score = std::numeric_limits<double>::max();
    std::pair<int, int> best_match;
    for (int j = 0; j < target_hists.size(); ++j)
    {
      double cur_score = HistMatchScore(source_hists[i], target_hists[j], 0.05);
      if (cur_score < cur_min_score)
      {
        cur_min_score = cur_score;
        best_match.first = i;
        best_match.second = j;
      }
    }
    // do not store all the best match
    // some of them are not resonable
    if (cur_min_score < 0.5)
    {
      source_to_target[best_match.first] = best_match.second;
    }
  }
  std::vector<std::pair<int, int> > target_to_source;
  std::vector<double2> crsp_pair(2, double2(0.0, 0.0));
  for (int i = 0; i < target_hists.size(); ++i)
  {
    double cur_min_score = std::numeric_limits<double>::max();
    std::pair<int, int> best_match;
    for (int j = 0; j < source_hists.size(); ++j)
    {
      double cur_score = HistMatchScore(target_hists[i], source_hists[j], 0.05);
      if (cur_score < cur_min_score)
      {
        cur_min_score = cur_score;
        best_match.first = i;
        best_match.second = j;
      }
    }

    // test if the best match from target to source
    // is also the best match from source to target
    map_iter = source_to_target.find(best_match.second);
    if (map_iter != source_to_target.end())
    {
      if (map_iter->second == best_match.first)
      {
        crsp_pair[0] = double2(
          source_hists[best_match.second][8],
          source_hists[best_match.second][9]);
        crsp_pair[1] = double2(
          target_hists[best_match.first][8],
          target_hists[best_match.first][9]);
        curves.push_back(crsp_pair);
      }
    }
  }
  std::cout<<curves.size()<<"\n";
}

void FeatureGuided::GetCrspPair(CURVES& curves)
{
  // return denormalized correspondence pair for shape optimization
  double2 target_translate = curve_translate;
  double target_scale = curve_scale;
  double2 source_translate = curve_translate;
  double source_scale = curve_scale;

  CURVES temp_curves;
  this->GetFittedCurves(temp_curves);
  CURVE source_crsp;
  CURVE target_crsp;
  for (size_t i = 0; i < temp_curves.size(); ++i)
  {
    source_crsp.push_back(temp_curves[i][0]);
    target_crsp.push_back(temp_curves[i][1]);
  }
  CurvesUtility::DenormalizedCurve(source_crsp, source_translate, source_scale);
  CurvesUtility::DenormalizedCurve(target_crsp, target_translate, target_scale);

  std::vector<double2> crsp_pair(2);
  for (size_t i = 0; i < source_crsp.size(); ++i)
  {
    crsp_pair[0] = source_crsp[i];
    crsp_pair[1] = target_crsp[i];
    curves.push_back(crsp_pair);
  }
}