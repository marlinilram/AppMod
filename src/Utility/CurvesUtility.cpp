#include "CurvesUtility.h"

#include "tele2d.h"

namespace CurvesUtility
{


bool isBoundary(cv::Mat& primitive_img, int x, int y)
{
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      int row_id = y + i - 1;
      int col_id = x + j - 1;
      row_id = std::max(std::min(row_id, primitive_img.rows - 1), 0);
      col_id = std::max(std::min(col_id, primitive_img.cols - 1), 0);

      if (!(row_id == y && col_id == x))
      {
        if (primitive_img.at<int>(row_id, col_id) < 0)
        {
          return true;
        }
      }
    }
  }
  return false;
}

void getBoundaryImg(cv::Mat& outputImg, cv::Mat& primitive_id_img)
{
  outputImg = cv::Mat::zeros(primitive_id_img.size(), CV_32FC1);
  std::vector<std::pair<int, int> > boundary_pts;
  for (int i = 0; i < primitive_id_img.rows; ++i)
  {
    for (int j = 0; j < primitive_id_img.cols; ++j)
    {
      if (primitive_id_img.at<int>(i, j) >= 0)
      {
        if (isBoundary(primitive_id_img, j, i))
        {
          // store as x, y
          boundary_pts.push_back(std::pair<int, int>(j, i));
          outputImg.at<float>(i, j) = 1.0;
        }
      }
    }
  }
}

double distInScalarMap(STLVectorf& scalar_map, int dim, double2& src_pt, double2& tar_pt, double threshold)
{
  // src_pt and tar_pt are normalized position
  // if the euclidean distance between them is larger than threshold
  // return infinite directly to save time
  // if the euclidean distance is less than threshold
  // we do the integration

  double e_dist = sqrt(pow(src_pt.x - tar_pt.x, 2) + pow(src_pt.y - tar_pt.y, 2));

  if (dim == 0)
  {
    // use euclidean distance
    return e_dist;
  }

  if (e_dist > threshold)
  {
    return std::numeric_limits<double>::max();
  }
  else
  {
    double step = 0.01;
    double xStep = 0.01 * (src_pt.x - tar_pt.x) / e_dist;
    double yStep = 0.01 * (src_pt.y - tar_pt.y) / e_dist;
    double dist = 0.0;
    for (int i = 1; i < e_dist / 0.01; ++i)
    {
      dist += 0.01 * (1 - scalar_map[int(src_pt.x + i * xStep) + int(src_pt.y + i * yStep) * dim]);
    }
    return dist;
  }
}

bool closestPtInCurves(double2& tar_pt, std::vector<std::vector<double2> >& src_curves,
  int& src_i, int& src_j, double& dis, 
  STLVectorf& scalar_map, int dim, double threshold)
{
  dis = std::numeric_limits<double>::max();
  for (size_t i = 0; i < src_curves.size(); ++i)
  {
    for (size_t j = 0; j < src_curves[i].size(); ++j)
    {
      double cur_dis = distInScalarMap(scalar_map, dim, src_curves[i][j], tar_pt, threshold);
      if (cur_dis < dis)
      {
        src_i = int(i);
        src_j = int(j);
        dis = cur_dis;
      }
    }
  }

  if (src_i != -1 && src_j != -1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool closestPtFromSaliencyCurves(double2& tar_pt, std::vector<std::vector<double2> >& src_curves,
  int& src_i, int& src_j, double& dis, std::vector<double>& paras)
{
  dis = std::numeric_limits<double>::min();
  for (size_t i = 0; i < src_curves.size(); ++i)
  {
    for (size_t j = 0; j < src_curves[i].size(); ++j)
    {
      double2 diff = tar_pt - src_curves[i][j];
      double cur_score = sqrt(diff.x * diff.x + diff.y * diff.y);
      cur_score = pow(paras[0], paras[1]) / pow(cur_score + 0.0001, paras[2]);
      if (cur_score > dis)
      {
        src_i = int(i);
        src_j = int(j);
        dis = cur_score;
      }
    }
  }

  if (src_i != -1 && src_j != -1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool closestPtInSaliencyCurves(double2& src_pt, std::vector<std::vector<double2> >& tar_curves, std::vector<std::vector<double> >& tar_sl,
  int& tar_i, int& tar_j, double& dis, std::vector<double>& paras)
{
  dis = std::numeric_limits<double>::min();
  for (size_t i = 0; i < tar_curves.size(); ++i)
  {
    for (size_t j = 0; j < tar_curves[i].size(); ++j)
    {
      double2 diff = src_pt - tar_curves[i][j];
      double cur_score = sqrt(diff.x * diff.x + diff.y * diff.y);
      cur_score = pow(tar_sl[i][j], paras[0]) / pow(cur_score + 0.0001, paras[1]);
      if (cur_score > dis)
      {
        tar_i = int(i);
        tar_j = int(j);
        dis = cur_score;
      }
    }
  }
  return true;
}

void mergeShapeEdges(std::vector<Edge>& edges, std::vector<STLVectori>& lines)
{
  lines.clear();
  for (size_t i = 0; i < edges.size(); ++i)
  {
    STLVectori temp_edges;
    temp_edges.push_back(edges[i].first);
    temp_edges.push_back(edges[i].second);
    lines.push_back(temp_edges);
  }
  // merge connected edge
  int tag = 0;
  size_t i = 0;
  while (i < lines.size())
  {
    int start = lines[i][0];
    int end   = lines[i][lines[i].size() - 1];

    for (size_t j = i + 1; j < lines.size(); ++j)
    {
      int cur_start = lines[j][0];
      int cur_end   = lines[j][lines[j].size() - 1];

      // four types
      if (start == cur_start)
      {
        int start_n = lines[i][1]; // the next v_id from start
        int cur_start_n = lines[j][1]; // the next v_id from start
        std::reverse(lines[j].begin(), lines[j].end());
        lines[i].insert(lines[i].begin(), lines[j].begin(), lines[j].end() - 1);
        lines.erase(lines.begin() + j);
        tag = 1;
        break;
      }
      else if (start == cur_end)
      {
        int start_n = lines[i][1]; // the next v_id from start
        int cur_end_p = lines[j][lines[j].size() - 2];
        lines[i].insert(lines[i].begin(), lines[j].begin(), lines[j].end() - 1);
        lines.erase(lines.begin() + j);
        tag = 1;
        break;
      }
      else if (end == cur_start)
      {
        int end_p = lines[i][lines[i].size() - 2];
        int cur_start_n = lines[j][1]; // the next v_id from start
        lines[i].insert(lines[i].end(), lines[j].begin() + 1, lines[j].end());
        lines.erase(lines.begin() + j);
        tag = 1;
        break;
      }
      else if (end == cur_end)
      {
        int end_p = lines[i][lines[i].size() - 2];
        int cur_end_p = lines[j][lines[j].size() - 2];
        std::reverse(lines[j].begin(), lines[j].end());
        lines[i].insert(lines[i].end(), lines[j].begin() + 1, lines[j].end());
        lines.erase(lines.begin() + j);
        tag = 1;
        break;
      }
    }

    if (tag == 1)
    {
      tag = 0;
    }
    else
    {
      ++i;
    }
  }
}


CURVES ReorganizeCurves(CURVES& curves, float sp_rate)
{
  // check if all curves are single connected
  for (int i = 0; i < curves.size(); ++i)
  {
    int j = 1;
    for (; j < curves[i].size(); ++j)
    {
      if (sqrt(pow(curves[i][j].x - curves[i][j - 1].x, 2) + pow(curves[i][j].y - curves[i][j - 1].y, 2)) > 1.5)
      {
        // if it's connected no larger than 1.414
        break;
      }
    }
    if (j != curves[i].size())
    {
      // break early, i is the one end 
      std::cerr<<"Not a single connected curve.\n";
    }
  }

  // reconnect possible curves
  int tag = 0;
  size_t i = 0;
  while (i < curves.size())
  {
    // only need to test the start point
    int x_cur = int(curves[i][0].x + 0.5);
    int y_cur = int(curves[i][0].y + 0.5);

    for (size_t j = i + 1; j < curves.size(); ++j)
    {
      int x_temp = int(curves[j][0].x + 0.5);
      int y_temp = int(curves[j][0].y + 0.5);

      if (abs(x_cur - x_temp) <= 1 && abs(y_cur - y_temp) <= 1)
      {
        // find a curve which should connect with the current curve
        // to keep the order of current curve
        // we insert the found curve backward into the head of current
        // curve
        std::reverse(curves[j].begin(), curves[j].end());
        curves[i].insert(curves[i].begin(), curves[j].begin(), curves[j].end());
        curves.erase(curves.begin() + j);
        tag = 1;
        break;
      }
    }
    if (tag == 1)
    {
      tag = 0;
    }
    else
    {
      ++i;
    }
  }

  // sample curve
  // don't save whole pixel position for the contour
  // lead to error in tele-reg
  CURVES reorganized;
  for (int i = 0; i < curves.size(); ++i)
  {
    //double cur_length = FeatureGuided::CurveLength(curves[i]);

    //if (cur_length > 0)
    {
      if (curves[i].size() > 20)
      {
        int step = int(sp_rate + 0.5);// 1;// + curves[i].size() / 50;
        int tail = 0;
        CURVE temp_curve;
        for (int j = 0; j < curves[i].size(); ++j)
        {
          if (j % step == 0)
          {
            temp_curve.push_back(curves[i][j]);
            ++tail;
          }
        }
        //curves[i].erase(curves[i].begin() + tail, curves[i].end());
        reorganized.push_back(temp_curve);
      }
    }
  }
  return reorganized;
}

CURVES SplitCurve(std::vector<double2> curve)
{
  CURVES curves;
  int i = 1;
  for (; i < curve.size(); ++i)
  {
    if (sqrt(pow(curve[i].x - curve[i - 1].x, 2) + pow(curve[i].y - curve[i - 1].y, 2)) > 1.5)
    {
      // if it's connected no larger than 1.414
      break;
    }
  }
  if (i == curve.size())
  {
    curves.push_back(curve);
    return curves;
  }
  else
  {
    // break early, i is the one end
    std::vector<double2> curve1(curve.begin() + i, curve.end());
    curves.push_back(std::vector<double2>(curve.begin(), curve.begin() + i));
    CURVES r_curves = SplitCurve(curve1);
    curves.insert(curves.end(), r_curves.begin(), r_curves.end());
    return curves;
  }
}

std::vector<double2> ConnectCurves(std::vector<double2> curve0, std::vector<double2> curve1,
                                                  int endtag0, int endtag1)
{
  const int endpid1_id = (endtag0%2 == 0) ? 0 : curve0.size() -1;
  const int endpid2_id = (endtag1%2 == 0) ? 0 : curve1.size() -1;

  double2 p0 = curve0[endpid1_id] ;
  double2 p1 = curve1[endpid2_id] ;
  double2 tan0, tan1 ;

  // slope of tangent
  if( endpid1_id == 0 )
    tan0 = curve0[0] - curve0[1] ;
  else
    tan0 = curve0.back() - curve0[curve0.size()-2] ;

  if( endpid2_id == 0 )
    tan1 = curve1[1] - curve1[0] ; 
  else
    tan1 = curve1[curve1.size()-2] - curve1.back() ;

  tan0.normalize();
  tan1.normalize();


  std::vector<double2>  nullcurve;
  for( double t=0.0; t<=1.0; t+=0.01 )
  {
    nullcurve.push_back( tele2d::get_hermite_value( p0, p1, tan0, tan1, t ) );
  }

  return nullcurve;
}

double CurveLength(std::vector<double2>& curve)
{
  return PartCurveLength(curve, curve.size() - 1);
}

double PartCurveLength(std::vector<double2>& curve, int sp_id)
{
  double cur_length = 0;
  for (int j = 1; j < curve.size() && j <= sp_id; ++j)
  {
    cur_length += 
      sqrt(pow(curve[j].x - curve[j - 1].x, 2)
      + pow(curve[j].y - curve[j - 1].y, 2));
  }
  return cur_length;
}

void NormalizedCurves(CURVES& curves)
{
  double2 normalize_translate;
  double normalize_scale;

  NormalizePara(curves, normalize_translate, normalize_scale);

  NormalizedCurves(curves, normalize_translate, normalize_scale);
}

void NormalizedCurves(CURVES& curves, double2 translate, double scale)
{
  for( int i=0; i<curves.size(); ++i )
    for( int j=0; j<curves[i].size(); ++j ){
      curves[i][j] = ( curves[i][j] + translate - double2(0.5, 0.5 ) ) * scale + double2(0.5, 0.5 );
    }
}

void NormalizedCurve(CURVE& curve, double2 translate, double scale)
{
  for (int i = 0; i < curve.size(); ++i)
  {
    curve[i] = ( curve[i] + translate - double2(0.5, 0.5 ) ) * scale + double2(0.5, 0.5 );
  }
}

void DenormalizedCurve(CURVE& curve, double2 translate, double scale)
{
  for (int i = 0; i < curve.size(); ++i)
  {
    curve[i] = (curve[i] - double2(0.5, 0.5)) / scale + double2(0.5, 0.5) - translate;
  }
}

void NormalizePara(CURVES& curves, double2& translate, double& scale)
{
  double2 minCorner ( 1e10, 1e10);
  double2 maxCorner (-1e10, -1e10);
  for( int i=0; i<curves.size(); ++i )
    for( int j=0; j<curves[i].size(); ++j ){
      if( curves[i][j].x < minCorner.x  ) minCorner.x =  curves[i][j].x ;
      if( curves[i][j].y < minCorner.y  ) minCorner.y =  curves[i][j].y ;
      if( curves[i][j].x > maxCorner.x  ) maxCorner.x =  curves[i][j].x ;
      if( curves[i][j].y > maxCorner.y  ) maxCorner.y =  curves[i][j].y ;
    }

    double2 cter = (minCorner+maxCorner) / 2 ;
    translate =  double2(0.5, 0.5 ) - cter ;
    scale = 0.6 / std::max( (maxCorner-minCorner).x, (maxCorner-minCorner).y  ) ;
}


void CurveSpTwoSideDist(std::vector<std::vector<double2> >& edges_sp_len, CURVES& curves)
{
  // compute the distance to two end of the curve for each sample
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
    edges_sp_len.push_back(temp_edge_sp_len);
  }
}

void CurveSpSaliency(std::vector<std::vector<double> >& edges_sp_sl, CURVES& curves, cv::Mat& saliency_img, std::vector<double>& edges_average_sp_sl)
{
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
          int img_i = saliency_img.rows - (curves[i][cur_idx].y + 0.5);
          int img_j = curves[i][cur_idx].x + 0.5;
          img_i = img_i < 0 ? 0 : (img_i < saliency_img.rows ? img_i : saliency_img.rows);
          img_j = img_j < 0 ? 0 : (img_j < saliency_img.cols ? img_j : saliency_img.cols);
          saliency += saliency_img.at<float>(img_i, img_j);
        }
      }
      temp_edge_sp_sl.push_back(saliency / 21);
    }
    edges_sp_sl.push_back(temp_edge_sp_sl);
  }

  std::ofstream f_debug("saliency.mat");
  if (f_debug)
  {
    for (size_t i = 0; i < edges_sp_sl.size(); ++i)
    {
      for (size_t j = 0; j < edges_sp_sl[i].size(); ++j)
      {
        f_debug << edges_sp_sl[i][j] << "\n";
      }
    }
    f_debug.close();
  }

  for(size_t i = 0; i < edges_sp_sl.size(); i ++)
  {
    double sum = 0.0;
    for(size_t j = 0; j < edges_sp_sl[i].size(); j ++)
    {
      sum += edges_sp_sl[i][j];
    }
    edges_average_sp_sl.push_back(sum / (edges_sp_sl[i].size() - 1));
  }
}


CURVE SmoothCurve(CURVE& curve_in, int win_size)
{
  int offset = win_size / 2;
  CURVE curve_out;
  for (size_t i = 0; i < curve_in.size(); ++i)
  {
    int n_pt = 0;
    double2 sum_pt(0, 0);
    for (int i_off = -offset; i_off <= offset; ++i_off)
    {
      if ((i + i_off) >= 0 && (i + i_off) < curve_in.size())
      {
        sum_pt = sum_pt + curve_in[i + i_off];
        ++n_pt;
      }
    }
    sum_pt = sum_pt / n_pt;
    curve_out.push_back(sum_pt);
  }
  return curve_out;
}

CURVES SmoothCurves(CURVES& curves_in, int win_size /* = 3 */)
{
  CURVES curves_out;
  for (size_t i = 0; i < curves_in.size(); ++i)
  {
    curves_out.push_back(SmoothCurve(curves_in[i], win_size));
  }
  return curves_out;
}


std::vector<int> DetectBreakPoint(CURVE& curve_in, int win_size /* = 3 */, double th /* = 0.5 */)
{
  int offset = win_size / 2;
  std::vector<int> bk_point;
  std::vector<int> id_cache; // to prevent the case that successive points are all break points
  for (int i = 0; i < curve_in.size(); ++i)
  {
    if ((i - offset) >= 0 && (i + offset) < curve_in.size())
    {
      double2 prev = curve_in[i] - curve_in[i - offset];
      double2 next = curve_in[i + offset] - curve_in[i];
      double cos_pt = ((prev.x * next.x + prev.y * next.y) / prev.norm() / next.norm());

      if (cos_pt < th)
      {
        if (id_cache.empty() || (id_cache.back() == (i - 1)))
        {
          id_cache.push_back(i);
        }
        else
        {
          bk_point.push_back(id_cache[id_cache.size() / 2]); // take the median one
          id_cache.clear();
          id_cache.push_back(i);
        }
      }
    }
  }
  if (!id_cache.empty())
  {
    bk_point.push_back(id_cache[id_cache.size() / 2]);
  }
  return bk_point;
}

std::vector<std::vector<int> > DetectBreakPointAll(CURVES& curves_in, int win_size /* = 3 */, double th /* = 0.5 */)
{
  std::vector<std::vector<int> > bk_points;
  for (size_t i = 0; i < curves_in.size(); ++i)
  {
    bk_points.push_back(DetectBreakPoint(curves_in[i], win_size, th));
  }
  return bk_points;
}

CURVES BreakCurves(CURVES& curves, std::vector<std::vector<int> >& bk_points)
{
  if (curves.size() != bk_points.size())
  {
    std::cout << "size of curves and bk_points doesn't match." << std::endl;
    return curves;
  }

  CURVES curves_out;
  for (size_t i = 0; i < curves.size(); ++i)
  {
    int start = 0;
    for (size_t j = 0; j < bk_points[i].size(); ++j)
    {
      if ((bk_points[i][j] - start) * 50 >= curves[i].size())
      {
        curves_out.push_back(CURVE(curves[i].begin() + start, curves[i].begin() + bk_points[i][j]));
        start = bk_points[i][j] + 1;
      }
    }
    curves_out.push_back(CURVE(curves[i].begin() + start, curves[i].end()));
  }
  return curves_out;
}

double2 ClosestConnection(double2& end_0_0, double2& end_0_1, double2& end_1_0, double2& end_1_1)
{
  double2 temp_0 = (end_0_0 - end_1_0).norm() < (end_0_0 - end_1_1).norm() ? (end_0_0 - end_1_0) : (end_0_0 - end_1_1);
  double2 temp_1 = (end_0_1 - end_1_0).norm() < (end_0_1 - end_1_1).norm() ? (end_0_1 - end_1_0) : (end_0_1 - end_1_1);
  return (temp_0.norm() < temp_1.norm() ? temp_0 : temp_1);
}

void CurvesAvgDir(CURVES& curves, std::vector<Vector2f>& cur_avg_dir, int sp_rate)
{
  for (size_t i = 0; i < curves.size(); ++i)
  {
    Vector2f dir(0, 0);
    for (int j = sp_rate; j < curves[i].size(); j += sp_rate)
    {
      dir += Vector2f(curves[i][j].x - curves[i][j - sp_rate].x,
                      curves[i][j].y - curves[i][j - sp_rate].y);
    }
    dir.normalize();
    cur_avg_dir.push_back(dir);
  }
}

} // namespace CurvesUtility
