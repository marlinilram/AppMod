#include "CurvesUtility.h"



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

  bool closestPtInSaliencyCurves(double2& tar_pt, std::vector<std::vector<double2> >& src_curves,
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
}
