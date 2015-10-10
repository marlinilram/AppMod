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
}
