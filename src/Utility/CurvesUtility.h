#ifndef CurvesUtility_H
#define CurvesUtility_H

#include <cv.h>
#include "BasicDataType.h"
#include "BasicHeader.h"


namespace CurvesUtility
{
  bool isBoundary(cv::Mat& primitive_img, int x, int y);

  void getBoundaryImg(cv::Mat& outputImg, cv::Mat& primitive_id_img);

  double distInScalarMap(STLVectorf& scalar_map, int dim, double2& src_pt, double2& tar_pt, double threshold);

  bool closestPtInCurves(double2& tar_pt, std::vector<std::vector<double2> >& src_curves,
    int& src_i, int& src_j, double& dis, 
    STLVectorf& scalar_map = STLVectorf(0), int dim = 0, double threshold = 0.0);

  void mergeShapeEdges(std::vector<Edge>& edges, std::vector<STLVectori>& lines);
}


#endif // !CurvesUtility_H
