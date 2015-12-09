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

bool closestPtFromSaliencyCurves(double2& tar_pt, std::vector<std::vector<double2> >& src_curves,
                               int& src_i, int& src_j, double& dis, std::vector<double>& paras);

bool closestPtInSaliencyCurves(double2& src_pt, std::vector<std::vector<double2> >& tar_curves, std::vector<std::vector<double> >& tar_sl,
                                      int& tar_i, int& tar_j, double& dis, std::vector<double>& paras);

void mergeShapeEdges(std::vector<Edge>& edges, std::vector<STLVectori>& lines);

// move from FeatureGuided static
CURVES ReorganizeCurves(CURVES& curves, float sp_rate);
CURVES SplitCurve(std::vector<double2> curve);
std::vector<double2> ConnectCurves(
  std::vector<double2> curve0, std::vector<double2> curve1,
  int endtag0, int endtag1);
double CurveLength(std::vector<double2>& curve);
double PartCurveLength(std::vector<double2>& curve, int sp_id);
void NormalizedCurves(CURVES& curves);
void NormalizedCurves(CURVES& curves, double2 translate, double scale);
void NormalizedCurve(CURVE& curve, double2 translate, double scale);
void DenormalizedCurve(CURVE& curve, double2 translate, double scale);
void NormalizePara(CURVES& curves, double2& translate, double& scale);

void CurveSpTwoSideDist(std::vector<std::vector<double2> >& edges_sp_len, CURVES& curves);
void CurveSpSaliency(std::vector<std::vector<double> >& edges_sp_sl, CURVES& curves, cv::Mat& saliency_img, std::vector<double>& edges_average_sp_sl);

CURVE SmoothCurve(CURVE& curve_in, int win_size = 3);
CURVES SmoothCurves(CURVES& curves_in, int win_size = 3);
std::vector<int> DetectBreakPoint(CURVE& curve_in, int win_size = 3, double th = 0.5);
std::vector<std::vector<int> > DetectBreakPointAll(CURVES& curves_in, int win_size = 3, double th = 0.5);
CURVES BreakCurves(CURVES& curves, std::vector<std::vector<int> >& bk_points);

double2 ClosestConnection(double2& end_0_0, double2& end_0_1, double2& end_1_0, double2& end_1_1);

void CurvesAvgDir(CURVES& curves, std::vector<Vector2f>& cur_avg_dir, int sp_rate);
}


#endif // !CurvesUtility_H
