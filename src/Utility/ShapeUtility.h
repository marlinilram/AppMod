#ifndef ShapeUtility_H
#define ShapeUtility_H

#include <memory>
#include <cv.h>

class Model;

namespace ShapeUtility
{
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);

  void computeNormalizedHeight(std::shared_ptr<Model> model);
  void computeMultiScaleSolidAngleCurvature(std::shared_ptr<Model> model);
  void computeDirectionalOcclusion(std::shared_ptr<Model> model);
  void computeSymmetry(std::shared_ptr<Model> model);
  void computeRMSCurvature(std::shared_ptr<Model> model);

  // an image tool
  void dilateImage(cv::Mat& mat, int max_n_dilate);
  void dilateImageMeetBoundary(cv::Mat& mat, cv::Mat& filled_mat, int i, int j);
}

#endif