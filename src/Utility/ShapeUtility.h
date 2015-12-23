#ifndef ShapeUtility_H
#define ShapeUtility_H

#include <memory>
#include <cv.h>
#include <set>

class Model;
namespace LG {
class PolygonMesh;
}
class ParaShape;

namespace ShapeUtility
{
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);

  void computeNormalizedHeight(std::shared_ptr<Model> model);
  void computeMultiScaleSolidAngleCurvature(std::shared_ptr<Model> model);
  void computeDirectionalOcclusion(std::shared_ptr<Model> model);
  void computeSymmetry(std::shared_ptr<Model> model);
  void computeRMSCurvature(std::shared_ptr<Model> model);

  void getNRingFacesAroundVertex(LG::PolygonMesh* poly_mesh, std::set<int>& f_id, int v_id, int n_ring);

  // an image tool
  void dilateImage(cv::Mat& mat, int max_n_dilate);
  void dilateImageMeetBoundary(cv::Mat& mat, cv::Mat& filled_mat, int i, int j);

  void matToMesh(cv::Mat& mat, LG::PolygonMesh& mesh, std::shared_ptr<Model> shape_model);

  void getFaceInPatchByFaceInMesh(int f_id, std::vector<ParaShape>& patches, int& f_id_patch, int& patch_id);
}

#endif