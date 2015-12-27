#ifndef ShapeUtility_H
#define ShapeUtility_H

#include <memory>
#include <cv.h>
#include <set>
#include "BasicHeader.h"

class Model;
class Shape;
class KDTreeWrapper;
namespace LG {
class PolygonMesh;
}
class ParaShape;

namespace ShapeUtility
{
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3]);

  void computeBaryCentreCoord(Vector2f& pt, Vector2f& v0, Vector2f& v1, Vector2f& v2, float lambd[3]);

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
  
  // test if two points lie on two side of a line
  bool twoSideOfLine(Vector2f& line_start, Vector2f& line_end, Vector2f& pt0, Vector2f& pt1);

  // Given three colinear points p, q, r, the function checks if
  // point q lies on line segment 'pr'
  bool onSegment(const Vector2f& p, const Vector2f& q, const Vector2f& r);

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int orientation(const Vector2f& p, const Vector2f& q, const Vector2f& r);

  // The main function that returns true if line segment 'p1q1'
  // and 'p2q2' intersect.
  bool doIntersect(const Vector2f& p1, const Vector2f& q1, const Vector2f& p2, const Vector2f& q2);

  bool findClosesttUVriangleFace(std::vector<float>& pt, ParaShape* para_shape, std::vector<float>& bary_coord, int& f_id, std::vector<int>& v_ids);

  bool findClosestUVFace(std::vector<float>& pt, ParaShape* para_shape, std::vector<float>& bary_coord, int& f_id, std::vector<int>& v_ids);

  void saveParameterization(std::string file_path, std::shared_ptr<Shape> shape, std::string fname);
  
  /*void findFaceId(int& x, int& y, int& resolution, std::shared_ptr<KDTreeWrapper> kdTree, AdjList& adjFaces_list, std::shared_ptr<Shape> shape,
                  int& face_id, float* lambda, int& id1, int& id2, int& id3); */
}

#endif