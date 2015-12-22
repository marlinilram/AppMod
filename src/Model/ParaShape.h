#ifndef ParaShape_H
#define ParaShape_H

#include <memory>
#include <set>
#include <vector>
#include "BasicHeader.h"

#include <cv.h>

class Shape;
class KDTreeWrapper;

class ParaShape
{
public:
  ParaShape();
  ~ParaShape();

public:
  FaceList cut_face_list; // triplets which store the old vertex id of the faces
  STLVectori vertex_set; // vertex id mapping from new id to old id
  STLVectori boundary_loop; // boundary vertex id in cut shape
  std::shared_ptr<Shape> cut_shape;
  std::shared_ptr<KDTreeWrapper> kdTree_UV;
  std::set<int> cut_faces; // face id in original model
  std::vector<cv::Mat> feature_map;
  std::vector<cv::Mat> detail_map;

private:
  ParaShape(const ParaShape&);
  void operator = (const ParaShape&);
};

#endif // !ParaShape_H
