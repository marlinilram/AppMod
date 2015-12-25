#ifndef ParaShape_H
#define ParaShape_H

#include <memory>
#include <set>
#include <vector>
#include "BasicHeader.h"

#include <cv.h>

class Shape;
class Model;
class KDTreeWrapper;

class ParaShape
{
public:
  ParaShape();
  ~ParaShape();

  void initUVKDTree();
  void initWithExtShape(std::shared_ptr<Model> model);

public:
  FaceList cut_face_list; // triplets which store the old vertex id of the faces
  STLVectori vertex_set; // vertex id mapping from new id to old id
  STLVectori boundary_loop; // boundary vertex id in cut shape
  std::shared_ptr<Shape> cut_shape;
  std::shared_ptr<KDTreeWrapper> kdTree_UV;
  std::shared_ptr<KDTreeWrapper> kdTree_UV_f; // now we store the center of each face here
  std::set<int> cut_faces; // face id in original model
  STLVectori face_set; // face id mapping from new id to old id
  std::vector<cv::Mat> feature_map;
  std::vector<cv::Mat> detail_map;
  int filled;
  float fill_ratio;

//private:
//  ParaShape(const ParaShape&);
//  void operator = (const ParaShape&);
};

#endif // !ParaShape_H
