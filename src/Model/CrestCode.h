#ifndef CrestCode_H
#define CrestCode_H

#include "BasicHeader.h"
#include <memory>

struct length_id
{
  int length;
  int id;
  bool operator < (const length_id& a) const
  {
    return a.length < length;
  }
};

class Shape;

class CrestCode
{
public:
  CrestCode();
  ~CrestCode();

  void setShape(std::shared_ptr<Shape> in_shape);
  void exportInputFile();
  std::vector<STLVectori>& getCrestLines();
  void computeCrestLines();
  void mergeCrestEdges();

private:
  std::shared_ptr<Shape> shape;
  std::vector<Edge> crest_edges;
  std::vector<STLVectori> crest_lines;
  std::vector<Vector3f> crest_lines_attributes;

};

#endif