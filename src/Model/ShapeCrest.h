#ifndef ShapeCrest_H
#define ShapeCrest_H

#include "BasicHeader.h"
#include "CrestCode.h"
#include <memory>

class Shape;

class ShapeCrest
{
public:
  ShapeCrest();
  ~ShapeCrest();

  void setShape(std::shared_ptr<Shape> in_shape);
  void buildCandidates();

  const std::vector<Edge>& getCrestEdge();
  std::vector<std::vector<Vector3f>>& getCrestLinesPoints(){ return crestLinesPoints; };
  std::vector<std::vector<int>>& getCrestLinesPointsId(){ return crestLinesPointsId; };
  void computeCrestLinesPoints();

public:
  std::vector<Edge> crest_edges;
  
private:
  std::shared_ptr<Shape> shape;
  std::shared_ptr<CrestCode> crest;
  std::vector<std::vector<int>> crestLinesPointsId;
  std::vector<std::vector<Vector3f>> crestLinesPoints;

private:
  ShapeCrest(const ShapeCrest&);
  void operator = (const ShapeCrest&);
};

#endif // !ShapeCrest_H
