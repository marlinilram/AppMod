#ifndef ShapeCrest_H
#define ShapeCrest_H

#include "BasicHeader.h"
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

public:
  std::vector<Edge> crest_edges;

private:
  std::shared_ptr<Shape> shape;

private:
  ShapeCrest(const ShapeCrest&);
  void operator = (const ShapeCrest&);
};

#endif // !ShapeCrest_H
