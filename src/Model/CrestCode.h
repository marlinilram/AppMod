#ifndef CrestCode_H
#define CrestCode_H

#include "BasicHeader.h"
#include <memory>

class Shape;

class CrestCode
{
public:
  CrestCode();
  ~CrestCode();

  void exportInputFile(std::shared_ptr<Shape> shape);
  void getCrestLinesPoints(std::vector<std::vector<Vector3f>>& crestLinesPoints);

};

#endif