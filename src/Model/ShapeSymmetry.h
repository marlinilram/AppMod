#ifndef ShapeSymmetry_H

#include "Shape.h"
#include "BasicHeader.h"

class ShapeSymmetry
{
public:
  ShapeSymmetry() {};
  ~ShapeSymmetry() {};

  void setShape(std::shared_ptr<Shape> _shape, std::string ext_info_path);
  bool loadSymmetryInfo(std::string fname);
  void setSymmetryPlane(float a, float b, float c, float d);
  void writeSymmetryPlane();

  void computeSymmetryPair();
  void getSymmetryPairs(std::set<STLPairii>& pairs);
  void getSymmetryPlane(std::vector<float>& plane);

private:
  std::shared_ptr<Shape> shape;
  std::vector<float> plane_coef;
  std::set<STLPairii> sym_pairs;

  std::string file_path;

private:
  ShapeSymmetry(const ShapeSymmetry&);
  void operator = (const ShapeSymmetry&);
};

#endif // !ShapeSymmetry_H
