#ifndef ScalarField_H
#define ScalarField_H

#include "BasicHeader.h"
#include "BasicDataType.h"

#include <memory>

class tele2d;

class ScalarField
{
public:
  ScalarField(int res, float rad);
  ~ScalarField();

  void init(int resolution);
  void setSearchRad(float radius);
  void setTeleRegister(std::shared_ptr<tele2d> tele);

  void computeVariationMap();
  void computeMatchingMap(std::vector<double2>& ext_vector_field);

public:
  int resolution;
  STLVectorf distance_map;
  STLVectorf variation_map;
  STLVectorf matching_map;

  float search_rad;

  std::shared_ptr<tele2d> tele_register;

private:
  ScalarField(const ScalarField&);
  void operator = (const ScalarField&);
};

#endif // !ScalarField_H
