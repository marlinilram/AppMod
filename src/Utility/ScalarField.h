#ifndef ScalarField_H
#define ScalarField_H

#include "BasicHeader.h"
#include "BasicDataType.h"

#include <memory>

class tele2d;
class FeatureGuided;

class ScalarField
{
public:
  ScalarField(int res, float rad);
  ~ScalarField();

  void init(int resolution);
  void initPara();
  void setSearchRad(float radius);
  void setTeleRegister(std::shared_ptr<tele2d> tele);

  void computeVariationMap();
  void computeMatchingMap(std::vector<double2>& ext_vector_field);
  void computeDistanceMap(FeatureGuided* feature_model);
  void updateDistanceMapGrad();
  void getDistanceMapGrad(double2& n_curve_pt, double& grad_x, double& grad_y);

  double curveIntegrate(std::vector<std::vector<double2> >& curves, FeatureGuided* feature_model);

public:
  int resolution;
  STLVectorf distance_map;
  STLVectorf variation_map;
  STLVectorf matching_map;
  STLVectorf distance_map_grad_x;
  STLVectorf distance_map_grad_y;

  float search_rad;
  float dist_attenuation;
  float para_a;
  float para_b;
  float para_w;

  // vis para
  float win_center;
  float win_width;

  std::shared_ptr<tele2d> tele_register;

private:
  ScalarField(const ScalarField&);
  void operator = (const ScalarField&);
};

#endif // !ScalarField_H
