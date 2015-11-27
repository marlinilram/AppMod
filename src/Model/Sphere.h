#ifndef Sphere_H
#define Sphere_H

#include "PolygonMesh.h"
#include <memory>

using namespace LG;

class Sphere
{
public:
  Sphere() : center(Eigen::Vector3f(0,0,0)), radius(1.0f) { Init(center, radius, 30, 15); }
  ~Sphere() {}
  Sphere(Eigen::Vector3f _center, float _radius, int numSlices, int numStacks) : center(_center), radius(_radius)
  {
    Init(center, radius, numSlices, numStacks);
  }

  void Init(Eigen::Vector3f _center, float _radius, int numSlices, int numStacks);
  LG::PolygonMesh* getPolygonMesh() { return poly_mesh.get(); };

private:
  void computeSHCoeffs();

public:
  Eigen::Matrix4f model_view;

private:
  Eigen::Vector3f center;
  float radius;

  std::shared_ptr<PolygonMesh> poly_mesh;
};


#endif // !Sphere_H
