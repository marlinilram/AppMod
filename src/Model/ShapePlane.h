#ifndef ShapePlane_H
#define ShapePlane_H

#include "BasicHeader.h"

#include <memory>
#include <vector>
#include <set>
#include <map>

class Shape;

class ShapePlane
{
public:
  ShapePlane() {};
  ~ShapePlane() {};

  void setShape(std::shared_ptr<Shape> _shape);
  void findFlats();
  std::vector<std::set<int> >& getFlats();
  void addTaggedPlane(int f_id);
  void clearTaggedPlanes();
  void getFlatSurfaceVertices(std::vector<std::vector<int> >& vertices, int tagged = 1);
  void setSymmetricPlane(double a, double b, double c, double d);
  void computePlaneCenter();
  std::vector<std::pair<Vector3f, Vector3f>>& getPlaneCenter();
  std::vector<std::pair<Vector3f, Vector3f>>& getOriginalPlaneCenter();
  std::vector<std::set<int>>& getFlatSurfaces();
  void findSymmetricPlane(int input_face_id, int& output_face_id);

private:
  void flatSurface(std::set<int>& surface, int f_id, float ref_normal[3]);

private:
  std::shared_ptr<Shape> shape;
  std::vector<std::set<int> > flat_surfaces; // face id here
  std::vector<bool> tagged_planes; // face id here
  std::map<int, int> face_plane_mapper;
  double symmetric_plane_a, symmetric_plane_b, symmetric_plane_c, symmetric_plane_d;
  Vector3f symmetric_plane_normal;
  std::vector<std::pair<Vector3f, Vector3f>> plane_center;
  std::vector<std::pair<Vector3f, Vector3f>> original_plane_center;

private:
  ShapePlane(const ShapePlane&);
  void operator = (const ShapePlane&);
};

#endif // !ShapePlane_H
