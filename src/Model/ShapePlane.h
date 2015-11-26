#ifndef ShapePlane_H
#define ShapePlane_H

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

private:
  void flatSurface(std::set<int>& surface, int f_id, float ref_normal[3]);

private:
  std::shared_ptr<Shape> shape;
  std::vector<std::set<int> > flat_surfaces; // face id here
  std::vector<bool> tagged_planes; // face id here
  std::map<int, int> face_plane_mapper;

private:
  ShapePlane(const ShapePlane&);
  void operator = (const ShapePlane&);
};

#endif // !ShapePlane_H
