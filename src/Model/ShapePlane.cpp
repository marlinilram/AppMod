#include "ShapePlane.h"
#include "Shape.h"

#include "Colormap.h"

void ShapePlane::setShape(std::shared_ptr<Shape> _shape)
{
  shape = _shape;

  this->findFlats();
}

std::vector<std::set<int> >& ShapePlane::getFlats()
{
  return flat_surfaces;
}

void ShapePlane::findFlats()
{
  const FaceList& face_list = shape->getFaceList();
  flat_surfaces.clear();
  std::set<int> flat_surface;
  
  for (size_t i = 0; i < face_list.size() / 3; ++i)
  {
    bool already_visited = false;
    for (size_t j = 0; j < flat_surfaces.size(); ++j)
    {
      if (flat_surfaces[j].find(i) != flat_surfaces[j].end())
      {
        already_visited = true;
        break;
      }
    }

    if (already_visited)
    {
      continue;
    }
    else
    {
      flat_surface.clear();
      float ref_normal[3];
      ref_normal[0] = shape->getFaceNormal()[3 * i + 0];
      ref_normal[1] = shape->getFaceNormal()[3 * i + 1];
      ref_normal[2] = shape->getFaceNormal()[3 * i + 2];
      this->flatSurface(flat_surface, i, ref_normal);
      flat_surfaces.push_back(flat_surface);
    }
  }

  tagged_planes = std::vector<bool>(flat_surfaces.size(), false);
}

void ShapePlane::flatSurface(std::set<int>& surface, int f_id, float ref_normal[3])
{
  if (surface.find(f_id) != surface.end())
  {
    return;
  }

  float face_cos = 0.0f;
  face_cos += shape->getFaceNormal()[3 * f_id + 0] * ref_normal[0];
  face_cos += shape->getFaceNormal()[3 * f_id + 1] * ref_normal[1];
  face_cos += shape->getFaceNormal()[3 * f_id + 2] * ref_normal[2];
  if (face_cos < 0.95)
  {
    return;
  }

  surface.insert(f_id);
  const AdjList& adj_list = shape->getFaceAdjList();
  for (size_t i = 0; i < adj_list[f_id].size(); ++i)
  {
    this->flatSurface(surface, adj_list[f_id][i], ref_normal);
  }
}

void ShapePlane::clearTaggedPlanes()
{
  tagged_planes = std::vector<bool>(flat_surfaces.size(), false);
  STLVectorf color_list(shape->getColorList().size(), 0.5f);
  shape->setColorList(color_list);
}

void ShapePlane::addTaggedPlane(int f_id)
{
  //std::cout<<"check vis for flat face.\n";
  int flat_id = -1;
  for (size_t i = 0; i < flat_surfaces.size(); ++i)
  {
    if (flat_surfaces[i].find(f_id) != flat_surfaces[i].end())
    {
      flat_id = i;
      break;
    }
  }

  if (flat_id == -1)
  {
    // the face is not in a flat surface
    // do nothing
    //std::cout<<"face not in flat surface.\n";
    return;
  }

  if (tagged_planes[flat_id] == true)
  {
    // the flat surface has been tagged before
    // do nothing
    //std::cout<<"face has been tagged.\n";
    return;
  }

  tagged_planes[flat_id] = true;
  STLVectorf color_list = shape->getColorList();
  //for (size_t i = 0; i < flats.size(); ++i)
  //{
    QColor color = 
      qtJetColor(double(flat_id)/flat_surfaces.size());

    for (auto j : flat_surfaces[flat_id])
    {
      // j is face id
      int v0 = shape->getFaceList()[3 * j + 0];
      int v1 = shape->getFaceList()[3 * j + 1];
      int v2 = shape->getFaceList()[3 * j + 2];

      color_list[3 * v0 + 0] = color.redF();
      color_list[3 * v0 + 1] = color.greenF();
      color_list[3 * v0 + 2] = color.blueF();

      color_list[3 * v1 + 0] = color.redF();
      color_list[3 * v1 + 1] = color.greenF();
      color_list[3 * v1 + 2] = color.blueF();

      color_list[3 * v2 + 0] = color.redF();
      color_list[3 * v2 + 1] = color.greenF();
      color_list[3 * v2 + 2] = color.blueF();
    }
  //}
  shape->setColorList(color_list);
  //std::cout<<"set new color for flat face.\n";
}

void ShapePlane::getFlatSurfaceVertices(std::vector<std::vector<int> >& vertices)
{
  vertices.clear();
  std::vector<int> cur_vertices;
  for (size_t i = 0; i < flat_surfaces.size(); ++i)
  {
    if (tagged_planes[i])
    {
      cur_vertices.clear();
      for (auto j : flat_surfaces[i])
      {
        // j is face id
        cur_vertices.push_back(shape->getFaceList()[3 * j + 0]);
        cur_vertices.push_back(shape->getFaceList()[3 * j + 1]);
        cur_vertices.push_back(shape->getFaceList()[3 * j + 2]);
      }

      std::sort(cur_vertices.begin(), cur_vertices.end());
      cur_vertices.erase(std::unique(cur_vertices.begin(), cur_vertices.end()), cur_vertices.end());
      cur_vertices.shrink_to_fit();
      vertices.push_back(cur_vertices);
    }
  }
}