#include "ShapePlane.h"
#include "Shape.h"
#include "Bound.h"
#include "Colormap.h"
#include <fstream>

void ShapePlane::setShape(std::shared_ptr<Shape> _shape, std::string ext_info_path)
{
  shape = _shape;

  if (!this->loadExtPlaneInfo(ext_info_path))
  {
    this->findFlats();
    this->writeExtPlaneInfo(ext_info_path);
  }

  this->setSymmetricPlane(0, 0, 1, 0);
  this->computePlaneCenter();
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
  //ref_normal[0] = shape->getFaceNormal()[3 * f_id + 0];
  //ref_normal[1] = shape->getFaceNormal()[3 * f_id + 1];
  //ref_normal[2] = shape->getFaceNormal()[3 * f_id + 2];
  if (face_cos < 0.8)
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
  STLVectorf face_color_list = shape->getFaceColorList();
  //for (size_t i = 0; i < flats.size(); ++i)
  //{
    QColor color = 
      qtJetColor(double(flat_id)/flat_surfaces.size());

    for (auto j : flat_surfaces[flat_id])
    {
      // j is face id
      //int v0 = shape->getFaceList()[3 * j + 0];
      //int v1 = shape->getFaceList()[3 * j + 1];
      //int v2 = shape->getFaceList()[3 * j + 2];

      //color_list[3 * v0 + 0] = color.redF();
      //color_list[3 * v0 + 1] = color.greenF();
      //color_list[3 * v0 + 2] = color.blueF();

      //color_list[3 * v1 + 0] = color.redF();
      //color_list[3 * v1 + 1] = color.greenF();
      //color_list[3 * v1 + 2] = color.blueF();

      //color_list[3 * v2 + 0] = color.redF();
      //color_list[3 * v2 + 1] = color.greenF();
      //color_list[3 * v2 + 2] = color.blueF();
      face_color_list[3 * j + 0] = color.redF();
      face_color_list[3 * j + 1] = color.greenF();
      face_color_list[3 * j + 2] = color.blueF();
    }
  //}
  //shape->setColorList(color_list);
  shape->setFaceColorList(face_color_list);
  //std::cout<<"set new color for flat face.\n";
}

void ShapePlane::getFlatSurfaceVertices(std::vector<std::vector<int> >& vertices, int tagged)
{
  vertices.clear();
  std::vector<int> cur_vertices;
  for (size_t i = 0; i < flat_surfaces.size(); ++i)
  {
    if (tagged_planes[i] || (tagged == 0))
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

void ShapePlane::setSymmetricPlane(double a, double b, double c, double d)
{
  symmetric_plane_a = a;
  symmetric_plane_b = b;
  symmetric_plane_c = c;
  symmetric_plane_d = d;
  symmetric_plane_normal << a, b, c;
  symmetric_plane_normal.normalized();
}

void ShapePlane::computePlaneCenter()
{
  plane_center.clear();
  original_plane_center.clear();
  FaceList face_list = shape->getFaceList();
  VertexList vertex_list = shape->getVertexList();
  NormalList normal_list = shape->getNormalList();
  NormalList face_normal = shape->getFaceNormal();
  Bound* bounding = shape->getBoundbox();
  for(size_t i = 0; i < flat_surfaces.size(); i ++)
  {
    Vector3f center_position, center_normal;
    center_position << 0, 0, 0;
    center_normal << 0, 0, 0;
    double x_max = std::numeric_limits<double>::min(), 
           x_min = std::numeric_limits<double>::max(),
           y_max = std::numeric_limits<double>::min(), 
           y_min = std::numeric_limits<double>::max(), 
           z_max = std::numeric_limits<double>::min(), 
           z_min = std::numeric_limits<double>::max();
    for(auto j : flat_surfaces[i])
    {
      Vector3f v1, v2, v3, n1, n2, n3;
      v1 << vertex_list[3 * face_list[3 * j]], vertex_list[3 * face_list[3 * j] + 1], vertex_list[3 * face_list[3 * j] + 2];
      v2 << vertex_list[3 * face_list[3 * j + 1]], vertex_list[3 * face_list[3 * j + 1] + 1], vertex_list[3 * face_list[3 * j + 1] + 2];
      v3 << vertex_list[3 * face_list[3 * j + 2]], vertex_list[3 * face_list[3 * j + 2] + 1], vertex_list[3 * face_list[3 * j + 2] + 2];
      /*n1 << normal_list[3 * face_list[3 * j]], normal_list[3 * face_list[3 * j] + 1], normal_list[3 * face_list[3 * j] + 2];
      n2 << normal_list[3 * face_list[3 * j + 1]], normal_list[3 * face_list[3 * j + 1] + 1], normal_list[3 * face_list[3 * j + 1] + 2];
      n3 << normal_list[3 * face_list[3 * j + 2]], normal_list[3 * face_list[3 * j + 2] + 1], normal_list[3 * face_list[3 * j + 2] + 2];*/
      center_position += (v1 + v2 + v3) / 3;
      /*center_normal += (n1 + n2 + n3) / 3;*/
      //for(int k = 0; k < 3; k ++)
      //{
      //  if(vertex_list[3 * face_list[3 * j + k]] > x_max)
      //  {
      //    x_max = vertex_list[3 * face_list[3 * j + k]];
      //  }
      //  if(vertex_list[3 * face_list[3 * j + k]] < x_min)
      //  {
      //    x_min = vertex_list[3 * face_list[3 * j + k]];
      //  }
      //  if(vertex_list[3 * face_list[3 * j + k] + 1] > y_max)
      //  {
      //    y_max = vertex_list[3 * face_list[3 * j + k] + 1];
      //  }
      //  if(vertex_list[3 * face_list[3 * j + k] + 1] < y_min)
      //  {
      //    y_min = vertex_list[3 * face_list[3 * j + k] + 1];
      //  }
      //  if(vertex_list[3 * face_list[3 * j + k] + 2] > z_max)
      //  {
      //    z_max = vertex_list[3 * face_list[3 * j + k] + 2];
      //  }
      //  if(vertex_list[3 * face_list[3 * j + k] + 2] < z_min)
      //  {
      //    z_min = vertex_list[3 * face_list[3 * j + k] + 2];
      //  }
      //}
      center_normal += Vector3f(face_normal[3 * j], face_normal[3 * j + 1], face_normal[3 * j + 2]);
    }
    center_position /= flat_surfaces[i].size();
    //center_position << (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2;
    center_normal /= flat_surfaces[i].size();
    center_normal.normalized();
    original_plane_center.push_back(std::pair<Vector3f, Vector3f>(center_position, center_normal));

    double distance = (symmetric_plane_a * center_position(0) + symmetric_plane_b * center_position(1) + symmetric_plane_c * center_position(2) + symmetric_plane_d)
      / sqrt(symmetric_plane_a * symmetric_plane_a + symmetric_plane_b * symmetric_plane_b + symmetric_plane_c * symmetric_plane_c);
    center_position += (-distance) * symmetric_plane_normal;
    if(center_normal.dot(symmetric_plane_normal) < 0)
    {
      distance = 2 * fabs(center_normal.dot(symmetric_plane_normal));
      center_normal += distance * symmetric_plane_normal;
      center_normal.normalized();
    }
    /*center_position << (center_position(0) - bounding->minX) / (bounding->maxX - bounding->minX), 
                       (center_position(1) - bounding->minY) / (bounding->maxY - bounding->minY),
                       (center_position(2) - bounding->minZ) / (bounding->maxZ - bounding->minZ);*/
    plane_center.push_back(std::pair<Vector3f, Vector3f>(center_position, center_normal));
  }
}

bool ShapePlane::loadExtPlaneInfo(std::string fname)
{
  std::cout << std::endl << "Try to load plane information." << std::endl;
  std::ifstream inFile(fname + "/plane_info.txt");
  if (!inFile.is_open())
  {
    std::cout << "Not existed or failed to open." << std::endl;
    return false;
  }

  std::cout << "Loading plane_info.txt." << std::endl;
  std::string line_str;

  getline(inFile, line_str);
  std::stringstream line_parser(line_str);
  int n_plane ;
  line_parser >> n_plane;
  std::cout << "Total " << n_plane << " patches." << std::endl;
  flat_surfaces.clear();
  flat_surfaces.resize(n_plane, std::set<int>());

  while (getline(inFile, line_str))
  {
    if (line_str.empty()) continue;
    std::stringstream plane_parser(line_str);
    int plane_id;
    int num_face;
    plane_parser >> plane_id >> num_face;
    for (int i = 0; i < num_face; ++i)
    {
      getline(inFile, line_str);
      std::stringstream face_parser(line_str);
      int face_id;
      face_parser >> face_id;
      flat_surfaces[plane_id].insert(face_id);
    }
  }
  inFile.close();
  std::cout << "Loading plane_info.txt finished." << std::endl;
  return true;
}

void ShapePlane::writeExtPlaneInfo(std::string fname)
{
  std::cout << std::endl << "Writing plane information." << std::endl;
  std::ofstream outFile(fname + "/plane_info.txt");
  if (!outFile.is_open())
  {
    std::cout << "failed to open plane_info.txt file, return." << std::endl;
    return;
  }

  outFile << flat_surfaces.size() << std::endl;
  for (size_t i = 0; i < flat_surfaces.size(); ++i)
  {
    outFile << i << " " << flat_surfaces[i].size() << std::endl;
    for (auto j : flat_surfaces[i])
    {
      outFile << j << std::endl;
    }
  }
  outFile.close();
  std::cout << "Writing plane_info.txt finished." << std::endl;
}

std::vector<std::pair<Vector3f, Vector3f>>& ShapePlane::getPlaneCenter()
{
  return this->plane_center;
}

std::vector<std::pair<Vector3f, Vector3f>>& ShapePlane::getOriginalPlaneCenter()
{
  return this->original_plane_center;
}

std::vector<std::set<int>>& ShapePlane::getFlatSurfaces()
{
  return this->flat_surfaces;
}

void ShapePlane::findSymmetricPlane(int input_face_id, int& output_face_id, std::vector<int>& candidate)
{
  double min = std::numeric_limits<double>::max();
  for(size_t i = 0; i < plane_center.size(); i ++)
  {
    if(i != input_face_id && candidate[i] != 0)
    {
      double distance = sqrt(pow(plane_center[i].first(0) - plane_center[input_face_id].first(0), 2)
                           + pow(plane_center[i].first(1) - plane_center[input_face_id].first(1), 2)
                           + pow(plane_center[i].first(2) - plane_center[input_face_id].first(2), 2)
                           + pow(plane_center[i].second(0) - plane_center[input_face_id].second(0), 2)
                           + pow(plane_center[i].second(1) - plane_center[input_face_id].second(1), 2)
                           + pow(plane_center[i].second(2) - plane_center[input_face_id].second(2), 2));
      if(distance < min)
      {
        min = distance;
        output_face_id = i;
      }
    }
  }
}