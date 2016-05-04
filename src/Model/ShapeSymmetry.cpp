#include "ShapeSymmetry.h"

#include "PolygonMesh.h"
#include "KDTreeWrapper.h"

#include <fstream>

using namespace LG;

void ShapeSymmetry::setShape(std::shared_ptr<Shape> _shape, std::string ext_info_path)
{
  shape = _shape;
  file_path = ext_info_path;

  if (!this->loadSymmetryInfo(file_path))
  {
    this->setSymmetryPlane(0, 1, 0, 0);
    this->writeSymmetryPlane();
  }
}

bool ShapeSymmetry::loadSymmetryInfo(std::string fname)
{
  std::cout << std::endl << "Try to load symmetry information." << std::endl;
  std::ifstream inFile(fname + "/symmetry_info.txt");
  if (!inFile.is_open())
  {
    std::cout << "Not existed or failed to open." << std::endl;
    return false;
  }

  std::cout << "Loading symmetry_info.txt." << std::endl;
  std::string line_str;

  getline(inFile, line_str);
  std::stringstream line_parser(line_str);
  double a, b, c, d;
  line_parser >> a >> b >> c >> d;
  plane_coef.clear();
  plane_coef.resize(4, 0);
  plane_coef[0] = a;
  plane_coef[1] = b;
  plane_coef[2] = c;
  plane_coef[3] = d;
  std::cout << "symmetry plane: " << a << " " << b << " " << c << " " << d << std::endl;

  inFile.close();
  std::cout << "Loading symmetry_info.txt finished." << std::endl;
  return true;
}

void ShapeSymmetry::setSymmetryPlane(float a, float b, float c, float d)
{
  plane_coef.clear();
  plane_coef.push_back(a);
  plane_coef.push_back(b);
  plane_coef.push_back(c);
  plane_coef.push_back(d);
}

void ShapeSymmetry::writeSymmetryPlane()
{
  std::cout << std::endl << "Writing symmetry information." << std::endl;
  std::ofstream outFile(file_path + "/symmetry_info.txt");
  if (!outFile.is_open())
  {
    std::cout << "failed to open symmetry_info.txt file, return." << std::endl;
    return;
  }

  outFile << plane_coef[0] << " " << plane_coef[1] << " " << plane_coef[2] << " " << plane_coef[3] << std::endl;
  outFile.close();
  std::cout << "Writing symmetry_info.txt finished." << std::endl;
}

void ShapeSymmetry::computeSymmetryPair()
{
  PolygonMesh* mesh = shape->getPolygonMesh();
  std::vector<int> mapper(mesh->n_vertices(), -1);

  std::vector<float> pt(3, 0);
  int v_id = 0;
  float normalizer = std::sqrt(std::pow(plane_coef[0], 2) + std::pow(plane_coef[1], 2) + std::pow(plane_coef[2], 2));
  Vector3f plane_normal(plane_coef[0], plane_coef[1], plane_coef[2]);
  plane_normal.normalize();
  for (auto i : mesh->vertices())
  {
    Vec3 pos = mesh->position(i);
    float dist = (pos[0] * plane_coef[0]
      + pos[1] * plane_coef[1]
      + pos[2] * plane_coef[2]
      + plane_coef[3]) / normalizer;
    pos = pos - 2 * dist * plane_normal;
    pt[0] = pos[0];
    pt[1] = pos[1];
    pt[2] = pos[2];
    shape->getKDTree()->nearestPt(pt, v_id);
    mapper[i.idx()] = v_id;
  }

  sym_pairs.clear();
  std::vector<int> visited(mapper.size(), 0);
  for (size_t i = 0; i < mapper.size(); ++i)
  {
    if (visited[i] == 0 && mapper[i] != -1)
    {
      if (mapper[mapper[i]] == int(i))
      {
        sym_pairs.insert(STLPairii(int(i), mapper[i]));
        visited[mapper[i]] = 1;
      }
      visited[i] = 1;
    }
  }
}

void ShapeSymmetry::getSymmetryPairs(std::set<STLPairii>& paris)
{
  pairs = sym_pairs;
}
