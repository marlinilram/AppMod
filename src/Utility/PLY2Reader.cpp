#include "PLY2Reader.h"
#include "Shape.h"

#include <fstream>
#include <sstream>
#include <iostream>

bool PLY2Reader::loadPLY2Mesh(std::shared_ptr<Shape> shape, std::string fname)
{
  std::ifstream f_handler(fname);
  if (!f_handler)
  {
    std::cout << "Open file failed.\n";
    return false;
  }

  std::string line;
  std::stringstream ss;

  int num_vertex = 0;
  int num_face = 0;
  VertexList vertexList;
  FaceList faceList;
  STLVectorf UVList;

  getline(f_handler, line);
  ss.str(line);
  ss >> num_vertex;
  getline(f_handler, line);
  ss.str(line);
  ss >> num_face;

  for (int i = 0; i < num_vertex; ++i)
  {
    float v;
    for (int j = 0; j < 3; ++j)
    {
      getline(f_handler, line);
      ss.str(line);
      ss >> v;
      vertexList.push_back(v);
    }
  }

  for (int i = 0; i < num_face; ++i)
  {
    int f;
    for (int j = 0; j < 3; ++j)
    {
      getline(f_handler, line);
      ss.str(line);
      ss >> f;
      faceList.push_back(f);
    }
  }

  shape->init(vertexList, faceList, UVList);
  return true;
}