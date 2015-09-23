#include "Model.h"
#include "Shape.h"
#include "Bound.h"
#include "tiny_obj_loader.h"
#include "obj_writer.h"
#include <time.h>
#include <QDir>

Model::Model()
{

}

Model::~Model()
{

}

Model::Model(const std::string path, const std::string name)
  : data_path(path), output_path(path), file_name(name)
{
  if (!loadOBJ(file_name, data_path))
  {
    std:: cerr << "Init model failed\n";
    return;
  }

  // make an output path
  char time_postfix[50];
  time_t current_time = time(NULL);
  strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
  output_path = data_path + "/output" + time_postfix;
  QDir dir;
  dir.mkpath(QString(output_path.c_str()));
}

bool Model::loadOBJ(const std::string name, const std::string path)
{
  std::cout << "Reading OBJ file " << name << " from disk.\n";

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err = tinyobj::LoadObj(shapes, materials, (path + "/" + name).c_str(), nullptr);
  if (!err.empty())
  {
    std::cerr << err << std::endl;
    return false;
  }

  shape.reset(new Shape());
  shape->init(shapes[0].mesh.positions, shapes[0].mesh.indices, shapes[0].mesh.texcoords);

  return true;
}

void Model::exportOBJ(int cur_iter)
{
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  tinyobj::shape_t obj_shape;

  obj_shape.mesh.positions = shape->getVertexList();
  obj_shape.mesh.indices = shape->getFaceList();
  obj_shape.mesh.texcoords = shape->getUVCoord();

  shapes.push_back(obj_shape);

  char time_postfix[50];
  time_t current_time = time(NULL);
  strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
  std::string file_time_postfix = time_postfix;

  std::string output_name = output_path + "/coarse_output" + file_time_postfix + ".obj";
  WriteObj(output_name, shapes, materials);
}

Bound* Model::getBoundBox()
{
  return shape->getBoundbox();
}

std::shared_ptr<Shape> Model::getShape()
{
  return shape;
}

std::string Model::getDataPath()
{
  return data_path;
}

std::string Model::getOutputPath()
{
  return output_path;
}

float Model::getModelAvgEdgeLength()
{
  return shape->avgEdgeLength();
}