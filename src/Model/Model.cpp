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
  // return the pixel length of average edge length of the model
  float avg_edge_length = shape->avgEdgeLength();

  float center[3];
  shape->getBoundbox()->getCenter(center);

  Vector4f v(center[0], center[1], center[2], 1.0);
  v = m_modelview * v;
  Vector4f vo = v;
  v[0] = v[0] + avg_edge_length;
  Vector4f vs = m_projection * v;
  Vector4f vso = m_projection * vo;
  vs[0] /= vs[3];
  vs[1] /= vs[3];
  vs[2] /= vs[3];
  vs[0] = vs[0] * 0.5 + 0.5;
  vs[1] = vs[1] * 0.5 + 0.5;
  vs[2] = vs[2] * 0.5 + 0.5;
  vs[0] = vs[0] * m_viewport[2] + m_viewport[0];
  vs[1] = vs[1] * m_viewport[3] + m_viewport[1];
  vso[0] /= vso[3];
  vso[1] /= vso[3];
  vso[2] /= vso[3];
  vso[0] = vso[0] * 0.5 + 0.5;
  vso[1] = vso[1] * 0.5 + 0.5;
  vso[2] = vso[2] * 0.5 + 0.5;
  vso[0] = vso[0] * m_viewport[2] + m_viewport[0];
  vso[1] = vso[1] * m_viewport[3] + m_viewport[1];

  return sqrt((vs[0] - vso[0]) * (vs[0] - vso[0]) + (vs[1] - vso[1]) * (vs[1] - vso[1]));
}

void Model::passCameraPara(float c_modelview[16], float c_projection[16], int c_viewport[4])
{

  m_modelview = Eigen::Map<Eigen::Matrix4f>(c_modelview, 4, 4);

  m_projection = Eigen::Map<Eigen::Matrix4f>(c_projection, 4, 4);

  m_inv_modelview_projection = (m_projection*m_modelview).inverse();

  m_viewport = Eigen::Map<Eigen::Vector4i>(c_viewport, 4, 1);

}