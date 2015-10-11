#include "Model.h"
#include "Shape.h"
#include "ShapeCrest.h"
#include "KDTreeWrapper.h"
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
  std::cout << "Deleted a Model.\n";
}

Model::Model(const std::string path, const std::string name)
  : data_path(path), output_path(path), file_name(name)
{
  if (!loadOBJ(file_name, data_path))
  {
    std:: cerr << "Init model failed\n";
    return;
  }

  shape_crest.reset(new ShapeCrest());
  shape_crest->setShape(shape);
  shape_crest->computeCrestLinesPoints();

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

std::shared_ptr<ShapeCrest> Model::getShapeCrest()
{
  return shape_crest;
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

  float avg_pix_length = sqrt((vs[0] - vso[0]) * (vs[0] - vso[0]) + (vs[1] - vso[1]) * (vs[1] - vso[1]));
  std::cout << "Average Pixel Length: " << avg_pix_length << "\n";
  return avg_pix_length;
}

void Model::passCameraPara(float c_modelview[16], float c_projection[16], int c_viewport[4])
{

  m_modelview = Eigen::Map<Eigen::Matrix4f>(c_modelview, 4, 4);

  m_projection = Eigen::Map<Eigen::Matrix4f>(c_projection, 4, 4);

  m_inv_modelview_projection = (m_projection*m_modelview).inverse();

  m_viewport = Eigen::Map<Eigen::Vector4i>(c_viewport, 4, 1);

}

bool Model::getWorldCoord(Vector3f rimg_coord, Vector3f &w_coord)
{
  // passed in screen coordinates is homogeneous
  rimg_coord(2) = (float)1.0 / rimg_coord(2);
  float winx = rimg_coord(0)*rimg_coord(2);
  float winy = rimg_coord(1)*rimg_coord(2);

  float winz = z_img.at<float>((int)(winy + 0.5), (int)(winx + 0.5));

  float obj_coord[3];

  if(!getUnprojectPt(winx, winy, winz, obj_coord))
    return false;

  w_coord = Eigen::Map<Vector3f>(obj_coord, 3, 1);

  return true;
}

bool Model::getUnprojectPt(float winx, float winy, float winz, float object_coord[3])
{
  // unproject visible point in render image back to world

  //Transformation of normalized coordinates between -1 and 1

  Vector4f in((winx - (float)m_viewport(0)) / (float)m_viewport(2) * 2.0 - 1.0,
              (winy - (float)m_viewport(1)) / (float)m_viewport(3) * 2.0 - 1.0,
              2.0*winz - 1.0,
              (float)1.0);

  //Objects coordinates
  Vector4f out = m_inv_modelview_projection*in;
  if (out(3) == 0.0)
  {
    return false;
  }

  out(3) = (float)1.0 / out(3);
  object_coord[0] = out(0) * out(3);
  object_coord[1] = out(1) * out(3);
  object_coord[2] = out(2) * out(3);
  return true;
}

int Model::getClosestVertexId(float world_pos[3], int x, int y)
{
  // given x, y in rendered image return the closest vertex id

  // compute world coordinate
  Eigen::Vector3f xy_img((float)x, (float)y, 1.0);
  Eigen::Vector3f xyz_model;
  this->getWorldCoord(xy_img, xyz_model);

  // find closest id
  std::vector<float> query(3, 0.0);
  int v_id;
  query[0] = xyz_model[0];
  query[1] = xyz_model[1];
  query[2] = xyz_model[2];
  shape->getKDTree()->nearestPt(query, v_id);
  world_pos[0] = query[0];
  world_pos[1] = query[1];
  world_pos[2] = query[2];
  return v_id;
}

void Model::getCameraOri(float camera_ori[3])
{
  Vector4f cam_ori(0.0f, 0.0f, 0.0f, 1.0f);
  cam_ori = m_modelview.inverse() * cam_ori;

  camera_ori[0] = cam_ori[0] / cam_ori[3];
  camera_ori[1] = cam_ori[1] / cam_ori[3];
  camera_ori[2] = cam_ori[2] / cam_ori[3];
}

void Model::getProjRay(float proj_ray[3], int x, int y)
{
  Eigen::Vector4f in;
  in << (x - (float)m_viewport(0)) / (float)m_viewport(2) * 2.0 - 1.0,
    (y - (float)m_viewport(1)) / (float)m_viewport(3) * 2.0 - 1.0,
    1.0f,
    1.0f;

  // get the ray direction in camera coordinate
  Eigen::Vector4f out = m_projection.inverse() * in;
  Eigen::Vector4f cam_ori(0.0f, 0.0f, 0.0f, 1.0f);
  cam_ori = m_modelview.inverse() * cam_ori;
  out = m_modelview.inverse() * out;

  proj_ray[0] = out[0] / out[3] - cam_ori[0] / cam_ori[3];
  proj_ray[1] = out[1] / out[3] - cam_ori[1] / cam_ori[3];
  proj_ray[2] = out[2] / out[3] - cam_ori[2] / cam_ori[3];
}