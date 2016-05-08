#include "AppearanceModel.h"

#include "Shape.h"
#include "PolygonMesh.h"
#include "ImageUtility.h"
#include "ShapeUtility.h"
#include "tiny_obj_loader.h"

using namespace LG;

void AppearanceModel::importAppMod(std::string file_name_, std::string file_path_)
{
  file_name = file_name_;
  file_path = file_path_;
  cv::FileStorage fs(file_path + '/' + file_name, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    std::cout << "Cannot open file: " << file_path + '/' + file_name << std::endl;
  }

  // load the appearance model

  // 1. mesh name
  std::cout << std::endl << "*** Reading Mesh File: " << mesh_file_name << " ***" << std::endl;
  fs["meshFileName"] >> mesh_file_name;
  base_mesh = std::make_unique<PolygonMesh>();
  if (!ShapeUtility::loadPolyMesh(base_mesh.get(), file_path + "/" + mesh_file_name))
  {
    std::cout << "Cannot open mesh file: " << file_path + '/' + mesh_file_name << std::endl;
  }

  // 2. vertex feature
  std::cout << std::endl << "*** Reading Mesh Features ***" << std::endl;
  this->readMeshFeatures(fs, base_mesh.get());

  // 3. load d0 feature maps
  std::cout << std::endl << "*** Reading D0 Features ***" << std::endl;
  this->readMaps(fs, d0_feature_maps, "d0Feature");
  // 4. d0 detail maps (displacement vector, 3 channels)
  // notice: if save the displacement vector in parametric mesh, voting is different
  std::cout << std::endl << "*** Reading D0 Details ***" << std::endl;
  this->readMaps(fs, d0_detail_maps, "d0Detail");
  // 5. d1 feature maps
  std::cout << std::endl << "*** Reading D1 Features ***" << std::endl;
  this->readMaps(fs, d1_feature_maps, "d1Feature");
  // 6. d1 detail maps (displacement and rgb, 4 channels)
  std::cout << std::endl << "*** Reading D1 Details ***" << std::endl;
  this->readMaps(fs, d1_detail_maps, "d1Detail");
  // 7.
  resolution = (int)fs["resolution"];
  // 8. cca mat
  std::cout << std::endl << "*** Reading CCA Matrices ***" << std::endl;
  fs["CCAMat"] >> cca_mat;

  // 9. cca min
  this->readVector(fs, cca_min, "cca_min");

  // 10. cca max
  this->readVector(fs, cca_max, "cca_max");

  // camera info
  std::cout << std::endl << "*** Reading Camera Information ***" << std::endl;
  this->readCameraInfo(fs);

  // photo
  std::cout << std::endl << "*** Reading Photo ***" << std::endl;
  fs["photo"] >> photo;
  fs["primitive_ID"] >> primitive_ID;

  fs.release();

  std::cout << std::endl << "*** Import Appearance Model Finished ***" << std::endl;
}

void AppearanceModel::readMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh)
{
  cv::FileNode fnode = fs["meshFeature"];
  cv::FileNodeIterator it = fnode.begin(), it_end = fnode.end();
  for (; it != it_end; ++it)
  {
    std::string feature_name;
    (*it)["name"] >> feature_name;
    int dim = (int)(*it)["dim"];
    std::vector<float> vertex_feature_list;
    (*it)["data"] >> vertex_feature_list;

    if (dim * mesh->n_vertices() != (int)vertex_feature_list.size())
    {
      std::cout << "read feature data error, dimensions don't match" << std::endl;
    }
    if (dim == 3)
    {
      PolygonMesh::Vertex_attribute<Vec3> feature_data = mesh->vertex_attribute<Vec3>(feature_name);
      for (int i = 0; i < mesh->n_vertices(); ++i)
      {
        feature_data[PolygonMesh::Vertex(i)][0] = vertex_feature_list[3 * i + 0];
        feature_data[PolygonMesh::Vertex(i)][1] = vertex_feature_list[3 * i + 1];
        feature_data[PolygonMesh::Vertex(i)][2] = vertex_feature_list[3 * i + 2];
      }
    }
  }
}

void AppearanceModel::readMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name)
{
  cv::FileNode fnode = fs[map_name];
  maps.clear();

  cv::FileNode maps_node = fnode["maps"];
  cv::FileNodeIterator it = maps_node.begin(), it_end = maps_node.end();
  int dim = (int)fnode["Dimension"];
  maps.resize(dim);
  for (int i = 0; it != it_end; ++it, ++i)
  {
    (*it)["map"] >> maps[i];
  }
}

void AppearanceModel::exportAppMod(std::string file_name_, std::string file_path_)
{
  file_name = file_name_;
  file_path = file_path_;
  cv::FileStorage fs(file_path + '/' + file_name, cv::FileStorage::WRITE);

  if (!fs.isOpened())
  {
    std::cout << "Cannot open file: " << file_path + '/' + file_name << std::endl;
  }

  // 1. mesh name
  fs << "meshFileName" << mesh_file_name;
  ShapeUtility::savePolyMesh(base_mesh.get(), file_path + "/" + mesh_file_name);

  // 2. vertex feature
  this->writeMeshFeatures(fs, base_mesh.get());

  // 3. d0 feature maps
  this->writeMaps(fs, d0_feature_maps, "d0Feature");

  // 4. d0 detail maps (displacement vector, 3 channels)
  // notice: if save the displacement vector in parametric mesh, voting is different
  this->writeMaps(fs, d0_detail_maps, "d0Detail");

  // 5. d1 feature maps
  this->writeMaps(fs, d1_feature_maps, "d1Feature");

  // 6. d1 detail maps (displacement and rgb, 4 channels)
  this->writeMaps(fs, d1_detail_maps, "d1Detail");

  // 7. resolution
  fs << "resolution" << resolution;

  // 8. cca mat
  fs << "CCAMat" << cca_mat;

  // 9. cca min
  this->writeVector(fs, cca_min, "cca_min");

  // 10. cca max
  this->writeVector(fs, cca_max, "cca_max");

  // camera info
  this->writeCameraInfo(fs);

  // photo
  fs << "photo" << photo;

  fs.release();
}

void AppearanceModel::writeMeshFeatures(cv::FileStorage& fs, LG::PolygonMesh* mesh)
{
  PolygonMesh::Vertex_attribute<Vec3> local_transform = mesh->vertex_attribute<Vec3>("v:local_transform");
  // ignore normal now, will be computed when loading
  // PolygonMesh::Vertex_attribute<Vec3> v_normals = mesh->vertex_attribute<Vec3>("v:normal");
  PolygonMesh::Vertex_attribute<Vec3> v_tangents = mesh->vertex_attribute<Vec3>("v:tangent");

  fs << "meshFeature" << "[:";

  std::vector<float> vertex_feature_list(3 * mesh->n_vertices(), 0.0);
  for (auto vit : mesh->vertices())
  {
    vertex_feature_list[3 * vit.idx() + 0] = local_transform[vit][0];
    vertex_feature_list[3 * vit.idx() + 1] = local_transform[vit][1];
    vertex_feature_list[3 * vit.idx() + 2] = local_transform[vit][2];
  }
  fs << "{:" << "name" << "v:local_transform" << "dim" << 3 << "data" << "[:";
  for (auto i : vertex_feature_list)
  {
    fs << i;
  }
  fs << "]" << "}";

  for (auto vit : mesh->vertices())
  {
    vertex_feature_list[3 * vit.idx() + 0] = v_tangents[vit][0];
    vertex_feature_list[3 * vit.idx() + 1] = v_tangents[vit][1];
    vertex_feature_list[3 * vit.idx() + 2] = v_tangents[vit][2];
  }
  fs << "{:" << "name" << "v:tangent" << "dim" << 3 << "data" << "[:";
  for (auto i : vertex_feature_list)
  {
    fs << i;
  }
  fs << "]" << "}";

  // TODO: more features ?

  fs << "]";
}

void AppearanceModel::writeMaps(cv::FileStorage& fs, std::vector<cv::Mat>& maps, std::string map_name)
{
  fs << map_name << "{:";
  int dim = (int)maps.size();
  fs << "Dimension" << dim << "maps" << "[:";
  for (int i = 0; i < dim; ++i)
  {
    fs << "{:" << "map" << maps[i] << "}";
  }
  fs << "]" << "}";
}

void AppearanceModel::setD0Features(std::vector<cv::Mat>& feature_maps)
{
  this->deepCopyCvMatVector(feature_maps, d0_feature_maps);
}

void AppearanceModel::setD0Details(std::vector<cv::Mat>& detail_maps)
{
  this->deepCopyCvMatVector(detail_maps, d0_detail_maps);
}

void AppearanceModel::setD1Features(std::vector<cv::Mat>& feature_maps)
{
  this->deepCopyCvMatVector(feature_maps, d1_feature_maps);
}

void AppearanceModel::setD1Details(std::vector<cv::Mat>& detail_maps)
{
  this->deepCopyCvMatVector(detail_maps, d1_detail_maps);
}

void AppearanceModel::getD0Features(std::vector<cv::Mat>& feature_maps)
{
  this->deepCopyCvMatVector(d0_feature_maps, feature_maps);
}

void AppearanceModel::getD0Details(std::vector<cv::Mat>& detail_maps)
{
  this->deepCopyCvMatVector(d0_detail_maps, detail_maps);
}

void AppearanceModel::getD1Features(std::vector<cv::Mat>& feature_maps)
{
  this->deepCopyCvMatVector(d1_feature_maps, feature_maps);
}

void AppearanceModel::getD1Details(std::vector<cv::Mat>& detail_maps)
{
  this->deepCopyCvMatVector(d1_detail_maps, detail_maps);
}

void AppearanceModel::deepCopyCvMatVector(std::vector<cv::Mat>& vector_in, std::vector<cv::Mat>& vector_out)
{
  vector_out.clear();
  for (size_t i = 0; i < vector_in.size(); ++i)
  {
    vector_out.push_back(vector_in[i].clone());
  }
}

void AppearanceModel::setBaseMesh(LG::PolygonMesh* mesh)
{
  base_mesh = std::make_unique<PolygonMesh>();
  (*base_mesh.get()) = (*mesh); // deep copy
}

void AppearanceModel::getBaseMesh(LG::PolygonMesh* mesh)
{
  (*mesh) = (*base_mesh.get());
}

LG::PolygonMesh* AppearanceModel::getBaseMesh()
{
  return base_mesh.get();
}

void AppearanceModel::setCCAMat(cv::Mat& mat)
{
  cca_mat = mat.clone();
}

void AppearanceModel::geteCCAMat(cv::Mat& mat)
{
  mat = cca_mat.clone();
}

void AppearanceModel::getCCAMin(std::vector<float>& vec)
{
  vec = cca_min;
}

void AppearanceModel::getCCAMax(std::vector<float>& vec)
{
  vec = cca_max;
}

void AppearanceModel::setCCAMin(std::vector<float>& vec)
{
  cca_min = vec;
}

void AppearanceModel::setCCAMax(std::vector<float>& vec)
{
  cca_max = vec;
}

template<typename T>
void AppearanceModel::writeVector(cv::FileStorage& fs, std::vector<T>& vec, std::string vec_name)
{
  fs << vec_name << "[:";
  for (int i = 0; i < vec.size(); ++i)
  {
    fs << vec[i];
  }
  fs << "]";
}

template<typename T>
void AppearanceModel::readVector(cv::FileStorage& fs, std::vector<T>& vec, std::string vec_name)
{
  vec.clear();
  fs[vec_name] >> vec;
}

void AppearanceModel::setCameraInfo(Matrix4f& modelview, Matrix4f& projection, Vector4i& viewport)
{
  m_modelview = modelview;
  m_projection = projection;
  m_viewport = viewport;

  m_inv_modelview_projection = (m_projection*m_modelview).inverse();
}

void AppearanceModel::setZImg(cv::Mat& z_img_)
{
  z_img = z_img_.clone();
}

void AppearanceModel::setPrimitiveID(cv::Mat& primitive_ID_)
{
  primitive_ID = primitive_ID_.clone();
}

void AppearanceModel::writeCameraInfo(cv::FileStorage& fs)
{
  fs << "z_img" << z_img;
  fs << "primitive_ID" << primitive_ID;
  std::vector<float> modelview_vec(m_modelview.data(), m_modelview.data() + m_modelview.size());
  std::vector<float> projection_vec(m_projection.data(), m_projection.data() + m_projection.size());
  std::vector<int> viewport_vec(m_viewport.data(), m_viewport.data() + m_viewport.size());

  this->writeVector(fs, modelview_vec, "m_modelview");
  this->writeVector(fs, projection_vec, "m_projection");
  this->writeVector(fs, viewport_vec, "m_viewport");
}

void AppearanceModel::readCameraInfo(cv::FileStorage& fs)
{
  fs["z_img"] >> z_img;
  fs["primitve_ID"] >> primitive_ID;

  std::vector<float> modelview_vec, projection_vec;
  this->readVector(fs, modelview_vec, "m_modelview");
  this->readVector(fs, projection_vec, "m_projection");
  m_modelview = Eigen::Map<Matrix4f>(&modelview_vec[0]);
  m_projection = Eigen::Map<Matrix4f>(&projection_vec[0]);

  std::vector<int> viewport_vec;
  this->readVector(fs, viewport_vec, "m_viewport");
  m_viewport = Eigen::Map<Vector4i>(&viewport_vec[0]); 
  m_inv_modelview_projection = (m_projection*m_modelview).inverse();
}

void AppearanceModel::getD1Reflectance(cv::Mat& reflectance)
{
  std::vector<cv::Mat> temp_reflectance;
  temp_reflectance.push_back(d1_detail_maps[2].clone());
  temp_reflectance.push_back(d1_detail_maps[1].clone());
  temp_reflectance.push_back(d1_detail_maps[0].clone());
  cv::merge(temp_reflectance, reflectance);
}

void AppearanceModel::setPhoto(cv::Mat& photo_)
{
  photo = photo_.clone();
}

void AppearanceModel::getPhoto(cv::Mat& photo_)
{
  photo_ = photo.clone();
}

void AppearanceModel::coordImgToUV(std::vector<CvPoint>& coords)
{
  if (coords.empty()) return;
  std::vector<int> records;
  for (auto i : coords)
  {
    int f_id = primitive_ID.at<int>(i.y, i.x);
    if (f_id >= 0) records.push_back(f_id);
  }

  // delete duplicated record
  size_t k = 1;
  for (size_t i = 1; i < records.size(); ++i)
  {
    if (records[k-1] != records[i])
    {
      records[k] = records[i];
      ++k;
    }
  }

  // convert face id to UV
  this->coordFaceToUV(coords, records);
}

void AppearanceModel::coordFaceToUV(std::vector<CvPoint>& coords, std::vector<int>& f_ids)
{
  coords.clear();
  coords.resize(f_ids.size());
  for (size_t i = 0; i < f_ids.size(); ++i)
  {
    Vector2f uv;
    ShapeUtility::getFaceUVCenter(base_mesh.get(), f_ids[i], uv);
    coords[i].x = int(uv[0] * resolution + 0.5);
    coords[i].y = resolution - (uv[1] * resolution + 0.5);
    if (coords[i].x >= resolution) coords[i].x = resolution - 1;
    if (coords[i].y >= resolution) coords[i].y = resolution - 1;
    if (coords[i].y < 0) coords[i].y = 0;
  }
}