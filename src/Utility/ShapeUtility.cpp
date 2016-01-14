#include "ShapeUtility.h"
#include "BasicHeader.h"

#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"
#include "Bound.h"
#include "ParaShape.h"
#include "Voxeler.h"

#include "Ray.h"
#include "SAMPLE.h"
#include "GenerateSamples.h"
#include "KDTreeWrapper.h"

#include "obj_writer.h"

using namespace LG;

namespace ShapeUtility
{
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3])
  {
    // v presents triangle vertex, pt locate inside triangle
    // pt[0]->x pt[1]->y pt[2]->z
    // v0[0]->x1 v0[1]->y1 v0[2]->z1
    // v1[0]->x2 v1[1]->y2 v1[2]->z2
    // v2[0]->x3 v2[1]->y3 v2[2]->z3
    Eigen::Vector3f e0(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
    Eigen::Vector3f e1(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
    Eigen::Vector3f e2(pt[0] - v0[0], pt[1] - v0[1], pt[2] - v0[2]);

    float d00 = e0.dot(e0);
    float d01 = e0.dot(e1);
    float d11 = e1.dot(e1);
    float d20 = e2.dot(e0);
    float d21 = e2.dot(e1);
    float denom = d00*d11 - d01*d01;

    if (denom == 0.0f)
    {
      lambd[1] = 0.33;

      lambd[2] = 0.33;

      lambd[0] = 1.0f - lambd[1] - lambd[2];
    }
    else
    {
      lambd[1] = (d11*d20 - d01*d21) / denom;

      lambd[2] = (d00*d21 - d01*d20) / denom;

      lambd[0] = 1.0f - lambd[1] - lambd[2];
    }

  }

  void computeBaryCentreCoord(Vector2f& pt, Vector2f& v0, Vector2f& v1, Vector2f& v2, float lambd[3])
  {
    float pte[3] = { pt[0], pt[1], 0 };
    float v0e[3] = { v0[0], v0[1], 0 };
    float v1e[3] = { v1[0], v1[1], 0 };
    float v2e[3] = { v2[0], v2[1], 0 };
    computeBaryCentreCoord(pte, v0e, v1e, v2e, lambd);
  }

  void computeNormalizedHeight(std::shared_ptr<Model> model)
  {
    Bound* boundbox = model->getBoundBox();
    PolygonMesh* mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Scalar> normalized_height = mesh->vertex_attribute<Scalar>("v:NormalizedHeight");

    for (auto vit : mesh->vertices())
    {
      normalized_height[vit] = (mesh->position(vit)[2] - boundbox->minZ) / (boundbox->maxZ - boundbox->minZ);
    }
  }

  void computeDirectionalOcclusion(std::shared_ptr<Model> model)
  {
    // only called when the shape is changed

    // 1. initialize BSPTree
    int num_band = 2;
    std::cout << "Initialize BSPTree.\n";
    std::shared_ptr<Ray> ray(new Ray);
    ray->passModel(model->getShapeVertexList(), model->getShapeFaceList());

    // 2. generate direction samples
    std::cout << "Generate samples.\n";
    int sqrtNumSamples = 10;
    int numSamples = sqrtNumSamples * sqrtNumSamples;
    std::vector<SAMPLE> samples(numSamples);
    GenerateSamples(sqrtNumSamples, num_band, &samples[0]);

    // 3. compute coeffs
    std::cout << "Compute Directional Occlusion Feature.\n";
    int numBand = num_band;
    int numFunctions = numBand * numBand;
    Bound* bound = model->getBoundBox();
    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<STLVectorf> shadowCoeff = poly_mesh->vertex_attribute<STLVectorf>("v:DirectionalOcclusion");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
    STLVectorf max_coeff(numFunctions, -std::numeric_limits<float>::max());
    STLVectorf min_coeff(numFunctions, std::numeric_limits<float>::max());
    float perc = 0;
    for (auto i : poly_mesh->vertices())
    {
      shadowCoeff[i].clear();
      shadowCoeff[i].resize(numFunctions, 0.0f);
      for (int k = 0; k < numSamples; ++k)
      {
        double dot = (double)samples[k].direction.dot(v_normals[i]);

        if (dot > 0.0)
        {
          Eigen::Vector3d ray_start = (poly_mesh->position(i) + 2 * 0.01 * bound->getRadius() * v_normals[i]).cast<double>();
          Eigen::Vector3d ray_end   = ray_start + (5 * bound->getRadius() * samples[k].direction).cast<double>();
          if (ray->intersectModel(ray_start, ray_end))
          {
            for (int l = 0; l < numFunctions; ++l)
            {
              shadowCoeff[i][l] += dot * samples[k].shValues[l];
            }
          }
        }
      }

      // rescale
      for (int l = 0; l < numFunctions; ++l)
      {
        shadowCoeff[i][l] *= 4.0 * M_PI / numSamples;

        if (shadowCoeff[i][l] > max_coeff[l]) max_coeff[l] = shadowCoeff[i][l];
        if (shadowCoeff[i][l] < min_coeff[l]) min_coeff[l] = shadowCoeff[i][l];
      }

      float cur_perc = (float)i.idx() / poly_mesh->n_vertices();
      if (cur_perc - perc >= 0.05)
      {
        perc = cur_perc;
        std::cout << perc << "...";
      }
    }
    std::cout << std::endl;

    for (auto i : poly_mesh->vertices())
    {
      for (int l = 0; l < numFunctions; ++l)
      {
        shadowCoeff[i][l] = (shadowCoeff[i][l] - min_coeff[l]) / (max_coeff[l] - min_coeff[l]);
      }
    }

    for (int l = 0; l < numFunctions; ++l)
    {
      std::cout<< "sh min: " << min_coeff[l] << " sh max: " << max_coeff[l] << std::endl;
    }
    std::cout << "Compute Directional Occlusion Feature finished.\n";
  }


  void computeSymmetry(std::shared_ptr<Model> model)
  {

    //std::cout << "Generate or Load symmetry.txt." << std::endl;
    //bool regenerate = false;
    //// test if the file exist
    //std::ifstream inFile(model->getDataPath() + "/symmetry.txt");
    //if (!inFile.is_open())
    //{
    //  std::cout << "Not existed or failed to load." << std::endl;
    //  regenerate = true;
    //}

    //PolygonMesh* poly_mesh = model->getPolygonMesh();
    //PolygonMesh::Vertex_attribute<Vec3> v_symmetry = poly_mesh->vertex_attribute<Vec3>("v:symmetry");

    //if (!regenerate)
    //{
    //  // symmetry information should use original coarse model
    //  // because after large alignment, symmetry information would be broken
    //  // we assume the ground plane are xoy plane
    //  // symmetry plane are yoz plane
    //  // symmetry information are stored like this
    //  // first two scalars store projection position on the symmetry plane (y,z)
    //  // last scalar stores the distance to symmetry plane |x|

    //  // no need to regenerate, read from file
    //  std::cout << "Loading symmetry.txt" << std::endl;
    //  std::string line_str;
    //  int n_line = 0;
    //  float min_v = std::numeric_limits<float>::max();
    //  float max_v = -std::numeric_limits<float>::max();
    //  for (auto vit : poly_mesh->vertices())
    //  {
    //    getline(inFile, line_str);
    //    std::stringstream line_parser(line_str);
    //    Vec3 cur_v_symmetry(0,0,0);
    //    line_parser >> cur_v_symmetry[0] >> cur_v_symmetry[1] >> cur_v_symmetry[2];
    //    v_symmetry[vit] = cur_v_symmetry;
    //    ++n_line;
    //  }
    //  
    //  inFile.close();
    //  if (n_line == poly_mesh->n_vertices()) 
    //  {
    //    std::cout << "Loading finished." << std::endl;
    //  }
    //  else 
    //  {
    //    std::cout << "Loading errors. Need to regenerate." << std::endl;
    //    regenerate = true;
    //  }
    //}
    //
    //if (regenerate)
    //{
    //  std::cout << "Generating Symmetry.txt." << std::endl;
    //  std::ofstream outFile(model->getDataPath() + "/symmetry.txt");
    //  // read from the file
    //  if (!outFile.is_open())
    //  {
    //    std::cout << "failed to open the symmetry.txt file, return." << std::endl;
    //    return;
    //  }
    //  for (auto vit : poly_mesh->vertices())
    //  {
    //    Vec3 pos = poly_mesh->position(vit);
    //    v_symmetry[vit] = Vec3(fabs(pos[1]), pos[0], pos[2]);
    //    outFile << fabs(pos[1]) << "\t" << pos[0] << "\t" << pos[2] << std::endl;
    //  }
    //  outFile.close();
    //  std::cout << "Generating finished." << std::endl;
    //}

    //// normalize the value
    //Vector3f mins(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    //Vector3f maxs(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    //for (auto vit : poly_mesh->vertices())
    //{
    //  Vec3 cur_sym = v_symmetry[vit];
    //  if (cur_sym[0] < mins[0]) mins[0] = cur_sym[0];
    //  if (cur_sym[0] > maxs[0]) maxs[0] = cur_sym[0];
    //  if (cur_sym[1] < mins[1]) mins[1] = cur_sym[1];
    //  if (cur_sym[1] > maxs[1]) maxs[1] = cur_sym[1];
    //  if (cur_sym[2] < mins[2]) mins[2] = cur_sym[2];
    //  if (cur_sym[2] > maxs[2]) maxs[2] = cur_sym[2];
    //}
    //for (auto vit : poly_mesh->vertices())
    //{
    //  Vec3 cur_sym = v_symmetry[vit];
    //  v_symmetry[vit] = ((cur_sym - mins).array() / (maxs - mins).array()).matrix();
    //}
    std::cout << "Generate or Load symmetry.txt." << std::endl;
    bool regenerate = false;
    // test if the file exist
    std::ifstream inFile(model->getDataPath() + "/symmetry.txt");
    //if (!inFile.is_open())
    {
      std::cout << "Not existed or failed to load." << std::endl;
      regenerate = true;
    }
    inFile.close();

    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<std::vector<float>> v_symmetry = poly_mesh->vertex_attribute<std::vector<float>>("v:symmetry");

    if (!regenerate)
    {
      // symmetry information should use original coarse model
      // because after large alignment, symmetry information would be broken
      // we assume the ground plane are xoy plane
      // symmetry plane are yoz plane
      // symmetry information are stored like this
      // first two scalars store projection position on the symmetry plane (y,z)
      // last scalar stores the distance to symmetry plane |x|

      // no need to regenerate, read from file
      std::cout << "Loading symmetry.txt" << std::endl;
      std::string line_str;
      int n_line = 0;
      float min_v = std::numeric_limits<float>::max();
      float max_v = -std::numeric_limits<float>::max();
      for (auto vit : poly_mesh->vertices())
      {
        getline(inFile, line_str);
        std::stringstream line_parser(line_str);
        //Vec3 cur_v_symmetry(0,0,0);
        std::vector<float> cur_v_symmetry;
        cur_v_symmetry.resize(5);
        line_parser >> cur_v_symmetry[0] >> cur_v_symmetry[1] >> cur_v_symmetry[2] >> cur_v_symmetry[3] >> cur_v_symmetry[4];
        v_symmetry[vit] = cur_v_symmetry;
        ++n_line;
      }
      
      inFile.close();
      if (n_line == poly_mesh->n_vertices()) 
      {
        std::cout << "Loading symmetry.txt finished." << std::endl;
      }
      else 
      {
        std::cout << "Loading errors. Need to regenerate symmetry.txt." << std::endl;
        regenerate = true;
      }
    }
    
    if (regenerate)
    {
      std::cout << "Generating Symmetry.txt." << std::endl;
      std::ofstream outFile(model->getDataPath() + "/symmetry.txt");
      // read from the file
      if (!outFile.is_open())
      {
        std::cout << "failed to open the symmetry.txt file, return." << std::endl;
        return;
      }
      std::cout << "Generate or Load symmetry_plane.txt." << std::endl;
      bool regenerate_sym_plane = false;
      // test if the file exist
      std::ifstream inFile(model->getDataPath() + "/symmetry_plane.txt");
      if (!inFile.is_open())
      {
        std::cout << "Not existed or failed to load symmetry_plane.txt." << std::endl;
        regenerate_sym_plane = true;
      }
      double a, b, c, d;
      if(!regenerate_sym_plane)
      {
        std::cout << "Loading symmetry_plane.txt" << std::endl;
        std::string line_str;
        getline(inFile, line_str);
        std::stringstream line_parser(line_str);
        line_parser >> a >> b >> c >> d;
      }
      else
      {
        a = 1;
        b = 0;
        c = 0;
        d = 0;
        std::cout << "Generating Symmetry_plane.txt." << std::endl;
        std::ofstream outFile_sym_plane(model->getDataPath() + "/symmetry_plane.txt");
        // read from the file
        if (!outFile_sym_plane.is_open())
        {
          std::cout << "failed to open the symmetry_plane.txt file, return." << std::endl;
          return;
        }
        outFile_sym_plane << a << "\t" << b << "\t" << c << "\t" << d << "\n";
        outFile_sym_plane.close();
        std::cout << "Generating symmetry_plane.txt finished." << std::endl;
      }
      std::vector<double> symmetric_plane_coef;
      symmetric_plane_coef.push_back(a);
      symmetric_plane_coef.push_back(b);
      symmetric_plane_coef.push_back(c);
      symmetric_plane_coef.push_back(d);
      PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
      Bound* bounding = model->getBoundBox();
      for (auto vit : poly_mesh->vertices())
      {
        Vec3 pos = poly_mesh->position(vit);
        pos << (pos(0) - bounding->minX) / (bounding->maxX - bounding->minX), 
                       (pos(1) - bounding->minY) / (bounding->maxY - bounding->minY),
                       (pos(2) - bounding->minZ) / (bounding->maxZ - bounding->minZ);
        Vec3 normal = v_normals[vit];
        ShapeUtility::computeVertexSymmetryProjection(pos, normal, symmetric_plane_coef);
        normal = (normal + Vec3(1, 1, 1)) / 2 ;

        std::vector<float> cur_v_symmetry;
        cur_v_symmetry.push_back(pos(0));
        cur_v_symmetry.push_back(pos(1));
        cur_v_symmetry.push_back(pos(2));
        cur_v_symmetry.push_back(normal(0));
        cur_v_symmetry.push_back(normal(1));
        //v_symmetry[vit] = Vec3(fabs(pos[1]), pos[0], pos[2]);
        v_symmetry[vit] = cur_v_symmetry;
        outFile << cur_v_symmetry[0] << "\t" << cur_v_symmetry[1] << "\t" << cur_v_symmetry[2] << "\t" << cur_v_symmetry[3] << "\t" << cur_v_symmetry[4] << std::endl;
      }
      outFile.close();
      std::cout << "Generating symmetrt.txt finished." << std::endl;
    }

    //// normalize the value
    //Vector3f mins(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    //Vector3f maxs(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    //for (auto vit : poly_mesh->vertices())
    //{
    //  Vec3 cur_sym = v_symmetry[vit];
    //  if (cur_sym[0] < mins[0]) mins[0] = cur_sym[0];
    //  if (cur_sym[0] > maxs[0]) maxs[0] = cur_sym[0];
    //  if (cur_sym[1] < mins[1]) mins[1] = cur_sym[1];
    //  if (cur_sym[1] > maxs[1]) maxs[1] = cur_sym[1];
    //  if (cur_sym[2] < mins[2]) mins[2] = cur_sym[2];
    //  if (cur_sym[2] > maxs[2]) maxs[2] = cur_sym[2];
    //}
    //for (auto vit : poly_mesh->vertices())
    //{
    //  Vec3 cur_sym = v_symmetry[vit];
    //  v_symmetry[vit] = ((cur_sym - mins).array() / (maxs - mins).array()).matrix();
    //}
  }

  void computeVertexSymmetryProjection(Vector3f& vertex, Vector3f& normal, std::vector<double>& plane_coef)
  {
    Vector3f symmetric_plane_normal;
    symmetric_plane_normal << plane_coef[0], plane_coef[1], plane_coef[2];
    double distance = (plane_coef[0] * vertex(0) + plane_coef[1] * vertex(1) + plane_coef[2] * vertex(2) + plane_coef[3])
      / sqrt(plane_coef[0] * plane_coef[0] + plane_coef[1] * plane_coef[1] + plane_coef[2] * plane_coef[3]);
    vertex += (-distance) * symmetric_plane_normal;
    if(normal.dot(symmetric_plane_normal) < 0)
    {
      distance = 2 * fabs(normal.dot(symmetric_plane_normal));
      normal += distance * symmetric_plane_normal;
      normal.normalized();
    }
  }

  void computeRMSCurvature(std::shared_ptr<Model> model)
  {
    // 1. compute 
  }

  void getNRingFacesAroundVertex(LG::PolygonMesh* poly_mesh, std::set<int>& f_id, int v_id, int n_ring)
  {
    // actually get (n_ring + 1) faces around vertex
    // first get the center faces around vertex 
    f_id.clear();
    for (auto fcc : poly_mesh->faces(PolygonMesh::Vertex(v_id)))
    {
      f_id.insert(fcc.idx());
    }

    std::set<int> cur_ring;
    std::set<int> last_ring = f_id;
    std::set<int>::iterator iter;
    for (int i = 0; i < n_ring; ++i)
    {
      for (auto j : last_ring)
      {
        for (auto hecc : poly_mesh->halfedges(PolygonMesh::Face(j)))
        {
          int cur_f_id = poly_mesh->face(poly_mesh->opposite_halfedge(hecc)).idx();
          iter = f_id.find(cur_f_id);
          if (iter == f_id.end())
          {
            cur_ring.insert(cur_f_id);
          }
        }
      }
      f_id.insert(cur_ring.begin(), cur_ring.end());
      last_ring.swap(cur_ring);
      cur_ring.clear();
    }
  }

  void dilateImage(cv::Mat& mat, int max_n_dilate)
  {
    // assume to be single channel float
    float* mat_ptr = (float*)mat.data;
    // let
    for (int n_dilate = 0; n_dilate < max_n_dilate; ++n_dilate)
    {
      cv::Mat temp_mat = cv::Mat::zeros(mat.rows, mat.cols, CV_32FC1);
      for (int i = 0; i < mat.rows; ++i)
      {
        for (int j = 0; j < mat.cols; ++j)
        {
          int offset = i * mat.cols + j;
          if (mat_ptr[offset] < 0)
          {
            dilateImageMeetBoundary(mat, temp_mat, i, j);
          }
        }
      }
      mat = mat + temp_mat;
    }
  }

  void dilateImageMeetBoundary(cv::Mat& mat, cv::Mat& filled_mat, int i, int j)
  {
    float* mat_ptr = (float*)mat.data;
    float* filled_mat_ptr = (float*)filled_mat.data;
    int offset = i * mat.cols + j;
    float value = 0;
    int n_value = 0;
    bool is_boundary = false;

    for (int ioff = -1; ioff <= 1; ++ioff)
    {
      for (int joff = -1; joff <= 1; ++joff)
      {
        int row_id = i + ioff;
        int col_id = j + joff;
        row_id = std::max(std::min(row_id, mat.rows - 1), 0);
        col_id = std::max(std::min(col_id, mat.cols - 1), 0);
        int ooffset = row_id * mat.cols + col_id;
        if (mat_ptr[ooffset] > 0)
        {
          is_boundary = true;
          break;
        }
      }
      if (is_boundary) break;
    }

    if (!is_boundary) return;

    for (int ioff = -3; ioff <= 3; ++ioff)
    {
      for (int joff = -3; joff <= 3; ++joff)
      {
        int row_id = i + ioff;
        int col_id = j + joff;
        row_id = std::max(std::min(row_id, mat.rows - 1), 0);
        col_id = std::max(std::min(col_id, mat.cols - 1), 0);
        int ooffset = row_id * mat.cols + col_id;
        if (mat_ptr[ooffset] >= 0)
        {
          value += mat_ptr[ooffset];
          ++n_value;
        }
      }
    }
    if (n_value != 0)
    {
      filled_mat_ptr[offset] = value / n_value;
    }
    else
    {
      filled_mat_ptr[offset] = 0;
    }
  }

  void fillImageWithMask(cv::Mat& mat, cv::Mat& mask)
  {
    bool full_in_mask = true;
    do
    {
      full_in_mask = true;
      cv::Mat this_iter_mat = mat.clone();
      for (int i = 0; i < mat.rows; i++)
      {
        for (int j = 0; j < mat.cols; j++)
        {
          if (mask.at<float>(i, j) > 0.5)
          {
            if (mat.at<float>(i, j) < 0)
            {
              dilateImageMeetBoundary(mat, this_iter_mat, i, j);
              full_in_mask = false;
            }
          }
        }
      }
      mat = this_iter_mat;
    } while (!full_in_mask);
  }

  void matToMesh(cv::Mat& mat, LG::PolygonMesh& mesh, std::shared_ptr<Model> shape_model)
  {
    int width = mat.size().width;
    int height = mat.size().height;
    double z_scale = shape_model->getZScale();
    std::cout << "z_scale is : " << z_scale << std::endl;
    Vector3f dir ;
    dir << 0, 0, -1;
    shape_model->getUnprojectVec(dir);
    dir.normalize();
    cv::Mat& mask = shape_model->getPrimitiveIDImg();

    for(int i = 0; i < width; i ++)
    {
      for(int j = 0; j < height; j ++)
      {
        Vector3f img_coord, w_coord;
        img_coord << i, j, 1;
        shape_model->getWorldCoord(img_coord, w_coord);
        Vector3f mesh_pt;
        /*mesh_pt = w_coord + dir * mat.at<float>(j, i) / z_scale;*/
        mesh_pt = w_coord + dir * mat.at<float>(j, i) / 30.0;
        mesh.add_vertex(Vec3(mesh_pt(0), mesh_pt(1), mesh_pt(2)));
        //mesh.add_vertex(Vec3(float(i), float(j), 10 * mat.at<float>(j, i)));
      }
    }
    for(int i = 0; i < width - 1; i ++)
    {
      for(int j = 0; j < height - 1; j ++)
      {
        std::vector<PolygonMesh::Vertex> vertices;
        vertices.clear();
        vertices.push_back(PolygonMesh::Vertex(height * i + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j + 1));
        vertices.push_back(PolygonMesh::Vertex(height * i + j + 1));
        mesh.add_face(vertices);
        vertices.clear();
        vertices.push_back(PolygonMesh::Vertex(height * i + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j + 1));
        mesh.add_face(vertices);
      }
    }

    std::vector<tinyobj::shape_t> shapes;
    tinyobj::shape_t obj_shape;
    std::vector<tinyobj::material_t> materials;
    VertexList vertex_list;
    vertex_list.clear();
    for (auto vit : mesh.vertices())
    {
      const Vec3& pt = mesh.position(vit);
      vertex_list.push_back(pt[0]);
      vertex_list.push_back(pt[1]);
      vertex_list.push_back(pt[2]);
    }
    obj_shape.mesh.positions = vertex_list;
    FaceList face_list;
    face_list.clear();
    for (auto fit : mesh.faces())
    {
      for (auto vfc_it : mesh.vertices(fit))
      {
        face_list.push_back(vfc_it.idx());
      }
    } 
    obj_shape.mesh.indices = face_list;
    shapes.push_back(obj_shape);
    WriteObj(shape_model->getOutputPath() + "/mat2mesh.obj", shapes, materials);
  }

  void heightToMesh(cv::Mat& mat, LG::PolygonMesh& mesh, std::shared_ptr<Model> shape_model)
  {
    int width = mat.size().width;
    int height = mat.size().height;
    double z_scale = shape_model->getZScale();
    std::cout << "z_scale is : " << z_scale << std::endl;

    for(int i = 0; i < width; i ++)
    {
      for(int j = 0; j < height; j ++)
      {
        float obj_coord[3];
        if(!shape_model->getUnprojectPt(i, j, (1 - mat.at<float>(j, i) / z_scale), obj_coord))
          return ;
        mesh.add_vertex(Vec3(obj_coord[0], obj_coord[1], obj_coord[2]));
      }
    }
    for(int i = 0; i < width - 1; i ++)
    {
      for(int j = 0; j < height - 1; j ++)
      {
        std::vector<PolygonMesh::Vertex> vertices;
        vertices.clear();
        vertices.push_back(PolygonMesh::Vertex(height * i + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j + 1));
        vertices.push_back(PolygonMesh::Vertex(height * i + j + 1));
        mesh.add_face(vertices);
        vertices.clear();
        vertices.push_back(PolygonMesh::Vertex(height * i + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j));
        vertices.push_back(PolygonMesh::Vertex(height * (i + 1) + j + 1));
        mesh.add_face(vertices);
      }
    }

    std::vector<tinyobj::shape_t> shapes;
    tinyobj::shape_t obj_shape;
    std::vector<tinyobj::material_t> materials;
    VertexList vertex_list;
    vertex_list.clear();
    for (auto vit : mesh.vertices())
    {
      const Vec3& pt = mesh.position(vit);
      vertex_list.push_back(pt[0]);
      vertex_list.push_back(pt[1]);
      vertex_list.push_back(pt[2]);
    }
    obj_shape.mesh.positions = vertex_list;
    FaceList face_list;
    face_list.clear();
    for (auto fit : mesh.faces())
    {
      for (auto vfc_it : mesh.vertices(fit))
      {
        face_list.push_back(vfc_it.idx());
      }
    } 
    obj_shape.mesh.indices = face_list;
    shapes.push_back(obj_shape);

    char time_postfix[50];
    time_t current_time = time(NULL);
    strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    std::string file_time_postfix = time_postfix;
    std::string output_name = shape_model->getOutputPath() + "/height2mesh" + file_time_postfix + ".obj";

    WriteObj(output_name, shapes, materials);
  }

  void getFaceInPatchByFaceInMesh(int f_id, std::vector<ParaShape>& patches, int& f_id_patch, int& patch_id)
  {
    // f_id is original mesh face id
    for (size_t i = 0; i < patches.size(); ++i)
    {
      int pos = std::find(patches[i].face_set.begin(), patches[i].face_set.end(), f_id) - patches[i].face_set.begin();
      if (pos < patches[i].face_set.size())
      {
        f_id_patch = pos;
        patch_id = i;
        return;
      }
    }
  }

  bool twoSideOfLine(Vector2f& line_start, Vector2f& line_end, Vector2f& pt0, Vector2f& pt1)
  {
    float a = (line_start[1] - line_end[1])*(pt0[0] - line_start[0]) + (line_end[0] - line_start[0])*(pt0[1] - line_start[1]);
    float b = (line_start[1] - line_end[1])*(pt1[0] - line_start[0]) + (line_end[0] - line_start[0])*(pt1[1] - line_start[1]);
    return (a * b) < 0 ? true : false;
  }

  bool onSegment(const Vector2f& p, const Vector2f& q, const Vector2f& r)
  {
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
      q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
      return true;

    return false;
  }

  int orientation(const Vector2f& p, const Vector2f& q, const Vector2f& r)
  {
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    float fval = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);

    if (fval > 0) return 1;
    else if (fval < 0) return 2;
    else return 0;

    int val = (fabs(fval) < 1e-9) ? 0 : ((fval < 0) ? -1 : 1); // how precise this is ?

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
  }

  bool doIntersect(const Vector2f& p1, const Vector2f& q1, const Vector2f& p2, const Vector2f& q2)
  {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
      return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
  }

  bool findClosesttUVriangleFace(std::vector<float>& pt, ParaShape* para_shape, std::vector<float>& bary_coord, int& f_id, std::vector<int>& v_ids)
  {
    bary_coord.clear();
    bary_coord.resize(3, 0);
    v_ids.clear();
    v_ids.resize(3, 0);

    // pt should be normalized to 0~1 uv coordinate
    int pt_id;
    float point[3] = { pt[0], pt[1], 1.0 };
    Vec2 pt_s_uv(pt[0], pt[1]);
    para_shape->kdTree_UV->nearestPt(pt, pt_id);

    // fast method to find which face the point lies in
    // first we test the faces around this vertex
    PolygonMesh* poly_mesh = para_shape->cut_shape->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
    const FaceList& f_list = para_shape->cut_shape->getFaceList();
    Vec2 pt_e_uv = tex_coords[PolygonMesh::Vertex(pt_id)];
    PolygonMesh::Face f_intersect;
    bool found_possible_face = false;
    for (auto hvc : poly_mesh->halfedges(PolygonMesh::Vertex(pt_id)))
    {
      if (poly_mesh->is_boundary(hvc)) continue;

      float l[3];
      Vec2 pt0 = tex_coords[poly_mesh->to_vertex(hvc)];
      Vec2 pt1 = tex_coords[poly_mesh->from_vertex(poly_mesh->prev_halfedge(hvc))];
      computeBaryCentreCoord(pt_s_uv, pt_e_uv, pt0, pt1, l);

      l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
      l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
      l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
      if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
      {
        f_id = poly_mesh->face(hvc).idx();
        bary_coord[0] = l[0];
        bary_coord[1] = l[1];
        bary_coord[2] = l[2];
        v_ids[0] = f_list[3 * f_id + 0];
        v_ids[1] = f_list[3 * f_id + 1];
        v_ids[2] = f_list[3 * f_id + 2];
        return true;
      }
      else
      {
        // test intersection
        if (doIntersect(pt_s_uv, pt_e_uv, pt0, pt1))
        {
          PolygonMesh::Halfedge possible_hf = poly_mesh->opposite_halfedge(poly_mesh->next_halfedge(hvc));
          if (!poly_mesh->is_boundary(possible_hf))
          {
            found_possible_face = true;
            f_intersect = poly_mesh->face(possible_hf);
            break;
          }
        }
      }
    }
    
    if (found_possible_face)
    {
      float l[3];
      std::vector<Vec2> pts;
      for (auto vfc : poly_mesh->vertices(f_intersect))
      {
        pts.push_back(tex_coords[vfc]);
      }
      computeBaryCentreCoord(pt_s_uv, pts[0], pts[1], pts[2], l);

      l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
      l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
      l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
      if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
      {
        f_id = f_intersect.idx();
        bary_coord[0] = l[0];
        bary_coord[1] = l[1];
        bary_coord[2] = l[2];
        v_ids[0] = f_list[3 * f_id + 0];
        v_ids[1] = f_list[3 * f_id + 1];
        v_ids[2] = f_list[3 * f_id + 2];
        return true;
      }
    }


    //PolygonMesh* poly_mesh = para_shape->cut_shape->getPolygonMesh();
    //const FaceList& f_list = para_shape->cut_shape->getFaceList();
    //const STLVectorf& uv_list = para_shape->cut_shape->getUVCoord();
    //bool meet_boundary = false;
    //std::set<int> visited;
    //for (auto fc : poly_mesh->faces(PolygonMesh::Vertex(pt_id)))
    //{
    //  visited.insert(fc.idx());
    //}
    //std::set<int> cur_ring;
    //std::set<int> last_ring = visited;
    //std::set<int>::iterator iter;
    //while (!last_ring.empty())
    //{
    //  for (auto i : last_ring)
    //  {
    //    // i is face id
    //    float l[3];
    //    int v1_id,v2_id,v3_id;
    //    v1_id = f_list[3 * i + 0];
    //    v2_id = f_list[3 * i + 1];
    //    v3_id = f_list[3 * i + 2];
    //    float v1[3] = { uv_list[2 * v1_id + 0], uv_list[2 * v1_id + 1], 1.0 };
    //    float v2[3] = { uv_list[2 * v2_id + 0], uv_list[2 * v2_id + 1], 1.0 };
    //    float v3[3] = { uv_list[2 * v3_id + 0], uv_list[2 * v3_id + 1], 1.0 };
    //    ShapeUtility::computeBaryCentreCoord(point,v1,v2,v3,l);
    //    l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
    //    l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
    //    l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
    //    if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
    //    {
    //      f_id = i;
    //      bary_coord[0] = l[0];
    //      bary_coord[1] = l[1];
    //      bary_coord[2] = l[2];
    //      v_ids[0] = v1_id;
    //      v_ids[1] = v2_id;
    //      v_ids[2] = v3_id;
    //      return true;
    //    }
    //    for (auto hc : poly_mesh->halfedges(PolygonMesh::Face(i)))
    //    {
    //      if (poly_mesh->is_boundary(poly_mesh->opposite_halfedge(hc)))
    //      {
    //        meet_boundary = true;
    //        continue;
    //      }
    //      int cur_f_id = poly_mesh->face(poly_mesh->opposite_halfedge(hc)).idx();
    //      iter = visited.find(cur_f_id);
    //      if (iter == visited.end())
    //      {
    //        cur_ring.insert(cur_f_id);
    //      }
    //    }
    //  }
    //  visited.insert(cur_ring.begin(), cur_ring.end());
    //  last_ring.swap(cur_ring);
    //  cur_ring.clear();
    //}

    return false;
  }

  bool findClosestUVFace(std::vector<float>& pt, ParaShape* para_shape, std::vector<float>& bary_coord, int& f_id, std::vector<int>& v_ids)
  {
    bary_coord.clear();
    bary_coord.resize(3, 0);
    v_ids.clear();
    v_ids.resize(3, 0);

    // pt should be normalized to 0~1 uv coordinate
    int closest_f_id;
    float point[3] = { pt[0], pt[1], 1.0 };
    Vec2 uv_pt(pt[0], pt[1]);
    para_shape->kdTree_UV_f->nearestPt(pt, closest_f_id);
    Vec2 uv_f_center_pt(pt[0], pt[1]);

    bool meet_boundary = false;
    PolygonMesh* poly_mesh = para_shape->cut_shape->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
    const FaceList& f_list = para_shape->cut_shape->getFaceList();
    int last_f_id = closest_f_id;
    int n_visited_f = 0;
    int max_visited_f = 5;
    while (!meet_boundary && n_visited_f < max_visited_f)
    {
      // test if the uv_pt is inside the face
      ++ n_visited_f;
      float l[3];
      std::vector<Vec2> pts;
      uv_f_center_pt = Vec2(0, 0);
      int n_pts = 0;
      for (auto vfc : poly_mesh->vertices(PolygonMesh::Face(closest_f_id)))
      {
        pts.push_back(tex_coords[vfc]);
        uv_f_center_pt += tex_coords[vfc];
        ++ n_pts;
      }
      uv_f_center_pt = uv_f_center_pt / n_pts;
      computeBaryCentreCoord(uv_pt, pts[0], pts[1], pts[2], l);

      l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
      l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
      l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
      if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
      {
        f_id = closest_f_id;
        bary_coord[0] = l[0];
        bary_coord[1] = l[1];
        bary_coord[2] = l[2];
        v_ids[0] = f_list[3 * f_id + 0];
        v_ids[1] = f_list[3 * f_id + 1];
        v_ids[2] = f_list[3 * f_id + 2];
        return true;
      }

      // not in the face, test which edge it intersects
      for (auto hefv : poly_mesh->halfedges(PolygonMesh::Face(closest_f_id)))
      {
        Vec2 pt0 = tex_coords[poly_mesh->to_vertex(hefv)];
        Vec2 pt1 = tex_coords[poly_mesh->from_vertex(hefv)];
        if (doIntersect(uv_pt, uv_f_center_pt, pt0, pt1))
        {
          // find the intersect halfedge then we search the other side face
          PolygonMesh::Halfedge oppo_he = poly_mesh->opposite_halfedge(hefv);
          if (poly_mesh->is_boundary(oppo_he))
          {
            meet_boundary = true;
            break;
          }
          else
          {
            closest_f_id = poly_mesh->face(oppo_he).idx();
            break;
          }
        }
      }
    }

    if (!meet_boundary && n_visited_f == max_visited_f)
    {
      // we use the last closest face
      float l[3];
      std::vector<Vec2> pts;
      for (auto vfc : poly_mesh->vertices(PolygonMesh::Face(closest_f_id)))
      {
        pts.push_back(tex_coords[vfc]);
      }
      computeBaryCentreCoord(uv_pt, pts[0], pts[1], pts[2], l);

      //l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
      //l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
      //l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
      //if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
      //{
      f_id = closest_f_id;
      bary_coord[0] = l[0];
      bary_coord[1] = l[1];
      bary_coord[2] = l[2];
      v_ids[0] = f_list[3 * f_id + 0];
      v_ids[1] = f_list[3 * f_id + 1];
      v_ids[2] = f_list[3 * f_id + 2];
      return true;
      //}
    }
    if(meet_boundary)
    {
      return false;
    }
  }


  void saveParameterization(std::string file_path, std::shared_ptr<Shape> shape, std::string fname)
  {
    STLVectorf vertex_list;
    const STLVectorf& UV_list = shape->getUVCoord();
    for (size_t i = 0; i < UV_list.size() / 2; ++i)
    {
      vertex_list.push_back(UV_list[2 * i + 0]);
      vertex_list.push_back(UV_list[2 * i + 1]);
      vertex_list.push_back(0.0f);
    }

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    tinyobj::shape_t obj_shape;

    obj_shape.mesh.positions = vertex_list;
    obj_shape.mesh.indices = shape->getFaceList();
    shapes.push_back(obj_shape);
    WriteObj(file_path + "/parameterization_" + fname + ".obj", shapes, materials);

    obj_shape.mesh.positions = shape->getVertexList();
    obj_shape.mesh.indices = shape->getFaceList();
    obj_shape.mesh.texcoords = shape->getUVCoord();
    shapes.pop_back();
    shapes.push_back(obj_shape);
    WriteObj(file_path + "/cutmesh_" + fname + ".obj", shapes, materials);
  }

  //void findFaceId(int& x, int& y, int& resolution, std::shared_ptr<KDTreeWrapper> kdTree, AdjList& adjFaces_list, std::shared_ptr<Shape> shape,
  //                int& face_id, float* lambda, int& id1, int& id2, int& id3)
  //{
  //  int pt_id;
  //  std::vector<float> pt;
  //  pt.resize(2);
  //  pt[0] = float(x) / resolution;
  //  pt[1] = float(y) / resolution;
  //  kdTree->nearestPt(pt,pt_id); // pt has been modified
  //  std::vector<int> adjFaces = adjFaces_list[pt_id];
  //  float point[3];
  //  point[0] = float(x) / resolution;//pt[0];
  //  point[1] = float(y) / resolution;;//pt[1];
  //  point[2] = 0;
  //  for(size_t i = 0; i < adjFaces.size(); i ++)
  //  {
  //    float l[3];
  //    int v1_id,v2_id,v3_id;
  //    float v1[3],v2[3],v3[3];
  //    v1_id = (shape->getFaceList())[3 * adjFaces[i]];
  //    v2_id = (shape->getFaceList())[3 * adjFaces[i] + 1];
  //    v3_id = (shape->getFaceList())[3 * adjFaces[i] + 2];
  //    v1[0] = (shape->getUVCoord())[2 * v1_id];
  //    v1[1] = (shape->getUVCoord())[2 * v1_id + 1];
  //    v1[2] = 0;
  //    v2[0] = (shape->getUVCoord())[2 * v2_id];
  //    v2[1] = (shape->getUVCoord())[2 * v2_id + 1];
  //    v2[2] = 0;
  //    v3[0] = (shape->getUVCoord())[2 * v3_id];
  //    v3[1] = (shape->getUVCoord())[2 * v3_id + 1];
  //    v3[2] = 0;
  //    ShapeUtility::computeBaryCentreCoord(point,v1,v2,v3,l);
  //    l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
  //    l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
  //    l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
  //    if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
  //    {
  //      face_id = adjFaces[i];
  //      lambda[0] = l[0];
  //      lambda[1] = l[1];
  //      lambda[2] = l[2];
  //      id1 = v1_id;
  //      id2 = v2_id;
  //      id3 = v3_id;
  //    }
  //  }
  //}

  void computeSolidAngleCurvature(std::shared_ptr<Model> model)
  {
    // voxelize the mesh
    double voxel_size = model->getBoundBox()->getRadius() / 200;
    VoxelerLibrary::Voxeler voxeler(model->getPolygonMesh(), voxel_size);
    std::vector<VoxelerLibrary::Voxel>& voxels = voxeler.fillInside();
    double voxelSize = voxeler.voxelSize;

    std::vector<tinyobj::shape_t> shapes(1, tinyobj::shape_t());
    std::vector<tinyobj::material_t> materials;
    shapes[0].mesh.positions.clear();

    KDTreeWrapper voxel_kd;
    std::vector<float> voxel_kd_data;

    for (size_t i = 0; i < voxels.size(); ++i)
    {
      Vector3f c(voxels[i].x, voxels[i].y, voxels[i].z);
      c *= voxelSize;

      voxel_kd_data.push_back(c(0));
      voxel_kd_data.push_back(c(1));
      voxel_kd_data.push_back(c(2));
    }

    voxel_kd.initKDTree(voxel_kd_data, voxels.size(), 3);
    double voxelVol = voxelSize * voxelSize * voxelSize;
    Vector4f sphereVol(1, 2, 3, 4);
    sphereVol = sphereVol * model->getBoundBox()->getRadius() / 200; //radius here
    Vector4f sphereR = sphereVol;
    sphereVol = (sphereVol.array() * sphereVol.array() * sphereVol.array()).matrix();
    sphereVol = 4 * M_PI / 3 * sphereVol; // volume

    Vector4f n_voxel_in_sphere = sphereVol / voxelVol;
    std::cout << "n voxels in shpere: " << n_voxel_in_sphere.transpose() << std::endl;

    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Vector4f> solid_angles = poly_mesh->vertex_attribute<Vector4f>("v:solid_angle");
    float perc = 0;
    std::vector<double> maxs(4, std::numeric_limits<double>::min());
    for (auto vit : poly_mesh->vertices())
    {
      Vec3 pos = poly_mesh->position(vit);
      std::vector<float> query(3, 0);
      query[0] = pos(0);
      query[1] = pos(1);
      query[2] = pos(2);

      std::vector<float> dis;
      voxel_kd.rNearestPt(sphereR(3) * sphereR(3), query, dis);
      std::cout << dis.size() << std::endl;system("pause");
      int r_id = 0;
      solid_angles[vit] << 0, 0, 0, 0;
      for (size_t i = 0; i < dis.size(); ++i)
      {
        if (dis[i] >= (pow(sphereR(r_id) - voxel_size, 2)))
        {
          solid_angles[vit](r_id) = voxelVol * i / sphereVol(r_id);
          if (maxs[r_id] < solid_angles[vit](r_id)) maxs[r_id] = solid_angles[vit](r_id);
          ++r_id;
          if (r_id == 3)
          {
            solid_angles[vit](3) = voxelVol * dis.size() / sphereVol(3);
            if (maxs[r_id] < solid_angles[vit](r_id)) maxs[r_id] = solid_angles[vit](r_id);
            break;
          }
        }
      }

      float cur_perc = (float)vit.idx() / poly_mesh->n_vertices();
      if (cur_perc - perc >= 0.05)
      {
        perc = cur_perc;
        std::cout << perc << "...";
      }
    }
    std::cout << std::endl;

    for (size_t i = 0; i < maxs.size(); ++i)
    {
      std::cout << maxs[i] << "\t";
    }
    std::cout << std::endl;
  }

  void computeCurvature(std::shared_ptr<Model> model)
  {
    computeHalfedgeAngle(model->getPolygonMesh());
    computeMeanCurvature(model);
    computeGaussianCurvature(model);
  }

  void computeMeanCurvature(std::shared_ptr<Model> model)
  {
    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Halfedge_attribute<Scalar> halfedge_angle = poly_mesh->halfedge_attribute<Scalar>("he:halfedge_angle");
    PolygonMesh::Vertex_attribute<Scalar> mean_curvature = poly_mesh->vertex_attribute<Scalar>("v:mean_curvature");

    Scalar min_curv = std::numeric_limits<Scalar>::max();
    Scalar max_curv = std::numeric_limits<Scalar>::min();
    
    for (auto vit : poly_mesh->vertices())
    {
      Vec3 delta(0, 0, 0);
      for (auto hevc : poly_mesh->halfedges(vit))
      {
        PolygonMesh::Vertex vj = poly_mesh->to_vertex(hevc);
        Scalar cot_alpha = 0;
        if (!poly_mesh->is_boundary(hevc))
        {
          cot_alpha = cos(halfedge_angle[hevc]) / sin(halfedge_angle[hevc]);
        }
        Scalar cot_beta = 0;
        if (!poly_mesh->is_boundary(poly_mesh->opposite_halfedge(hevc)))
        {
          cot_beta = cos(halfedge_angle[poly_mesh->opposite_halfedge(hevc)]) / sin(halfedge_angle[poly_mesh->opposite_halfedge(hevc)]);
        }
        delta += (cot_alpha + cot_beta) * (poly_mesh->position(vj) - poly_mesh->position(vit));
      }
      mean_curvature[vit] = (delta / 2).norm() / 2;
      if (mean_curvature[vit] > max_curv) max_curv = mean_curvature[vit];
      if (mean_curvature[vit] < min_curv) min_curv = mean_curvature[vit];
    }

    for (auto vit : poly_mesh->vertices())
    {
      mean_curvature[vit] = (mean_curvature[vit] - min_curv) / (max_curv - min_curv);
    }

    std::cout << "max mean curvature: " << max_curv << "\tmin mean curvature: " << min_curv << std::endl;
  }

  void computeGaussianCurvature(std::shared_ptr<Model> model)
  {
    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Halfedge_attribute<Scalar> halfedge_angle = poly_mesh->halfedge_attribute<Scalar>("he:halfedge_angle");
    PolygonMesh::Vertex_attribute<Scalar> gaussian_curvature = poly_mesh->vertex_attribute<Scalar>("v:gaussian_curvature");

    Scalar min_curv = std::numeric_limits<Scalar>::max();
    Scalar max_curv = std::numeric_limits<Scalar>::min();

    for (auto vit : poly_mesh->vertices())
    {
      Scalar theta_j = 0;
      for (auto hevc : poly_mesh->halfedges(vit))
      {
        if (!poly_mesh->is_boundary(hevc))
        {
          theta_j += halfedge_angle[poly_mesh->next_halfedge(hevc)];
        }
      }
      gaussian_curvature[vit] = 2 * M_PI - theta_j;

      if (gaussian_curvature[vit] > max_curv) max_curv = gaussian_curvature[vit];
      if (gaussian_curvature[vit] < min_curv) min_curv = gaussian_curvature[vit];
    }

    for (auto vit : poly_mesh->vertices())
    {
      gaussian_curvature[vit] = (gaussian_curvature[vit] - min_curv) / (max_curv - min_curv);
    }

    std::cout << "max gaussian curvature: " << max_curv << "\tmin gaussian curvature: " << min_curv << std::endl;
  }

  void computeHalfedgeAngle(LG::PolygonMesh* poly_mesh)
  {
    PolygonMesh::Halfedge_attribute<Scalar> halfedge_angle = poly_mesh->halfedge_attribute<Scalar>("he:halfedge_angle");
    for (auto heit : poly_mesh->halfedges())
    {
      if (poly_mesh->is_boundary(heit))
      {
        halfedge_angle[heit] = 0;
      }
      else
      {
        // assume it is triangle mesh
        Vec3 v0 = poly_mesh->position(poly_mesh->from_vertex(heit));
        Vec3 v1 = poly_mesh->position(poly_mesh->to_vertex(heit));
        Vec3 v2 = poly_mesh->position(poly_mesh->to_vertex(poly_mesh->next_halfedge(heit)));
        Scalar c = (v1 - v0).norm();
        Scalar a = (v2 - v1).norm();
        Scalar b = (v0 - v2).norm();
        halfedge_angle[heit] = acos((a * a + b * b - c * c) / (2 * a * b));
      }
    }
  }

  void computeLocalTransform(PolygonMesh* src_mesh, PolygonMesh* tar_mesh)
  {
    // src_mesh is the base mesh, the coarse one
    // tar_mesh is the transformed mesh, the deformed one
    // src_mesh and tar_mesh should be with same mesh only with different vertex position

    // we define the local coordinate system: tangent -> x, normal cross tangent -> y, normal -> z

    PolygonMesh::Vertex_attribute<Vec3> local_transform = tar_mesh->vertex_attribute<Vec3>("v:local_transform");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = src_mesh->vertex_attribute<Vec3>("v:normal");
    PolygonMesh::Vertex_attribute<Vec3> v_tangents = src_mesh->vertex_attribute<Vec3>("v:tangent");
    for (auto vit : tar_mesh->vertices())
    {
      Vec3 new_v = tar_mesh->position(vit);
      Vec3 old_v = src_mesh->position(vit);

      //Vec3 centroid(0, 0, 0);
      //int n_cnt = 0;
      //for (auto vvc : src_mesh->vertices(vit))
      //{
      //  centroid += src_mesh->position(vvc);
      //  ++n_cnt;
      //}
      //centroid = centroid / n_cnt;

      //Vec3 tangent = centroid - old_v; tangent.normalize();
      //Vec3 normal = v_normals[vit]; normal.normalize();

      Vec3 tangent = v_tangents[vit]; tangent.normalize();
      if (isnan(tangent(0)))
      {
        tangent(0) = (rand() / double(RAND_MAX));
        tangent(1) = (rand() / double(RAND_MAX));
        tangent(2) = (rand() / double(RAND_MAX));
        tangent.normalize();
      }
      Vec3 normal = v_normals[vit]; normal.normalize();
      
      Vec3 yy = normal.cross(tangent);
      tangent = yy.cross(normal);
      Matrix3f local_trans_mat;
      local_trans_mat << tangent, yy, normal;
      local_transform[vit] = local_trans_mat.inverse() * (new_v - old_v);
      if (isnan(local_transform[vit](0)))
      {
        std::cout << "nan happens in local transform computing, vid: " << vit.idx() << std::endl;
      }
    }
  }

  void applyLocalTransform(std::shared_ptr<Shape> src_shape, std::shared_ptr<Shape> tar_shape)
  {
    PolygonMesh* src_mesh = src_shape->getPolygonMesh();
    PolygonMesh* tar_mesh = tar_shape->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Vec3> local_transform = src_mesh->vertex_attribute<Vec3>("v:local_transform");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = tar_mesh->vertex_attribute<Vec3>("v:normal");
    VertexList new_vertices = tar_shape->getVertexList();
    for (auto vit : tar_mesh->vertices())
    {
      Vec3 t_v = tar_mesh->position(vit);

      Vec3 centroid(0, 0, 0);
      int n_cnt = 0;
      for (auto vvc : tar_mesh->vertices(vit))
      {
        centroid += tar_mesh->position(vvc);
        ++n_cnt;
      }
      centroid = centroid / n_cnt;

      Vec3 tangent = centroid - t_v; tangent.normalize();
      Vec3 normal = v_normals[vit]; normal.normalize();

      Vec3 yy = normal.cross(tangent);
      tangent = yy.cross(normal);
      Matrix3f local_trans_mat;
      local_trans_mat << tangent, yy, normal;
      Vec3 n_t_v = local_trans_mat * local_transform[vit] + t_v;
      new_vertices[3 * vit.idx() + 0] = n_t_v(0);
      new_vertices[3 * vit.idx() + 1] = n_t_v(1);
      new_vertices[3 * vit.idx() + 2] = n_t_v(2);
    }

    tar_shape->updateShape(new_vertices);
  }

  void applyLocalTransform(PolygonMesh* src_mesh, PolygonMesh* tar_mesh)
  {
    PolygonMesh::Vertex_attribute<Vec3> local_transform = src_mesh->vertex_attribute<Vec3>("v:local_transform");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = tar_mesh->vertex_attribute<Vec3>("v:normal");
    VertexList new_vertices(3 * tar_mesh->n_vertices(), 0);

    for (auto vit : tar_mesh->vertices())
    {
      Vec3 t_v = tar_mesh->position(vit);

      Vec3 centroid(0, 0, 0);
      int n_cnt = 0;
      for (auto vvc : tar_mesh->vertices(vit))
      {
        centroid += tar_mesh->position(vvc);
        ++n_cnt;
      }
      centroid = centroid / n_cnt;

      Vec3 tangent = centroid - t_v; tangent.normalize();
      Vec3 normal = v_normals[vit]; normal.normalize();
      Vec3 yy = normal.cross(tangent);
      tangent = yy.cross(normal);
      Matrix3f local_trans_mat;
      local_trans_mat << tangent, yy, normal;
      Vec3 n_t_v = local_trans_mat * local_transform[vit] + t_v;
      new_vertices[3 * vit.idx() + 0] = n_t_v(0);
      new_vertices[3 * vit.idx() + 1] = n_t_v(1);
      new_vertices[3 * vit.idx() + 2] = n_t_v(2);
    }

    for (auto vit : tar_mesh->vertices())
    {
      tar_mesh->position(vit) = Vec3(new_vertices[3 * vit.idx() + 0], new_vertices[3 * vit.idx() + 1], new_vertices[3 * vit.idx() + 2]);
    }
  }

  void prepareLocalTransform(PolygonMesh* src_mesh, PolygonMesh* tar_mesh, const std::vector<STLVectori>& src_v_ids, const STLVectori& v_ids, STLVectorf& new_v_list, float scale)
  {
    PolygonMesh::Vertex_attribute<Vec3> local_transform = src_mesh->vertex_attribute<Vec3>("v:local_transform");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = tar_mesh->vertex_attribute<Vec3>("v:normal");
    PolygonMesh::Vertex_attribute<Vec3> v_tangents = tar_mesh->vertex_attribute<Vec3>("v:tangent");

    new_v_list.clear();
    new_v_list.resize(3 * v_ids.size(), 0);
    for (size_t i = 0; i < v_ids.size(); ++i)
    {
      PolygonMesh::Vertex cur_v(v_ids[i]);

      Vec3 tangent = v_tangents[cur_v]; tangent.normalize();
      if (isnan(tangent(0)))
      {
        tangent(0) = (rand() / double(RAND_MAX));
        tangent(1) = (rand() / double(RAND_MAX));
        tangent(2) = (rand() / double(RAND_MAX));
        tangent.normalize();
      }
      Vec3 normal = v_normals[cur_v]; normal.normalize();
      Vec3 yy = normal.cross(tangent);
      tangent = yy.cross(normal);
      Matrix3f local_trans_mat;
      local_trans_mat << tangent, yy, normal;
      Vec3 t_v = tar_mesh->position(cur_v);

      Vec3 n_t_v(0, 0, 0);
      int n_cnt = 0;
      for (size_t j = 0; j < src_v_ids[i].size(); ++j)
      {
        PolygonMesh::Vertex cur_src_v(src_v_ids[i][j]);
        Vec3 cur_n_t_v = local_trans_mat * (scale * local_transform[cur_src_v]) + t_v;
        n_t_v += cur_n_t_v;
        ++n_cnt;

        if (isnan(cur_n_t_v(0)))
        {
          std::cout << "nan happens in local transform transferring, src_vid: " << cur_src_v.idx() << "\ttar_vid: " << cur_v.idx() << std::endl;
        }
      }
      n_t_v = n_t_v / n_cnt;
      //tar_mesh->position(cur_v) = t_v + 5 * v_normals[cur_v];continue;
      
      //Vec3 centroid(0, 0, 0);
      //int n_cnt = 0;

      //for (auto vvc : tar_mesh->vertices(cur_v))
      //{
      //  centroid += tar_mesh->position(vvc);
      //  ++n_cnt;
      //}
      //centroid = centroid / n_cnt;

      //Vec3 tangent = centroid - t_v; tangent.normalize();
      //Vec3 normal = v_normals[cur_v]; normal.normalize();

      new_v_list[3 * i + 0] = n_t_v(0);
      new_v_list[3 * i + 1] = n_t_v(1);
      new_v_list[3 * i + 2] = n_t_v(2);
    }

    for (size_t i = 0; i < v_ids.size(); ++i)
    {
      //tar_mesh->position(PolygonMesh::Vertex(v_ids[i])) = Vec3(new_v_list[3 * i + 0], new_v_list[3 * i + 1], new_v_list[3 * i + 2]);
    }
  }

  void savePolyMesh(LG::PolygonMesh* poly_mesh, std::string fName)
  {
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    tinyobj::shape_t obj_shape;

    obj_shape.mesh.positions.resize(3 * poly_mesh->n_vertices(), 0);
    obj_shape.mesh.indices.resize(3 * poly_mesh->n_faces(), 0);
    obj_shape.mesh.uv_indices.resize(3 * poly_mesh->n_faces(), 0);

    std::vector<Vec2>& he_texcoord = poly_mesh->get_attribute<std::vector<Vec2> >("he:texcoord");
    for (size_t i = 0; i < he_texcoord.size(); ++i)
    {
      obj_shape.mesh.texcoords.push_back(he_texcoord[i](0));
      obj_shape.mesh.texcoords.push_back(he_texcoord[i](1));
    }
    for (auto vit : poly_mesh->vertices())
    {
      Vec3 cur_v = poly_mesh->position(vit);
      obj_shape.mesh.positions[3 * vit.idx() + 0] = cur_v(0);
      obj_shape.mesh.positions[3 * vit.idx() + 1] = cur_v(1);
      obj_shape.mesh.positions[3 * vit.idx() + 2] = cur_v(2);
    }
    PolygonMesh::Halfedge_attribute<int> f_uv_id = poly_mesh->halfedge_attribute<int>("he:uv_id");
    for (auto fit : poly_mesh->faces())
    {
      int n_cnt = 0;
      for (auto hefc : poly_mesh->halfedges(fit))
      {
        obj_shape.mesh.indices[3 * fit.idx() + n_cnt] = poly_mesh->to_vertex(hefc).idx();
        obj_shape.mesh.uv_indices[3 * fit.idx() + n_cnt] = f_uv_id[hefc];
        ++n_cnt;
      }
    }

    shapes.push_back(obj_shape);

    WriteObj(fName, shapes, materials);
  }

  int findLeftTopUVVertex(LG::PolygonMesh* poly_mesh, std::set<int>& f_ids)
  {
    float max_v = std::numeric_limits<float>::min();
    int v_id = -1;
    std::vector<Vec2>& he_texcoord = poly_mesh->get_attribute<std::vector<Vec2> >("he:texcoord");
    PolygonMesh::Halfedge_attribute<int> f_uv_id = poly_mesh->halfedge_attribute<int>("he:uv_id");
    for (auto f : f_ids)
    {
      for (auto hefc : poly_mesh->halfedges(PolygonMesh::Face(f)))
      {
        Vec2 cur_uv = he_texcoord[f_uv_id[hefc]];
        if (cur_uv(1) > max_v)
        {
          max_v = cur_uv(1);
          v_id = poly_mesh->to_vertex(hefc).idx();
        }
      }
    }
    return v_id;
  }

  int closestVertex(PolygonMesh* src_mesh, std::vector<int>& src_v_ids, PolygonMesh* tar_mesh, int tar_v_id)
  {
    int closest_v_id = -1;
    Scalar min_dist = std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < src_v_ids.size(); ++i)
    {
      Scalar cur_dist = (src_mesh->position(PolygonMesh::Vertex(src_v_ids[i])) - tar_mesh->position(PolygonMesh::Vertex(tar_v_id))).norm();
      if (cur_dist < min_dist)
      {
        min_dist = cur_dist;
        closest_v_id = src_v_ids[i];
      }
    }
    return closest_v_id;
  }

  void getAverageNormalAroundVertex(LG::PolygonMesh* poly_mesh, int v_id, LG::Vec3& normal, int n_ring)
  {
    //
    PolygonMesh::Face_attribute<Vec3> f_normals = poly_mesh->face_attribute<Vec3>("f:normal");
    
    std::set<int> f_ids;
    getNRingFacesAroundVertex(poly_mesh, f_ids, v_id, n_ring - 1);

    normal = LG::Vec3(0, 0, 0);
    for (auto i : f_ids)
    {
      normal += f_normals[PolygonMesh::Face(i)];
    }
    normal.normalize();
  }

  void initBSPTreeRayFromPolyMesh(Ray* ray, LG::PolygonMesh* poly_mesh)
  {
    VertexList vertex_list;
    for (auto vit : poly_mesh->vertices())
    {
      const Vec3& pt = poly_mesh->position(vit);
      vertex_list.push_back(pt[0]);
      vertex_list.push_back(pt[1]);
      vertex_list.push_back(pt[2]);
    }
    FaceList face_list;
    for (auto fit : poly_mesh->faces())
    {
      for (auto vfc_it : poly_mesh->vertices(fit))
      {
        face_list.push_back(vfc_it.idx());
      }
    } 
    ray->passModel(vertex_list, face_list); // initialize the displacement mesh ray
  }

  void visibleFacesInModel(std::shared_ptr<Model> model, std::set<int>& visible_faces)
  {
    cv::Mat& primitive_ID_img = model->getPrimitiveIDImg();
    visible_faces.clear();
    for (int i = 0; i < primitive_ID_img.rows; ++i)
    {
      for (int j = 0; j < primitive_ID_img.cols; ++j)
      {
        int face_id = primitive_ID_img.at<int>(i, j);
        if (face_id >= 0)
        {
          visible_faces.insert(face_id);
        }
      }
    }
  }
  void visibleVerticesInModel(std::shared_ptr<Model> model, std::set<int>& visible_vertices)
  {
    std::set<int> visible_faces;
    visible_vertices.clear();
    const FaceList& face_list = model->getShapeFaceList();
    for (auto i : visible_faces)
    {
      visible_vertices.insert(face_list[3 * i + 0]);
      visible_vertices.insert(face_list[3 * i + 1]);
      visible_vertices.insert(face_list[3 * i + 2]);
    }
  }

  void nRingVertices(LG::PolygonMesh* poly_mesh, int v_id, std::set<int>& vertices, int n_ring)
  {
    std::set<int> f_ids;
    getNRingFacesAroundVertex(poly_mesh, f_ids, v_id, n_ring);

    vertices.clear();
    for (auto i : f_ids)
    {
      for (auto vfc : poly_mesh->vertices(PolygonMesh::Face(i)))
      {
        vertices.insert(vfc.idx());
      }
    }
  }

  int getVisiblePatchIDinPatches(std::vector<ParaShape>& patches, std::set<int>& ori_visible_faces)
  {
    int best_id = 0;
    int best_face_cnt = 0;
    for (size_t i = 0; i < patches.size(); ++i)
    {
      int cur_face_cnt = 0;
      for (auto j : ori_visible_faces)
      {
        if (patches[i].cut_faces.find(j) != patches[i].cut_faces.end())
        {
          ++cur_face_cnt;
        }
      }
      if (cur_face_cnt > best_face_cnt)
      {
        best_face_cnt = cur_face_cnt;
        best_id = int(i);
      }
    }
    return best_id;
  }
}