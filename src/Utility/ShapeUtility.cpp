#include "ShapeUtility.h"
#include "BasicHeader.h"

#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"
#include "Bound.h"
#include "ParaShape.h"

#include "Ray.h"
#include "SAMPLE.h"
#include "GenerateSamples.h"

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

    lambd[1] = (d11*d20 - d01*d21) / denom;

    lambd[2] = (d00*d21 - d01*d20) / denom;

    lambd[0] = 1.0f - lambd[1] - lambd[2];

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
    int sqrtNumSamples = 50;
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
          //Eigen::Vector3d ray_start = (poly_mesh->position(i) + 2 * 0.01 * bound->getRadius() * v_normals[i]).cast<double>();
          //Eigen::Vector3d ray_end   = ray_start + (5 * bound->getRadius() * samples[k].direction).cast<double>();
          //if (ray->intersectModel(ray_start, ray_end))
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

    for (auto i : poly_mesh->vertices())
    {
      for (int l = 0; l < numFunctions; ++l)
      {
        shadowCoeff[i][l] = (shadowCoeff[i][l] - min_coeff[l]) / (max_coeff[l] - min_coeff[l]);
      }
    }

    std::cout << "Compute Directional Occlusion Feature finished.\n";
  }


  void computeSymmetry(std::shared_ptr<Model> model)
  {
    std::cout << "Generate or Load symmetry.txt." << std::endl;
    bool regenerate = false;
    // test if the file exist
    std::ifstream inFile(model->getDataPath() + "/symmetry.txt");
    if (!inFile.is_open())
    {
      std::cout << "Not existed or failed to load." << std::endl;
      regenerate = true;
    }

    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Vec3> v_symmetry = poly_mesh->vertex_attribute<Vec3>("v:symmetry");

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
        Vec3 cur_v_symmetry(0,0,0);
        line_parser >> cur_v_symmetry[0] >> cur_v_symmetry[1] >> cur_v_symmetry[2];
        v_symmetry[vit] = cur_v_symmetry;
        ++n_line;
      }
      
      inFile.close();
      if (n_line == poly_mesh->n_vertices()) 
      {
        std::cout << "Loading finished." << std::endl;
      }
      else 
      {
        std::cout << "Loading errors. Need to regenerate." << std::endl;
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
      for (auto vit : poly_mesh->vertices())
      {
        Vec3 pos = poly_mesh->position(vit);
        v_symmetry[vit] = Vec3(fabs(pos[1]), pos[0], pos[2]);
        outFile << fabs(pos[1]) << "\t" << pos[0] << "\t" << pos[2] << std::endl;
      }
      outFile.close();
      std::cout << "Generating finished." << std::endl;
    }

    // normalize the value
    Vector3f mins(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vector3f maxs(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (auto vit : poly_mesh->vertices())
    {
      Vec3 cur_sym = v_symmetry[vit];
      if (cur_sym[0] < mins[0]) mins[0] = cur_sym[0];
      if (cur_sym[0] > maxs[0]) maxs[0] = cur_sym[0];
      if (cur_sym[1] < mins[1]) mins[1] = cur_sym[1];
      if (cur_sym[1] > maxs[1]) maxs[1] = cur_sym[1];
      if (cur_sym[2] < mins[2]) mins[2] = cur_sym[2];
      if (cur_sym[2] > maxs[2]) maxs[2] = cur_sym[2];
    }
    for (auto vit : poly_mesh->vertices())
    {
      Vec3 cur_sym = v_symmetry[vit];
      v_symmetry[vit] = ((cur_sym - mins).array() / (maxs - mins).array()).matrix();
    }
  }

  void computeRMSCurvature(std::shared_ptr<Model> model)
  {
    // 1. compute 
  }

  void getNRingFacesAroundVertex(LG::PolygonMesh* poly_mesh, std::set<int>& f_id, int v_id, int n_ring)
  {
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
          if (mat_ptr[offset] <= 0)
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
        if (mat_ptr[ooffset] > 0)
        {
          value += mat_ptr[ooffset];
          ++n_value;
        }
      }
    }
    if (n_value != 0)
    {
      filled_mat_ptr[offset] = 1 + value / n_value;
    }
  }

  void matToMesh(cv::Mat& mat, LG::PolygonMesh& mesh, std::shared_ptr<Model> shape_model)
  {
    int width = mat.size().width;
    int height = mat.size().height;
    Vector3f dir ;
    dir << 0, 0, -1;
    shape_model->getUnprojectVec(dir);
    dir.normalize();

    for(int i = 0; i < width; i ++)
    {
      for(int j = 0; j < height; j ++)
      {
        Vector3f img_coord, w_coord;
        img_coord << i, j, 1;
        shape_model->getWorldCoord(img_coord, w_coord);
        Vector3f mesh_pt;
        mesh_pt = w_coord + 0.1 * dir * mat.at<float>(j, i);
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

}