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

    lambd[1] = (d11*d20 - d01*d21) / denom;

    lambd[2] = (d00*d21 - d01*d20) / denom;

    lambd[0] = 1.0f - lambd[1] - lambd[2];

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
    while (!meet_boundary)
    {
      // test if the uv_pt is inside the face
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

    if (meet_boundary)
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

    return false;
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
}