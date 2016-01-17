#include "MeshParameterization.h"
#include "Model.h"
#include "Shape.h"
#include "ParaShape.h"

#include "obj_writer.h"
#include "CurvesUtility.h"
#include "ShapeUtility.h"
#include "KDTreeWrapper.h"
#include "PolygonMesh.h"

#include <cv.h>
#include <set>

using namespace LG;

MeshParameterization::MeshParameterization()
{
  this->init();
}

MeshParameterization::~MeshParameterization()
{

}

void MeshParameterization::init()
{
  seen_part = nullptr;
  unseen_part = nullptr;
}

void MeshParameterization::doMeshParameterization(std::shared_ptr<Model> model)
{
  seen_part.reset(new ParaShape);
  unseen_part.reset(new ParaShape);
  this->cutMesh(model);
  this->prepareCutShape(model, seen_part->cut_face_list, seen_part->vertex_set, seen_part->cut_shape);
  this->findBoundary(seen_part->cut_shape, seen_part->boundary_loop);
  this->computeBaryCentericPara(seen_part->cut_shape, seen_part->boundary_loop);
  ShapeUtility::saveParameterization(model->getOutputPath(), seen_part->cut_shape, "seen");
  
  this->expandCutShape(model, unseen_part->cut_faces);
  this->prepareCutShape(model, unseen_part->cut_face_list, unseen_part->vertex_set, unseen_part->cut_shape);
  this->findBoundary(unseen_part->cut_shape, unseen_part->boundary_loop);
  this->computeBaryCentericPara(unseen_part->cut_shape, unseen_part->boundary_loop);
  ShapeUtility::saveParameterization(model->getOutputPath(), unseen_part->cut_shape, "unseen");
  this->buildKDTree_UV();
  this->getNormalOfOriginalMesh(model);
  this->getVertexOfOriginalMesh(model);
}

void MeshParameterization::cutMesh(std::shared_ptr<Model> model)
{
  // detect visible faces
  cv::Mat& primitive_ID_img = model->getPrimitiveIDImg();
  seen_part->cut_faces.clear();
  for (int i = 0; i < primitive_ID_img.rows; ++i)
  {
    for (int j = 0; j < primitive_ID_img.cols; ++j)
    {
      int face_id = primitive_ID_img.at<int>(i, j);
      if (face_id >= 0)

      {
        seen_part->cut_faces.insert(face_id);
      }
    }
  }

  // fill holes
  const AdjList& f_adjList = model->getShapeFaceAdjList();
  this->fillHoles(seen_part->cut_faces, f_adjList);

  // delete single face
  std::vector<std::set<int> > connected_components;
  this->connectedComponents(connected_components, seen_part->cut_faces, f_adjList);

  // find largest component
  int largest_id = this->findLargestComponent(connected_components);
  if (largest_id != -1)
  {
    //seen_part->cut_faces = connected_components[largest_id];
  }

  // get complement of visible faces to the full faces
  std::set<int> full_faces;
  std::set<int> complement_faces;
  for (size_t i = 0; i < f_adjList.size(); ++i)
  {
    full_faces.insert(i);
  }
  std::set_difference(full_faces.begin(), full_faces.end(),
                      seen_part->cut_faces.begin(), seen_part->cut_faces.end(),
                      std::inserter(complement_faces, complement_faces.begin()));

  // merge complement faces into connected components
  connected_components.clear();
  this->connectedComponents(connected_components, complement_faces, f_adjList);

  // insert all components into visible faces except the largest one 
  // so finally we cut the mesh into two part
  largest_id = this->findLargestComponent(connected_components);
  if (largest_id != -1)
  {
    for (size_t i = 0; i < connected_components.size(); ++i)
    {
      if (i != largest_id)
      {
        seen_part->cut_faces.insert(connected_components[i].begin(), connected_components[i].end());
      }
    }
  }

  // deal with single face leads to bad case in parameterization
  this->eliminateSingleFaceAll(model, seen_part->cut_faces);

  // save cut_face_list
  seen_part->cut_face_list.clear();
  seen_part->face_set.clear();
  const FaceList& ori_face_list = model->getShapeFaceList();
  for (auto i : seen_part->cut_faces)
  {
    seen_part->cut_face_list.push_back(ori_face_list[3 * i + 0]);
    seen_part->cut_face_list.push_back(ori_face_list[3 * i + 1]);
    seen_part->cut_face_list.push_back(ori_face_list[3 * i + 2]);
    seen_part->face_set.push_back(i);
  }

  // get hidden face_list
  unseen_part->cut_faces.clear();
  std::set_difference(full_faces.begin(), full_faces.end(),
                      seen_part->cut_faces.begin(), seen_part->cut_faces.end(),
                      std::inserter(unseen_part->cut_faces, unseen_part->cut_faces.begin()));
  unseen_part->cut_face_list.clear();
  unseen_part->face_set.clear();
  for (auto i : unseen_part->cut_faces)
  {
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 0]);
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 1]);
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 2]);
    unseen_part->face_set.push_back(i);
  }

  //std::vector<tinyobj::shape_t> shapes;
  //std::vector<tinyobj::material_t> materials;
  //tinyobj::shape_t obj_shape;

  //obj_shape.mesh.positions = model->getShapeVertexList();
  //obj_shape.mesh.indices = cut_face_list;
  //shapes.push_back(obj_shape);
  //WriteObj(model->getOutputPath() + "/cutface.obj", shapes, materials);
}

void MeshParameterization::expandCutShape(std::shared_ptr<Model> model, std::set<int>& f_id_set)
{
  // expand the boundary

  // first we generate the initial shape
  this->prepareCutShape(model, unseen_part->cut_face_list, unseen_part->vertex_set, unseen_part->cut_shape);
  this->findBoundary(unseen_part->cut_shape, unseen_part->boundary_loop);

  // now we have the boundary id
  PolygonMesh* poly_mesh = model->getPolygonMesh();
  std::set<int> new_f_id;
  for (auto i : unseen_part->boundary_loop)
  {
    // insert all n-ring neighbor faces around vertex into the f_id_set
    ShapeUtility::getNRingFacesAroundVertex(poly_mesh, new_f_id, unseen_part->vertex_set[i], 3);
    f_id_set.insert(new_f_id.begin(), new_f_id.end());
    new_f_id.clear();
  }

  // build face
  this->eliminateSingleFaceAll(model, f_id_set);

  // build final face
  //f_id_set.swap(left_f);
  const FaceList& ori_face_list = model->getShapeFaceList();
  unseen_part->cut_face_list.clear();
  unseen_part->face_set.clear();
  for (auto i : f_id_set)
  {
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 0]);
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 1]);
    unseen_part->cut_face_list.push_back(ori_face_list[3 * i + 2]);
    unseen_part->face_set.push_back(i);
  }
}

bool MeshParameterization::eliminateSingleFace(std::shared_ptr<Model> model, std::set<int>& f_id_set)
{
  const FaceList& ori_face_list = model->getShapeFaceList();
  FaceList cut_face_list;
  STLVectori face_set;
  for (auto i : f_id_set)
  {
    cut_face_list.push_back(ori_face_list[3 * i + 0]);
    cut_face_list.push_back(ori_face_list[3 * i + 1]);
    cut_face_list.push_back(ori_face_list[3 * i + 2]);
    face_set.push_back(i);
  }

  STLVectori vertex_set;
  std::shared_ptr<Shape> cut_shape;
  this->prepareCutShape(model, cut_face_list, vertex_set, cut_shape);
  std::set<int> invalid_f;
  PolygonMesh* cut_poly_mesh = cut_shape->getPolygonMesh();
  for (auto fit : cut_poly_mesh->faces())
  {
    int b_edge = 0;
    for (auto hefc : cut_poly_mesh->halfedges(fit))
    {
      if (cut_poly_mesh->is_boundary(cut_poly_mesh->edge(hefc)))
      {
        ++b_edge;
      }
    }
    if (b_edge > 1)
    {
      invalid_f.insert(face_set[fit.idx()]);
    }
  }
  std::set<int> left_f;
  std::set_difference(f_id_set.begin(), f_id_set.end(), invalid_f.begin(), invalid_f.end(), std::inserter(left_f, left_f.begin()));

  f_id_set.swap(left_f);
  if (invalid_f.empty()) return false;
  else return true;
}

void MeshParameterization::eliminateSingleFaceAll(std::shared_ptr<Model> model, std::set<int>& f_id_set)
{
  this->eliminateSingleFace(model, f_id_set);
  //while (this->eliminateSingleFace(model, f_id_set))
  //{
  //}
}

void MeshParameterization::prepareCutShape(std::shared_ptr<Model> model, FaceList& f_list, STLVectori& v_set, std::shared_ptr<Shape>& shape)
{
  //get all parameterization related vertices
  const FaceList& face_list = model->getShapeFaceList();

  // vertex_set store the vertex id map from old full mesh to new cut mesh
  v_set.clear();
  for (auto i : f_list)
  {
    v_set.push_back(i);
  }
  std::sort(v_set.begin(), v_set.end());
  v_set.erase(std::unique(v_set.begin(), v_set.end()), v_set.end());
  v_set.shrink_to_fit();

  const VertexList& vertex_list = model->getShapeVertexList();
  VertexList new_vertex_list;
  for (auto i : v_set)
  {
    new_vertex_list.push_back(vertex_list[3 * i + 0]);
    new_vertex_list.push_back(vertex_list[3 * i + 1]);
    new_vertex_list.push_back(vertex_list[3 * i + 2]);
  }
  FaceList new_face_list;
  for (auto i : f_list)
  {
    size_t id = std::distance(v_set.begin(), std::find(v_set.begin(), v_set.end(), i));
    new_face_list.push_back(id);
  }
  STLVectorf UVList(v_set.size() * 2, 0.0f);
  FaceList UVIdList;

  shape.reset(new Shape);
  shape->init(new_vertex_list, new_face_list, UVIdList, UVList);
}

void MeshParameterization::findBoundary(std::shared_ptr<Shape> shape, STLVectori& b_loop, int start_v_id)
{
  // find all boundaries
  //PolygonMesh* poly_mesh = shape->getPolygonMesh();
  //std::vector<STLVectori> boundary_lines;
  //for (auto vit : poly_mesh->vertices())
  //{
  //  if (poly_mesh->is_boundary(vit))
  //  {
  //    bool visited = false;
  //    for (size_t i = 0; i < boundary_lines.size(); ++i)
  //    {
  //      if (std::find(boundary_lines[i].begin(), boundary_lines[i].end(), vit.idx()) != boundary_lines[i].end())
  //      {
  //        visited = true;
  //        break;
  //      }
  //    }
  //    if (!visited)
  //    {
  //      PolygonMesh::Halfedge b_halfedge = poly_mesh->halfedge(vit);
  //      PolygonMesh::Halfedge b_iter_halfedge = b_halfedge;
  //      STLVectori cur_b_loop;
  //      do 
  //      {
  //        cur_b_loop.push_back(poly_mesh->to_vertex(b_halfedge).idx());
  //        b_iter_halfedge = poly_mesh->next_halfedge(b_iter_halfedge);
  //      } while (b_iter_halfedge != b_halfedge);
  //      boundary_lines.push_back(cur_b_loop);
  //    }
  //  }
  //}

  const STLVectori& edge_connectivity = shape->getEdgeConnectivity();
  const FaceList& face_list = shape->getFaceList();
  std::vector<Edge> boundary_edges;
  int inner_index[6] = {0, 1, 1, 2, 2, 0};
  for (size_t i = 0; i < edge_connectivity.size(); ++i)
  {
    if (edge_connectivity[i] == -1)
    {
      // a boundary edge
      int face_id = i / 3;
      int inner_id = i % 3;
      int v0 = face_list[3 * face_id + inner_index[2 * inner_id + 0]];
      int v1 = face_list[3 * face_id + inner_index[2 * inner_id + 1]];
      boundary_edges.push_back(Edge(v0, v1));
    }
  }
  std::vector<STLVectori> boundary_lines;
  CurvesUtility::mergeShapeEdges(boundary_edges, boundary_lines);
  // use the longest boundary for parameterization
  int longest_id = -1;
  size_t longest_len = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < boundary_lines.size(); ++i)
  {
    if (boundary_lines[i].size() > longest_len)
    {
      longest_len = boundary_lines[i].size();
      longest_id = i;
    }
  }
  STLVectori longest_b_loop;
  if (longest_id != -1)
  {
    longest_b_loop = boundary_lines[longest_id];
  }

  PolygonMesh* poly_mesh = shape->getPolygonMesh();
  if (!poly_mesh->is_boundary(PolygonMesh::Vertex(longest_b_loop[0])))
  {
    std::cout << "\n Error in find boundary !!!\n";
  }
  else
  {
    PolygonMesh::Halfedge b_he_start = poly_mesh->halfedge(PolygonMesh::Vertex(longest_b_loop[0]));
    PolygonMesh::Halfedge b_he_iter = b_he_start;
    b_loop.clear();
    do 
    {
      b_loop.push_back(poly_mesh->to_vertex(b_he_iter).idx());
      b_he_iter = poly_mesh->next_halfedge(b_he_iter);
    } while (b_he_iter != b_he_start);

    if (start_v_id == -1)
    {
      int shuffle = std::min(int((rand() / double(RAND_MAX)) * b_loop.size()), int(b_loop.size() - 1));
      std::rotate(b_loop.begin(), b_loop.begin() + shuffle, b_loop.end());
    }
    else
    {
      size_t pos = std::distance(b_loop.begin(), std::find(b_loop.begin(), b_loop.end(), start_v_id));
      if (pos == b_loop.size())
      {
        std::cout << "\nThe start v_id in boundary loop not found!!!\n";
      }
      else
      {
        std::rotate(b_loop.begin(), b_loop.begin() + pos, b_loop.end());
      }
    }

    //std::cout << longest_b_loop.size() << "\t" << b_loop.size() << std::endl;
    //std::cout << longest_b_loop[0] << "\t" << longest_b_loop[longest_b_loop.size() - 1] << std::endl;
  }
  //b_loop = longest_b_loop;
}

void MeshParameterization::computeBaryCentericPara(std::shared_ptr<Shape>& shape, STLVectori& b_loop)
{
  // map boundary loop to unit circle in texture domain
  const VertexList& vertex_list = shape->getVertexList();
  STLVectorf UV_list(2 * vertex_list.size() / 3, 0.0f);
  this->mapBoundary(UV_list, b_loop, vertex_list, 1);

  // setup matrix and rhs
  // 1. delete boundary loop vertex from the vertex list
  STLVectori free_vertices;
  for (size_t i = 0; i < vertex_list.size() / 3; ++i)
  {
    if (std::find(b_loop.begin(), b_loop.end(), i) == b_loop.end())
    {
      free_vertices.push_back(i);
    }
  }

  // 2. fill matrix
  const AdjList& v_adjlist = shape->getVertexAdjList();
  size_t N = free_vertices.size();
  SparseMatrix A(N, N);
  TripletList A_triplets;
  std::vector<VectorXf> b(2, VectorXf(N));
  std::vector<VectorXf> x(2, VectorXf(N));
  std::map<int, float> row;
  std::map<int, float>::iterator r_it;
  for (size_t i = 0; i < N; ++i)
  {
    row.clear();
    this->computeLaplacianWeight(free_vertices[i], row, shape);
    b[0][i] = 0.0; b[1][i] = 0.0;

    for (r_it = row.begin(); r_it != row.end(); ++r_it)
    {
      if (std::find(b_loop.begin(), b_loop.end(), r_it->first) != b_loop.end())
      {
        b[0][i] -= r_it->second * UV_list[2 * r_it->first + 0];
        b[1][i] -= r_it->second * UV_list[2 * r_it->first + 1];
      }
      else
      {
        size_t idx = std::distance(free_vertices.begin(), std::find(free_vertices.begin(), free_vertices.end(), r_it->first));
        A_triplets.push_back(Triplet(i, idx, r_it->second));
      }
    }
  }

  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::SimplicialLLT<SparseMatrix> solver(A);

  x[0] = solver.solve(b[0]);
  x[1] = solver.solve(b[1]);

  for (size_t i = 0; i < N; ++i)
  {
    UV_list[2 * free_vertices[i] + 0] = x[0][i];
    UV_list[2 * free_vertices[i] + 1] = x[1][i];
  }

  shape->setUVCoord(UV_list);
}

void MeshParameterization::mapBoundary(STLVectorf& UV_list, const STLVectori& boundary_loop, const VertexList& vertex_list, int b_type /* = 0 */)
{
  // map the boundary loop to a kind of shape

  // compute the length of boundary loop
  float length = 0.0;
  size_t n = boundary_loop.size();
  for (size_t i = 0; i < n; ++i)
  {
    int v_0 = boundary_loop[i];
    int v_1 = boundary_loop[(i + 1) % n];

    Vector3f diff(vertex_list[3 * v_0 + 0] - vertex_list[3 * v_1 + 0],
      vertex_list[3 * v_0 + 1] - vertex_list[3 * v_1 + 1],
      vertex_list[3 * v_0 + 2] - vertex_list[3 * v_1 + 2]);

    length += diff.norm();
  }

  if (b_type == 0)
  {
    // map boundary to unit circle
    float l = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
      float angle = l / length * 2.0 * M_PI + 5 * M_PI / 4;
      UV_list[2 * boundary_loop[i] + 0] = 0.5 * cos(angle) + 0.5;
      UV_list[2 * boundary_loop[i] + 1] = 0.5 * sin(angle) + 0.5;

      int v_0 = boundary_loop[i];
      int v_1 = boundary_loop[(i + 1) % n];

      Vector3f diff(vertex_list[3 * v_0 + 0] - vertex_list[3 * v_1 + 0],
        vertex_list[3 * v_0 + 1] - vertex_list[3 * v_1 + 1],
        vertex_list[3 * v_0 + 2] - vertex_list[3 * v_1 + 2]);

      l += diff.norm();
    }
  }
  else if (b_type == 1)
  {
    // map boundary to unit square
    float l = 0.0;
    float side_length = length / 4;
    Vector2f start(0.0, 0.0);
    Vector2f dir(1.0, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
      float offset = l / side_length;
      UV_list[2 * boundary_loop[i] + 0] = start[0] + offset * dir[0];
      UV_list[2 * boundary_loop[i] + 1] = start[1] + offset * dir[1];

      int v_0 = boundary_loop[i];
      int v_1 = boundary_loop[(i + 1) % n];

      Vector3f diff(vertex_list[3 * v_0 + 0] - vertex_list[3 * v_1 + 0],
        vertex_list[3 * v_0 + 1] - vertex_list[3 * v_1 + 1],
        vertex_list[3 * v_0 + 2] - vertex_list[3 * v_1 + 2]);

      l += diff.norm();

      if (l > side_length)
      {
        l -= side_length;
        start += dir;
        dir = Vector2f(- dir[1], dir[0]);
      }
    }
  }
}

void MeshParameterization::computeLaplacianWeight(int v_id, std::map<int, float>& weight, std::shared_ptr<Shape> shape)
{
  //const STLVectori& v_ring = shape->getVertexAdjList()[v_id];
  //const VertexList& vertex_list = shape->getVertexList();
  //float wi = 0.0f;

  //for (size_t i = 0; i < v_ring.size(); ++i)
  //{
  //  STLVectori share_vertex;
  //  this->findShareVertex(v_id, v_ring[i], share_vertex, shape);

  //  float wij = 0.0f;
  //  if (share_vertex.size() == 2) 
  //  {
  //    wij = computeWij(&vertex_list[3 * v_id], &vertex_list[3 * v_ring[i]], 
  //      &vertex_list[3 * share_vertex[0]], &vertex_list[3 * share_vertex[1]]);
  //  }
  //  else wij = computeWij(&vertex_list[3 * v_id], &vertex_list[3 * v_ring[i]], &vertex_list[3*share_vertex[0]]);

  //  weight[v_ring[i]] = -wij;
  //  wi += wij;
  //}
  //weight[v_id] = wi;

  float wi = 0.0f;
  PolygonMesh* mesh = shape->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  PolygonMesh::Halfedge_around_vertex_circulator hec, hec_end;
  hec = hec_end = mesh->halfedges(PolygonMesh::Vertex(v_id));
  do 
  {
    int vj = mesh->to_vertex(*hec).idx();
    double wij = laplacian_cot[mesh->edge(*hec)];
    weight[vj] = -(float)wij;
    wi += (float)wij;
  } while (++hec != hec_end);
  weight[v_id] = wi;
}

void MeshParameterization::findShareVertex(int pi, int pj, STLVectori& share_vertex, std::shared_ptr<Shape> shape)
{
  STLVectori pi_f_ring = shape->getVertexShareFaces()[pi];
  STLVectori pj_f_ring = shape->getVertexShareFaces()[pj];
  std::sort(pi_f_ring.begin(), pi_f_ring.end());
  std::sort(pj_f_ring.begin(), pj_f_ring.end());

  STLVectori faces;
  std::set_intersection(pi_f_ring.begin(), pi_f_ring.end(), pj_f_ring.begin(), pj_f_ring.end(), back_inserter(faces));

  const FaceList& face_list = shape->getFaceList();
  for (size_t i = 0; i < faces.size(); ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      if (face_list[3 * faces[i] + j] != pi && face_list[3 * faces[i] + j] != pj)
      {
        share_vertex.push_back(face_list[3 * faces[i] + j]);
        break;
      }
    }
  }

  if (share_vertex.size() > 2)
  {
    std::cout << "share vertices number warning: " << share_vertex.size() << std::endl;
  }
}

float MeshParameterization::computeWij(const float *p1, const float *p2, const float *p3, const float *p4)
{
  float e1 = sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
  float e2 = sqrt((p1[0]-p3[0])*(p1[0]-p3[0])+(p1[1]-p3[1])*(p1[1]-p3[1])+(p1[2]-p3[2])*(p1[2]-p3[2]));
  float e3 = sqrt((p3[0]-p2[0])*(p3[0]-p2[0])+(p3[1]-p2[1])*(p3[1]-p2[1])+(p3[2]-p2[2])*(p3[2]-p2[2]));
  float alpha_cos = fabs((e3*e3+e2*e2-e1*e1)/(2*e3*e2));
  float beta_cos = 0;
  if (p4 != nullptr) {
    float e4 = sqrt((p1[0]-p4[0])*(p1[0]-p4[0])+(p1[1]-p4[1])*(p1[1]-p4[1])+(p1[2]-p4[2])*(p1[2]-p4[2]));
    float e5 = sqrt((p4[0]-p2[0])*(p4[0]-p2[0])+(p4[1]-p2[1])*(p4[1]-p2[1])+(p4[2]-p2[2])*(p4[2]-p2[2]));
    beta_cos = fabs((e4*e4+e5*e5-e1*e1)/(2*e4*e5));
  }
  return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
}

void MeshParameterization::findConnectedFaces(int f_id, std::set<int>& connected_faces, const std::set<int>& visible_faces, const AdjList& adj_list)
{
  if (connected_faces.find(f_id) != connected_faces.end())
  {
    return;
  }

  connected_faces.insert(f_id);
  for (size_t i = 0; i < adj_list[f_id].size(); ++i)
  {
    if (visible_faces.find(adj_list[f_id][i]) != visible_faces.end())
    {
      this->findConnectedFaces(adj_list[f_id][i], connected_faces, visible_faces, adj_list);
    }
  }
  return;
}

void MeshParameterization::connectedComponents(std::vector<std::set<int> >& components, const std::set<int>& visible_faces, const AdjList& adj_list)
{
  std::set<int> component;
  for (auto i : visible_faces)
  {
    bool already_visited = false;
    for (size_t j = 0; j < components.size(); ++j)
    {
      if (components[j].find(i) != components[j].end())
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
      component.clear();
      this->findConnectedFaces(i, component, visible_faces, adj_list);
      if (component.size() > 0)
      {
        components.push_back(component);
      }
    }
  }
}

void MeshParameterization::fillHoles(std::set<int>& visible_faces, const AdjList& f_adjList)
{
  std::set<int> hole_faces;
  do
  {
    hole_faces.clear();
    for (auto i : visible_faces)
    {
      for (size_t j = 0; j < f_adjList[i].size(); ++j)
      {
        int adj_f = f_adjList[i][j];
        if (visible_faces.find(adj_f) == visible_faces.end())
        {
          // test if the adjacent faces of this new face is in visible_faces
          // if so, insert it into hole_faces
          int cnt_faces_in_visible = 0;
          for (size_t k = 0; k < f_adjList[adj_f].size(); ++k)
          {
            if (visible_faces.find(f_adjList[adj_f][k]) != visible_faces.end())
            {
              ++cnt_faces_in_visible;
            }
          }
          if (cnt_faces_in_visible >= 2)
          {
            hole_faces.insert(adj_f);
          }


          // old method which assuming all faces have 3 adjacent faces
          // and this is not suitable
          //int fs[2];
          //if (f_adjList[adj_f][0] == i) { fs[0] = f_adjList[adj_f][1]; fs[1] = f_adjList[adj_f][2]; }
          //else if (f_adjList[adj_f][1] == i) { fs[0] = f_adjList[adj_f][0]; fs[1] = f_adjList[adj_f][2]; }
          //else if (f_adjList[adj_f][2] == i) { fs[0] = f_adjList[adj_f][0]; fs[1] = f_adjList[adj_f][1]; }

          //if (visible_faces.find(fs[0]) != visible_faces.end() || visible_faces.find(fs[1]) != visible_faces.end())
          //{
          //  hole_faces.insert(adj_f);
          //}
        }
      }
    }

    visible_faces.insert(hole_faces.begin(), hole_faces.end());
  } while (!hole_faces.empty());
}

int MeshParameterization::findLargestComponent(const std::vector<std::set<int> >& components)
{
  int largest_id = -1;
  size_t largest_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < components.size(); ++i)
  {
    if (components[i].size() > largest_size)
    {
      largest_size = components[i].size();
      largest_id = i;
    }
  }
  return largest_id;
}

void MeshParameterization::buildKDTree_UV()
{
  seen_part->initUVKDTree();
  unseen_part->initUVKDTree();
}

void MeshParameterization::getNormalOfOriginalMesh(std::shared_ptr<Model> model)
{
  normal_original_mesh = model->getShapeNormalList();
}

void MeshParameterization::getVertexOfOriginalMesh(std::shared_ptr<Model> model)
{
  vertex_original_mesh = model->getShapeVertexList();
}

void MeshParameterization::doMeshParamterizationPatch(std::shared_ptr<Model> model, int plane_id, ParaShape* one_patch)
{
  // prepare the (plane) patch for parameterization

  this->doMeshParamterizationPatch(model, model->getPlaneFaces()[plane_id], one_patch);

  // save the parametrization
  ShapeUtility::saveParameterization(model->getOutputPath(), one_patch->cut_shape, "plane_" + std::to_string(plane_id));
}

void MeshParameterization::doMeshParamterizationPatch(std::shared_ptr<Model> model, const std::set<int>& f_ids, ParaShape* one_patch, int start_v_id)
{
  // get the patch face list
  one_patch->cut_faces = f_ids;
  one_patch->cut_face_list.clear();
  one_patch->face_set.clear();
  const FaceList& ori_face_list = model->getShapeFaceList();
  for (auto i : one_patch->cut_faces)
  {
    one_patch->cut_face_list.push_back(ori_face_list[3 * i + 0]);
    one_patch->cut_face_list.push_back(ori_face_list[3 * i + 1]);
    one_patch->cut_face_list.push_back(ori_face_list[3 * i + 2]);
    one_patch->face_set.push_back(i);
  }

  // build the new shape
  one_patch->vertex_set.clear(); // v_id mapper from new shape to original shape
  // new shape for parameterization
  this->prepareCutShape(model, one_patch->cut_face_list, one_patch->vertex_set, one_patch->cut_shape);

  // find boundary
  if (start_v_id == -1)
  {
    this->findBoundary(one_patch->cut_shape, one_patch->boundary_loop);
  }
  else
  {
    size_t pos = std::distance(one_patch->vertex_set.begin(), std::find(one_patch->vertex_set.begin(), one_patch->vertex_set.end(), start_v_id));
    if (pos == one_patch->vertex_set.size())
    {
      std::cout << "\nThe start v_id not found in para shape!!!\n";
      this->findBoundary(one_patch->cut_shape, one_patch->boundary_loop);
    }
    else
    {
      this->findBoundary(one_patch->cut_shape, one_patch->boundary_loop, int(pos));
    }
  }
  this->computeBaryCentericPara(one_patch->cut_shape, one_patch->boundary_loop);

  one_patch->initUVKDTree();
}