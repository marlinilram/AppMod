#include "ParaShape.h"
#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"

#include "KDTreeWrapper.h"
#include "ShapeUtility.h"

using namespace LG;

ParaShape::ParaShape()
{
  cut_shape = nullptr;
  kdTree_UV = nullptr;
}

ParaShape::~ParaShape()
{

}

void ParaShape::initUVKDTree()
{
  kdTree_UV.reset(new KDTreeWrapper);
  kdTree_UV->initKDTree(std::vector<float>(cut_shape->getUVCoord()), cut_shape->getUVCoord().size() / 2, 2);

  // if it's synthesis target uv could complicated stored in halfedges
  PolygonMesh* poly_mesh = cut_shape->getPolygonMesh();
  PolygonMesh::Vertex_attribute<Vec2> tex_coords = poly_mesh->vertex_attribute<Vec2>("v:texcoord");
  std::vector<float> uv_f_center(2 * poly_mesh->n_faces(), 0);
  for (auto fit : poly_mesh->faces())
  {
    Vec2 uv_center(0, 0);
    int n_pts = 0;
    for (auto vfc : poly_mesh->vertices(fit))
    {
      uv_center += tex_coords[vfc];
      ++ n_pts;
    }
    uv_center = uv_center / n_pts;
    uv_f_center[2 * fit.idx() + 0] = uv_center[0];
    uv_f_center[2 * fit.idx() + 1] = uv_center[1];
  }
  kdTree_UV_f.reset(new KDTreeWrapper);
  kdTree_UV_f->initKDTree(uv_f_center, poly_mesh->n_faces(), 2);
}

void ParaShape::initWithExtShape(std::shared_ptr<Model> model)
{
  this->initWithExtPolygonMesh(model->getPolygonMesh());

  ShapeUtility::saveParameterization(model->getOutputPath(), cut_shape, "ext_target");
}

void ParaShape::initWithExtPolygonMesh(PolygonMesh* poly_mesh)
{
  // we regenerate a shape
  // probably we need to regenerate it by uv?
  // let the v list is same to uv list and face list is same to uvid list
  cut_shape.reset(new Shape);

  // we need to duplicate the vertices for each triangle...
  PolygonMesh::Halfedge_attribute<int> f_uv_id = poly_mesh->halfedge_attribute<int>("he:uv_id");
  std::vector<Vec2>& f_uv_coord = poly_mesh->get_attribute<std::vector<Vec2> >("he:texcoord");
  int n_faces = poly_mesh->n_faces();
  VertexList v_list(3 * f_uv_coord.size(), 0);
  FaceList f_list(3 * n_faces, 0);
  STLVectorf uv_list(2 * f_uv_coord.size(), 0);
  vertex_set.clear();
  vertex_set.resize(f_uv_coord.size(), 0);
  cut_faces.clear();
  face_set.clear();
  //PolygonMesh::Halfedge_attribute<Vec2> f_uv_coord = poly_mesh->halfedge_attribute<Vec2>("he:face_uv");
  for (int i = 0; i < n_faces; ++i)
  {
    int n_face_pt = 0;
    for (auto hefc : poly_mesh->halfedges(PolygonMesh::Face(i)))
    {
      int cur_uv_id = f_uv_id[hefc];

      v_list[3 * cur_uv_id + 0] = f_uv_coord[cur_uv_id][0];
      v_list[3 * cur_uv_id + 1] = f_uv_coord[cur_uv_id][1];
      v_list[3 * cur_uv_id + 2] = 0.0;
      uv_list[2 * cur_uv_id + 0] = f_uv_coord[cur_uv_id][0];
      uv_list[2 * cur_uv_id + 1] = f_uv_coord[cur_uv_id][1];
      f_list[3 * i + n_face_pt] = cur_uv_id;
      vertex_set[cur_uv_id] = poly_mesh->to_vertex(hefc).idx();

      ++n_face_pt;
    }
    cut_faces.insert(i);
    face_set.push_back(i);
  }
  cut_shape->init(v_list, f_list, f_list, uv_list);
  cut_face_list = cut_shape->getFaceList();

  //VertexList v_list = model->getShapeVertexList();
  //FaceList   f_list = model->getShapeFaceList();
  //STLVectorf uv_list = model->getShapeUVCoord();
  //cut_shape->init(v_list, f_list, f_list, uv_list);

  //cut_face_list = cut_shape->getFaceList();

  //vertex_set.clear();
  //for (int i = 0; i < cut_shape->getPolygonMesh()->n_vertices(); ++i)
  //{
  //  vertex_set.push_back(i);
  //}

  //boundary_loop.clear();

  //cut_faces.clear();
  //face_set.clear();
  //for (int i = 0; i < cut_shape->getPolygonMesh()->n_faces(); ++i)
  //{
  //  cut_faces.insert(i);
  //  face_set.push_back(i);
  //}

  initUVKDTree();
}