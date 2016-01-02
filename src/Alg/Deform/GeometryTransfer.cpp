#include "GeometryTransfer.h"

#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"

#include "Solver.h"
#include "ARAP.h"
#include "MoveConstraint.h"

#include "KDTreeWrapper.h"

using namespace LG;

GeometryTransfer::GeometryTransfer()
{

}

GeometryTransfer::~GeometryTransfer()
{

}

void GeometryTransfer::prepareSampleVertex(std::shared_ptr<Model> tar_model, std::vector<int>& v_ids)
{
  // load sampled mesh

  PolygonMesh sample_mesh;
  read_poly(sample_mesh, tar_model->getDataPath() + "/sampled.obj");

  v_ids.clear();
  std::shared_ptr<Shape> tar_shape = tar_model->getShape();
  for (auto vit : sample_mesh.vertices())
  {
    STLVectorf query(3, 0.0);
    Vec3 cur_pt = sample_mesh.position(vit);
    query[0] = cur_pt(0);
    query[1] = cur_pt(1);
    query[2] = cur_pt(2);
    int v_id = 0;
    tar_shape->getKDTree()->nearestPt(query, v_id);
    v_ids.push_back(v_id);
  }

  std::sort(v_ids.begin(), v_ids.end());
  v_ids.erase(std::unique(v_ids.begin(), v_ids.end()), v_ids.end());
}

void GeometryTransfer::transferDeformation(std::shared_ptr<Model> tar_model, const std::vector<int>& v_ids, const std::vector<float>& v_list)
{
  FaceList face_list = tar_model->getShapeFaceList();
  VertexList vertex_list = tar_model->getShapeVertexList();
  NormalList normal_list = tar_model->getShapeNormalList();
  AdjList vertex_shared_faces = tar_model->getShapeVertexShareFaces();
  AdjList adj_list = tar_model->getShapeVertexAdjList();

  std::shared_ptr<Solver> solver(new Solver);
  std::shared_ptr<ARAP> arap(new ARAP);
  std::shared_ptr<MoveConstraint> move_constraint(new MoveConstraint);

  solver->problem_size = vertex_list.size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());

  arap->setSolver(solver);
  arap->initConstraint(vertex_list, face_list, adj_list);
  arap->setLamdARAP(5.0f);

  move_constraint->setSolver(solver);
  move_constraint->initMatrix(v_ids, v_list);

  solver->initCholesky();
  int max_iter = 20;
  int cur_iter = 0;
  do 
  {
    solver->runOneStep();
    ++cur_iter;

    std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

  } while (cur_iter < max_iter);

  std::vector<float> new_vertex_list(solver->P_Opt.data(), solver->P_Opt.data() + solver->P_Opt.rows() * solver->P_Opt.cols());

  tar_model->updateShape(new_vertex_list);
  // map new texture
  //model->updateColor(); // this is for build uv coordinates
  //model->updateSHColor();

  std::cout << "Update geometry finished...\n";
}