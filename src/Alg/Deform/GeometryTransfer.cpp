#include "GeometryTransfer.h"

#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"

#include "Solver.h"
#include "ARAP.h"
#include "FastMassSpring.h"
#include "MoveConstraint.h"

#include "KDTreeWrapper.h"
#include "obj_writer.h"

#include <fstream>

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

void GeometryTransfer::transferDeformation(std::shared_ptr<Model> tar_model, const std::vector<int>& v_ids, const std::vector<float>& v_list, float lamd_move)
{
  FaceList face_list = tar_model->getShapeFaceList();
  VertexList vertex_list = tar_model->getShapeVertexList();
  NormalList normal_list = tar_model->getShapeNormalList();
  AdjList vertex_shared_faces = tar_model->getShapeVertexShareFaces();
  AdjList adj_list = tar_model->getShapeVertexAdjList();

  std::shared_ptr<Solver> solver(new Solver);
  std::shared_ptr<FastMassSpring> fms(new FastMassSpring);
  std::shared_ptr<MoveConstraint> move_constraint(new MoveConstraint);

  solver->problem_size = vertex_list.size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());
  solver->addConstraint(fms);
  solver->addConstraint(move_constraint);

  fms->setSolver(solver);
  fms->initEdgeGraph(face_list, vertex_list, vertex_shared_faces);
  fms->buildMatrix();
  fms->setkStrech(1.0);
  fms->setkBending(5.0);

  move_constraint->setSolver(solver);
  move_constraint->initMatrix(v_ids, v_list);
  move_constraint->setLamdMove(lamd_move);

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

  //std::ofstream fdebug(tar_model->getOutputPath() + "/v_ids.txt");
  //if (fdebug)
  //{
  //  for (size_t i = 0; i < v_ids.size(); ++i)
  //  {
  //    fdebug << v_ids[i] << std::endl;
  //  }
  //  
  //  fdebug.close();
  //}

  tar_model->updateShape(new_vertex_list);
    // map new texture
  //model->updateColor(); // this is for build uv coordinates
  //model->updateSHColor();

  tar_model->exportOBJ(0);

  std::cout << "Update geometry finished...\n";
}

void GeometryTransfer::debugDeformation(std::shared_ptr<Model> tar_model)
{
  FaceList face_list = tar_model->getShapeFaceList();
  VertexList vertex_list = tar_model->getShapeVertexList();
  NormalList normal_list = tar_model->getShapeNormalList();
  AdjList vertex_shared_faces = tar_model->getShapeVertexShareFaces();
  AdjList adj_list = tar_model->getShapeVertexAdjList();

  int v_index = 0;
  for (auto i : face_list)
  {
    if (i > v_index) v_index = i;
  }
  std::cout << v_index << "\t" << tar_model->getPolygonMesh()->n_vertices() << std::endl;

  std::vector<int> v_ids;
  std::vector<float> v_list;
  std::ifstream fdebug(tar_model->getDataPath() + "/move_debug.txt");
  if (fdebug.is_open())
  {
    std::string line;
    while (getline(fdebug, line))
    {
      std::stringstream parser(line);
      int v_id = 0;
      STLVectorf v_pos(3, 0);
      parser >> v_id >> v_pos[0] >> v_pos[1] >> v_pos[2];
      v_ids.push_back(v_id);
      v_list.push_back(v_pos[0]);
      v_list.push_back(v_pos[1]);
      v_list.push_back(v_pos[2]);
    }

    fdebug.close();
  }

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::shape_t obj_shape;

  obj_shape.mesh.positions = v_list;
  shapes.push_back(obj_shape);
  WriteObj(tar_model->getOutputPath() + "/debug_deformation.obj", shapes, materials);//return;


  std::shared_ptr<Solver> solver(new Solver);
  std::shared_ptr<ARAP> arap(new ARAP);
  std::shared_ptr<MoveConstraint> move_constraint(new MoveConstraint);

  solver->problem_size = vertex_list.size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());
  solver->addConstraint(arap);
  solver->addConstraint(move_constraint);

  arap->setSolver(solver);
  //arap->initConstraint(vertex_list, face_list, adj_list);
  arap->initConstraint(tar_model->getPolygonMesh());
  arap->setLamdARAP(5.0f);

  move_constraint->setSolver(solver);
  move_constraint->initMatrix(v_ids, v_list);
  move_constraint->setLamdMove(5.0f);

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

  std::ofstream ofdebug(tar_model->getOutputPath() + "/P_Opt.txt");
  if (ofdebug)
  {
    for (size_t i = 0; i < new_vertex_list.size() / 3; ++i)
    {
      ofdebug << new_vertex_list[3 * i + 0] << "\t" << new_vertex_list[3 * i + 1] << "\t" << new_vertex_list[3 * i + 2] << std::endl;
    }
    
    ofdebug.close();
  }

  tar_model->updateShape(new_vertex_list);
  // map new texture
  //model->updateColor(); // this is for build uv coordinates
  //model->updateSHColor();

  tar_model->exportOBJ(0);

  std::cout << "Update geometry finished...\n";
}