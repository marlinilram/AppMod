#include "ProjOptimize.h"
#include "FeatureGuided.h"
#include "Model.h"
#include "viewer.h"
#include "Solver.h"
#include "FastMassSpring.h"
#include "ProjConstraint.h"

ProjOptimize::ProjOptimize()
{

}

ProjOptimize::~ProjOptimize()
{

}

void ProjOptimize::updateShape(FeatureGuided* feature_guided, Model* model)
{
  CURVES crsp_pairs;
  feature_guided->GetCrspPair(crsp_pairs);

  cv::Mat r_img_syn = cv::Mat::ones(model->getRImg().size(), CV_32FC3);
  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  std::vector<std::pair<int, int> > boundary_pts;
  for (int i = 0; i < primitive_id_img.rows; ++i)
  {
    for (int j = 0; j < primitive_id_img.cols; ++j)
    {
      if (primitive_id_img.at<int>(i, j) >= 0)
      {
        if (this->isBoundary(primitive_id_img, j, i))
        {
        // store as x, y
          boundary_pts.push_back(std::pair<int, int>(j, i));
          r_img_syn.at<cv::Vec3f>(i, j) = cv::Vec3f(0.5, 0.5, 0.5);
        }
      }
    }
  }
  //cv::imwrite(model->getDataPath() + "/boundary.png", r_img_syn * 255);
  kdtree::KDTree* source_KDTree;
  kdtree::KDTreeArray kdTree_data;
  kdTree_data.resize(boost::extents[boundary_pts.size()][2]);
  for (size_t i = 0; i < boundary_pts.size(); ++i)
  {
    kdTree_data[i][0] = boundary_pts[i].first;
    kdTree_data[i][1] = boundary_pts[i].second;
  }
  source_KDTree = new kdtree::KDTree(kdTree_data);

  std::vector<int> constrained_vertex_id;
  std::vector<float> constrained_ray;
  for (size_t i = 0; i < crsp_pairs.size(); ++i)
  {
    // get the vertex id
    int img_xy[2];
    img_xy[0] = int(crsp_pairs[i][0].x + 0.5);
     // the y coordinate start from bottom in FeatureGuided
     // but from top in cv::Mat
    img_xy[1] = primitive_id_img.rows - 1 - int(crsp_pairs[i][0].y + 0.5);

    float world_pos[3];
    if (primitive_id_img.at<int>(img_xy[1], img_xy[0]) >= 0)
    {
      // if it indeed falls in the object region
      // get the vertex id directly
      constrained_vertex_id.push_back(model->getClosestVertexId(world_pos, img_xy[0], img_xy[1]));
    }
    else
    {
      // if not we find closest boundary point
      std::vector<float> query(2, 0.0);
      kdtree::KDTreeResultVector result;
      query[0] = img_xy[0];
      query[1] = img_xy[1];
      source_KDTree->n_nearest(query, 1, result);
      // get correct boundary img_xy
      img_xy[0] = source_KDTree->the_data[result[0].idx][0];
      img_xy[1] = source_KDTree->the_data[result[0].idx][1];
      if (primitive_id_img.at<int>(img_xy[1], img_xy[0]) >= 0)
      {
        // make sure it really falls in the object region now
        constrained_vertex_id.push_back(model->getClosestVertexId(world_pos, img_xy[0], img_xy[1]));
      }
      else
      {
        std::cout << "Error: can not find correct correspondence in world space.\n";
      }
    }
#define DEBUG
#ifdef DEBUG
    model->getRenderer()->addDrawablePoint(world_pos[0], world_pos[1], world_pos[2], 1, 0, 0);
#endif

    // get ray corresponding to the vertex
    img_xy[0] = int(crsp_pairs[i][1].x + 0.5);
    // the y coordinate start from bottom in FeatureGuided
    // but from top in cv::Mat
    img_xy[1] = primitive_id_img.rows - 1 - int(crsp_pairs[i][1].y + 0.5);
    float proj_ray[3];
    model->getProjRay(proj_ray, img_xy[0], img_xy[1]);
    constrained_ray.push_back(proj_ray[0]);
    constrained_ray.push_back(proj_ray[1]);
    constrained_ray.push_back(proj_ray[2]);
  }
  float camera_ori[3];
  model->getCameraOri(camera_ori);

  std::vector<int> new_constrained_vertex_id;
  std::vector<float> new_constrained_ray;
  std::vector<int>::iterator iter;
  for (size_t i = 0; i < constrained_vertex_id.size(); ++i)
  {
    iter = std::find(new_constrained_vertex_id.begin(), new_constrained_vertex_id.end(), constrained_vertex_id[i]);
    if (iter == new_constrained_vertex_id.end())
    {
      new_constrained_vertex_id.push_back(constrained_vertex_id[i]);
      new_constrained_ray.push_back(constrained_ray[3 * i + 0]);
      new_constrained_ray.push_back(constrained_ray[3 * i + 1]);
      new_constrained_ray.push_back(constrained_ray[3 * i + 2]);

#ifdef DEBUG
      float camera_ori[3];
      float proj_ray[3] = {constrained_ray[3 * i + 0], constrained_ray[3 * i + 1], constrained_ray[3 * i + 2]};
      model->getCameraOri(camera_ori);
      proj_ray[0] = camera_ori[0] + 10 * proj_ray[0];
      proj_ray[1] = camera_ori[1] + 10 * proj_ray[1];
      proj_ray[2] = camera_ori[2] + 10 * proj_ray[2];
      float c[3] = {1.0f, 0.0f, 0.0f};
      model->getRenderer()->addDrawableLine(camera_ori, proj_ray, c, c);
#endif
      
    }
  }

#ifdef DEBUG
  std::ofstream debug_output(model->getOutputPath() + "/vertexlist.txt");
  if (debug_output)
  {
    for (auto& i : new_constrained_vertex_id)
    {
      debug_output << i << "\n";
    }
    debug_output.close();
  }
#endif


  std::cout << "\nBegin update geometry...\n";

  //typedef std::pair<int, int> Edge;

  std::vector<unsigned int> *face_list = model->getFaceList();
  std::vector<float> *vertex_list = model->getVertexList();
  std::vector<float> *normal_list = model->getNormalList();
  std::vector<std::vector<int> > *vertices_share_faces = model->getVertexShareFaces();

  Solver* solver = new Solver;
  FastMassSpring* fsm = new FastMassSpring;
  ProjConstraint* proj_constraint = new ProjConstraint;

  // init solver info
  solver->problem_size = (*vertex_list).size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(*vertex_list)[0], (*vertex_list).size());

  // init fast mass spring
  fsm->setSolver(solver);
  fsm->initEdgeGraph((*face_list), (*vertex_list), (*vertices_share_faces));
  fsm->buildMatrix();
  fsm->setkStrech(10.0f); //10.0f;
  fsm->setkBending(15.0f); //15.0f;

  // init projection constraint
  proj_constraint->setSolver(solver);
  proj_constraint->initMatrix(new_constrained_ray, new_constrained_vertex_id, camera_ori);
  proj_constraint->setLamdProj(30.0f);

  // add constraints to solver
  solver->addConstraint(fsm);
  solver->addConstraint(proj_constraint);

  // solve
  solver->initCholesky();
  int max_iter = 20; //20;
  int cur_iter = 0;
  do
  {
    solver->runOneStep();
    ++cur_iter;

    std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

    updateScreenShape(model, solver->P_Opt);

  } while (cur_iter < max_iter);

  delete proj_constraint;
  delete fsm;
  delete solver;
  delete source_KDTree;
  std::cout << "Update geometry finished...\n";
}

bool ProjOptimize::isBoundary(cv::Mat& primitive_img, int x, int y)
{
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      int row_id = y + i - 1;
      int col_id = x + j - 1;
      row_id = std::max(std::min(row_id, primitive_img.rows - 1), 0);
      col_id = std::max(std::min(col_id, primitive_img.cols - 1), 0);

      if (!(row_id == y && col_id == x))
      {
        if (primitive_img.at<int>(row_id, col_id) < 0)
        {
          return true;
        }
      }
    }
  }
  return false;
}

void ProjOptimize::updateScreenShape(Model* model, Eigen::VectorXf& P_Opt)
{
  std::vector<float> *vertex_list = model->getVertexList();

  for (decltype((*vertex_list).size()) i = 0; i < (*vertex_list).size(); ++i)
  {
    (*vertex_list)[i] = P_Opt(i);
  }

  model->computeFaceNormal();

  model->computeVertexNormal();

  model->updateBSPtree();

  //model->computeLight();

  model->getRenderer()->getModel(model);

  model->getRenderer()->UpdateGLOutside();
}