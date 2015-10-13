#include "ProjOptimize.h"
#include "FeatureGuided.h"
#include "Model.h"
#include "Shape.h"
#include "MainCanvasViewer.h"
#include "Solver.h"
#include "FastMassSpring.h"
#include "ProjConstraint.h"
#include "ARAP.h"
#include "CurvesUtility.h"

ProjOptimize::ProjOptimize()
{
  actors.push_back(GLActor(ML_POINT, 5.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 1.0f));

  solver = nullptr;
  arap= nullptr;
  proj_constraint = nullptr;
}

ProjOptimize::~ProjOptimize()
{

}

void ProjOptimize::updateShape(std::shared_ptr<FeatureGuided> feature_guided, std::shared_ptr<Model> model)
{

  actors[0].clearElement();
  actors[1].clearElement();
  actors[2].clearElement();

#ifdef USE_AUTO
  CURVES crsp_pairs;
  feature_guided->BuildSourceEdgeKDTree();
  feature_guided->GetUserCrspPair(crsp_pairs, model->getModelAvgEdgeLength());
//
  cv::Mat r_img_syn = cv::Mat::ones(model->getRImg().size(), CV_32FC3);
  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  std::vector<std::pair<int, int> > boundary_pts;
  for (int i = 0; i < primitive_id_img.rows; ++i)
  {
    for (int j = 0; j < primitive_id_img.cols; ++j)
    {
      if (primitive_id_img.at<int>(i, j) >= 0)
      {
        if (CurvesUtility::isBoundary(primitive_id_img, j, i))
        {
        // store as x, y
          boundary_pts.push_back(std::pair<int, int>(j, i));
          r_img_syn.at<cv::Vec3f>(i, j) = cv::Vec3f(0.5, 0.5, 0.5);
        }
      }
    }
  }
  //cv::imwrite(model->getDataPath() + "/boundary.png", r_img_syn * 255);
  std::shared_ptr<kdtree::KDTree> source_KDTree;
  kdtree::KDTreeArray kdTree_data;
  kdTree_data.resize(boost::extents[boundary_pts.size()][2]);
  for (size_t i = 0; i < boundary_pts.size(); ++i)
  {
    kdTree_data[i][0] = boundary_pts[i].first;
    kdTree_data[i][1] = boundary_pts[i].second;
  }
  source_KDTree.reset(new kdtree::KDTree(kdTree_data));

  // prepare curve on model
  // the feature line defined by user need to be resampled


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
    actors[0].addElement(world_pos[0], world_pos[1], world_pos[2], 1, 0, 0);
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

      actors[2].addElement(camera_ori[0], camera_ori[1], camera_ori[2], 1.0, 0.0, 0.0);
      actors[2].addElement(proj_ray[0], proj_ray[1], proj_ray[2], 1.0, 0.0, 0.0);
#endif
      
    }
  }

#ifdef DEBUG
  //std::ofstream debug_output(model->getOutputPath() + "/vertexlist.txt");
  //if (debug_output)
  //{
  //  for (auto& i : new_constrained_vertex_id)
  //  {
  //    debug_output << i << "\n";
  //  }
  //  debug_output.close();
  //}
#endif
#endif // USE_AUTO

#define USE_AUTO_2
#ifdef USE_AUTO_2

  std::vector<std::pair<int, double2> > crsp_list;
  feature_guided->GetCurrentCrspList(crsp_list);

  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  constrained_vertex_id.clear();
  constrained_ray.clear();

  for (size_t i = 0; i < crsp_list.size(); ++i)
  {
    constrained_vertex_id.push_back(crsp_list[i].first);

    int p_x = int(crsp_list[i].second.x + 0.5);
    // the y coordinate start from bottom in FeatureGuided
    // but from top in cv::Mat
    int p_y = primitive_id_img.rows - 1 - int(crsp_list[i].second.y + 0.5);
    float proj_ray[3];
    model->getProjRay(proj_ray, p_x, p_y);
    constrained_ray.push_back(proj_ray[0]);
    constrained_ray.push_back(proj_ray[1]);
    constrained_ray.push_back(proj_ray[2]);
  }

  float camera_ori[3];
  model->getCameraOri(camera_ori);

#endif // USE_AUTO_2



#ifdef USE_MANUL

  std::vector<CvPoint3D32f>& pts3d = model->getRenderer()->pts3d;
  std::vector<CvPoint2D32f>& pts2d = model->getRenderer()->pts2d;

  std::vector<float> new_constrained_ray;
  std::vector<int> new_constrained_vertex_id;
  float camera_ori[3];
  model->getCameraOri(camera_ori);

  for (decltype(pts2d.size()) i = 0; i < pts2d.size(); ++i)
  {
    float proj_ray[3];
    model->getProjRay(proj_ray, int(pts2d[i].x + 0.5), int(pts2d[i].y + 0.5));
    new_constrained_ray.push_back(proj_ray[0]);
    new_constrained_ray.push_back(proj_ray[1]);
    new_constrained_ray.push_back(proj_ray[2]);
    proj_ray[0] = camera_ori[0] + 10 * proj_ray[0];
    proj_ray[1] = camera_ori[1] + 10 * proj_ray[1];
    proj_ray[2] = camera_ori[2] + 10 * proj_ray[2];
    float c[3] = {1.0f, 0.0f, 0.0f};
    model->getRenderer()->addDrawableLine(camera_ori, proj_ray, c, c);

    float world_pos[3] = { pts3d[i].x, pts3d[i].y, pts3d[i].z };
    new_constrained_vertex_id.push_back(model->getClosestVertexId(world_pos));
    model->getRenderer()->addDrawablePoint(world_pos[0], world_pos[1], world_pos[2], 1, 0, 0);
  }

#endif // USE_MANUL
//
//
  std::cout << "\nBegin update geometry...\n";

  //typedef std::pair<int, int> Edge;

  std::vector<unsigned int> face_list = model->getShape()->getFaceList();
  std::vector<float> vertex_list = model->getShape()->getVertexList();
  std::vector<float> normal_list = model->getShape()->getNormalList();
  std::vector<std::vector<int> > vertices_share_faces = model->getShape()->getVertexShareFaces();
  std::vector<std::vector<int> > adj_list = model->getShape()->getVertexAdjList();

  if (!solver)
  {
    solver.reset(new Solver);
  }
  if (!proj_constraint)
  {
    proj_constraint.reset(new ProjConstraint);
    // add constraints to solver
    solver->addConstraint(proj_constraint);
  }
  if (!arap)
  {
    arap.reset(new ARAP);
    // add constraints to solver
    solver->addConstraint(arap);
  }

  // init solver info
  solver->problem_size = (vertex_list).size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());

  // init fast mass spring
  //fsm->setSolver(solver);
  //fsm->initEdgeGraph((face_list), (vertex_list), (vertices_share_faces));
  //fsm->buildMatrix();
  //fsm->setkStrech(10.0f); //10.0f;
  //fsm->setkBending(15.0f); //15.0f;

  // init arap
  arap->setSolver(solver);
  arap->initConstraint(vertex_list, face_list, adj_list);
  arap->setLamdARAP(10.0f);

  // init projection constraint
  proj_constraint->setSolver(solver);
  proj_constraint->initMatrix(constrained_ray, constrained_vertex_id, camera_ori);
  proj_constraint->setLamdProj(30.0f);

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

    //model->getRenderer()->saveScreen(model->getOutputPath() + "/" + std::to_string(cur_iter) + ".png");

  } while (cur_iter < max_iter);

  std::cout << "Update geometry finished...\n";
}

void ProjOptimize::updateShapeFromInteraction(std::shared_ptr<FeatureGuided> feature_guided, std::shared_ptr<Model> model)
{
  // assume the optimization has been initialized before by updateShape()
  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  int user_v_id = feature_guided->user_constrained_src_v;
  double2 user_win = feature_guided->user_constrained_tar_p;

  for (size_t i = 0; i < constrained_vertex_id.size(); ++i)
  {
    if (constrained_vertex_id[i] == user_v_id)
    {
      int p_x = int(user_win.x + 0.5);
      // the y coordinate start from bottom in FeatureGuided
      // but from top in cv::Mat
      int p_y = primitive_id_img.rows - 1 - int(user_win.y + 0.5);
      float proj_ray[3];
      model->getProjRay(proj_ray, p_x, p_y);
      constrained_ray[3 * i + 0] = proj_ray[0];
      constrained_ray[3 * i + 1] = proj_ray[1];
      constrained_ray[3 * i + 2] = proj_ray[2];
    }
  }

  float camera_ori[3];
  model->getCameraOri(camera_ori);

  // update the system matrix for proj_constraint
  proj_constraint->initMatrix(constrained_ray, constrained_vertex_id, camera_ori);

  // since the solver has been initialized before and we don't change the pattern of the 
  // system matrix, no need to do initCholesky. Use preFactorize() instead
  solver->preFactorize();
  int max_iter = 20; //20;
  int cur_iter = 0;
  do
  {
    solver->runOneStep();
    ++cur_iter;

    std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

    updateScreenShape(model, solver->P_Opt);

    //model->getRenderer()->saveScreen(model->getOutputPath() + "/" + std::to_string(cur_iter) + ".png");

  } while (cur_iter < max_iter);

  std::cout << "Update geometry finished...\n";
}

void ProjOptimize::updateScreenShape(std::shared_ptr<Model> model, Eigen::VectorXf& P_Opt)
{
  std::vector<float> new_vertex_list(P_Opt.data(), P_Opt.data() + P_Opt.rows() * P_Opt.cols());

  model->getShape()->updateShape(new_vertex_list);
}

void ProjOptimize::getDrawableActors(std::vector<GLActor>& actors)
{
  actors = this->actors;
}