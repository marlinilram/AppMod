#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "ScalarField.h"
#include "Model.h"
#include "PolygonMesh.h"

#include "nlopt.hpp"
#include <ctime>
#include <fstream>

#include "ParameterMgr.h"
#include "BasicHeader.h"
#include "LOG.h"

using namespace LG;

void LargeFeatureReg::runReg(int method_id)
{
  std::cout << "begin Large Feature Rigid Registration" << std::endl;
  unsigned time1 = clock();

  // start point
  int x_dim = 6;
  std::vector<double> x0(x_dim, 0.0);
  //x0[x_dim - 1] = 1.0; // scale start from 1

  double start_func_val = this->energyFunc(x0);
  //std::cout << "Start f(x): " << start_func_val << std::endl;

  // set nlopt handle
  if (method_id >= nlopt::NUM_ALGORITHMS)
  {
    std::cout << "Required method does not exist. Use default method.\n";
    method_id = 0;
  }
  nlopt::opt opt(nlopt::algorithm(method_id), x_dim);

  // set bound
  std::vector<double> lb(x_dim);
  std::vector<double> ub(x_dim);
  for (size_t i = 0; i < x_dim; ++i)
  {
    if (i < 3)
    {
      lb[i] = -0.5 * modelRadius();
      ub[i] = 0.5 * modelRadius();
    }
    else if (i < 6)
    {
      lb[i] = -M_PI / 15;
      ub[i] = M_PI / 15;
    }
    //else
    //{
    //  lb[i] = 0.5;
    //  ub[i] = 1.5;
    //}
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  // set objective function
  int SField_type = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("SField:Type");
  if (SField_type == 0 || SField_type == 1)
  {
    opt.set_min_objective(LFReg::efunc, this);
  }
  else if (SField_type == 2)
  {
    opt.set_min_objective(LFReg::efunc, this);
  }

  // set stop criteria
  opt.set_ftol_rel(0.001);
  opt.set_xtol_rel(0.001);
  opt.set_maxtime(30);

  // get result
  double maxf;
  nlopt::result result;
  try{
    result = opt.optimize(x0, maxf);
  }catch (std::exception& e){  std::cerr << "exception caught: " << e.what() << '\n'; }
  Matrix4f& cur_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  cur_transform = 
    (Eigen::Translation3f(x0[0], x0[1], x0[2])
    * Eigen::AngleAxisf(x0[5], Vector3f::UnitZ())
    * Eigen::AngleAxisf(x0[4], Vector3f::UnitY())
    * Eigen::AngleAxisf(x0[3], Vector3f::UnitZ())).matrix();
    //* Eigen::Scaling(float(x0[6]))).matrix();

  std::stringstream log_stream;
  log_stream << "LFReg Log:" << std::endl;
  log_stream << "Method: " << method_id << " " << opt.get_algorithm_name() << std::endl;
  log_stream << "Start f(x): " << start_func_val << "\tStop f(x): " << maxf << std::endl;
  log_stream << "optimized transform: " << std::endl;
  log_stream << cur_transform << std::endl;
  log_stream << "registration time = " << (double)(clock() - time1) / CLOCKS_PER_SEC << " s" << std::endl;
  std::cout << "end registration" << std::endl;

  std::string log_fname;

  if (SField_type == 0)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegDistLog.txt";
  }
  else if (SField_type == 1)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegTunedKDLog.txt";
  }
  else if (SField_type == 2)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegMLEqLog.txt";
  }
  LOG::Instance(log_fname.c_str())->OutputMisc(log_stream.str().c_str());
}

void LargeFeatureReg::testNlopt()
{
  int num_method = nlopt::NUM_ALGORITHMS;
  for (int i = 0; i < num_method; ++i)
  {
    this->runReg(i);
  }
}



void LargeFeatureReg::runRegNonRigid(int method_id)
{
  std::cout << "begin Large Feature Non-Rigid Registration" << std::endl;
  unsigned time1 = clock();

  // start point
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  Matrix4f& cur_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  MatrixXf v_with_transform = cur_transform * (MatrixXf(4, mesh->n_vertices()) << 
                                                Eigen::Map<const MatrixXf>(&feature_model->source_model->getShapeVertexList()[0], 3, mesh->n_vertices()),
                                                Eigen::RowVectorXf::Ones(mesh->n_vertices())).finished();
  P_init = v_with_transform.block(0, 0, 3, mesh->n_vertices());
  ARAP_R.resize(mesh->n_vertices(), Matrix3f::Identity());
  updateARAPLMatrix(ARAP_L_matrix);
  lamd_ARAP = 0.5;

  P_plane_proj = P_init;
  updateFlatCoefs(flat_coefs);
  feature_model->source_model->getPlaneVertices(flat_vertices);
  P_plane_proj_new.resize(flat_vertices.size());
  lamd_flat = 0.5;

  int x_dim = 3 * mesh->n_vertices();
  std::vector<double> x0;
  for (size_t i = 0; i < mesh->n_vertices(); ++i)
  {
    x0.push_back(P_init(0, i));
    x0.push_back(P_init(1, i));
    x0.push_back(P_init(2, i));
  }
  //x0[x_dim - 1] = 1.0; // scale start from 1

  double start_func_val = this->energyFuncNonRigid(x0);
  //std::cout << "Start f(x): " << start_func_val << std::endl;

  // set nlopt handle
  if (method_id >= nlopt::NUM_ALGORITHMS)
  {
    std::cout << "Required method does not exist. Use default method.\n";
    method_id = 0;
  }
  nlopt::opt opt(nlopt::algorithm(method_id), x_dim);

  // set bound
  //std::vector<double> lb(x_dim);
  //std::vector<double> ub(x_dim);
  //for (size_t i = 0; i < x_dim; ++i)
  //{
  //  if (i < 3)
  //  {
  //    lb[i] = -0.5 * modelRadius();
  //    ub[i] = 0.5 * modelRadius();
  //  }
  //  else if (i < 6)
  //  {
  //    lb[i] = -M_PI / 15;
  //    ub[i] = M_PI / 15;
  //  }
  //  //else
  //  //{
  //  //  lb[i] = 0.5;
  //  //  ub[i] = 1.5;
  //  //}
  //}
  //opt.set_lower_bounds(lb);
  //opt.set_upper_bounds(ub);

  // set objective function
  int SField_type = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("SField:Type");
  if (SField_type == 0 || SField_type == 1)
  {
    opt.set_min_objective(LFReg::efuncNonRigid, this);
  }
  else if (SField_type == 2)
  {
    opt.set_min_objective(LFReg::efuncNonRigid, this);
  }

  // set stop criteria
  opt.set_ftol_rel(0.001);
  opt.set_xtol_rel(0.001);
  opt.set_maxtime(60);

  // get result
  double maxf;
  nlopt::result result;
  try{
    result = opt.optimize(x0, maxf);
  }catch (std::exception& e){  std::cerr << "exception caught: " << e.what() << '\n'; }
  for (size_t i = 0; i < mesh->n_vertices(); ++i)
  {
    P_init(0, i) = x0[3 * i + 0];
    P_init(1, i) = x0[3 * i + 1];
    P_init(2, i) = x0[3 * i + 2];
  }
  v_with_transform = cur_transform.inverse() * (MatrixXf(4, mesh->n_vertices()) << P_init, Eigen::RowVectorXf::Ones(mesh->n_vertices())).finished();
  std::vector<float> new_shape;
  for (size_t i = 0; i < mesh->n_vertices(); ++i)
  {
    new_shape.push_back(v_with_transform(0, i));
    new_shape.push_back(v_with_transform(1, i));
    new_shape.push_back(v_with_transform(2, i));
  }
  feature_model->source_model->updateShape(new_shape);

  std::stringstream log_stream;
  log_stream << "LFReg Log:" << std::endl;
  log_stream << "Method: " << method_id << " " << opt.get_algorithm_name() << std::endl;
  log_stream << "Start f(x): " << start_func_val << "\tStop f(x): " << maxf << std::endl;
  log_stream << "optimized transform: " << std::endl;
  log_stream << cur_transform << std::endl;
  log_stream << "registration time = " << (double)(clock() - time1) / CLOCKS_PER_SEC << " s" << std::endl;
  std::cout << "end registration" << std::endl;

  std::string log_fname;

  if (SField_type == 0)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegNonRigidDistLog.txt";
  }
  else if (SField_type == 1)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegTunedKDNonRigidLog.txt";
  }
  else if (SField_type == 2)
  {
    log_fname = feature_model->source_model->getDataPath() + "/LFRegMLEqNonRigidLog.txt";
  }
  LOG::Instance(log_fname.c_str())->OutputMisc(log_stream.str().c_str());
}