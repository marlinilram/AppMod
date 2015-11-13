#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "ScalarField.h"
#include "Model.h"
#include "Bound.h"

#include "nlopt.hpp"
#include <ctime>

#include "ParameterMgr.h"
#include "BasicHeader.h"
#include "LOG.h"


namespace LFReg {

  const static double energyGradAngleStep = 0.5 * M_PI / 180.0; // 0.1 degree
  const static double energyGradTransStep = 0.01; // assume the mesh is normalized
  const static double energyGradScaleStep = 0.01;

  double efunc(const std::vector<double>&x, std::vector<double>& grad, void *func_data)
  {
    std::vector<double> x0 = x;
    LargeFeatureReg* lf_reg = (LargeFeatureReg*)func_data;

    double fx0 = lf_reg->energyFunc(x0);

    grad.resize(x.size());
    std::vector<double> cppx;

    for (size_t i = 0; i < x.size(); ++i)
    {
      cppx = x0;

      double step;
      if (i < 3)
      {
        step = energyGradTransStep * lf_reg->modelRadius();
      }
      else if (i < 6)
      {
        step = energyGradAngleStep;
      }
      //else
      //{
      //  step = energyGradScaleStep;
      //}

      cppx[i] += step;
      grad[i] = (lf_reg->energyFunc(cppx) - fx0) / step;
    }

    //std::cout << "f(X) = " << fx0 << std::endl;
    return fx0;
  }
};

double LargeFeatureReg::energyFunc(const std::vector<double>& X)
{
  // for the rigid part
  // three for rotation
  // one for scaling
  // three for translation

  // first step is to find the new visible curves given the current X
  // build transform matrix from X
  Matrix4f& cur_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  cur_transform = 
    (Eigen::Translation3f(X[0], X[1], X[2])
    * Eigen::AngleAxisf(X[5], Vector3f::UnitZ())
    * Eigen::AngleAxisf(X[4], Vector3f::UnitY())
    * Eigen::AngleAxisf(X[3], Vector3f::UnitZ())).matrix();
    //* Eigen::Scaling(float(X[6]))).matrix();

  // update the renderer here
  // if we assume the visible vertices will not change, this can be ignored

  // second step is to compute
  //update the source curve
  const std::vector<STLVectori>& crest_lines = feature_model->source_model->getShapeVisbleCrestLine();
  const VertexList& vertex_list = feature_model->source_model->getShapeVertexList();
  CURVES curves;
  std::vector<double2> curve;
  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    curve.clear();
    for (size_t j = 0; j < crest_lines[i].size(); ++j)
    {
      Vector4f v;
      v[0] = vertex_list[3 * crest_lines[i][j] + 0];
      v[1] = vertex_list[3 * crest_lines[i][j] + 1];
      v[2] = vertex_list[3 * crest_lines[i][j] + 2];
      v[3] = 1.0f;
      v = cur_transform * v;
      v = v / v[3];
      float winx, winy;
      feature_model->source_model->getProjectPt(v.data(), winx, winy);
      curve.push_back(double2(winx, feature_model->target_img.rows - winy));
    }
    curves.push_back(curve);
  }
  
  return feature_model->target_scalar_field->curveIntegrate(curves, feature_model);
}

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
  opt.set_min_objective(LFReg::efunc, this);

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
  log_stream << "Start f(x): " << start_func_val << "\tMax f(x): " << maxf << std::endl;
  log_stream << "optimized transform: " << std::endl;
  log_stream << cur_transform << std::endl;
  log_stream << "registration time = " << (double)(clock() - time1) / CLOCKS_PER_SEC << " s" << std::endl;
  std::cout << "end registration" << std::endl;

  LOG::Instance((feature_model->source_model->getDataPath() + "/LFRegDistLog.txt").c_str())->OutputMisc(log_stream.str().c_str());
}

double LargeFeatureReg::modelRadius()
{
  return feature_model->source_model->getBoundBox()->getRadius();
}

void LargeFeatureReg::testNlopt()
{
  int num_method = nlopt::NUM_ALGORITHMS;
  for (int i = 0; i < num_method; ++i)
  {
    this->runReg(i);
  }
}