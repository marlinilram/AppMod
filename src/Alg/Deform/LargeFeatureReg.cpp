#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "ScalarField.h"
#include "Model.h"
#include "Bound.h"
#include "PolygonMesh.h"

#include "nlopt.hpp"
#include <ctime>
#include <fstream>

#include "ParameterMgr.h"
#include "BasicHeader.h"
#include "LOG.h"
#include "WunderSVD3x3.h"

using namespace LG;


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

  double efuncNonRigid(const std::vector<double>&x, std::vector<double>& grad, void *func_data)
  {
    LargeFeatureReg* lf_reg = (LargeFeatureReg*)func_data;

    double fx0 = 0;
    double fx0_SField = 0;
    fx0_SField = lf_reg->energyScalarField(x); // to estimate grad of SField
    fx0 += fx0_SField;
    fx0 += lf_reg->lamd_ARAP * lf_reg->energyARAP(x);


    grad.resize(x.size());
    std::vector<double> cppx;
    std::vector<double> ARAP_grad;
    lf_reg->updateARAPGrad(x, ARAP_grad);

    for (size_t i = 0; i < x.size(); ++i)
    {
      cppx = x;

      double step = 0.01;

      cppx[i] += step;
      grad[i] = ((lf_reg->energyScalarField(cppx) - fx0_SField) / step) + lf_reg->lamd_ARAP * ARAP_grad[i];
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
  Matrix4f& cur_transform = GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
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
  int SField_type = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("SField:Type");
  if (SField_type == 0 || SField_type == 1)
  {
    opt.set_min_objective(LFReg::efunc, this);
  }
  else if (SField_type == 2)
  {
    opt.set_max_objective(LFReg::efunc, this);
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

double LargeFeatureReg::energyFuncNonRigid(const std::vector<double>& X)
{
  double energy = 0;

  // energy for Scalar Field
  energy += energyScalarField(X);

  // energy for As Rigid As Possible
  energy += lamd_ARAP * energyARAP(X);

  // energy for local flatness

  return energy;
}

double LargeFeatureReg::energyScalarField(const std::vector<double>& X)
{
  const std::vector<STLVectori>& crest_lines = feature_model->source_model->getShapeVisbleCrestLine();
  CURVES curves;
  std::vector<double2> curve;
  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    curve.clear();
    for (size_t j = 0; j < crest_lines[i].size(); ++j)
    {
      Vector4f v;
      v[0] = X[3 * crest_lines[i][j] + 0];
      v[1] = X[3 * crest_lines[i][j] + 1];
      v[2] = X[3 * crest_lines[i][j] + 2];
      float winx, winy;
      feature_model->source_model->getProjectPt(v.data(), winx, winy);
      curve.push_back(double2(winx, feature_model->target_img.rows - winy));
    }
    curves.push_back(curve);
  }
  return feature_model->target_scalar_field->curveIntegrate(curves, feature_model);
}

double LargeFeatureReg::energyARAP(const std::vector<double>& X)
{
  // E = sum_i { w_i * sum_j { w_ij * |R_i(v_i - v_j) - (v_i^' - v_j^')|^2 } }
  updateARAPRotation(X);

  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  double sum = 0.0;
  double wi_sum = 0.0;
  for (; vit != vend; ++vit)
  {
    wi_sum = 0.0;
    double temp_sum = 0.0;

    PolygonMesh::Halfedge_around_vertex_circulator hec, hec_end;
    hec = hec_end = mesh->halfedges(*vit);
    do 
    {
      double wij = laplacian_cot[mesh->edge(*hec)];
      int vi = (*vit).idx();
      int vj = mesh->to_vertex(*hec).idx();
      Vector3f vdiff(X[3 * vi + 0] - X[3 * vj + 0],
                     X[3 * vi + 1] - X[3 * vj + 1],
                     X[3 * vi + 2] - X[3 * vj + 2]);
      temp_sum += wij * (ARAP_R[(*vit).idx()] * (P_init.col(vi) - P_init.col(vj)) - vdiff).squaredNorm();
      wi_sum += wij;
    } while (++hec != hec_end);

    sum += temp_sum;
  }
  return sum;
}

void LargeFeatureReg::updateARAPRotation(const std::vector<double>& X)
{
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  Matrix3f Si;
  MatrixXf Di;
  Matrix3Xf Pi_Prime;
  Matrix3Xf Pi;

  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  for (; vit != vend; ++vit)
  {
    // compute Ri for each vertex
    // 1. compute one ring size
    int ring_cnt = 0; 
    PolygonMesh::Halfedge_around_vertex_circulator hec, hec_end;
    hec = hec_end = mesh->halfedges(*vit);
    do 
    {
      ++ring_cnt;
    } while (++hec != hec_end);

    // 2. init Di, Pi_Prime, Pi
    Di = MatrixXf::Zero(ring_cnt, ring_cnt);
    Pi_Prime.resize(3, ring_cnt);
    Pi.resize(3, ring_cnt);

    // 3. fill matrix
    ring_cnt = 0;
    do 
    {
      // 3.1 get Laplacian weight wij
      Di(ring_cnt, ring_cnt) = laplacian_cot[mesh->edge(*hec)];
      int vi = (*vit).idx();
      int vj = mesh->to_vertex(*hec).idx();
      Pi_Prime.col(ring_cnt) = Vector3f(X[3 * vi + 0] - X[3 * vj + 0],
                                        X[3 * vi + 1] - X[3 * vj + 1],
                                        X[3 * vi + 2] - X[3 * vj + 2]);
      Pi.col(ring_cnt) = P_init.col(vi) - P_init.col(vj);

      ++ring_cnt;
    } while (++hec != hec_end);

    // 4. compute closest rotation
    Si = Pi * Di * Pi_Prime.transpose();
    Matrix3f Ui;
    Vector3f Wi;
    Matrix3f Vi;
    wunderSVD3x3(Si, Ui, Wi, Vi);
    ARAP_R[(*vit).idx()] = Vi * Ui.transpose();

    if (ARAP_R[(*vit).idx()].determinant() < 0)
    {
      std::cout << "determinant is negative!" << std::endl;
    }
  }
}

void LargeFeatureReg::updateARAPGrad(const std::vector<double>& X, std::vector<double>& grad)
{
  // 4 * L * X - 4 * b
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  Matrix3Xf b = Eigen::Matrix3Xf::Zero(3, mesh->n_vertices());

  for (; vit != vend; ++vit)
  {
    PolygonMesh::Halfedge_around_vertex_circulator hec, hec_end;
    hec = hec_end = mesh->halfedges(*vit);
    do 
    {
      double wij = laplacian_cot[mesh->edge(*hec)];
      int vi = (*vit).idx();
      int vj = mesh->to_vertex(*hec).idx();

      b.col(vi) += (wij / 2) * (ARAP_R[vi] + ARAP_R[vj]) * (P_init.col(vi) - P_init.col(vj));
    } while (++hec != hec_end);
  }

  std::vector<float> X_cast(X.begin(), X.end());
  VectorXf grad_cast = 4 * ARAP_L_matrix * Eigen::Map<const VectorXf>(&X_cast[0], X.size()) - 4 * Eigen::Map<VectorXf>(b.data(), 3 * mesh->n_vertices(), 1);
  X_cast = std::vector<float>(grad_cast.data(), grad_cast.data() + grad_cast.size());
  grad = std::vector<double>(X_cast.begin(), X_cast.end());
}

void LargeFeatureReg::updateARAPLMatrix(SparseMatrix& matrix)
{
  TripletList L_triplets;
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  double sum = 0.0;
  double wi_sum = 0.0;
  for (; vit != vend; ++vit)
  {
    wi_sum = 0.0;
    double temp_sum = 0.0;
    int vi = (*vit).idx();

    PolygonMesh::Halfedge_around_vertex_circulator hec, hec_end;
    hec = hec_end = mesh->halfedges(*vit);
    do 
    {
      int vj = mesh->to_vertex(*hec).idx();
      double wij = laplacian_cot[mesh->edge(*hec)];
      L_triplets.push_back(Triplet(3 * vi + 0, 3 * vj + 0, -wij));
      L_triplets.push_back(Triplet(3 * vi + 1, 3 * vj + 1, -wij));
      L_triplets.push_back(Triplet(3 * vi + 2, 3 * vj + 2, -wij));
      wi_sum += wij;
    } while (++hec != hec_end);

    L_triplets.push_back(Triplet(3 * vi + 0, 3 * vi + 0, wi_sum));
    L_triplets.push_back(Triplet(3 * vi + 1, 3 * vi + 1, wi_sum));
    L_triplets.push_back(Triplet(3 * vi + 2, 3 * vi + 2, wi_sum));
  }
  ARAP_L_matrix.resize(3 * mesh->n_vertices(), 3 * mesh->n_vertices());
  ARAP_L_matrix.setFromTriplets(L_triplets.begin(), L_triplets.end());
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
    opt.set_max_objective(LFReg::efuncNonRigid, this);
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