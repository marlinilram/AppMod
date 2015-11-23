#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "ScalarField.h"
#include "Model.h"
#include "PolygonMesh.h"

#include "ParameterMgr.h"
#include "BasicHeader.h"
#include "WunderSVD3x3.h"

using namespace LG;

namespace LFReg {

  double efuncNonRigid(const std::vector<double>&x, std::vector<double>& grad, void *func_data)
  {
    LargeFeatureReg* lf_reg = (LargeFeatureReg*)func_data;

    double fx0_SField = lf_reg->lamd_SField * lf_reg->energyScalarField(x); // to estimate grad of SField
    double fx0_ARAP = lf_reg->lamd_ARAP * lf_reg->energyARAP(x);
    double fx0_flat = lf_reg->lamd_flat * lf_reg->energyFlatNew(x);
    double fx0_data = lf_reg->lamd_data * lf_reg->energyDataTerm(x);
    double fx0 = fx0_SField + fx0_ARAP + fx0_flat + fx0_data;
    std::cout << "SField: " << fx0_SField << "\tARAP: " << fx0_ARAP << "\tflat: " << fx0_flat << "\tdata: " << fx0_data << std::endl;


    grad.resize(x.size());
    std::vector<double> SField_grad;
    std::vector<double> ARAP_grad;
    std::vector<double> flat_grad;
    std::vector<double> data_grad;
    lf_reg->updateScalarFieldGrad(x, SField_grad);
    lf_reg->updateARAPGrad(x, ARAP_grad);
    lf_reg->updateFlatGradNew(x, flat_grad);
    lf_reg->updateDataTermGrad(x, data_grad);

    for (size_t i = 0; i < x.size(); ++i)
    {
      //cppx = x;

      //double step = 0.01;

      //cppx[i] += step;
      //((lf_reg->lamd_SField * lf_reg->energyScalarField(cppx) - fx0_SField) / step)
      //lf_reg->lamd_SField * SField_grad[i]
      grad[i] = lf_reg->lamd_SField * SField_grad[i] + lf_reg->lamd_ARAP * ARAP_grad[i] + lf_reg->lamd_flat * flat_grad[i] + lf_reg->lamd_data * data_grad[i];
    }

    //std::cout << "f(X) = " << fx0 << std::endl;
    return fx0;
  }
};

double LargeFeatureReg::energyFuncNonRigid(const std::vector<double>& X)
{
  double energy = 0;

  // energy for Scalar Field
  energy += lamd_SField * energyScalarField(X);

  // energy for As Rigid As Possible
  energy += lamd_ARAP * energyARAP(X);

  // energy for local flatness
  energy += lamd_flat * energyFlatNew(X);

  // energy for data term
  energy += lamd_data * energyDataTerm(X);

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

void LargeFeatureReg::updateScalarFieldGrad(const std::vector<double>& X, std::vector<double>& grad)
{
  grad.resize(X.size(), 0.0);
  const std::vector<STLVectori>& crest_lines = feature_model->source_model->getShapeVisbleCrestLine();
  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    for (size_t j = 0; j < crest_lines[i].size(); ++j)
    {
      int vid = crest_lines[i][j];
      Vector4f v_proj = vpPMV_mat * Vector4f(X[3 * vid + 0], X[3 * vid + 1], X[3 * vid + 2], 1.0);
      double2 n_curve_pt = (double2(v_proj[0] / v_proj[3], v_proj[1] / v_proj[3]) + feature_model->curve_translate - double2(0.5, 0.5)) * feature_model->curve_scale + double2(0.5, 0.5);
      double field_grad_x, field_grad_y;
      feature_model->target_scalar_field->getDistanceMapGrad(n_curve_pt, field_grad_x, field_grad_y);
      grad[3 * vid + 0] = field_grad_x * (v_proj[3] * vpPMV_mat(0, 0) - v_proj[0] * vpPMV_mat(3, 0)) / (v_proj[3] * v_proj[3])
                        + field_grad_y * (v_proj[3] * vpPMV_mat(1, 0) - v_proj[1] * vpPMV_mat(3, 0)) / (v_proj[3] * v_proj[3]);
      grad[3 * vid + 1] = field_grad_x * (v_proj[3] * vpPMV_mat(0, 1) - v_proj[0] * vpPMV_mat(3, 1)) / (v_proj[3] * v_proj[3])
                        + field_grad_y * (v_proj[3] * vpPMV_mat(1, 1) - v_proj[1] * vpPMV_mat(3, 1)) / (v_proj[3] * v_proj[3]);
      grad[3 * vid + 2] = field_grad_x * (v_proj[3] * vpPMV_mat(0, 2) - v_proj[0] * vpPMV_mat(3, 2)) / (v_proj[3] * v_proj[3])
                        + field_grad_y * (v_proj[3] * vpPMV_mat(1, 2) - v_proj[1] * vpPMV_mat(3, 2)) / (v_proj[3] * v_proj[3]);
    }
  }
  //std::cout << std::endl;
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

void LargeFeatureReg::updateFlatCoefs(std::vector<double>& coefs)
{
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  coefs.resize(mesh->n_vertices());
  PolygonMesh::Face_attribute<Vec3> f_normals = mesh->face_attribute<Vec3>("f:normal");
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  double min_coef = std::numeric_limits<double>::max();
  for (; vit != vend; ++vit)
  {
    int vi = (*vit).idx();
    PolygonMesh::Face_around_vertex_circulator fc, fc_end;
    fc = fc_end = mesh->faces(*vit);
    int ring_cnt = 0;
    do 
    {
      ++ring_cnt;
    } while (++fc != fc_end);

    Matrix3Xf var(3, ring_cnt);
    ring_cnt = 0;
    do 
    {
      var.col(ring_cnt) = f_normals[*fc];
      ++ring_cnt;
    } while (++fc != fc_end);

    VectorXf cos_list(ring_cnt);
    cos_list(0) = (var.col(0).dot(var.col(ring_cnt - 1)));
    for (int i = 1; i < ring_cnt; ++i)
    {
      cos_list(i) = (var.col(i).dot(var.col(i - 1)));
    }
    //coefs[vi] = std::exp(-(cos_list.array() - cos_list.mean()).matrix().squaredNorm() / ring_cnt);
    coefs[vi] = cos_list.minCoeff() < 0 ? 0 : cos_list.minCoeff();
    //if (coefs[vi] < min_coef) min_coef = coefs[vi];
  }

  //std::cout << "min flat coef: " << min_coef << std::endl;
}

void LargeFeatureReg::updateFlatProj(const std::vector<double>& X)
{
  PolygonMesh* mesh = feature_model->source_model->getPolygonMesh();
  P_plane_proj.resize(3, mesh->n_vertices());
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  for (; vit != vend; ++vit)
  {
    int vi = (*vit).idx();
    std::set<int> neighbors;
    std::vector<int> cur_ring;
    std::vector<int> next_ring;
    cur_ring.push_back(vi);
    neighbors.insert(vi);
    PolygonMesh::Vertex_around_vertex_circulator vc, vc_end;
    for (int i_ring = 0; i_ring < 1; ++i_ring)
    {
      for (size_t i = 0; i < cur_ring.size(); ++i)
      {
        vc = vc_end = mesh->vertices(PolygonMesh::Vertex(cur_ring[i]));
        do 
        {
          int cur_v = (*vc).idx();
          if (neighbors.find(cur_v) == neighbors.end())
          {
            neighbors.insert(cur_v);
            next_ring.push_back(cur_v);
          }
        } while (++vc != vc_end);
      }
      cur_ring.swap(next_ring);
      next_ring.clear();
    }

    Matrix3Xf C(3, neighbors.size());
    int neighbor_cnt = 0;
    int vi_id = 0;
    for (auto i : neighbors)
    {
      if (i == vi) vi_id = neighbor_cnt;
      C.col(neighbor_cnt) =  Vector3f(X[3 * i + 0], X[3 * i + 1], X[3 * i + 2]);
      ++neighbor_cnt;
    }
    //Vector3f cur_pos(X[3 * vi + 0], X[3 * vi + 1], X[3 * vi + 2]); // the vertex self also need to be considered

    Vector3f center = C * VectorXf::Ones(neighbors.size()) / (neighbors.size());
    C = ((MatrixXf::Identity(neighbors.size(), neighbors.size()) - MatrixXf::Ones(neighbors.size(), neighbors.size()) / neighbors.size()) * C.transpose()).transpose();
    Matrix3f C_cov = C * C.transpose();
    Matrix3f U;
    Vector3f W;
    Matrix3f V;
    wunderSVD3x3(C_cov, U, W, V);

    Matrix3Xf U_part(3, 2);
    U_part.col(0) = U.col(0);
    U_part.col(1) = U.col(1);
    P_plane_proj.col(vi) = U_part * U_part.transpose() * C.col(vi_id) + center;
  }
}

void LargeFeatureReg::updateFlatGrad(const std::vector<double>& X, std::vector<double>& grad)
{
  grad.resize(X.size());
  for (size_t i = 0; i < P_plane_proj.cols(); ++i)
  {
    grad[3 * i + 0] = 2 * flat_coefs[i] * (X[3 * i + 0] - P_plane_proj(0, i));
    grad[3 * i + 1] = 2 * flat_coefs[i] * (X[3 * i + 1] - P_plane_proj(1, i));
    grad[3 * i + 2] = 2 * flat_coefs[i] * (X[3 * i + 2] - P_plane_proj(2, i));
  }
}

double LargeFeatureReg::energyFlat(const std::vector<double>& X)
{
  updateFlatProj(X);

  double sum = 0;
  for (size_t i = 0; i < P_plane_proj.cols(); ++i)
  {
    sum += flat_coefs[i] * pow(X[3 * i + 0] - P_plane_proj(0, i), 2);
    sum += flat_coefs[i] * pow(X[3 * i + 1] - P_plane_proj(1, i), 2);
    sum += flat_coefs[i] * pow(X[3 * i + 2] - P_plane_proj(2, i), 2);
  }
  return sum;
}

double LargeFeatureReg::energyFlatNew(const std::vector<double>& X)
{
  updateFlatProjNew(X);

  double sum = 0;
  for (size_t i = 0; i < flat_vertices.size(); ++i)
  {
    for (size_t j = 0; j < flat_vertices[i].size(); ++j)
    {
      sum += pow(X[3 * flat_vertices[i][j] + 0] - P_plane_proj_new[i](0, j), 2);
      sum += pow(X[3 * flat_vertices[i][j] + 1] - P_plane_proj_new[i](1, j), 2);
      sum += pow(X[3 * flat_vertices[i][j] + 2] - P_plane_proj_new[i](2, j), 2);
    }
  }
  return sum;
}

void LargeFeatureReg::updateFlatProjNew(const std::vector<double>& X)
{
  for (size_t i = 0; i < flat_vertices.size(); ++i)
  {
    size_t plane_size = flat_vertices[i].size();
    P_plane_proj_new[i].resize(3, plane_size);
    Matrix3Xf C(3, plane_size);
    for (size_t j = 0; j < plane_size; ++j)
    {
      C.col(j) =  Vector3f(X[3 * flat_vertices[i][j] + 0], X[3 * flat_vertices[i][j] + 1], X[3 * flat_vertices[i][j] + 2]);
    }

    Vector3f center = C * VectorXf::Ones(plane_size) / (plane_size);
    C = ((MatrixXf::Identity(plane_size, plane_size) - MatrixXf::Ones(plane_size, plane_size) / plane_size) * C.transpose()).transpose();
    Matrix3f C_cov = C * C.transpose();
    Matrix3f U;
    Vector3f W;
    Matrix3f V;
    wunderSVD3x3(C_cov, U, W, V);

    Matrix3Xf U_part(3, 2);
    U_part.col(0) = U.col(0);
    U_part.col(1) = U.col(1);
    P_plane_proj_new[i] = U_part * U_part.transpose() * C;
    for (size_t j = 0; j < plane_size; ++j)
    {
      P_plane_proj_new[i].col(j) += center;
    }
  }
}

void LargeFeatureReg::updateFlatGradNew(const std::vector<double>& X, std::vector<double>& grad)
{
  grad.resize(X.size(), 0);
  for (size_t i = 0; i < flat_vertices.size(); ++i)
  {
    for (size_t j = 0; j < flat_vertices[i].size(); ++j)
    {
      grad[3 * flat_vertices[i][j] + 0] += 2 * (X[3 * flat_vertices[i][j] + 0] - P_plane_proj_new[i](0, j));
      grad[3 * flat_vertices[i][j] + 1] += 2 * (X[3 * flat_vertices[i][j] + 1] - P_plane_proj_new[i](1, j));
      grad[3 * flat_vertices[i][j] + 2] += 2 * (X[3 * flat_vertices[i][j] + 2] - P_plane_proj_new[i](2, j));
    }
  }
}

void LargeFeatureReg::updateDataCrsp(const std::vector<double>& X)
{
  // 1. update the source curves based on new X
  CURVES src_new_curves = feature_model->source_curves;
  for (size_t i = 0; i < src_new_curves.size(); ++i)
  {
    for (size_t j = 0; j < src_new_curves[i].size(); ++j)
    {
      int vid = feature_model->src_vid_mapper[STLPairii(int(i), int(j))];      
      Vector4f v_proj = vpPMV_mat * Vector4f(X[3 * vid + 0], X[3 * vid + 1], X[3 * vid + 2], 1.0);
      src_new_curves[i][j] = double2(v_proj[0] / v_proj[3], v_proj[1] / v_proj[3]);
    }
  }
  
  // 2. find correspondence
  data_crsp.clear();
  feature_model->BuildClosestPtPair(src_new_curves, data_crsp);
}

double LargeFeatureReg::energyDataTerm(const std::vector<double>& X)
{
  //updateDataCrsp(X);

  double sum = 0.0;
  for (auto i : data_crsp)
  {
    Vector4f v_proj = vpPMV_mat * Vector4f(X[3 * i.first + 0], X[3 * i.first + 1], X[3 * i.first + 2], 1.0);
    sum += (Vector2f(v_proj[0] / v_proj[3], v_proj[1] / v_proj[3]) - i.second).squaredNorm();
  }
  return sum;
}

void LargeFeatureReg::updateDataTermGrad(const std::vector<double>& X, std::vector<double>& grad)
{
  grad.resize(X.size(), 0.0);
  for (auto i : data_crsp)
  {
    int vid = i.first;
    Vector4f v_proj = vpPMV_mat * Vector4f(X[3 * vid + 0], X[3 * vid + 1], X[3 * vid + 2], 1.0);
    grad[3 * vid + 0] = 2 * (v_proj[0] / v_proj[3] - i.second[0]) * (v_proj[3] * vpPMV_mat(0, 0) - v_proj[0] * vpPMV_mat(3, 0)) / (v_proj[3] * v_proj[3])
                      + 2 * (v_proj[1] / v_proj[3] - i.second[1]) * (v_proj[3] * vpPMV_mat(1, 0) - v_proj[1] * vpPMV_mat(3, 0)) / (v_proj[3] * v_proj[3]);
    grad[3 * vid + 1] = 2 * (v_proj[0] / v_proj[3] - i.second[0]) * (v_proj[3] * vpPMV_mat(0, 1) - v_proj[0] * vpPMV_mat(3, 1)) / (v_proj[3] * v_proj[3])
                      + 2 * (v_proj[1] / v_proj[3] - i.second[1]) * (v_proj[3] * vpPMV_mat(1, 1) - v_proj[1] * vpPMV_mat(3, 1)) / (v_proj[3] * v_proj[3]);
    grad[3 * vid + 2] = 2 * (v_proj[0] / v_proj[3] - i.second[0]) * (v_proj[3] * vpPMV_mat(0, 2) - v_proj[0] * vpPMV_mat(3, 2)) / (v_proj[3] * v_proj[3])
                      + 2 * (v_proj[1] / v_proj[3] - i.second[1]) * (v_proj[3] * vpPMV_mat(1, 2) - v_proj[1] * vpPMV_mat(3, 2)) / (v_proj[3] * v_proj[3]);
  }
}