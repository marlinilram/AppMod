#include "LargeFeatureReg.h"
#include "FeatureGuided.h"
#include "ScalarField.h"
#include "Model.h"
#include "Bound.h"

#include "ParameterMgr.h"
#include "BasicHeader.h"

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

  double curve_integrate = feature_model->target_scalar_field->curveIntegrate(curves, feature_model);
  //return curve_integrate;

  double sum = 0.0;
  for (auto i : data_crsp)
  {
    Vector4f v_proj = vpPMV_mat * cur_transform * Vector4f(vertex_list[3 * i.first + 0], vertex_list[3 * i.first + 1], vertex_list[3 * i.first + 2], 1.0);
    Vector2f diff = Vector2f(v_proj[0] / v_proj[3], v_proj[1] / v_proj[3]) - i.second.first;
    sum += diff.squaredNorm() - pow(diff.dot(i.second.second), 2); // point to line distance
  }

  return lamd_SField * curve_integrate + lamd_data * sum;
}

double LargeFeatureReg::modelRadius()
{
  return feature_model->source_model->getBoundBox()->getRadius();
}