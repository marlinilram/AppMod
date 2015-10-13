#include "ScalarField.h"
#include "VectorVariation.h"
#include "tele2d.h"

#include <cv.h>
#include <highgui.h>

ScalarField::ScalarField(int res, float rad)
{
  this->init(res);
  this->setSearchRad(rad);
}

ScalarField::~ScalarField()
{

}

void ScalarField::setTeleRegister(std::shared_ptr<tele2d> tele)
{
  tele_register = tele;
}

void ScalarField::init(int resolution)
{
  this->resolution = resolution;

  distance_map.resize(resolution * resolution, 0);
  variation_map.resize(resolution * resolution, 0);
  matching_map.resize(resolution * resolution, 0);
}

void ScalarField::setSearchRad(float radius)
{
  this->search_rad = radius;
}

void ScalarField::computeVariationMap()
{
  std::vector<double2>& vector_field = tele_register->vector_field;
  float max_var = std::numeric_limits<float>::min();
  float min_var = std::numeric_limits<float>::max();
  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {

      std::vector<double2> neighbour_area;
      double2 center = vector_field[i + j * resolution];
      for (int i_step = i - search_rad; i_step <= i + search_rad; ++i_step)
      {
        for (int j_step = j - search_rad; j_step <= j + search_rad; ++j_step)
        {
          if (i_step >= 0 && i_step < resolution
           && j_step >= 0 && j_step < resolution
           && i_step != i && j_step != j)
          {
            neighbour_area.push_back(vector_field[i_step + j_step * resolution]);
          }
        }
      }
      variation_map[i + j * resolution] = (float)computeVariation(neighbour_area, center);
      if (variation_map[i + j * resolution] > max_var)
      {
        max_var = variation_map[i + j * resolution];
      }
      if (variation_map[i + j * resolution] < min_var)
      {
        min_var = variation_map[i + j * resolution];
      }
    }
  }

  // normalize the variation map
  for (size_t i = 0; i < variation_map.size(); ++i)
  {
    variation_map[i] = (variation_map[i] - min_var) / (max_var - min_var);
  }

  //cv::imshow("variation map", cv::Mat(resolution, resolution, CV_32FC1, &variation_map[0]));
}

void ScalarField::computeMatchingMap(std::vector<double2>& ext_vector_field)
{
  std::vector<double2>& vector_field = tele_register->vector_field;
  float max_var = std::numeric_limits<float>::min();
  float min_var = std::numeric_limits<float>::max();

  if (vector_field.size() != ext_vector_field.size())
  {
    std::cerr << "Dimension of vector field doesn't match.\n";
    return;
  }

  for (size_t i = 0; i < vector_field.size(); ++i)
  {
    double cur_cos = vector_field[i].x * ext_vector_field[i].x + vector_field[i].y * ext_vector_field[i].y;
    cur_cos = fabs(cur_cos);
    matching_map[i] = (float)cur_cos;
    if (cur_cos > max_var)
    {
      max_var = cur_cos;
    }
    if (cur_cos < min_var)
    {
      min_var = cur_cos;
    }
  }

  // normalize the variation map
  for (size_t i = 0; i < matching_map.size(); ++i)
  {
    matching_map[i] = 1 - (matching_map[i] - min_var) / (max_var - min_var);
  }
}