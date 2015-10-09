#ifndef VectorVariation_H
#define VectorVariation_H

#include "BasicDataType.h"
#include <vector>

double computeVariation(std::vector<double2>& vector_list)
{
  std::vector<double> angles;
  double avg_angle = 0.0;
  for (size_t i = 0; i < vector_list.size(); ++i)
  {
    double2 dir = vector_list[i];
    double angle = dir.x / sqrt(dir.x * dir.x + dir.y * dir.y);

    avg_angle += angle;
    angles.push_back(angle);
  }

  avg_angle /= angles.size();
  double var = 0.0;
  for (size_t i = 0; i < angles.size(); ++i)
  {
    var += pow(angles[i] - avg_angle, 2);
  }
  var /= angles.size();
  return var;
}

double computeVariation(std::vector<double2>& vector_list, double2& center)
{
  std::vector<double> angles;
  double avg_angle = 0.0;
  for (size_t i = 0; i < vector_list.size(); ++i)
  {
    double2 dir = vector_list[i];
    double angle = dir.x * center.x + dir.y * center.y;

    avg_angle += angle;
    angles.push_back(angle);
  }

  avg_angle /= angles.size();
  double var = 0.0;
  for (size_t i = 0; i < angles.size(); ++i)
  {
    var += pow(angles[i] - avg_angle, 2);
  }
  var /= angles.size();
  return var;
}

#endif // !VectorVariation_H
