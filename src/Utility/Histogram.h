#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include "BasicDataType.h"

int ProjectDirToBin2D(double2 dir)
{
  // project a direction in 2D space into n bins
  // return start from 0 and counter clockwise
  double angle = atan(fabs(dir.x / dir.y));
  if (dir.x > 0 && dir.y >= 0)
  {
    if (angle < M_PI / 4)
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }
  else if (dir.x <= 0 && dir.y > 0)
  {
    if (angle < M_PI / 4)
    {
      return 3;
    }
    else
    {
      return 2;
    }
  }
  else if (dir.x < 0 && dir.y <= 0)
  {
    if (angle < M_PI / 4)
    {
      return 4;
    }
    else
    {
      return 5;
    }
  }
  else
  {
    if (angle < M_PI / 4)
    {
      return 7;
    }
    else
    {
      return 6;
    }
  }
}
double ComputeAngle(double2 dir)
{
  double angle = atan(fabs(dir.x / dir.y));
  if (dir.x >= 0 && dir.y >= 0)
  {
    return angle;
  }
  else if (dir.x < 0 && dir.y >= 0)
  {
    return M_PI - angle;
  }
  else if (dir.x < 0 && dir.y < 0)
  {
    return M_PI + angle;
  }
  else
  {
    return 2 * M_PI - angle;
  }
}
int ProjectDirToBin2D(double angle, int nbin)
{
  return angle / (2 * M_PI / nbin);
}
int ProjectRelativeDirToBin2D(double2 dir, double2 center_dir)
{
  double angle1 = ComputeAngle(dir);
  double angle2 = ComputeAngle(center_dir);
  return ProjectDirToBin2D(
    angle1 - angle2 < 0 ? angle1 - angle2 + 2 * M_PI : angle1 - angle2, 8);
}


double HistMatchScore(std::vector<double>& hist1, std::vector<double>& hist2, double r)
{
  if (hist1.size() != hist2.size())
  {
    std::cout<<"dimension not match.\n";
    return 0.0;
  }

  // assume the last two elements of the hist store the space coordinate
  double space_dist = 
    pow(hist1[hist1.size() - 1] - hist2[hist2.size() - 1], 2);
  space_dist +=
    pow(hist1[hist1.size() - 2] - hist2[hist2.size() - 2], 2);
  if (sqrt(space_dist) > r)
  {
    return std::numeric_limits<double>::max();
  }

  double score = 0.0;
  for (size_t i = 0; i < hist1.size() - 2; ++i)
  {
    score += (hist1[i] - hist2[i]) * (hist1[i] - hist2[i]);
  }
  return sqrt(score);
}

#endif