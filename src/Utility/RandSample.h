#ifndef RandSample_H
#define RandSample_H

#include "BasicHeader.h"

#include <set>

void RandSample(int r_min, int r_max, int num, std::vector<int>& output)
{
  std::set<int> generated;
  for (int i = 0; i < num;)
  {
    int v = rand() % r_max + r_min;
    if (generated.find(v) == generated.end())
    {
      output.push_back(v);
      ++i;
      generated.insert(v);
    }
  }
  std::sort(output.begin(), output.end());
}

#endif // !RandSample_H
