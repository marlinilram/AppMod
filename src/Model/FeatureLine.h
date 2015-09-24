#ifndef FeatureLine_H
#define FeatureLine_H

#include "BasicHeader.h"
#include "BasicDataType.h"

class FeatureLine
{
public:
  FeatureLine();
  ~FeatureLine();

public:
  std::vector<std::vector<double2> > lines;

private:
  FeatureLine(const FeatureLine&);
  void operator = (const FeatureLine&);
};
#endif // !FeatureLine_H
