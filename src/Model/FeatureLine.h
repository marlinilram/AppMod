#ifndef FeatureLine_H
#define FeatureLine_H

#include "BasicHeader.h"

class FeatureLine
{
public:
  FeatureLine();
  ~FeatureLine();

public:
  std::vector<std::vector<Vector2f> > lines;

private:
  FeatureLine(const FeatureLine&);
  void operator = (const FeatureLine&);
};
#endif // !FeatureLine_H
