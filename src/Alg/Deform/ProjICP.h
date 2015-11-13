#ifndef PROJICP_H
#define PROJICP_H

#include <memory>

class FeatureGuided;

class ProjICP
{
public:
  ProjICP();
  ~ProjICP();

  void buildCrsp(std::shared_ptr<FeatureGuided> feature_model);

  void testCrsp(std::shared_ptr<FeatureGuided> feature_model);

private:
  ProjICP(const ProjICP&);
  void operator=(const ProjICP&);
};


#endif // !PROJICP_H
