#ifndef KDTreeWrapper_H
#define KDTreeWrapper_H

#include "kdtree.h"

#include <memory>
#include <vector>

class KDTreeWrapper
{
public:
  KDTreeWrapper();
  ~KDTreeWrapper();

  void initKDTree(std::vector<float>& data, size_t num_pts, int dim);
  void nearestPt(std::vector<float>& pt);
  void nearestPt(std::vector<float>& pt, int& pt_id);
  void nearestPt(std::vector<float>& pt, int& pt_id, float& dis);
  void nearestPt(int n_neighbor, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis, std::vector<int>& pt_id);
  float nearestDis(std::vector<float>& pt);
  void rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis);
  void rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis, std::vector<int>& pt_id);
  int rNearestPt(float r, std::vector<float>& pt_in);
  void rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& dis);
  int nDataPt();
  bool has(std::vector<float>& pt_in, float epsilon = 1e-7);

private:
  std::shared_ptr<kdtree::KDTree> kdTree;
  kdtree::KDTreeArray kdTree_data;
private:
  KDTreeWrapper(const KDTreeWrapper&);
  void operator = (const KDTreeWrapper&);
};

#endif // !KDTreeWrapper_H
