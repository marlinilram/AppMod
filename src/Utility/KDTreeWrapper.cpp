#include "KDTreeWrapper.h"
#include <iostream>

KDTreeWrapper::KDTreeWrapper()
{

}

KDTreeWrapper::~KDTreeWrapper()
{

}

void KDTreeWrapper::initKDTree(std::vector<float>& data, size_t num_pts, int dim)
{
  std::cout << "Building KDTree...\n";
  kdTree_data.resize(boost::extents[num_pts][dim]);
  int cnt_edge_points = 0;
  for (int i = 0; i < data.size() / dim; ++i)
  {
    for (int j = 0; j < dim; ++j)
    {
      kdTree_data[i][j] = data[dim * i + j];
    }
  }

  kdTree.reset(new kdtree::KDTree(kdTree_data));
  std::cout << "Build KDTree finished.\n";
}

void KDTreeWrapper::nearestPt(std::vector<float>& pt)
{
  kdtree::KDTreeResultVector result;
  kdTree->n_nearest(pt, 1, result);
  for (int i = 0; i < kdTree->dim; ++i)
  {
    pt[i] = kdTree->the_data[result[0].idx][i];
  }
}

float KDTreeWrapper::nearestDis(std::vector<float>& pt)
{
  kdtree::KDTreeResultVector result;
  kdTree->n_nearest(pt, 1, result);
  return result[0].dis;
}

void KDTreeWrapper::nearestPt(std::vector<float>& pt, int& pt_id)
{
  kdtree::KDTreeResultVector result;
  kdTree->n_nearest(pt, 1, result);
  for (int i = 0; i < kdTree->dim; ++i)
  {
    pt[i] = kdTree->the_data[result[0].idx][i];
  }

  pt_id = result[0].idx;
}

void KDTreeWrapper::nearestPt(std::vector<float>& pt, int& pt_id, float& dis)
{
  kdtree::KDTreeResultVector result;
  kdTree->n_nearest(pt, 1, result);
  for (int i = 0; i < kdTree->dim; ++i)
  {
    pt[i] = kdTree->the_data[result[0].idx][i];
  }

  pt_id = result[0].idx;
  dis = result[0].dis;
}