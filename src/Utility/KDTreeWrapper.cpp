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
  //std::cout << "Building KDTree...\n";
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
  kdTree->sort_results = true;
  //std::cout << "Build KDTree finished.\n";
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

void KDTreeWrapper::rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis)
{
  kdtree::KDTreeResultVector result;
  kdTree->r_nearest(pt_in, r, result);
  for (size_t i = 0; i < result.size(); ++i)
  {
    for (int j = 0; j < kdTree->dim; ++j)
    {
      pt_out.push_back(kdTree->the_data[result[i].idx][j]);
    }
    dis.push_back(result[i].dis);
  }
}

void KDTreeWrapper::rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis, std::vector<int>& pt_id)
{
  kdtree::KDTreeResultVector result;
  kdTree->r_nearest(pt_in, r, result);
  for (size_t i = 0; i < result.size(); ++i)
  {
    for (int j = 0; j < kdTree->dim; ++j)
    {
      pt_out.push_back(kdTree->the_data[result[i].idx][j]);
    }
    dis.push_back(result[i].dis);
    pt_id.push_back(result[i].idx);
  }
}

int KDTreeWrapper::rNearestPt(float r, std::vector<float>& pt_in)
{
  kdtree::KDTreeResultVector result;
  kdTree->r_nearest(pt_in, r, result);
  return (int)result.size();
}

void KDTreeWrapper::rNearestPt(float r, std::vector<float>& pt_in, std::vector<float>& dis)
{
  kdtree::KDTreeResultVector result;
  kdTree->r_nearest(pt_in, r, result);
  for (size_t i = 0; i < result.size(); ++i)
  {
    dis.push_back(result[i].dis);
  }
}

void KDTreeWrapper::nearestPt(int n_neighbor, std::vector<float>& pt_in, std::vector<float>& pt_out, std::vector<float>& dis, std::vector<int>& pt_id)
{
  kdtree::KDTreeResultVector result;
  kdTree->n_nearest(pt_in, n_neighbor, result);
  for (size_t i = 0; i < result.size(); ++i)
  {
    for (int j = 0; j < kdTree->dim; ++j)
    {
      pt_out.push_back(kdTree->the_data[result[i].idx][j]);
    }
    dis.push_back(result[i].dis);
    pt_id.push_back(result[i].idx);
  }
}

int KDTreeWrapper::nDataPt()
{
  return kdTree->N;
}

bool KDTreeWrapper::has(std::vector<float>& pt_in, float epsilon)
{
  kdtree::KDTreeResultVector result;
  kdTree->r_nearest(pt_in, epsilon, result);
  if (result.empty()) return false;
  else return true;
}