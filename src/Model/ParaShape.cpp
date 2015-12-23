#include "ParaShape.h"
#include "KDTreeWrapper.h"
#include "Shape.h"

ParaShape::ParaShape()
{
  cut_shape = nullptr;
  kdTree_UV = nullptr;
}

ParaShape::~ParaShape()
{

}

void ParaShape::initUVKDTree()
{
  kdTree_UV.reset(new KDTreeWrapper);
  kdTree_UV->initKDTree(std::vector<float>(cut_shape->getUVCoord()), cut_shape->getUVCoord().size() / 2, 2);
}