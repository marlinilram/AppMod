#ifndef Ray_H
#define Ray_H

#include "Eigen\Eigen"
#include <vector>

// parametric equation of ray point(t) = P + t * D
bool intersectTriangle(Eigen::Vector3f p, Eigen::Vector3f d,
    Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2);

bool intersectModel(Eigen::Vector3f p, Eigen::Vector3f d, std::vector<float> &vertices, std::vector<unsigned int> &faces);

#endif