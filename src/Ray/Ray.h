#ifndef Ray_H
#define Ray_H

#include "Eigen\Eigen"
#include <vtkSmartPointer.h>
#include <vtkModifiedBSPTree.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vector>

// parametric equation of ray point(t) = P + t * D
bool intersectTriangle(Eigen::Vector3f &p, Eigen::Vector3f &d,
    Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f &v2);

bool intersectModel(Eigen::Vector3f &p, Eigen::Vector3f &d, std::vector<float> &vertices, std::vector<unsigned int> &faces);

class Ray
{
public:
	Ray(){};
	~Ray(){};

	bool intersectModel(Eigen::Vector3d &ray_start, Eigen::Vector3d &ray_end);
	void passModel(std::vector<float> &vertices, std::vector<unsigned int> &faces);

public:
	vtkSmartPointer< vtkModifiedBSPTree > bsptree;
	vtkSmartPointer< vtkPolyData > poly_mesh;
	double intersect_point[3];
	double pt_coord[3];
	double t;
	int sub_id;
};

#endif