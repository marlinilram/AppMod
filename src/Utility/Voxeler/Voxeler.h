#pragma once

#include "Voxel.h"

#include "KDTreeWrapper.h"

#define glv(v) glVertex3fv(v.data())
#define gln glNormal3f

namespace VoxelerLibrary{

class Voxeler
{
private:
    LG::PolygonMesh * mesh;
    KDTreeWrapper kd;
	  LG::PolygonMesh::Vertex_attribute<Vec3> points;

	// Special voxels
    KDTreeWrapper outerVoxels, innerVoxels;

public:
    Voxeler( LG::PolygonMesh * src_mesh, double voxel_size, bool verbose = false);

	FaceBounds findFaceBounds( LG::PolygonMesh::Face f );
	bool isVoxelIntersects( const Voxel & v, LG::PolygonMesh::Face f );
	
	void update();
	void computeBounds();

	// Grow larger by one voxel
	void grow();

	// Find inside and outside of mesh surface
	std::vector< Voxel > fillOther();
  std::vector< Voxel > fillInside();
  void fillOuter(KDTreeWrapper & outside);

	// Intersection
	std::vector<Voxel> Intersects(Voxeler * other);
	std::map<int, Voxel> around(Point p);

	// Visualization:
	void draw();
	void setupDraw();
	static void drawVoxels( const std::vector< Voxel > & voxels, double voxel_size = 1.0);

  KDTreeWrapper corner_kd;
	std::vector< Point > corners;
	std::vector< std::vector<int> > cornerIndices;
	std::vector< int > cornerCorrespond;
	std::vector< Point > getCorners(int vid);
	int getClosestVoxel(Eigen::Vector3f point);
	int getEnclosingVoxel( Eigen::Vector3f point );

	std::vector< Point > getVoxelCenters();

	std::vector< Voxel > voxels;
	int getVoxelIndex(Voxel v);
	unsigned int d1, d2;
	bool isVerbose;
	bool isReadyDraw;

	double voxelSize;

	Voxel minVox;
	Voxel maxVox;
};

}
