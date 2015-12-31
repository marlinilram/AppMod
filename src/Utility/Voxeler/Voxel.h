#pragma once
#include <float.h>
#include "PolygonMesh.h"
#include "BasicHeader.h"
using namespace LG;

namespace VoxelerLibrary{

struct Voxel{ 
	int x, y, z;
	int flag;

	Voxel(){ x = y = z = flag = 0; }
	Voxel(int X, int Y, int Z) : x(X), y(Y), z(Z){ flag = 1; } 

	// Operators
    operator const Vec3() const{ return Vec3(x,y,z); }

	Voxel & operator+= (const Voxel & other){
		x += other.x;	y += other.y;	z += other.z;
		return *this;
	}

	Voxel operator+(const Voxel & other) const{
		return Voxel(*this) += other;
	}

	bool operator== (const Voxel & other) const{
		return this->x == other.x && this->y == other.y && this->z == other.z;
	}

	// Useful for bounds
    inline void toMax(const Voxel & v){ x = std::max(x, v.x); y = std::max(y, v.y); z = std::max(z, v.z); }
    inline void toMin(const Voxel & v){ x = std::min(x, v.x); y = std::min(y, v.y); z = std::min(z, v.z); }
};

struct FaceBounds { int minX, minY, minZ; int maxX, maxY, maxZ; };

}
