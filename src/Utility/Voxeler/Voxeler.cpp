#include "qgl.h"
#include "Voxeler.h"
using namespace VoxelerLibrary;

#include "BoundingBox.h"
#include <stack>
#include <qset.h>

#include "voxel_weld.h"

Voxeler::Voxeler( LG::PolygonMesh * src_mesh, double voxel_size, bool verbose /*= false*/ )
{
	this->mesh = src_mesh;

  points = mesh->vertex_attribute<Point>("v:point");

	this->voxelSize = voxel_size;
	this->isVerbose = verbose;
	this->isReadyDraw = false;

	if(mesh == NULL)
		return;

	//if(isVerbose) qDebug() << "Computing voxels..";

	// For each face in mesh
  for (auto fit : mesh->faces())
  {
    FaceBounds fb = findFaceBounds( fit );

    for(int x = fb.minX; x <= fb.maxX; x++)
    {
      for(int y = fb.minY; y <= fb.maxY; y++)
      {
        for(int z = fb.minZ; z <= fb.maxZ; z++)
        {
          Voxel v(x,y,z);

          if(isVoxelIntersects(v, fit))
            voxels.push_back( v );
        }
      }
    }
  }
	
	// Combine into a set of voxels
	std::vector<size_t> xrefs;
  weld(voxels, xrefs, std::hash_VoxelerLibraryVoxel(), std::equal_to<Voxel>());

	// Add voxels to KD-tree and build
  {
    std::vector<float> kd_data;
    for (size_t i = 0; i < voxels.size(); ++i)
    {
      kd_data.push_back(voxels[i].x);
      kd_data.push_back(voxels[i].y);
      kd_data.push_back(voxels[i].z);
    }
    kd.initKDTree(kd_data, voxels.size(), 3);
  }

	computeBounds();

//	if(isVerbose) qDebug() << "Voxel count = " << (int)voxels.size();
}

void Voxeler::update()
{
	// Compute bounds
	computeBounds();

	// Setup visualization
	setupDraw();
}

void Voxeler::computeBounds()
{
	minVox = Voxel(INT_MAX, INT_MAX, INT_MAX);
	maxVox = Voxel(-INT_MAX, -INT_MAX, -INT_MAX);

	for(int i = 0; i < (int)voxels.size(); i++)
	{
		Voxel v = voxels[i];

		minVox.toMin(v);
		maxVox.toMax(v);
	}
}

FaceBounds Voxeler::findFaceBounds( PolygonMesh::Face f )
{
	FaceBounds fb;

	double minx = 0, miny = 0, minz = 0;
	double maxx = 0, maxy = 0, maxz = 0;

	// Collect points
  std::vector<Vector3f> f_vec;
  for (auto vfc : mesh->vertices(f))
  {
    f_vec.push_back(points[vfc]);
  }

	minx = maxx = f_vec[0](0);
	miny = maxy = f_vec[0](1);
	minz = maxz = f_vec[0](2);

	for(int v = 0; v < 3; v++)
	{
		Vector3f vec = f_vec[v];

		if (vec(0) < minx) minx = vec(0);
		if (vec(0) > maxx) maxx = vec(0);

		if (vec(1) < miny) miny = vec(1);
		if (vec(1) > maxy) maxy = vec(1);

		if (vec(2) < minz) minz = vec(2);
		if (vec(2) > maxz) maxz = vec(2);
	}

	fb.minX = floor(minx / voxelSize);
	fb.minY = floor(miny / voxelSize);
	fb.minZ = floor(minz / voxelSize);

	fb.maxX = ceil(maxx / voxelSize);
	fb.maxY = ceil(maxy / voxelSize);
	fb.maxZ = ceil(maxz / voxelSize);

	return fb;
}

bool Voxeler::isVoxelIntersects( const Voxel& v, PolygonMesh::Face f )
{
	Vector3f center = Vector3f(v.x * voxelSize, v.y * voxelSize, v.z * voxelSize);

	double s = voxelSize * 0.5;

	BoundingBox b(center, s,s,s);

	// Collect points
  std::vector<Vector3f> f_vec;
  for (auto vfc : mesh->vertices(f))
  {
    f_vec.push_back(points[vfc]);
  }

	return b.containsTriangle(f_vec[0], f_vec[1], f_vec[2]);
}

void Voxeler::draw()
{
	if(!isReadyDraw)
		update();

	glEnable(GL_LIGHTING);
	glShadeModel(GL_FLAT);

	glColor3f(0, 0.8f, 0);
	//glCallList(d1);

	glDisable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);

	glColor3f(0, 1, 0);
	glLineWidth(1.0);
	glCallList(d2);

	/*for(int i = 0; i < (int) temp2.size(); i++){
		Vector3d c = temp2[i];
		c *= voxelSize;
		if(temp2[i].x() < 0)
			SimpleDraw::DrawSolidBox(c, voxelSize, voxelSize, voxelSize, 1,0,0);
	}*/

	glEnable(GL_LIGHTING);
}

void Voxeler::setupDraw()
{
	double s = voxelSize * 0.5;
	int n = (int)voxels.size();
	
	std::vector< std::vector<Vector3f> > corner(n, std::vector<Vector3f>(8));

	// Find corners
	for(int i = 0; i < n; i++)
	{
		Vector3f c(voxels[i].x, voxels[i].y, voxels[i].z);	c *= voxelSize;
		corner[i][0] = Vector3f(s, s, s) + c;		corner[i][1] = Vector3f(-s, s, s) + c;
		corner[i][2] = Vector3f(-s, -s, s) + c;	corner[i][3] = Vector3f(s, -s, s) + c;
		corner[i][4] = Vector3f(s, s, -s) + c;		corner[i][5] = Vector3f(-s, s, -s) + c;
		corner[i][6] = Vector3f(-s, -s, -s) + c;	corner[i][7] = Vector3f(s, -s, -s) + c;
	}

	// Save corners
	corners.clear();
	cornerIndices.clear();
	cornerCorrespond.resize(n*8);

	// Build corner_kd
	std::vector<Vector3f> cornerPnts;
	for(int i = 0; i < n; i++)
		for(int j = 0; j < 8; j++)
			cornerPnts.push_back(corner[i][j]);

	std::vector<size_t> xrefs;
    weld(cornerPnts, xrefs, std::hash_Vector3f(), std::equal_to<Vector3f>());

  {
    std::vector<float> corner_kd_data;
    for (size_t i = 0; i < cornerPnts.size(); ++i)
    {
      corner_kd_data.push_back(cornerPnts[i](0));
      corner_kd_data.push_back(cornerPnts[i](1));
      corner_kd_data.push_back(cornerPnts[i](2));
    }
    corner_kd.initKDTree(corner_kd_data, cornerPnts.size(), 3);
  }


	for(int i = 0; i < n; i++)
	{
		std::vector<int> p(8, 0);

		for(int j = 0; j < 8; j++)
		{
      std::vector<float> query(3, 0);
      query[0]= corner[i][j](0);
      query[1]= corner[i][j](1);
      query[2]= corner[i][j](2);
      corner_kd.nearestPt(query, p[j]);
			cornerCorrespond[p[j]] = i; // will belong to last one
		}

		cornerIndices.push_back(p);
	}

	d1 = glGenLists(1);

	// Faces
	glNewList(d1, GL_COMPILE);
	glBegin(GL_QUADS);
	for(int i = 0; i < (int)voxels.size(); i++)
	{
		// top, left, right, bottom
		gln(0,0,1); glv(corner[i][0]); glv(corner[i][1]); glv(corner[i][2]); glv(corner[i][3]);
		gln(0,1,0); glv(corner[i][0]); glv(corner[i][1]); glv(corner[i][5]); glv(corner[i][4]);
		gln(0,-1,0); glv(corner[i][2]); glv(corner[i][3]); glv(corner[i][7]); glv(corner[i][6]);
		gln(0,0,-1); glv(corner[i][4]); glv(corner[i][5]); glv(corner[i][6]); glv(corner[i][7]);

		// front, back
		gln(1,0,0); glv(corner[i][0]); glv(corner[i][3]); glv(corner[i][7]); glv(corner[i][4]);
		gln(-1,0,0); glv(corner[i][1]); glv(corner[i][2]); glv(corner[i][6]); glv(corner[i][5]);
	}
	glEnd();
	glEndList();

	d2 = glGenLists(1);

	// Lines
	glNewList(d2, GL_COMPILE);
	glBegin(GL_LINES);
	for(int i = 0; i < (int)voxels.size(); i++)
	{
		glv(corner[i][0]);glv(corner[i][4]);glv(corner[i][1]);glv(corner[i][5]);
		glv(corner[i][2]);glv(corner[i][6]);glv(corner[i][3]);glv(corner[i][7]);
		glv(corner[i][0]);glv(corner[i][1]);glv(corner[i][2]);glv(corner[i][3]);
		glv(corner[i][0]);glv(corner[i][3]);glv(corner[i][1]);glv(corner[i][2]);
		glv(corner[i][4]);glv(corner[i][5]);glv(corner[i][6]);glv(corner[i][7]);
		glv(corner[i][4]);glv(corner[i][7]);glv(corner[i][5]);glv(corner[i][6]);
	}
	glEnd();
	glEndList();

	isReadyDraw = true;
}

void Voxeler::drawVoxels( const std::vector< Voxel > & voxels, double voxel_size )
{
	double s = voxel_size * 0.5;
	int n = (int)voxels.size();

	std::vector<Vector3f> c1(n), c2(n), c3(n), c4(n);
	std::vector<Vector3f> bc1(n), bc2(n), bc3(n), bc4(n);

	// Find corners
	for(int i = 0; i < (int)voxels.size(); i++){
		Vector3f c(voxels[i].x, voxels[i].y, voxels[i].z);	c *= voxel_size;
		c1[i] = Vector3f(s, s, s) + c; c2[i] = Vector3f(-s, s, s) + c;
		c3[i] = Vector3f(-s, -s, s) + c; c4[i] = Vector3f(s, -s, s) + c;
		bc1[i] = Vector3f(s, s, -s) + c; bc2[i] = Vector3f(-s, s, -s) + c;
		bc3[i] = Vector3f(-s, -s, -s) + c; bc4[i] = Vector3f(s, -s, -s) + c;
	}

	glColor3d(1,0,0);
	glLineWidth(3.0f);
	glDisable(GL_LIGHTING);

	// Lines
	glBegin(GL_LINES);
	for(int i = 0; i < (int)voxels.size(); i++){
		glv(c1[i]);glv(bc1[i]);glv(c2[i]);glv(bc2[i]);
		glv(c3[i]);glv(bc3[i]);glv(c4[i]);glv(bc4[i]);
		glv(c1[i]);glv(c2[i]);glv(c3[i]);glv(c4[i]);
		glv(c1[i]);glv(c4[i]);glv(c2[i]);glv(c3[i]);
		glv(bc1[i]);glv(bc2[i]);glv(bc3[i]);glv(bc4[i]);
		glv(bc1[i]);glv(bc4[i]);glv(bc2[i]);glv(bc3[i]);
	}
	glEnd();

	glEnable(GL_LIGHTING);
}

std::vector<Voxel> Voxeler::fillOther()
{
	std::vector<Voxel> filled;

	for(int x = minVox.x - 1; x <= maxVox.x + 1; x++){
		for(int y = minVox.y - 1; y <= maxVox.y + 1; y++){
			for(int z = minVox.z - 1; z <= maxVox.z + 1; z++){
                std::vector<float> v(3, 0);
                v[0] = x;
                v[1] = y;
                v[2] = z;
                if(!kd.has(v))
					filled.push_back(Voxel(x,y,z));
			}
		}
	}

	return filled;
}

std::vector<Voxel> Voxeler::fillInside()
{
	printf("Computing inside, outside..");

	std::vector<Voxel> innerVoxels;

	KDTreeWrapper outside;
	fillOuter(outside);

	// Compute inner as complement of outside
	for(int x = minVox.x - 1; x <= maxVox.x + 1; x++){
		for(int y = minVox.y - 1; y <= maxVox.y + 1; y++){
			for(int z = minVox.z - 1; z <= maxVox.z + 1; z++){
                std::vector<float> v(3, 0);
                v[0] = x;
                v[1] = y;
                v[2] = z;
                if(!outside.has(v)){
					innerVoxels.push_back(Voxel(x,y,z));
          voxels.push_back(Voxel(x,y,z));
				}
			}
		}
	}

	return innerVoxels;
}

void Voxeler::fillOuter(KDTreeWrapper & outside)
{
	std::stack<Voxel> stack;

	stack.push(maxVox + Voxel(1,1,1));

	std::vector<Voxel> outterVoxels;

	QSet<QString> seenSet;

//	qDebug() << "Filling outside..";

  std::vector<float> outside_data;
	while(!stack.empty())
	{
		// Get next square
		Voxel c = stack.top(); // get current voxel
		stack.pop();

		// Bad: using strings for now
		QString cellString = QString("%1,%2,%3").arg(c.x).arg(c.y).arg(c.z);

		std::vector<float> p(3, 0);
    p[0] = c.x;
    p[1] = c.y;
    p[2] = c.z;

		// Base case:
		if( !kd.has(p) && !seenSet.contains(cellString) )
		{
			seenSet.insert( cellString );

      outside_data.push_back(p[0]);
      outside_data.push_back(p[1]);
      outside_data.push_back(p[2]);

			// Visit neighbors
			if(c.x < maxVox.x + 1) stack.push( c + Voxel( 1, 0, 0) );
			if(c.y < maxVox.y + 1) stack.push( c + Voxel( 0, 1, 0) );
			if(c.z < maxVox.z + 1) stack.push( c + Voxel( 0, 0, 1) );

			if(c.x > minVox.x - 1) stack.push( c + Voxel(-1, 0, 0) );
			if(c.y > minVox.y - 1) stack.push( c + Voxel( 0,-1, 0) );
			if(c.z > minVox.z - 1) stack.push( c + Voxel( 0, 0,-1) );
		}
	}

	outside.initKDTree(outside_data, outside_data.size() / 3, 3);

//	qDebug() << "Outside voxels filled!";
}

std::vector<Voxel> Voxeler::Intersects(Voxeler * other)
{
	std::vector<Voxel> intersection;

	Voxeler *minVoxeler = this, *maxVoxeler = other;

	// Swap with minimum
	if(other->voxels.size() < this->voxels.size()){
		minVoxeler = other;
		maxVoxeler = this;
	}

	for(int i = 0; i < (int) minVoxeler->voxels.size(); i++)
	{
		Voxel v = minVoxeler->voxels[i];
        
        std::vector<float> vv(3, 0);
        vv[0] = v.x;
        vv[1] = v.y;
        vv[2] = v.z;
        if(maxVoxeler->kd.has(vv))
			intersection.push_back(v);
	}

	return intersection;
}

std::map<int, Voxel> Voxeler::around(Point p)
{
	std::map<int, Voxel> result;

	int x = p.x() / voxelSize;
	int y = p.y() / voxelSize;
	int z = p.z() / voxelSize;

	for(int i = -1; i <= 1; i += 1){
		for(int j = -1; j <= 1; j += 1){
			for(int k = -1; k <= 1; k += 1){
				Voxel v(x + i, y + j, z + k);

				Vector3f vpos(v.x, v.y, v.z);
        std::vector<float> vv(3, 0);
        vv[0] = v.x;
        vv[1] = v.y;
        vv[2] = v.z;

                if(kd.has(vv)){
          int idx = 0;
          std::vector<float> vpos(vv);
          kd.nearestPt(vpos, idx);
					result[idx] = v;
				}
			}
		}
	}

	return result;
}

void Voxeler::grow()
{
	int N = (int)voxels.size();

	std::vector<Voxel> newVox;

	for(int i = 0; i < N; i++)
	{
		Voxel curVoxel = voxels[i];

		for(int x = -1; x <= 1; x += 1){
			for(int y = -1; y <= 1; y++){
				for(int z = -1; z <= 1; z++){
					Voxel v(curVoxel.x + x, curVoxel.y + y, curVoxel.z + z);
					newVox.push_back(v);
				}
			}
		}
	}

	// Combine into set of voxels
	std::vector<size_t> xrefs;
    weld(voxels, xrefs, std::hash_VoxelerLibraryVoxel(), std::equal_to<Voxel>());

	// Clear old, add new points and build
  std::vector<float> kd_data;
  for (size_t i = 0; i < voxels.size(); ++i)
  {
    kd_data.push_back(voxels[i].x);
    kd_data.push_back(voxels[i].y);
    kd_data.push_back(voxels[i].z);
  }
	kd.initKDTree(kd_data, voxels.size(), 3);

	printf("\nVoxler grown from (%d) to (%d).\n", N, (int)voxels.size());

	isReadyDraw = false;
}

std::vector< Point > Voxeler::getCorners( int vid )
{
	std::vector< Point > result;

	for(int i = 0; i < 8; i++)
		result.push_back(corners[cornerIndices[vid][i]]);

	return result;
}

int Voxeler::getClosestVoxel( Vector3f point )
{
  int idx = 0;
  std::vector<float> pt_in;
  pt_in[0] = point(0);
  pt_in[1] = point(1);
  pt_in[2] = point(2);
  corner_kd.nearestPt(pt_in, idx);
	return cornerCorrespond[ idx ];
}

int Voxeler::getEnclosingVoxel( Vector3f point )
{
	int N = (int)voxels.size();

	double s = voxelSize * 0.5;

	double minDist = DBL_MAX;
	int closestVoxel = -1;

	for(int i = 0; i < N; i++)
	{
		Voxel curVoxel = voxels[i];

		Point voxelCenter(curVoxel.x * s, curVoxel.y * s, curVoxel.z * s);

		double curDist = (voxelCenter - point).norm();

		if(curDist < minDist){
			closestVoxel = i;
			minDist = curDist;
		}
	}

	return closestVoxel;
}

int Voxeler::getVoxelIndex( Voxel v )
{
	for(int i = 0; i < (int) voxels.size(); i++)
	{
		Voxel w = voxels[i];

		if(w.x == v.x && w.y == v.y && w.z == v.z)
			return i;
	}

	return -1;
}

std::vector< Point > Voxeler::getVoxelCenters()
{
	std::vector< Point > pnts;

	for(int i = 0; i < (int) voxels.size(); i++){
		Voxel w = voxels[i];
		pnts.push_back(Point(w.x * voxelSize, w.y* voxelSize, w.z* voxelSize) );
	}

	return pnts;
}
