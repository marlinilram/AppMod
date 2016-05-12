#include "poly_mesh.h"
#include <gl/glut.h>
#include <algorithm>
PolyMesh::PolyMesh(const std::string& name)
: wireframe_color_(0, 0, 1, 1)
{
	bbx_computed_ = false;
}

PolyMesh::~PolyMesh(void) {

	this->clear();
}


void PolyMesh::draw(bool wireframe/* = false*/) const {
	
		for (unsigned int i=0; i<faces_.size(); ++i) {
			Facet* f = faces_[i];
			std::vector<Vertex*>& vts = f->vertices();
			const Vector3_f& normal = f->normal();
			glColor3fv(f->get_color().data());
			glBegin(GL_POLYGON);
			glNormal3f(normal.x(), normal.y(), normal.z());
			const Colorf& color = faces_[i]->get_color();
			glColor3f(color.r(), color.g(), color.b());
			for (unsigned int j=0; j<vts.size(); ++j) {
				const Point3f& v = vts[j]->point();
				glVertex3f(v.x(), v.y(), v.z());
			}
			glEnd();
		}

}

void PolyMesh::add_facet(const std::vector<unsigned int>& indices) {
	if (indices.size() >= 3) {
		std::vector<Vertex*> vts;
		for (unsigned int i=0; i<indices.size(); ++i) {
			unsigned int index = indices[i];
			vts.push_back(vertices_[index]);
		}
		Facet* f = new Facet(vts);
		add_facet(f);
	}
}

void PolyMesh::set_color_by_normal()
{
	for (unsigned int i = 0; i < faces_.size(); ++i) {
		Facet* f = faces_[i];
		const Vector3_f& n = f->normal();
		int r = int(std::min(std::max(128.0 * (n.x() + 1.0), 0.0), 255.99));
		int g = int(std::min(std::max(128.0 * (n.y() + 1.0), 0.0), 255.99));
		int b = int(std::min(std::max(128.0 * (n.z() + 1.0), 0.0), 255.99));
		f->set_color(Colorf(r / 256.0f, g / 256.0f, b / 256.0f));
	}
};

Vertex* PolyMesh::add_vertex(const Point3f& p)
{
	Vertex* v = new Vertex(p);
	vertices_.push_back(v);
	return v;
};

const Bbox3f& PolyMesh::bbox() {
	if (bbx_computed_)
		return bbox_;

	compute_bbox();
	bbx_computed_ = true;
	return bbox_;
}

void PolyMesh::compute_bbox()
{
	float minx = 99999999;
	float maxx = -minx;

	float miny = 99999999;
	float maxy = -miny;

	float minz = 99999999;
	float maxz = -minz;

	for (unsigned int i = 0; i < this->vertices_.size();i++)
	{
		const Point3f& p = vertices_[i]->point();
		float x = p.x();
		float y = p.y();
		float z = p.z();

		if (x < minx)
		{
			minx = x;
		}
		if (x >maxx)
		{
			maxx = x;
		}

		if (y < miny)
		{
			miny = y;
		}
		if (y > maxy)
		{
			maxy = y;
		}

		if (z < minz)
		{
			minz = z;
		}
		if (z > maxz)
		{
			maxz = z;
		}
	}
	this->bbox_ = Bbox3f(minx, miny, minz, maxx, maxy, maxz);
	bbx_computed_ = true;
}