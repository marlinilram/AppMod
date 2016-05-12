#include "facet.h"
#include "vertex.h"
#include "color_table.h"
Facet::Facet(const std::vector<Vertex*>& vertices) 
: vertices_(vertices)
{
	if (vertices.size() >= 3) {
		Point3f A = vertices[0]->point();
		Point3f B = vertices[1]->point();
		Point3f C = vertices[2]->point();
		normal_ = CGAL::cross_product(C-B, A-B);
		GLOBAL::normalize(normal_);
	}
	color_ = Colorf(0.0, 0.0, 0.0);
}

Facet::Facet(const std::vector<Vertex*>& vertices, const Vector3_f& n)
:vertices_(vertices)
, normal_(n)
{
}


Plane3f Facet::supporting_plane() const {
	Point3f p = vertices_[0]->point();
	return Plane3f(p, normal_);
}


void Facet::draw(bool wireframe/* = false*/) const {
	if (wireframe) {
		glDisable(GL_LIGHTING);
	} else {
		glEnable(GL_LIGHTING);
		glColor4fv(color_.data());
	}

	glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);

	glBegin(GL_POLYGON);
	glNormal3f(normal_.x(), normal_.y(), normal_.z());
	for (unsigned int i=0; i<vertices_.size(); ++i) {
		const Point3f& v = vertices_[i]->point();
		glVertex3f(v.x(), v.y(), v.z());
	}
	glEnd();
}

void Facet::draw_with_name(unsigned int i) const {
	glPushName(i);
	if (is_visible()) {
		draw(false);
	}
	glPopName();
}