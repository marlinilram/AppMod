#ifndef _VERTEX_H_
#define _VERTEX_H_
#include "color.h"
#include "geometry_types.h"

struct vote
{
	int support_;
	float weight_;
	vote(int support,
	float weight)
	{
		support_ = support;
		weight_ = weight;
	}
};
class Vertex
{
public:
	Vertex(const Point3f& p, const Colorf& c = Colorf(0, 0, 0, 1))
	: point_(p)
		, color_(c)
	{
	}

	const Point3f& point() const     { return point_; }
	Point3f& point()                 { return point_; }
	void set_point(const Point3f& p) { point_ = p; }

	const Vector3_f& normal() const		{ return normal_; }
	Vector3_f& normal()					{ return normal_; }
	void set_normal(const Vector3_f& n)	{ normal_ = n; }

	inline void draw(bool wireframe = false) const {
		if (visible_) {
			glColor4fv(color_.data());
			glNormal3d(normal_.x(), normal_.y(), normal_.z());
			glVertex3d(point_.x(), point_.y(), point_.z());
		}		
	}

	inline void draw_with_name(unsigned int i) const {
		glPushName(i);
		if (visible_) {
			glRasterPos3d(point_.x(), point_.y(), point_.z());
		}
		glPopName();
	}

	virtual void translate(const Vector3_f& d) {	point_ = point_ + d; }

	const Colorf& color() const { return color_; } 
	void set_color(const Colorf& c) { color_ = c; } 


private:

	Point3f		 point_ ;
	Vector3_f     normal_;
	Colorf       color_;
	bool		visible_;
} ;

#endif