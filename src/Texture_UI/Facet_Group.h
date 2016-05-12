#pragma once
#include "geometry_types.h"
#include "facet.h"
#include "color.h"
class Facet_Group
{
public:
	Facet_Group(void);
	~Facet_Group(void);
	std::string facet_group_name_;
	Point3f barycenter_origin_;
	Point3f barycenter_transformed_;
	Point3f barycenter_for_drawing_;
	Colorf					color_;
	
	void addfacet(Facet* face)
	{
		face->set_color(color_);
		faces_.push_back(face);
	}
	std::vector<Facet*>& getfaces(){return faces_;};
private:
	std::vector<Facet*> faces_;
};
