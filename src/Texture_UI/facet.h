#ifndef _FACET_H_
#define _FACET_H_

#include "vertex.h"
#include "color.h"
#include "geometry_types.h"
class Facet 
{
public:
// 	Facet(const std::vector<Point3f>& points) ;
// 	Facet(const std::vector<Point3f>& points, const Vector3_f& n);
	Facet(const std::vector<Vertex*>& vertices) ;
	Facet(const std::vector<Vertex*>& vertices, const Vector3_f& n);
	~Facet() {}

	Plane3f supporting_plane() const ;

	std::vector<Vertex*>&  vertices() { return vertices_; }	
	const std::vector<Vertex*>&  vertices() const { return vertices_; }

	unsigned int size() const { return vertices_.size(); }

	Vector3_f& normal() { return normal_; }
	const Vector3_f& normal() const { return normal_; }

	void set_normal(const Vector3_f& n) { normal_ = n; }
	void reverse_normal() { normal_ = -normal_; }

	void set_color(Colorf color){color_ = color;};
	const Colorf& get_color(){return color_;};

	void set_facet_segment_name(std::string& seg_name){facet_segment_name_ = seg_name;};
	const std::string& get_facet_segment_name(){return facet_segment_name_ ;};


	bool is_in_facet(const Point3f& p, double squared_dist_thresh) const ;

	virtual void draw(bool wireframe = false) const ;
	virtual void draw_with_name(unsigned int i) const ;

	bool is_visible() const{ return this->m_visileb_; };
	void set_visible(bool b){ this->m_visileb_ = b; };
private:
		std::string				facet_segment_name_;
		void create_edges_from_vertices();
		Colorf					color_;
protected:
	std::vector<Vertex*>	vertices_; //
	Vector3_f				normal_;
	bool					m_visileb_;
};



#endif