#ifndef _POLY_MESH_H_
#define _POLY_MESH_H_

#include <string>
#include "facet.h"
#include "color.h"
#include "Facet_Group.h"
#include "geometry_types.h"
#include <string>
class PolyMesh 
{
public:
	PolyMesh(const std::string& name = "");
	~PolyMesh(void);

	virtual std::string title() { return "[PolyMesh]: "; }

	void draw(bool wireframe = false) const ;
	int num_facets(){return faces_.size();}
	void add_facet(const std::vector<unsigned int>& indices) ;
	void clear(){	
	for (unsigned int i=0; i<faces_.size(); ++i) {
		delete (faces_[i]);
	}
	faces_.clear() ;
	};
	void add_facet(const std::vector<unsigned int>& indices, const std::string& facet_type);

	std::vector<Facet*>& faces() { return faces_; }
	const std::vector<Facet*>& faces() const { return faces_; }
	
	void set_face_groups(const std::vector<Facet_Group>& face_groups){face_groups_ = face_groups;};
	std::vector<Facet_Group>& get_face_groups(){return this->face_groups_;}
	virtual Vertex* add_vertex(const Point3f& p);
	void set_color_by_normal();

	void compute_bbox();
	const Bbox3f& bbox();
	void set_file_name(const std::string&);
	const std::string& get_file_name();

	bool get_need_to_be_deleted();
	void set_need_to_be_deleted(bool b);
private:
	void add_facet(Facet* f) { faces_.push_back(f); }
	std::vector<Facet*>  faces_;
	Colorf wireframe_color_;
	std::vector<Facet_Group> face_groups_;
	std::vector<Vertex*>	 vertices_;
	Bbox3f					bbox_;
	bool					bbx_computed_;
	std::string				m_file_name_;
	bool					m_need_to_be_deleted_;
};



#endif