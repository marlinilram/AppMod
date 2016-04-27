#include "DispObject.h"
#include "Bound.h"

DispObject::DispObject():
m_viewer_(NULL),
m_model_(NULL)
{

}

DispObject::~DispObject()
{

}

QGLViewer* DispObject::viewer()
{
	return this->m_viewer_;
};
void DispObject::set_viewer(QGLViewer* g)
{
	this->m_viewer_ = g;
};
void DispObject::setModel(std::shared_ptr<Model> shared_model)
{
	this->m_model_ = shared_model;
	this->m_model_->set_dis_obj(this);
};
std::shared_ptr<Model> DispObject::getModel() 
{ 
	return this->m_model_;
};