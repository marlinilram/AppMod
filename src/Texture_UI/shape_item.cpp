#include "shape_item.h"
#include <QListWidgetItem>
#include <QListWidget>
ShapeItem::ShapeItem(QListWidget * parent, int type)
:QListWidgetItem(parent, type),
m_mini_texture_(NULL)
{
	this->m_shape_ = NULL;
	this->m_mini_texture_ = NULL;
};
ShapeItem::~ShapeItem()
{
	if (this->m_mini_texture_)
	{
		delete this->m_mini_texture_;
	}
};
Render_Shape* ShapeItem::get_shape()
{
	return this->m_shape_;
};
void ShapeItem::set_shape(Render_Shape* s)
{
	this->m_shape_ = s;
};

void ShapeItem::set_texture(MiniTexture* m)
{
	this->m_mini_texture_ = m;
};
MiniTexture* ShapeItem::get_texture()
{
	return this->m_mini_texture_;
};