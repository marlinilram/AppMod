#include "shape_list.h"
#include "shape_item.h"
#include "../Model/Shape.h"
#include "../Model/Model.h"
#include "../DispModule/Viewer/DispObject.h"
#include "../UI/PolygonMesh_Manipulator.h"
#include "color.h"
#include "mini_texture.h"
ShapeList::ShapeList(QWidget * parent)
	:QListWidget(parent)
{
	m_texture_list_ = new QListWidget(parent);
	connect_signal();
};
ShapeList::~ShapeList()
{
	delete this->m_texture_list_;
};
void ShapeList::connect_signal()
{
	connect(this, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(item_changed(QListWidgetItem *)));
	connect(this->m_texture_list_, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(texture_item_clicked(QListWidgetItem *)));

};
void ShapeList::setParent(QWidget * parent)
{
	QListWidget::setParent(parent);
	this->m_texture_list_->setParent(parent);
}
void ShapeList::setGeometry(int x, int y, int w, int h)
{
	QListWidget::setGeometry(x, y, w, h);
	this->m_texture_list_->setGeometry(x+w, y, w, h);
};
void ShapeList::item_changed(QListWidgetItem* current)
{
	ShapeItem* si = dynamic_cast<ShapeItem*>(current);

	Render_Shape* sp = si->get_shape();
	if (sp == NULL)
	{
		return;
	}
	PolygonMesh_Manipulator::set_color(sp->getPolygonMesh(), LG::Vec3(1, 0, 0));
	sp->get_model()->get_dis_obj()->updateModelBuffer();
};
void ShapeList::texture_item_clicked(QListWidgetItem* item)
{
	if (item == NULL)
	{
		return;
	}
	int index = this->m_texture_list_->row(item);
	if (index < 0)
	{
		return;
	}
	ShapeItem* sp = dynamic_cast<ShapeItem*>(this->item(index));
	MiniTexture* mini = sp->get_texture();
	if (mini == NULL)
	{
		return;
	}
	mini->hide();
	MINITEXTURE_SHOW_MODE m = mini->get_show_mode();
	mini->show_origin_image_d0();
	mini->set_show_mode(m);
	mini->show();
};
void ShapeList::set_texture(ShapeItem * item, MiniTexture* texture)
{

	MiniTexture* m = item->get_texture();
	item->set_texture(texture);

	int index = this->row(item);
	if (index < 0)
	{
		return;
	}
	QListWidgetItem* it = this->m_texture_list_->item(index);

	if (texture != NULL)
	{
		it->setText("Texture setted");
		it->setBackground(QBrush(QColor(255,255,0)));
		item->setBackground(QBrush(QColor(255, 255, 0)));
		item->setIcon(QIcon(texture->get_file_name()));
		it->setIcon(QIcon(texture->get_file_name()));
	}
	else
	{
		it->setText("Texture not setted!");
		it->setBackground(QBrush(QColor(255, 0, 0)));
		item->setBackground(QBrush(QColor(255, 0, 0)));

		item->setIcon(QIcon());
		it->setIcon(QIcon());
	}
};

MiniTexture* ShapeList::get_mini_texture(int item_id)
{
	if (this->size().height() < 1 || item_id > (this->size().height() - 1))
	{
		return NULL;
	}

	ShapeItem* spi = dynamic_cast<ShapeItem*>(this->item(item_id));

	MiniTexture* mini = spi->get_texture();
	return mini;
};

std::string ShapeList::getTexturePath(int item_id)
{
  if (this->size().height() < 1 || item_id > (this->size().height() - 1))
  {
    return std::string("");
  }

  ShapeItem* spi = dynamic_cast<ShapeItem*>(this->item(item_id));

  MiniTexture* mini = spi->get_texture();
  if (mini == NULL)
  {
    return std::string("");
  }
  QString file_path = mini->get_file_name();
  std::string std_file_path = file_path.toStdString().substr(0, file_path.toStdString().find_last_of('/'));
  return std_file_path;
}

void ShapeList::clear()
{
// 	this->m_items_.clear();
// 	QListWidget::clear();
};
QListWidget* ShapeList::texture_list()
{
	return this->m_texture_list_;
};
void ShapeList::show()
{
	QListWidget::show();
	this->m_texture_list_->show();
};