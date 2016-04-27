#include "mini_texture.h"
#include <QApplication>
#include <QDesktopWidget>
MiniTexture::MiniTexture(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{

	m_with_image_ = false;
	m_new_image_ = false;
	this->m_image_file_.clear();
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	m_texture_ = NULL;
};
MiniTexture::~MiniTexture()
{
	
};
QString MiniTexture::get_file_name()
{
	return this->m_image_file_;
};

void MiniTexture::set_file(QString s)
{
	this->m_image_file_ = s;
	this->load_texture();
};
void MiniTexture::clear()
{
	QLabel::clear();
	this->m_image_ = QImage();
	
	this->m_image_file_ = "";

	m_with_image_ = false;
	m_new_image_ = true;
};
void MiniTexture::set_image(const QImage& img)
{
	this->m_image_ = img;
	this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
	m_new_image_ = true;
};
void MiniTexture::load_texture()
{
	if (this->m_image_file_.isEmpty())
	{
		return;
	}
	else
	{

	}
};