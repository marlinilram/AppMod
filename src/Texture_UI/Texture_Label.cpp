#include "Texture_Label.h"
#include <QApplication>
#include <QDesktopWidget>
#include "mini_texture.h"

Texture_Label::Texture_Label(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{
	m_full_image_label_ = NULL;
	m_with_image_ = false;
	m_new_image_ = false;
	//this->setScaledContents(true);
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
};
Texture_Label::~Texture_Label()
{
	if (this->m_full_image_label_)
	{
		delete this->m_full_image_label_;
		this->m_full_image_label_ = NULL;
	}
};

void Texture_Label::set_file(QString s)
{
	this->m_image_file_ = s;
	this->m_image_ = QImage(s); 
	this->set_image(this->m_image_);
};
void Texture_Label::clear()
{
	QLabel::clear();
	this->m_image_ = QImage();
	if (m_full_image_label_ != NULL)
	{
		this->m_full_image_label_->clear();
	}
	
	this->m_image_file_ = "";

	m_with_image_ = false;
	m_new_image_ = true;
};
void Texture_Label::set_image(const QImage& img)
{
	this->m_image_ = img;
	this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
	m_new_image_ = true;
};
void Texture_Label::mouseReleaseEvent(QMouseEvent * event)
{
	if (event->button() == Qt::RightButton)
	{
		if (m_full_image_label_ != NULL)
		{
			m_full_image_label_->hide();
		}
	}
	this->setStyleSheet("border: 1px groove gray;");
};
void Texture_Label::mousePressEvent(QMouseEvent * event)
{
	if (event->button() == Qt::RightButton)
	{
		if (m_full_image_label_ == NULL)
		{
			m_full_image_label_ = new QLabel();
		}

		if (this->m_with_image_ == false && m_new_image_ == true)
		{
			m_full_image_label_->setPixmap(QPixmap::fromImage(this->m_image_));
			int wid = this->m_image_.width();
			int hei = this->m_image_.height();
			this->m_with_image_ = true;

			QRect clientRect = QApplication::desktop()->availableGeometry();
			QPoint center = clientRect.center();
			this->m_full_image_label_->setGeometry(center.x() - wid / 2, center.y() - hei / 2, wid, hei);
			this->m_full_image_label_->setWindowFlags(Qt::FramelessWindowHint);
		}
		this->setStyleSheet("border: 3px groove red;");
		this->m_full_image_label_->show();
	}
	else if (event->button() == Qt::LeftButton)
	{
		this->setStyleSheet("border: 5px groove green;");
	}
};


void Texture_Label::mouseDoubleClickEvent(QMouseEvent * event)
{
	if (event->button() == Qt::LeftButton && !this->m_image_file_.isEmpty())
	{
		MiniTexture* mini = new MiniTexture(NULL);
		mini->setPixmap(QPixmap::fromImage(this->m_image_));
// 		int wid = this->m_image_.width();
// 		int hei = this->m_image_.height();
		int wid = 512;
		int hei = 512;

		QRect clientRect = QApplication::desktop()->availableGeometry();
		QPoint center = clientRect.center();
		mini->setGeometry(center.x() - wid / 2, center.y() - hei / 2, wid, hei);
		mini->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		mini->setParent(NULL);
		mini->set_file(m_image_file_);
		mini->show_origin_image_d0();
		mini->show();
//  		mini->load_texture();
//  	mini->show_mesh_image_d0();
		emit cut_selected(mini);
// 		MiniTexture*  mini2 = new MiniTexture();
// 		mini->clone(*mini2);
// 
// 		mini2->hide();
// 		mini2->show_origin_image_d0();
// 		mini2->show();
// 		mini2->setWindowTitle("mini2");
// 		emit cut_selected(mini2);
	}
};
void Texture_Label::select_cut(MiniTexture*)
{

};