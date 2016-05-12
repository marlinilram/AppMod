#include "texture_points_corres.h"
#include <QApplication>
#include <QDesktopWidget>
#include "PolygonMesh.h"
#include <QGLViewer/qglviewer.h>
#include "color_table.h"
#include <QMouseEvent>
#include <QAction>
#include <QMenu>
#include "AppearanceModel.h"
#include "ShapeUtility.h"
#include "ParaShape.h"
#include "TexSynHandler.h"
#include "Texture_Viewer.h"
#include<opencv2/highgui/highgui.hpp>
Texture_Mesh_Corres::Texture_Mesh_Corres(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{
	m_left_button_ = false;
	m_new_image_ = false;
	m_mesh_ = NULL;
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	this->setWindowFlags(Qt::SubWindow);

	this->m_width_icon_ = 100;
	this->m_height_icon_ = 100;

	QAction*	m_delete_action_ = NULL;
	QMenu*	   m_menu_ = NULL;

	this->m_show_mode_ = -1;
	this->resize(this->m_width_icon_, this->m_height_icon_);

	m_label_full_image_ = NULL;
	m_focus_on_ = false;
	m_viewer_ = NULL;
};

Texture_Mesh_Corres::~Texture_Mesh_Corres()
{
	
};



void Texture_Mesh_Corres::timerEvent(QTimerEvent* e)
{
	if (this->timer_id == e->timerId())
	{
		int num_mode = this->m_show_mode_ + 1;
		if (num_mode > 2)
		{
			num_mode = 0;
		}
		this->set_show_mode(num_mode);
		this->m_show_mode_ = num_mode;
	}
	QLabel::timerEvent(e);
};


void Texture_Mesh_Corres::creat_menu()
{
	m_menu_ = new QMenu(this);
	QAction* delete_action = new QAction(tr("delete this"), m_menu_);
	m_menu_->addAction(delete_action);
	connect(delete_action, SIGNAL(triggered()), this, SLOT(delete_this()));
};

void Texture_Mesh_Corres::set_image(const QImage& img)
{
	this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
	m_new_image_ = true;
};
void Texture_Mesh_Corres::dropEvent(QDropEvent * event)
{
	QLabel::dropEvent(event);
};

void Texture_Mesh_Corres::paintEvent(QPaintEvent * event)
{
	QLabel::paintEvent(event);
};
void Texture_Mesh_Corres::delete_this()
{
	emit delete_coress(this);
};

void Texture_Mesh_Corres::set_data(
	LG::PolygonMesh* mesh, 
	const std::vector<int>& faces_selected, 
	const QImage& image, 
	const QString& file_dir, 
	QGLViewer* m,
	const cv::Mat& mask,
	TexSynHandler* tex_syn_handler
	)
{
	if (mesh == NULL)
	{
		return;
	}
	this->m_mesh_ = mesh;

	this->m_face_ids_in_mesh_.clear();
	this->m_face_ids_in_mesh_ = faces_selected;
	
	this->m_image_file_ = file_dir;
	this->compute_faces_centers();
	this->compute_mesh_center();
	this->set_image(image);
	this->m_viewer_ = m;
	this->m_color_ = GLOBAL::random_color(1);
	this->m_mask_source_ = mask.clone();







	std::shared_ptr<AppearanceModel> am = tex_syn_handler->get_syn_app_mod();
	std::vector<cv::Mat> tar_feature_map;
	am->getD0Features(tar_feature_map);

	int w = tar_feature_map[0].cols;
	int h = tar_feature_map[0].rows;
	m_mask_target_ = cv::Mat(w, h, CV_32FC1, 1);;
	//cv::Mat mask_d0_all = cv::Mat(w, h, CV_32FC1, 1);;
	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			m_mask_target_.at<float>(i, j) = 0;
			//mask_d0_all.at<float>(i, j) = 0;
		}
	}
	const std::vector<bool>& face_ids = static_cast<Texture_Viewer*>(this->m_viewer_)->get_face_selected();
	LG::PolygonMesh pl;
	am->getBaseMesh(&pl);
	float resolution = am->getResolution();

	std::shared_ptr<ParaShape> tar_para_shape(new ParaShape);
	tar_para_shape->initWithExtPolygonMesh(&pl);

	std::vector<int>  select_faces;
	for (unsigned int i = 0; i < face_ids.size(); i++)
	{
		if (face_ids[i])
		{
			select_faces.push_back(i);
		}
	}

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{

			std::vector<float> uv_coor;
			uv_coor.push_back(1.0*i / resolution);
			uv_coor.push_back(1.0*(h - j -1) / resolution);

			std::vector<float> bary_coord;
			int f_id;
			std::vector<int> v_ids;
			bool b = ShapeUtility::findClosestUVFace(uv_coor, tar_para_shape.get(), bary_coord, f_id, v_ids);
			if (b)
			{
				//mask_d0_all.at<float>(i, j) = 1;
				if (std::find(select_faces.begin(), select_faces.end(), f_id) != select_faces.end())
				{
					m_mask_target_.at<float>(j, i) = 1;
				}

			}

		}
	}

// 	IplImage iplImg = IplImage(mask_d0);
// 	cvShowImage("mask_faces_selected", &iplImg);

// 	IplImage iplImg2 = IplImage(mask_d0_all);
// 	cvShowImage("mask_faces_all", &iplImg2);


	this->setStyleSheet("border-width: 3px;border-style: solid; border-color: rgb(" + QString::number(255 * m_color_.r()) + "," + QString::number(255 * m_color_.g())
		+ "," + QString::number(255 * m_color_.b()) + ")");

	this->creat_menu();
};


void Texture_Mesh_Corres::show_origin_image()
{
	this->set_image(this->m_origin_image_);
	this->m_show_mode_ = 0;
};
void Texture_Mesh_Corres::show_mesh_image()
{
	this->set_image(this->m_mesh_image_);
	this->m_show_mode_ = 1;
};
void Texture_Mesh_Corres::show_masked_image()
{
	this->set_image(this->m_masked_image_);
	this->m_show_mode_ = 2;
};
void Texture_Mesh_Corres::set_show_mode(int m)
{
	if (m == 0)
	{
		show_origin_image();
	}
	else if (m == 1)
	{
		show_mesh_image();
	}
	else if (m == 2)
	{
		show_masked_image();
	}
};

void Texture_Mesh_Corres::set_origin_image(const QImage& m)
{
	this->m_origin_image_ = m;
};
void Texture_Mesh_Corres::set_mesh_image(const QImage& m)
{
	this->m_mesh_image_ = m;
};
void Texture_Mesh_Corres::set_masked_image(const QImage& m)
{
	this->m_masked_image_ = m;
};

QString Texture_Mesh_Corres::file_path()
{
	return this->m_image_file_;
};

cv::Mat	Texture_Mesh_Corres::get_mask_target()
{
	return this->m_mask_target_;
};
cv::Mat	Texture_Mesh_Corres::get_mask_source()
{
	return this->m_mask_source_;
};
void Texture_Mesh_Corres::compute_mesh_center()
{
	if (m_mesh_ == NULL)
	{
		return;
	}

	LG::Vec3 v_total(0, 0, 0);
	int num = 0;
	for (unsigned int i = 0; i < m_face_centers_.size(); i++)
	{
		LG::Vec3 p = m_face_centers_[i];
		v_total = v_total + p;
		num++;
	}
	v_total = v_total / num;
	this->m_mesh_center_ = v_total;
};
void Texture_Mesh_Corres::moveEvent(QMoveEvent * event)
{
	QLabel::moveEvent(event);
};
void Texture_Mesh_Corres::mouseMoveEvent(QMouseEvent * e)
{
	if (this->m_left_button_)
	{
		QPoint p = e->globalPos();
		QPoint p_step = p - this->m_p_previous_;
		this->setGeometry(this->geometry().x() + p_step.x(), this->geometry().y() + p_step.y(), this->width(), this->height());
		this->m_viewer_->updateGL();
		this->m_p_previous_ = p;
	}
};
void Texture_Mesh_Corres::mousePressEvent(QMouseEvent * e)
{
	if (e->button() == Qt::LeftButton)
	{
		m_p_previous_ = e->globalPos();
		m_left_button_ = true;
		m_focus_on_ = true;
		if (this->m_viewer_)
		{
			this->m_viewer_->updateGL();
		}

		return;
	}

	if (m_label_full_image_ == NULL)
	{
		m_label_full_image_ = new QLabel(NULL);
		m_label_full_image_->setWindowFlags(Qt::SubWindow);
		QRect clientRect = QApplication::desktop()->screenGeometry();
		QPoint center = clientRect.center();

		int num_width = 600;
		int num_height = 600;

		float scale1 = this->m_origin_image_.width() / num_width;
		float scale2 = this->m_origin_image_.height() / num_height;

		float scale = scale1 > scale2?scale1:scale2;
		num_width = this->m_origin_image_.width() / scale;
		num_height = this->m_origin_image_.height() / scale;

		m_label_full_image_->resize(num_width, num_height);
		m_label_full_image_->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		m_label_full_image_->setPixmap(QPixmap::fromImage(this->m_origin_image_).scaled(m_label_full_image_->width(), m_label_full_image_->height(), Qt::KeepAspectRatio));
		m_label_full_image_->setGeometry(center.x() - m_label_full_image_->width() / 2, center.y() - m_label_full_image_->height() / 2, m_label_full_image_->width(), m_label_full_image_->height());
	}


	m_label_full_image_->show();
	//this->timer_id = this->startTimer(1000);
};
void Texture_Mesh_Corres::mouseReleaseEvent(QMouseEvent * e)
{
	if (e->button() == Qt::LeftButton)
	{
		m_focus_on_ = false;
		m_left_button_ = false;
		QRect r(0, 0, this->width(), this->height());

		if (r.contains(e->pos()) && this->m_menu_ != NULL)
		{
			this->m_menu_->exec(e->globalPos());
		}
		if (this->m_viewer_)
		{
			this->m_viewer_->updateGL();
		}
		
		return;
	}
	if (m_label_full_image_ != NULL)
	{
		m_label_full_image_->hide();
	}
	
// 	this->resize(this->m_width_icon_, this->m_height_icon_);
// 	this->setPixmap(QPixmap::fromImage(this->m_origin_image_).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
	//killTimer(this->timer_id);
};
void Texture_Mesh_Corres::draw_points()
{
	if (this->m_mesh_ == NULL)
	{
		return;
	}
	if (m_focus_on_)
	{
		glColor3f(1.0, 1.0, 0);
	}
	else
		glColor3fv(this->m_color_.data());
	glPointSize(4);
	glBegin(GL_POINTS);
	for (int f_id = 0; f_id < m_face_centers_.size(); f_id++)
	{
	 	glVertex3f(m_face_centers_[f_id].x(), m_face_centers_[f_id].y(), m_face_centers_[f_id].z());
	}
	glEnd();

};

void Texture_Mesh_Corres::draw_line()
{
	if (m_mesh_ == NULL)
	{
		return;
	}

	if (!this->isVisible())
	{
		return;
	}


	qglviewer::Vec c_tmp(m_mesh_center_.x(), m_mesh_center_.y(), m_mesh_center_.z());
	qglviewer::Vec vec_mesh_center = this->m_viewer_->camera()->projectedCoordinatesOf(c_tmp);

	QPoint this_center(this->geometry().x() + this->width() / 2, this->geometry().y() + this->height() / 2);

// 	std::cout << this_center.x() << "**" << this_center.y()<< "\n";
// 	std::cout << vec_mesh_center.x << "**" << vec_mesh_center.y << "\n";

	m_viewer_->startScreenCoordinatesSystem();
	glColor3fv(this->m_color_.data());
	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex2i(this_center.x(), this_center.y());
	glVertex2i(vec_mesh_center.x, vec_mesh_center.y);
	glEnd();
	m_viewer_->stopScreenCoordinatesSystem();
};
void Texture_Mesh_Corres::compute_faces_centers()
{
	if (m_mesh_ == NULL)
	{
		return;
	}

	m_face_centers_.resize(m_face_ids_in_mesh_.size());
	for (unsigned int i = 0; i < this->m_face_ids_in_mesh_.size(); i++)
	{
		LG::Vec3 v_total(0, 0, 0);
		int f_id = this->m_face_ids_in_mesh_[i];
		int num = 0;
		for (auto vfc : m_mesh_->vertices(LG::PolygonMesh::Face(f_id)))
		{
			LG::Vec3 p = m_mesh_->position(vfc);
			v_total = v_total + p;
			num++;
		}
		v_total = v_total / num;
		m_face_centers_[i] = v_total;
	}
};