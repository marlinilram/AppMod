#include "mini_texture.h"
#include <QApplication>
#include <QDesktopWidget>
#include <highgui.h>
#include <QPainter>
#include <QMessageBox>
#include "viewer_selector.h"
MiniTexture::MiniTexture(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{

	this->m_image_file_.clear();
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0;// 0->origin. 1->mesh;

	m_r_previous_ = -1;
	m_g_previous_ = -1;
	m_b_previous_ = -1;

	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;

	reset_window_for_stroke();
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
	this->m_origin_image_d0_ = QImage(s);
	
};
void MiniTexture::clear()
{
	QLabel::clear();
	this->m_mesh_image_d0_ = QImage();
	this->m_origin_image_d0_ = QImage();
	this->m_image_file_ = "";
};
void MiniTexture::set_image(const QImage& img)
{
	this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
};

bool MiniTexture::load_texture_d0()
{
	if (this->m_image_file_.isEmpty())
	{
		return false;
	}
	else
	{
		if (true)
		{
			QString file_path = this->m_image_file_;
			std::string std_file_path = file_path.toStdString().substr(0, file_path.toStdString().find_last_of('/'));

			std::vector<cv::Mat> detail_maps;
			MiniTexture::readMaps(std_file_path + "/" + "app_model.xml", detail_maps, "d1Detail");

			if (detail_maps.size() < 1)
			{
				return false;
			}

			QImage im(detail_maps[0].rows, detail_maps[0].cols, QImage::Format_RGB32);
			for (int i = 0; i < im.width(); i++)
			{
				for (int j = 0; j < im.height(); j++)
				{
					im.setPixel(i, j, qRgb(detail_maps[0].at<float>(j, i) * 255, detail_maps[1].at<float>(j, i) * 255, detail_maps[2].at<float>(j, i) * 255));
					if (detail_maps[0].at<float>(j, i) < 0)
					{
						im.setPixel(i, j, qRgb(0, 0, 0));
					}
				}
			}

			this->setFixedSize(im.width(), im.height());
			m_mesh_image_d0_ = im;
			MiniTexture::generate_mask(m_mesh_image_d0_, m_mask_image_d0_);
			m_r_previous_ = -1;
			m_g_previous_ = -1;
			m_b_previous_ = -1;

			m_mask_d0_.setTo(cv::Scalar(0));
			m_mask_tmp_d0_.setTo(cv::Scalar(0));
		}
	}
	return true;
};
bool MiniTexture::load_texture_d1()
{
	if (this->m_image_file_.isEmpty())
	{
		return false;
	}
	else
	{
		if (true)
		{
			this->m_origin_image_d1_ = this->m_origin_image_d0_;
			QString file_path = this->m_image_file_;
			std::string std_file_path = file_path.toStdString().substr(0, file_path.toStdString().find_last_of('/'));

			//d1Feature
			std::vector<cv::Mat> detail_maps;
			MiniTexture::readMaps(std_file_path + "/" + "app_model.xml", detail_maps, "d1Detail");

// 			IplImage iplImg = IplImage(detail_maps[0]);
// 			cvShowImage("load_texture_d1", &iplImg);

			if (detail_maps.size() < 1)
			{
				return false;
			}
			QImage im(detail_maps[0].rows, detail_maps[0].cols, QImage::Format_RGB32);
			for (int i = 0; i < im.width(); i++)
			{
				for (int j = 0; j < im.height(); j++)
				{
					im.setPixel(i, j, qRgb(detail_maps[0].at<float>(j, i) * 255, detail_maps[1].at<float>(j, i) * 255, detail_maps[2].at<float>(j, i) * 255));
					if (detail_maps[0].at<float>(j, i) < 0)
					{
						im.setPixel(i, j, qRgb(0, 0, 0));
					}
				}
			}
			this->setFixedSize(im.width(), im.height());
			m_mesh_image_d1_ = im;
			MiniTexture::generate_mask(m_mesh_image_d1_, m_mask_image_d1_);
			m_mask_d1_.setTo(cv::Scalar(0));
			m_mask_tmp_d1_.setTo(cv::Scalar(0));
		}
	}
	return true;
};

bool MiniTexture::load_texture()
{
	if (!this->m_mask_image_d0_.isNull())
	{
		return false;
	}

	 bool	b = this->load_texture_d0();
	 if (!b)
	 {
		 return b;
	 }
			b = this->load_texture_d1();
	 if (!b)
	 {
		 return b;
	 }
	m_r_previous_ = -1;
	m_g_previous_ = -1;
	m_b_previous_ = -1;
	return true;
};


void MiniTexture::generate_mask(const QImage& im, QImage& mask)
{
	int w = im.width();
	int h = im.height();

	mask = QImage(w, h, QImage::Format_RGB32);
	for (int i = 0; i < im.width(); i++)
	{
		for (int j = 0; j < im.height(); j++)
		{
			mask.setPixel(i, j, qRgb(0, 0, 0));
		}
	}



	std::vector<std::vector<bool>> checked(w, std::vector<bool>(h, false));
	int num_texture = 0;
	QRgb c = qRgb(rand() % 256, rand() % 256, rand() % 256);
	for (int i = 0; i < im.width(); i++)
	{
		for (int j = 0; j < im.height(); j++)
		{
			if (checked[i][j])
			{
				continue;;
			}
			if (im.pixel(i, j) != qRgb(0, 0, 0))
			{
				std::vector<int> ws;
				ws.push_back(i);
				std::vector<int> hs;
				hs.push_back(j);
				c = qRgb(rand() % 256, rand() % 256, rand() % 256);
				num_texture++;
				for (unsigned int k = 0; k < ws.size(); k++)
				{
					int x = ws[k];
					int y = hs[k];

					if (im.pixel(x, y) == qRgb(0, 0, 0))
					{
						checked[x][y] = true;
						continue;
					}
					mask.setPixel(x, y, c);
					checked[x][y] = true;


						if (x - 1 >= 0 && y - 1 >= 0 && !checked[x-1][y-1])
						{
							ws.push_back(x-1);
							hs.push_back(y-1);
							checked[x -1][y - 1] = true;
						}
						if (y - 1 >= 0 && !checked[x][y - 1])
						{
							ws.push_back(x);
							hs.push_back(y - 1);
							checked[x][y -1] = true;
						}
						if (x + 1 < w && y - 1 >= 0 && !checked[x + 1][y - 1])
						{
							ws.push_back(x + 1);
							hs.push_back(y - 1);
							checked[x+1][y - 1] = true;
						}
						if (x - 1 >= 0 && !checked[x - 1][y])
						{
							ws.push_back(x - 1);
							hs.push_back(y );
							checked[x-1][y] = true;
						}
						if (x + 1 < w && !checked[x + 1][y])
						{
							ws.push_back(x + 1);
							hs.push_back(y );
							checked[x+1][y] = true;
						}
						if (x - 1 >= 0 && y + 1 < h && !checked[x - 1][y + 1])
						{
							ws.push_back(x - 1);
							hs.push_back(y + 1);
							checked[x-1][y+1] = true;
						}
						if (y + 1 < h && !checked[x][y+1])
						{
							ws.push_back(x);
							hs.push_back(y+1);
							checked[x][y+1] = true;
						}
						if (x + 1 < w && y + 1 < h && !checked[x + 1][y + 1])
						{
							ws.push_back(x + 1);
							hs.push_back(y + 1);
							checked[x+1][y+1] = true;
						}
					
				}
				int a = 0;
			}
			else
			{
				checked[i][j] = true;
			}
		}
	}

	std::cout << "num_texture :" << num_texture << "\n";

};

void MiniTexture::show_origin_image_d0()
{
	this->set_image(this->m_origin_image_d0_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0;
	this->setWindowTitle("Mask for D0.........");
};
void MiniTexture::show_mesh_image_d0()
{
	this->set_image(this->m_mesh_image_d0_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::PARA_MESH_D0;
	this->setWindowTitle("Mask for D0.........");
};
void MiniTexture::show_stroke_image_d0()
{
	this->set_image(this->m_for_stroke_image_d0_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::MASK_STROKE_D0;
	this->setWindowTitle("Mask for D0.........");
};

void MiniTexture::show_origin_image_d1()
{
	this->set_image(this->m_origin_image_d0_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1;
	this->setWindowTitle("Mask for D1.........");
};
void MiniTexture::show_mesh_image_d1()
{
	this->set_image(this->m_mesh_image_d1_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::PARA_MESH_D1;
	this->setWindowTitle("Mask for D1.........");
};
void MiniTexture::show_stroke_image_d1()
{
	this->set_image(this->m_for_stroke_image_d1_);
	m_shown_mode_ = MINITEXTURE_SHOW_MODE::MASK_STROKE_D1;
	this->setWindowTitle("Mask for D1.........");
};




void MiniTexture::set_center_pos(int x, int y)
{
	this->setGeometry(x - this->width() / 2, y- this->height()/2, this->width(), this->height());
};


MINITEXTURE_SHOW_MODE MiniTexture::get_show_mode()
{
	return this->m_shown_mode_;
};
void MiniTexture::set_show_mode(MINITEXTURE_SHOW_MODE m)
{
	
	if (m == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0)
	{
		show_origin_image_d0();
	}
	else if (m == MINITEXTURE_SHOW_MODE::PARA_MESH_D0)
	{
		show_mesh_image_d0();;
	}
	else if (m == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		show_stroke_image_d0();
	}
	else if (m == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1)
	{
		show_origin_image_d1();
	}
	else if (m == MINITEXTURE_SHOW_MODE::PARA_MESH_D1)
	{
		show_mesh_image_d1();
	}
	else if (m == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		show_stroke_image_d1();
	}
	else
	{
		return;
	}
	this->m_shown_mode_ = m;

};
void MiniTexture::mouseDoubleClickEvent(QMouseEvent * event)
{
	if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0)
	{
		double_Click_On_Origin_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D0)
	{
		double_Click_On_PARA_MESH_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		double_Click_On_MASK_STROKE_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1)
	{
		double_Click_On_Origin_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D1)
	{
		double_Click_On_PARA_MESH_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		double_Click_On_MASK_STROKE_D1(event);
	}
	else
	{
		QLabel::mouseDoubleClickEvent(event);
	}

};

void MiniTexture::mouseMoveEvent(QMouseEvent * event)
{
	if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0)
	{
		move_On_Origin_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D0)
	{
		move_On_PARA_MESH_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		move_On_MASK_STROKE_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1)
	{
		move_On_Origin_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D1)
	{
		move_On_PARA_MESH_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		move_On_MASK_STROKE_D1(event);
	}
	else
	{
		QLabel::mouseMoveEvent(event);
	}
};

void MiniTexture::mousePressEvent(QMouseEvent * event)
{
	if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0)
	{
		press_On_Origin_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D0)
	{
		press_On_PARA_MESH_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		press_On_MASK_STROKE_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1)
	{
		press_On_Origin_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D1)
	{
		press_On_PARA_MESH_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		press_On_MASK_STROKE_D1(event);
	}
	else
	{
		QLabel::mousePressEvent(event);
	}
};
void MiniTexture::mouseReleaseEvent(QMouseEvent *event)
{
	if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0)
	{
		release_On_Origin_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D0)
	{
		release_On_PARA_MESH_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		release_On_MASK_STROKE_D0(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1)
	{
		release_On_Origin_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::PARA_MESH_D1)
	{
		release_On_PARA_MESH_D1(event);
	}
	else if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		release_On_MASK_STROKE_D1(event);
	}
	else
	{
		QLabel::mouseReleaseEvent(event);
	}

	
};

void MiniTexture::readMaps(std::string xml_file, std::vector<cv::Mat>& maps, std::string map_name)
{
	cv::FileStorage fs(xml_file, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Cannot open file: " << xml_file << std::endl;
	}

	cv::FileNode fnode = fs[map_name];
	maps.clear();

	cv::FileNode maps_node = fnode["maps"];
	cv::FileNodeIterator it = maps_node.begin(), it_end = maps_node.end();
	int dim = (int)fnode["Dimension"];
	maps.resize(dim);
	for (int i = 0; it != it_end; ++it, ++i)
	{
		(*it)["map"] >> maps[i];
	}
}
void MiniTexture::set_mask_d0(cv::Mat& m)
{
	this->m_mask_d0_ = m;
};
cv::Mat MiniTexture::get_mask_d0()
{
	cv::Mat dis;
	cv::transpose(this->m_mask_d0_, dis);
	return dis;
};


void MiniTexture::set_mask_d1(cv::Mat& m)
{
	this->m_mask_d1_ = m;
};
cv::Mat MiniTexture::get_mask_d1()
{
	cv::Mat dis;
	cv::transpose(this->m_mask_d1_, dis);
	return dis;
};


void MiniTexture::reset_window_for_stroke()
{
	m_x_min_for_mask_ = INT_MAX;
	m_y_min_for_mask_ = INT_MAX;
	m_x_max_for_mask_ = INT_MIN;
	m_y_max_for_mask_ = INT_MIN;
};
void MiniTexture::update_window_for_stroke(int x, int y)
{
	if (x > m_x_max_for_mask_)
	{
		m_x_max_for_mask_ = x;
	}
	if (x < m_x_min_for_mask_)
	{
		m_x_min_for_mask_ = x;
	}
	if (y > m_y_max_for_mask_)
	{
		m_y_max_for_mask_ = y;
	}
	if (y < m_y_min_for_mask_)
	{
		m_y_min_for_mask_ = y;
	}
};
void MiniTexture::paintEvent(QPaintEvent * event)
{
	QLabel::paintEvent(event);
	if (this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0 || this->m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		QPainter painter(this);
		QRgb color = qRgb(255, 0, 0);
		painter.setPen(color);
		if (m_stroke_points_.size() <3)
		{
			return;
		}
		for (int i = 0; i < m_stroke_points_.size(); i++)
		{
			int x = m_stroke_points_[i].x(); int y = m_stroke_points_[i].y();
			int x1 = m_stroke_points_[(i + 1) % m_stroke_points_.size()].x(); int y1 = m_stroke_points_[(i + 1) % m_stroke_points_.size()].y();
			painter.drawLine(x, y, x1, y1);
		}
	}	
};
QImage MiniTexture::merge_image(const QImage& im, const cv::Mat& mask, QRgb rgb)
{
	QImage image_return = im;
	float origin_weight = 0.8;
	float mask_weight = 1.0 - origin_weight;

	for (int i = 0; i < im.width(); i++)
	{
		for (int j = 0; j < im.height(); j++)
		{
			if (mask.at<float>(i,j) > 0)
			{
				QRgb rgb_tmp = image_return.pixel(i, j);
				int r = qRed(rgb_tmp)* origin_weight + qRed(rgb) * mask_weight;
				int g = qGreen(rgb_tmp)* origin_weight + qGreen(rgb) * mask_weight;
				int b = qBlue(rgb_tmp)* origin_weight + qBlue(rgb) * mask_weight;
				image_return.setPixel(i, j, qRgb(r, g, b));
			}
		}
	}
	return image_return;
};

void MiniTexture::double_Click_On_Origin_D0(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);
	if (m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D0 && event->button() == Qt::LeftButton)
	{
		if (!this->m_mesh_image_d0_.isNull())
		{
			this->show_mesh_image_d0();
			m_mask_d0_.setTo(cv::Scalar(0));
			m_mask_tmp_d0_.setTo(cv::Scalar(0));
		}

	}
};
void MiniTexture::double_Click_On_PARA_MESH_D0(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);
	this->reset_window_for_stroke();
	if (event->button() == Qt::LeftButton)
	{
		QPoint p = event->pos();
		QRgb rgb = this->m_mask_image_d0_.pixel(p);

		if (rgb == qRgb(0, 0, 0))
		{
			return;
		}
		std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
		m_for_stroke_image_d0_ = QImage(this->m_mask_image_d0_.width(), this->m_mask_image_d0_.height(), QImage::Format_RGB32);
		m_mask_d0_ = cv::Mat(m_for_stroke_image_d0_.width(), m_for_stroke_image_d0_.height(), CV_32FC1, 1);
		m_mask_tmp_d0_ = cv::Mat(m_for_stroke_image_d0_.width(), m_for_stroke_image_d0_.height(), CV_32FC1, 1);
		for (int i = 0; i < m_for_stroke_image_d0_.width(); i++)
		{
			for (int j = 0; j < m_for_stroke_image_d0_.height(); j++)
			{
				m_for_stroke_image_d0_.setPixel(i, j, qRgb(0, 0, 0));
				m_mask_d0_.at<float>(i, j) = 0;
				m_mask_tmp_d0_.at<float>(i, j) = 0;
				if (rgb == this->m_mask_image_d0_.pixel(i, j))
				{
					m_for_stroke_image_d0_.setPixel(i, j, this->m_mesh_image_d0_.pixel(i, j));
					m_mask_d0_.at<float>(i, j) = 1;
					m_mask_tmp_d0_.at<float>(i, j) = 1;
					//this->update_window_for_stroke(i,j);
				}
			}
		}
		this->show_stroke_image_d0();
		m_mask_d0_.setTo(cv::Scalar(0));
	}
	else if (event->button() == Qt::RightButton)
	{
		this->show_origin_image_d0();
		m_mask_d0_.setTo(cv::Scalar(0));
		m_mask_tmp_d0_.setTo(cv::Scalar(0));
	}
};
void MiniTexture::double_Click_On_MASK_STROKE_D0(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);
	if (m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D0)
	{
		if (event->button() == Qt::LeftButton)
		{
			cv::Mat m_dis = this->get_mask_d0();
			IplImage iplImg = IplImage(m_dis);
			cvShowImage("mask", &iplImg);

			if (QMessageBox::question(this, "Submit??", "Are you sure use these regions you selected?") == QMessageBox::Yes)
			{
				emit mask_selected_d0();
				this->show_origin_image_d1();
			}
		}
		else if (event->button() == Qt::RightButton)
		{
			this->show_mesh_image_d0();
			m_mask_d0_.setTo(cv::Scalar(0));
			m_mask_tmp_d0_.setTo(cv::Scalar(0));
		}
	}
};
void MiniTexture::double_Click_On_Origin_D1(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);

	if (m_shown_mode_ == MINITEXTURE_SHOW_MODE::ORIGIN_FOR_D1 && event->button() == Qt::LeftButton)
	{
		if (!this->m_mesh_image_d1_.isNull())
		{
			this->show_mesh_image_d1();
			m_mask_d1_.setTo(cv::Scalar(0));
			m_mask_tmp_d1_.setTo(cv::Scalar(0));
		}

	}
};
void MiniTexture::double_Click_On_PARA_MESH_D1(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);

	this->reset_window_for_stroke();
	if (event->button() == Qt::LeftButton)
	{
		QPoint p = event->pos();
		QRgb rgb = this->m_mask_image_d1_.pixel(p);

		if (rgb == qRgb(0, 0, 0))
		{
			return;
		}
		std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
		m_for_stroke_image_d1_ = QImage(this->m_mask_image_d1_.width(), this->m_mask_image_d1_.height(), QImage::Format_RGB32);
		m_mask_d1_ = cv::Mat(m_for_stroke_image_d1_.width(), m_for_stroke_image_d1_.height(), CV_32FC1, 1);
		m_mask_tmp_d1_ = cv::Mat(m_for_stroke_image_d1_.width(), m_for_stroke_image_d1_.height(), CV_32FC1, 1);
		for (int i = 0; i < m_for_stroke_image_d1_.width(); i++)
		{
			for (int j = 0; j < m_for_stroke_image_d1_.height(); j++)
			{
				m_for_stroke_image_d1_.setPixel(i, j, qRgb(0, 0, 0));
				m_mask_d1_.at<float>(i, j) = 0;
				m_mask_tmp_d1_.at<float>(i, j) = 0;
				if (rgb == this->m_mask_image_d1_.pixel(i, j))
				{
					m_for_stroke_image_d1_.setPixel(i, j, this->m_mesh_image_d1_.pixel(i, j));
					m_mask_d1_.at<float>(i, j) = 1;
					m_mask_tmp_d1_.at<float>(i, j) = 1;
					//this->update_window_for_stroke(i,j);
				}
			}
		}
		this->show_stroke_image_d1();
		m_mask_d1_.setTo(cv::Scalar(0));
	}
	else if (event->button() == Qt::RightButton)
	{
		this->show_origin_image_d1();
		m_mask_d1_.setTo(cv::Scalar(0));
		m_mask_tmp_d1_.setTo(cv::Scalar(0));
	}
};
void MiniTexture::double_Click_On_MASK_STROKE_D1(QMouseEvent * event)
{
	QLabel::mouseDoubleClickEvent(event);
	if (m_shown_mode_ == MINITEXTURE_SHOW_MODE::MASK_STROKE_D1)
	{
		if (event->button() == Qt::LeftButton)
		{
			cv::Mat m_dis = this->get_mask_d1();
			IplImage iplImg = IplImage(m_dis);
			cvShowImage("mask", &iplImg);

			if (QMessageBox::question(this, "Submit??", "Are you sure use these regions you selected?") == QMessageBox::Yes)
			{
				emit mask_selected_d1();
			}
		}
		else if (event->button() == Qt::RightButton)
		{
			this->show_mesh_image_d1();
			m_mask_d1_.setTo(cv::Scalar(0));
			m_mask_tmp_d1_.setTo(cv::Scalar(0));
		}
	}
};

void MiniTexture::press_On_Origin_D0(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};
void MiniTexture::press_On_PARA_MESH_D0(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};
void MiniTexture::press_On_MASK_STROKE_D0(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};
void MiniTexture::press_On_Origin_D1(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};
void MiniTexture::press_On_PARA_MESH_D1(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};
void MiniTexture::press_On_MASK_STROKE_D1(QMouseEvent * event)
{
	QLabel::mousePressEvent(event);
	if (event->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}

	if (event->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = true;
	}
};


void MiniTexture::release_On_Origin_D0(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
};
void MiniTexture::release_On_PARA_MESH_D0(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
	m_r_previous_ = -1;
	m_g_previous_ = -1;
	m_b_previous_ = -1;
	this->show_mesh_image_d0();
	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;
};
void MiniTexture::release_On_MASK_STROKE_D0(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
	if (this->m_stroke_points_.size() < 3)
	{
		return;
	}
	bool changed = false;
	if (event->button() == Qt::LeftButton && this->m_left_button_down_)
	{
		for (int i = 0; i < this->m_mask_d0_.cols; i++)
		{
			for (int j = 0; j < this->m_mask_d0_.rows; j++)
			{
				if (this->m_mask_tmp_d0_.at<float>(i, j) > 0)
				{
					QPoint p(i, j);
					if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
					{
						this->m_mask_d0_.at<float>(i, j) = 1;
						changed = true;
					}
				}
			}
		}
	}
	else if (event->button() == Qt::RightButton && this->m_right_button_down_)
	{
		for (int i = 0; i < this->m_mask_d0_.cols; i++)
		{
			for (int j = 0; j < this->m_mask_d0_.rows; j++)
			{
				if (this->m_mask_tmp_d0_.at<float>(i, j) > 0)
				{
					QPoint p(i, j);
					if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
					{
						this->m_mask_d0_.at<float>(i, j) = 0;
						changed = true;
					}
				}

			}
		}
	}
	this->m_stroke_points_.clear();
	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;
	if (changed)
	{
		m_masked_image_d0_ = MiniTexture::merge_image(this->m_for_stroke_image_d0_, this->m_mask_d0_, qRgb(255, 0, 0));
		this->set_image(m_masked_image_d0_);

	}
	this->update();
};
void MiniTexture::release_On_Origin_D1(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
};
void MiniTexture::release_On_PARA_MESH_D1(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
	m_r_previous_ = -1;
	m_g_previous_ = -1;
	m_b_previous_ = -1;
	this->show_mesh_image_d1();
	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;
};
void MiniTexture::release_On_MASK_STROKE_D1(QMouseEvent * event)
{
	QLabel::mouseReleaseEvent(event);
	if (this->m_stroke_points_.size() < 3)
	{
		return;
	}
	bool changed = false;
	if (event->button() == Qt::LeftButton && this->m_left_button_down_)
	{
		for (int i = 0; i < this->m_mask_d1_.cols; i++)
		{
			for (int j = 0; j < this->m_mask_d1_.rows; j++)
			{
				if (this->m_mask_tmp_d1_.at<float>(i, j) > 0)
				{
					QPoint p(i, j);
					if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
					{
						this->m_mask_d1_.at<float>(i, j) = 1;
						changed = true;
					}
				}
			}
		}
	}
	else if (event->button() == Qt::RightButton && this->m_right_button_down_)
	{
		for (int i = 0; i < this->m_mask_d1_.cols; i++)
		{
			for (int j = 0; j < this->m_mask_d1_.rows; j++)
			{
				if (this->m_mask_tmp_d1_.at<float>(i, j) > 0)
				{
					QPoint p(i, j);
					if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
					{
						this->m_mask_d1_.at<float>(i, j) = 0;
						changed = true;
					}
				}

			}
		}
	}
	this->m_stroke_points_.clear();
	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;
	if (changed)
	{
		QImage im = MiniTexture::merge_image(this->m_for_stroke_image_d1_, this->m_mask_d1_, qRgb(255, 0, 0));
		this->set_image(im);
	}
	this->update();
};

void MiniTexture::move_On_Origin_D0(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);
};
void MiniTexture::move_On_PARA_MESH_D0(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);

	if (this->m_right_button_down_)
	{
		QPoint p = event->pos();

		if (p.x() < 0 || p.x() >= this->width() || p.y() < 0 || p.y() >= this->height())
		{
			return;
		}

		QRgb rgb = this->m_mask_image_d0_.pixel(p);

		if (rgb == qRgb(m_r_previous_, m_g_previous_, m_b_previous_))
		{
			return;
		}

		if (rgb == qRgb(0, 0, 0))
		{
			this->set_image(this->m_mesh_image_d0_);
			return;
		}

		float origin_weight = 0.8;
		float mask_weight = 1.0 - origin_weight;
		QImage img(m_mask_image_d0_.width(), this->m_mask_image_d0_.height(), QImage::Format_RGB32);
		for (int i = 0; i < img.width(); i++)
		{
			for (int j = 0; j < img.height(); j++)
			{
				//img.setPixel(i, j, qRgb(0, 0, 0));

				if (rgb == this->m_mask_image_d0_.pixel(i, j))
				{
					QRgb rgb_tmp = this->m_mesh_image_d0_.pixel(i, j);
					int r = qRed(rgb_tmp)* origin_weight + 255 * mask_weight;
					int g = qGreen(rgb_tmp)* origin_weight + 0 * mask_weight;
					int b = qBlue(rgb_tmp)* origin_weight + 0 * mask_weight;
					img.setPixel(i, j, qRgb(r, g, b));
				}
				else
				{
					QRgb rgb_tmp = this->m_mesh_image_d0_.pixel(i, j);
					img.setPixel(i, j, rgb_tmp);
				}
			}
		}

		this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
		m_r_previous_ = qRed(rgb);
		m_g_previous_ = qGreen(rgb);
		m_b_previous_ = qBlue(rgb);
	}
};
void MiniTexture::move_On_MASK_STROKE_D0(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);
	if ((this->m_left_button_down_ || this->m_right_button_down_))
	{
		this->m_stroke_points_.push_back(event->pos());
		this->update();
	}
};
void MiniTexture::move_On_Origin_D1(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);
};
void MiniTexture::move_On_PARA_MESH_D1(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);
	if (this->m_right_button_down_)
	{
		QPoint p = event->pos();

		if (p.x() < 0 || p.x() >= this->width() || p.y() < 0 || p.y() >= this->height())
		{
			return;
		}

		QRgb rgb = this->m_mask_image_d1_.pixel(p);

		if (rgb == qRgb(m_r_previous_, m_g_previous_, m_b_previous_))
		{
			return;
		}

		if (rgb == qRgb(0, 0, 0))
		{
			this->set_image(this->m_mesh_image_d1_);
			return;
		}

		float origin_weight = 0.8;
		float mask_weight = 1.0 - origin_weight;
		QImage img(m_mask_image_d1_.width(), this->m_mask_image_d1_.height(), QImage::Format_RGB32);
		for (int i = 0; i < img.width(); i++)
		{
			for (int j = 0; j < img.height(); j++)
			{
				//img.setPixel(i, j, qRgb(0, 0, 0));

				if (rgb == this->m_mask_image_d1_.pixel(i, j))
				{
					QRgb rgb_tmp = this->m_mesh_image_d1_.pixel(i, j);
					int r = qRed(rgb_tmp)* origin_weight + 255 * mask_weight;
					int g = qGreen(rgb_tmp)* origin_weight + 0 * mask_weight;
					int b = qBlue(rgb_tmp)* origin_weight + 0 * mask_weight;
					img.setPixel(i, j, qRgb(r, g, b));
				}
				else
				{
					QRgb rgb_tmp = this->m_mesh_image_d1_.pixel(i, j);
					img.setPixel(i, j, rgb_tmp);
				}
			}
		}

		this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
		m_r_previous_ = qRed(rgb);
		m_g_previous_ = qGreen(rgb);
		m_b_previous_ = qBlue(rgb);
	}
};
void MiniTexture::move_On_MASK_STROKE_D1(QMouseEvent * event)
{
	QLabel::mouseMoveEvent(event);
	if ((this->m_left_button_down_ || this->m_right_button_down_))
	{
		this->m_stroke_points_.push_back(event->pos());
		this->update();
	}
};