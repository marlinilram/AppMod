#include "mini_texture.h"
#include <QApplication>
#include <QDesktopWidget>
#include <highgui.h>
#include <QPainter>
#include "viewer_selector.h"
MiniTexture::MiniTexture(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{

	this->m_image_file_.clear();
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	this->m_texture_ = NULL;
	m_shown_mode_ = 0;// 0->origin. 1->mesh;

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
	this->m_origin_image_ = QImage(s);
	
};
void MiniTexture::clear()
{
	QLabel::clear();
	this->m_mesh_image_ = QImage();
	this->m_origin_image_ = QImage();
	this->m_image_file_ = "";
};
void MiniTexture::set_image(const QImage& img)
{
	this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
};
void MiniTexture::load_texture()
{
	if (this->m_image_file_.isEmpty())
	{
		return;
	}
	else
	{
		if (true)
		{
			QString file_path = this->m_image_file_;
			std::string std_file_path = file_path.toStdString().substr(0, file_path.toStdString().find_last_of('/'));

			std::vector<cv::Mat> detail_maps;
			MiniTexture::readMaps(std_file_path + "/" + "app_model.xml", detail_maps, "d1Detail");

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
			m_mesh_image_ = im;
			MiniTexture::generate_mask(m_mesh_image_, m_mask_image_);
			m_r_previous_ = -1;
			m_g_previous_ = -1;
			m_b_previous_ = -1;

			m_mask_.setTo(cv::Scalar(0));
			m_mask_tmp_.setTo(cv::Scalar(0));
		}
	}
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

void MiniTexture::show_origin_image()
{
	this->set_image(this->m_origin_image_);
	m_shown_mode_ = 0;// 0->origin. 1->mesh; 2-> mask stroke;
};
void MiniTexture::show_mesh_image()
{
	//std::cout << this->m_mesh_image_.width() << " " << this->m_mesh_image_.height() << "\n";
	this->set_image(this->m_mesh_image_);
	m_shown_mode_ = 1;// 0->origin. 1->mesh; 2-> mask stroke;
};

void MiniTexture::set_center_pos(int x, int y)
{
	this->setGeometry(x - this->width() / 2, y- this->height()/2, this->width(), this->height());
};

void MiniTexture::show_stroke_image()
{
	this->set_image(this->m_for_stroke_image_);
	m_shown_mode_ = 2;// 0->origin. 1->mesh;2-> mask stroke;
};

void MiniTexture::mouseDoubleClickEvent(QMouseEvent * event)
{
	
	if (m_shown_mode_ == 1)
	{

		this->reset_window_for_stroke();
		if (event->button() == Qt::LeftButton)
		{
			QPoint p = event->pos();
			QRgb rgb = this->m_mask_image_.pixel(p);

			if (rgb == qRgb(0, 0, 0))
			{
				return;
			}
			std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
			m_for_stroke_image_ = QImage(this->m_mask_image_.width(), this->m_mask_image_.height(), QImage::Format_RGB32);
			m_mask_ = cv::Mat(m_for_stroke_image_.width(), m_for_stroke_image_.height(), CV_32FC1, 1);
			m_mask_tmp_ = cv::Mat(m_for_stroke_image_.width(), m_for_stroke_image_.height(), CV_32FC1, 1);
			for (int i = 0; i < m_for_stroke_image_.width(); i++)
			{
				for (int j = 0; j < m_for_stroke_image_.height(); j++)
				{
					m_for_stroke_image_.setPixel(i, j, qRgb(0, 0, 0));
					m_mask_.at<float>(i, j) = 0;
					m_mask_tmp_.at<float>(i, j) = 0;
					if (rgb == this->m_mask_image_.pixel(i, j))
					{
						m_for_stroke_image_.setPixel(i, j, this->m_mesh_image_.pixel(i, j));
						m_mask_.at<float>(i, j) = 1;
						m_mask_tmp_.at<float>(i, j) = 1;
						//this->update_window_for_stroke(i,j);
					}
				}
			}
			this->show_stroke_image();
			m_mask_.setTo(cv::Scalar(0));
		}
		else if (event->button() == Qt::RightButton)
		{
			this->show_origin_image();
			m_mask_.setTo(cv::Scalar(0));
			m_mask_tmp_.setTo(cv::Scalar(0));
		}

// 		std::cout << "Double click\n";
// 		cv::Mat m_dis; cv::transpose(m_mask_tmp_, m_dis);
// 		IplImage iplImg = IplImage(m_dis);
// 		cvShowImage("m_mask_tmp_", &iplImg);
	}
	else if (m_shown_mode_ == 2 )
	{
		if (event->button() == Qt::LeftButton)
		{
			cv::Mat m_dis; cv::transpose(m_mask_, m_dis);
			IplImage iplImg = IplImage(m_dis);

			cvShowImage("mask", &iplImg);

// 			m_mask_.setTo(cv::Scalar(0));
// 			m_mask_tmp_.setTo(cv::Scalar(0));


			emit mask_selected();
		}
		else if (event->button() == Qt::RightButton)
		{
			this->show_mesh_image();
			m_mask_.setTo(cv::Scalar(0));
			m_mask_tmp_.setTo(cv::Scalar(0));
		}
	}

	else
	{
		QLabel::mouseDoubleClickEvent(event);
	}
	
};

void MiniTexture::mouseMoveEvent(QMouseEvent * event)
{

	if (m_shown_mode_ == 1 && this->m_right_button_down_)
	{
		QPoint p = event->pos();

		if (p.x() < 0 || p.x() >= this->width() || p.y() < 0 || p.y() >= this->height())
		{
			return;
		}

		QRgb rgb = this->m_mask_image_.pixel(p);

		if (rgb == qRgb(m_r_previous_, m_g_previous_, m_b_previous_))
		{
			return;
		}

		if (rgb == qRgb(0, 0, 0))
		{
			this->set_image(this->m_mesh_image_);
			return;
		}

		float origin_weight = 0.8;
		float mask_weight = 1.0 - origin_weight;
		QImage img(m_mask_image_.width(), this->m_mask_image_.height(), QImage::Format_RGB32);
		for (int i = 0; i < img.width(); i++)
		{
			for (int j = 0; j < img.height(); j++)
			{
				//img.setPixel(i, j, qRgb(0, 0, 0));

				if (rgb == this->m_mask_image_.pixel(i, j))
				{
					QRgb rgb_tmp = this->m_mesh_image_.pixel(i, j);
					int r = qRed(rgb_tmp)* origin_weight + 255 * mask_weight;
					int g = qGreen(rgb_tmp)* origin_weight + 0 * mask_weight;
					int b = qBlue(rgb_tmp)* origin_weight + 0 * mask_weight;
					img.setPixel(i, j, qRgb(r, g, b));
				}
				else
				{
					QRgb rgb_tmp = this->m_mesh_image_.pixel(i, j);
					img.setPixel(i, j, rgb_tmp);
				}
			}
		}

		this->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
		m_r_previous_ = qRed(rgb);
		m_g_previous_ = qGreen(rgb);
		m_b_previous_ = qBlue(rgb);
	}
	if (m_shown_mode_ == 2 && (this->m_left_button_down_||this->m_right_button_down_))
	{
		this->m_stroke_points_.push_back(event->pos());
		this->update();

	}
	else
	{
		QLabel::mouseMoveEvent(event);
	}
		
};

void MiniTexture::mousePressEvent(QMouseEvent * event)
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

// 	if (event->button() == Qt::RightButton)
// 	{
// 
// 		QPoint p = event->pos();
// 		QRgb rgb = this->m_mask_image_.pixel(p);
// 
// 		if (rgb == qRgb(0, 0, 0))
// 		{
// 			return;
// 		}
// 
// 		QImage img(m_mask_image_.width(), this->m_mask_image_.height(), QImage::Format_RGB32);
// 		for (int i = 0; i < img.width(); i++)
// 		{
// 			for (int j = 0; j < img.height(); j++)
// 			{
// 				img.setPixel(i, j, qRgb(0, 0, 0));
// 				if (rgb == this->m_mask_image_.pixel(i, j))
// 				{
// 					QRgb rgb = this->m_mesh_image_.pixel(i, j);
// 					int r = qRed(rgb)* 0.8 + 255 * 0.2;
// 					int g = qGreen(rgb)* 0.8 + 0 * 0.2;
// 					int b = qBlue(rgb)* 0.8 + 0 * 0.2;
// 					img.setPixel(i, j, qRgb(r, g, b));
// 				}
// 			}
// 		}
// 
// 	}
};
void MiniTexture::mouseReleaseEvent(QMouseEvent *event)
{
	QLabel::mouseReleaseEvent(event);

	if (this->m_shown_mode_ == 1)
	{
		m_r_previous_ = -1;
		m_g_previous_ = -1;
		m_b_previous_ = -1;
		this->show_mesh_image();
		this->m_left_button_down_ = false;
		this->m_right_button_down_ = false;
	}
	else if (this->m_shown_mode_ == 2)
	{
		
		if (this->m_stroke_points_.size() < 3)
		{
			return;
		}
		bool changed = false;
		if (event->button() == Qt::LeftButton && this->m_left_button_down_)
		{
			for (int i = 0; i < this->m_mask_.cols; i++)
			{
				for (int j = 0; j < this->m_mask_.rows; j++)
				{
					if (this->m_mask_tmp_.at<float>(i,j) > 0)
					{
						QPoint p(i, j);
						if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
						{
							this->m_mask_.at<float>(i, j) = 1;
							changed = true;
						}
					}
					
				}
			}
		}
		else if (event->button() == Qt::RightButton && this->m_right_button_down_)
		{
			for (int i = 0; i < this->m_mask_.cols; i++)
			{
				for (int j = 0; j < this->m_mask_.rows; j++)
				{
					if (this->m_mask_tmp_.at<float>(i, j) > 0)
					{
						QPoint p(i, j);
						if (Viewer_Selector::is_point_in_polygon(p, this->m_stroke_points_))
						{
							this->m_mask_.at<float>(i, j) = 0;
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
			QImage im = MiniTexture::merge_image(this->m_for_stroke_image_, this->m_mask_, qRgb(255,0, 0));
			this->set_image(im);
		}
		this->update();
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
void MiniTexture::set_mask(cv::Mat& m)
{
	this->m_mask_ = m;
};
cv::Mat MiniTexture::get_mask()
{
	cv::Mat dis;
	cv::transpose(this->m_mask_, dis);
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
	if (this->m_shown_mode_ == 2)
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