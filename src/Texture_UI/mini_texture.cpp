#include "mini_texture.h"
#include <QApplication>
#include <QDesktopWidget>
#include <highgui.h>
MiniTexture::MiniTexture(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{

	this->m_image_file_.clear();
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	this->m_texture_ = NULL;
	this->m_selection_label_ = new QLabel(this);
	m_shown_mode_ = 0;// 0->origin. 1->mesh;
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
	m_shown_mode_ = 0;// 0->origin. 1->mesh;
};
void MiniTexture::show_mesh_image()
{
	this->set_image(this->m_mesh_image_);
	m_shown_mode_ = 1;// 0->origin. 1->mesh;
};

void MiniTexture::set_center_pos(int x, int y)
{
	this->setGeometry(x - this->width() / 2, y- this->height()/2, this->width(), this->height());
};

void MiniTexture::mouseDoubleClickEvent(QMouseEvent * event)
{
	if (this->m_shown_mode_ != 1)
	{
		return;
	}

	if (event ->button() == Qt::LeftButton)
	{
		QPoint p = event->pos();
		QRgb rgb = this->m_mask_image_.pixel(p);

		if (rgb == qRgb(0, 0, 0))
		{
			return;
		}

		std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
		QImage im(this->m_mask_image_.width(), this->m_mask_image_.height(), QImage::Format_RGB32);
		m_mask_ = cv::Mat(im.width(), im.height(), CV_32FC1, 1);
		for (int i = 0; i < im.width(); i++)
		{
			for (int j = 0; j < im.height(); j++)
			{
				im.setPixel(i, j, qRgb(0, 0, 0));
				m_mask_.at<float>(i, j) = 0;
				if (rgb == this->m_mask_image_.pixel(i, j))
				{
					im.setPixel(i, j, qRgb(255, 0, 0));
					m_mask_.at<float>(i, j) = 1;
				}
			}
		}
		emit mask_selected();

	}
	else if (event->button() == Qt::RightButton)
	{
		//this->hide();
	}

	
};



void MiniTexture::mousePressEvent(QMouseEvent * event)
{
	if (this->m_shown_mode_ != 1)
	{
		return;
	}
	if (event->button() == Qt::RightButton)
	{

		QPoint p = event->pos();
		QRgb rgb = this->m_mask_image_.pixel(p);

		if (rgb == qRgb(0, 0, 0))
		{
			return;
		}

		QImage img(m_mask_image_.width(), this->m_mask_image_.height(), QImage::Format_RGB32);
		for (int i = 0; i < img.width(); i++)
		{
			for (int j = 0; j < img.height(); j++)
			{
				img.setPixel(i, j, qRgb(0, 0, 0));
				if (rgb == this->m_mask_image_.pixel(i, j))
				{
					img.setPixel(i, j, this->m_mesh_image_.pixel(i, j));
				}
			}
		}

		//this->m_selection_label_->setGeometry(this->geometry());
		this->m_selection_label_->setPixmap(QPixmap::fromImage(img).scaled(this->width(), this->height(), Qt::KeepAspectRatio));
		this->m_selection_label_->hide();
		this->m_selection_label_->show();
	}
};
void MiniTexture::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == Qt::RightButton)
	{
		this->m_selection_label_->hide();
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
const cv::Mat& MiniTexture::get_mask()
{
	return this->m_mask_;
};