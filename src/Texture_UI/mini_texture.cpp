#include "mini_texture.h"
#include <QApplication>
#include <QDesktopWidget>
#include <highgui.h>
MiniTexture::MiniTexture(QWidget * parent, Qt::WindowFlags f)
	:QLabel(parent, f)
{

	m_with_image_ = false;
	m_new_image_ = false;
	this->m_image_file_.clear();
	this->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	this->setStyleSheet("border: 1px groove gray;");
	this->m_texture_ = NULL;
	this->m_appearance_model_ = NULL;
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
		if (this->m_appearance_model_ == NULL)
		{
			QString file_path = this->m_image_file_;
			std::string std_file_path = file_path.toStdString().substr(0, file_path.toStdString().find_last_of('/'));
			this->m_appearance_model_ = new AppearanceModel();
			this->m_appearance_model_->importAppMod("app_model.xml", std_file_path);
			std::vector<cv::Mat> detail_maps;
			this->m_appearance_model_->getD1Details(detail_maps);

			cv::merge(detail_maps, m_show_images_);
			cv::imwrite("D:/test.jpg", m_show_images_);

			QImage im(m_show_images_.cols, m_show_images_.rows, QImage::Format_RGB32);
			for (int i = 0; i < im.width(); i++)
			{
				for (int j = 0; j < im.height(); j++)
				{
					im.setPixel(i, j, qRgb(detail_maps[0].at<float>(i, j) * 255, detail_maps[1].at<float>(i, j) * 255, detail_maps[2].at<float>(i, j) * 255));
					if (detail_maps[0].at<float>(i, j) < 0)
					{
						im.setPixel(i, j, qRgb(0, 0, 0));
					}
				}
			}

			/*QImage im(m_show_images_.data, m_show_images_.cols, m_show_images_.rows, QImage::Format_RGB32);*/
			//this->setPixmap(QPixmap::fromImage(im).scaled(im.width(), im.height(), Qt::KeepAspectRatio));
			this->setFixedSize(im.width(), im.height());

			MiniTexture::generate_mask(im, m_image_mask_);
			/*m_image_mask_.save("d:/mask.jpg");*/
			this->set_image(im);

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

void MiniTexture::mouseDoubleClickEvent(QMouseEvent * event)
{
	QPoint p = event->pos();
	QRgb rgb = this->m_image_mask_.pixel(p);
	std::cout << qRed(rgb) << " " << qGreen(rgb) << " " << qBlue(rgb) << "\n";
	QImage im(this->m_image_mask_.width(), this->m_image_mask_.height(), QImage::Format_RGB32);

	for (int i = 0; i < im.width(); i++)
	{
		for (int j = 0; j < im.height(); j++)
		{
			im.setPixel(i, j, qRgb(0, 0, 0));

			if (rgb == this->m_image_mask_.pixel(i, j))
			{
				im.setPixel(i, j, qRgb(255, 0, 0));
			}
		}
	}
	im.save("mask.jpg");
	
	IplImage* iplImg = cvLoadImage("mask.jpg", 1);
	cv::Mat mtx(iplImg);
	cvShowImage("mask",iplImg);
};