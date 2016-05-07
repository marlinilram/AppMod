#ifndef MiniTexture_H
#define MiniTexture_H
#include <QLabel>
#include <QObject>
#include <QString>
#include <QImage>
#include <QWidget>
#include <QMouseEvent>
#include "AppearanceModel.h"
#include <QPaintDevice>
class AppearanceModel;
class QPainter;
class MiniTexture : public QLabel
{
  Q_OBJECT

public:
	MiniTexture(QWidget * parent = 0, Qt::WindowFlags f = 0);
	~MiniTexture();
	
public:
	void set_center_pos(int x, int y);
	void set_file(QString);
	QString get_file_name();
	
	void show_origin_image();
	void show_mesh_image();
	void show_stroke_image();

	static void generate_mask(const QImage& im, QImage& mask);
	static void readMaps(std::string xml_file, std::vector<cv::Mat>& maps, std::string map_name);
	void load_texture();

private:
	void set_image(const QImage&);
	void reset_window_for_stroke();
	void update_window_for_stroke(int x, int y);
	void paintEvent(QPaintEvent * event);
	

public:
	void mouseDoubleClickEvent(QMouseEvent * event);
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);
	void set_mask(cv::Mat&);
	cv::Mat get_mask();
	static QImage merge_image(const QImage& im, const cv::Mat& mask, QRgb rgb);
public slots:
	void clear();


private:
	QString m_image_file_;

	QImage  m_origin_image_;
	QImage  m_mesh_image_;
	QImage  m_mask_image_;
	QImage  m_for_stroke_image_;

	void* m_texture_;
	cv::Mat m_mask_;
	cv::Mat m_mask_tmp_;
	int m_shown_mode_;// 0->origin. 1->para_mesh_image; 2-> mask stroke

	int m_r_previous_;
	int m_g_previous_;
	int m_b_previous_;

	bool m_left_button_down_;
	bool m_right_button_down_;
	int  m_x_min_for_mask_;
	int  m_y_min_for_mask_;
	int  m_x_max_for_mask_;
	int  m_y_max_for_mask_;
	std::vector<QPoint> m_stroke_points_;


	//QPainter* m_painter_;
signals:
	void mask_selected();
};


#endif // 
