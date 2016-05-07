#ifndef MiniTexture_H
#define MiniTexture_H
#include <QLabel>
#include <QObject>
#include <QString>
#include <QImage>
#include <QWidget>
#include <QMouseEvent>
#include <QThread>
#include "AppearanceModel.h"
class AppearanceModel;
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
	static void generate_mask(const QImage& im, QImage& mask);
	static void readMaps(std::string xml_file, std::vector<cv::Mat>& maps, std::string map_name);
	void load_texture();
private:
	void set_image(const QImage&);
	


public:
	void mouseDoubleClickEvent(QMouseEvent * event);
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);

	void set_mask(cv::Mat&);
	const cv::Mat& get_mask();
public slots:
	void clear();


private:
	QString m_image_file_;

	QImage  m_origin_image_;
	QImage  m_mesh_image_;
	QImage  m_mask_image_;

	void* m_texture_;
	cv::Mat m_mask_;
	int m_shown_mode_;// 0->origin. 1->mesh;

	int m_r_previous_;
	int m_g_previous_;
	int m_b_previous_;

signals:
	void mask_selected();
};


#endif // 
