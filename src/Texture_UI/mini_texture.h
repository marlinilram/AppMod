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

enum MINITEXTURE_SHOW_MODE
{
	ORIGIN_FOR_D0 = 0,
	PARA_MESH_D0 = 1,
	MASK_STROKE_D0 = 2,
	ORIGIN_FOR_D1 = 3,
	PARA_MESH_D1 =4,
	MASK_STROKE_D1 = 5
	//0->origin. 1->para_mesh_image; 2->mask_stroke_for_d0; 3->
};
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
	
	void show_origin_image_d0();
	void show_mesh_image_d0();
	void show_stroke_image_d0();

	void show_origin_image_d1();
	void show_mesh_image_d1();
	void show_stroke_image_d1();

	static void generate_mask(const QImage& im, QImage& mask);
	static void readMaps(std::string xml_file, std::vector<cv::Mat>& maps, std::string map_name);
	bool load_texture();
	bool load_texture_d0();
	bool load_texture_d1();
	MINITEXTURE_SHOW_MODE get_show_mode();
	void set_show_mode(MINITEXTURE_SHOW_MODE);
	const QImage& get_masked_image_D0(){ return this->m_masked_image_d0_; };
	const QImage& get_origin_image_D0(){ return this->m_origin_image_d0_; };
	const QImage& get_mesh_image_D0(){ return this->m_mesh_image_d0_; };

	void clone(MiniTexture&);
public:
	void set_new_image(const QImage&);
	void set_image(const QImage&);
	void reset_window_for_stroke();
	void update_window_for_stroke(int x, int y);
	void paintEvent(QPaintEvent * event);
	void set_mask_d0(cv::Mat&);
	void set_mask_d1(cv::Mat&);
	cv::Mat get_mask_d1();
	cv::Mat get_mask_d0();

	void double_Click_On_Origin_D0(QMouseEvent * event);
	void double_Click_On_PARA_MESH_D0(QMouseEvent * event);
	void double_Click_On_MASK_STROKE_D0(QMouseEvent * event);
	void double_Click_On_Origin_D1(QMouseEvent * event);
	void double_Click_On_PARA_MESH_D1(QMouseEvent * event);
	void double_Click_On_MASK_STROKE_D1(QMouseEvent * event);

	void press_On_Origin_D0(QMouseEvent * event);
	void press_On_PARA_MESH_D0(QMouseEvent * event);
	void press_On_MASK_STROKE_D0(QMouseEvent * event);
	void press_On_Origin_D1(QMouseEvent * event);
	void press_On_PARA_MESH_D1(QMouseEvent * event);
	void press_On_MASK_STROKE_D1(QMouseEvent * event);


	void release_On_Origin_D0(QMouseEvent * event);
	void release_On_PARA_MESH_D0(QMouseEvent * event);
	void release_On_MASK_STROKE_D0(QMouseEvent * event);
	void release_On_Origin_D1(QMouseEvent * event);
	void release_On_PARA_MESH_D1(QMouseEvent * event);
	void release_On_MASK_STROKE_D1(QMouseEvent * event);

	void move_On_Origin_D0(QMouseEvent * event);
	void move_On_PARA_MESH_D0(QMouseEvent * event);
	void move_On_MASK_STROKE_D0(QMouseEvent * event);
	void move_On_Origin_D1(QMouseEvent * event);
	void move_On_PARA_MESH_D1(QMouseEvent * event);
	void move_On_MASK_STROKE_D1(QMouseEvent * event);
private:
	void mouseDoubleClickEvent(QMouseEvent * event);
	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);
	static QImage merge_image(const QImage& im, const cv::Mat& mask, QRgb rgb);
public slots:
	void clear();


private:
	QString m_image_file_;

	QImage  m_origin_image_d0_;
	QImage  m_mesh_image_d0_;
	QImage  m_mask_image_d0_;
	QImage  m_for_stroke_image_d0_;
	QImage  m_masked_image_d0_;
	cv::Mat m_mask_d0_;
	cv::Mat m_mask_tmp_d0_;

	QImage  m_origin_image_d1_;
	QImage  m_mesh_image_d1_;
	QImage  m_mask_image_d1_;
	QImage  m_for_stroke_image_d1_;
	cv::Mat m_mask_d1_;
	cv::Mat m_mask_tmp_d1_;



	MINITEXTURE_SHOW_MODE m_shown_mode_;// 0->origin. 1->para_mesh_image; 2->mask_stroke_for_d0; 3->
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
	float m_image_scale_;

	int m_width_ = 500;
	int m_height_ = 500;
signals:
	void mask_selected_d0();
	void mask_selected_d1();
};


#endif // 
