#ifndef Texture_Mesh_Corres_H
#define Texture_Mesh_Corres_H
#include <QLabel>
#include <QString>
#include <QImage>
#include <QAction>
#include "color.h"
#include "PolygonMesh.h"
#include <opencv2/core/core.hpp>
class MiniTexture;
class PolygonMesh;
class QGLViewer;
class QMoveEvent;
class QMenu;
class TexSynHandler;
class Texture_Mesh_Corres: public QLabel
{
  Q_OBJECT

public:
	Texture_Mesh_Corres(QWidget * parent = 0, Qt::WindowFlags f = 0);
	~Texture_Mesh_Corres();
public:
	void set_data(LG::PolygonMesh* mesh, 
		const std::vector<bool>& m_face_selected, 
		const QImage& image, 
		const QString& file_dir, 
		QGLViewer*, 
		const std::vector<int>&,
		const cv::Mat& mask,
		TexSynHandler*
		);

	QString file_path();
	cv::Mat	get_mask_source();
	cv::Mat	get_mask_target();
	const std::vector<int>&	get_face_ids_in_mesh(){ return this->m_face_ids_in_mesh_; };

	void draw_line();
	void draw_points();
private:
	void set_file(QString);
	void set_image(const QImage&);
	void compute_mesh_center();
	void compute_faces_centers();
	void paintEvent(QPaintEvent * event);
	void moveEvent(QMoveEvent * event);

	void mousePressEvent(QMouseEvent *);
	void mouseReleaseEvent(QMouseEvent *);
	void mouseMoveEvent(QMouseEvent *);
	void timerEvent(QTimerEvent *);
	void dropEvent(QDropEvent * event);

	void creat_menu();
	QString m_image_file_;
	QImage  m_image_;
	bool	m_new_image_;
private:
	LG::PolygonMesh* m_mesh_;
	LG::Vec3	 m_mesh_center_;
	std::vector<int>	m_face_ids_in_mesh_;
	std::vector<int>	m_face_boundaries_in_mesh_;
	std::vector<LG::Vec3>	m_face_centers_;
	cv::Mat	   m_mask_source_;
	cv::Mat	   m_mask_target_;

	Colorf		m_color_;
	QGLViewer*	m_viewer_;

	int m_width_icon_;
	int m_height_icon_;

	bool m_left_button_;
	QPoint m_p_previous_;
	QMenu*	   m_menu_;
	

private slots:
	void delete_this();
signals :
	void delete_coress(Texture_Mesh_Corres*);

};
#endif // !DispModuleHandler_H
