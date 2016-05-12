#pragma once
#include <vector>
#include <QGLViewer/qglviewer.h>
class QGLViewer;
class QPoint;

class Viewer_Selector
{
public:
	Viewer_Selector();
	~Viewer_Selector();

	void set_viewer(QGLViewer*);
	QGLViewer*  get_viewer();

public:

	static bool delete_points_in_polygon(std::vector<qglviewer::Vec>&, std::vector<int>& ids, const std::vector<QPoint>& polygon,QGLViewer* v);

	static bool is_point_in_polygon(const QPoint& P, const std::vector<QPoint>& polygon, float scale = 1);
	static bool is_point_in_polygon(const qglviewer::Vec& P, const std::vector<QPoint>& polygon, QGLViewer* v);
private:
	QGLViewer* m_gl_viewer_;
};

