#ifndef CANVAS_MINIVIEW_H
#define CANVAS_MINIVIEW_H

#include <glew-1.11.0/include/GL/glew.h>
#include "QGLViewer/qglviewer.h"
#include "poly_mesh.h"
#include <QLabel>
#include <QObject>
class MainWindow_Texture;
class Texture_Viewer;
class Canvas_Miniview : public QGLViewer
{
	Q_OBJECT

public:
	Canvas_Miniview(const QGLFormat& format, QWidget *parent, const QGLWidget* shareWidget);
	~Canvas_Miniview();

public:
	void fitScreen();
	void load_obj(QString obj_File);
	void clear();
	void set_mainwindow(MainWindow_Texture*);
private:
	virtual void draw();
	virtual void init();
private:
	virtual void mouseDoubleClickEvent (QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent *e);
	virtual void mousePressEvent(QMouseEvent *e);
	virtual void mouseMoveEvent(QMouseEvent *e);

	
	static bool do_read(std::istream& input, PolyMesh* mesh);
	static bool do_read(const std::string& file_name, PolyMesh* mesh);
private:

	QLabel* m_label_dragging_;
	QPoint  m_clicked_mouse_;
	QPoint  m_label_topleft_;
	MainWindow_Texture* m_mainWindow_;
	bool	m_left_button_on_;
	bool	m_right_button_on_;
	PolyMesh* m_mesh_;
	QString m_file_name_;
signals:
	void selected_obj(QPoint, QString);
};


#endif // CANVAS_MINIVIEW_H
