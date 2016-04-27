#ifndef Shape_Manipulator_H
#define Shape_Manipulator_H
#include "Shape.h"
#include "geometry_types.h"
#include <vector>
#include <QGLViewer/qglviewer.h>
#include <QPoint>
class Shape_Manipulator
{
public:
	Shape_Manipulator();
	~Shape_Manipulator();
public:
	Shape* get_shape();
	void set_shape(Shape* s);
	bool double_click(QMouseEvent* e,int& activated);
	int mouse_press(QMouseEvent* e);
	int mouse_move(QMouseEvent* e, Vector3_f& vt);
	int	release(QMouseEvent *e);
	bool wheel(QWheelEvent *e, Vector3_f& v_line, float& angle, Point3f& center_scale, float& scale, int& scale_rotate);
	void draw();
private:
	Shape* m_shape_;

	std::vector<qglviewer::Vec> m_axises_;
	int m_axis_selected_;
	QPoint	start_pos_;

	Point3f center_;
	Point3f	center_previous_;
	bool  left_button_down_;
	bool  right_button_down_;
	float m_beishu_;
};
#endif
