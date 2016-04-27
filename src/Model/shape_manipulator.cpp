#include "shape_manipulator.h"
#include <QGLViewer/qglviewer.h>
#include <Bound.h>
#include "geometry_types.h"
Shape_Manipulator::Shape_Manipulator()
{
	m_axises_.resize(3);
	m_axis_selected_ = -1;
	m_axises_[0] = qglviewer::Vec(1, 0, 0);
	m_axises_[1] = qglviewer::Vec(0, 1, 0);
	m_axises_[2] = qglviewer::Vec(0, 0, 1);
	this->m_shape_ = NULL;
	this->m_beishu_ = 1.5;
}


Shape_Manipulator::~Shape_Manipulator()
{
}
Shape* Shape_Manipulator::get_shape()
{
	return this->m_shape_;
};
void Shape_Manipulator::set_shape(Shape* s)
{
	this->m_shape_ = s;
};

void Shape_Manipulator::draw()
{
	if (this->get_shape() == NULL)
	{
		return;
	}
	if (this->get_shape()->glviewer() == NULL)
	{
		return;
	}

	Bound* bbb = this->get_shape()->getBoundbox();
	CvPoint3D32f center = bbb->centroid;
	float r = bbb->radius;

	std::vector<Colorf>	colors(3, Colorf(1, 0, 0, 1));
	colors[1] = Colorf(0, 1, 0, 1);
	colors[2] = Colorf(0, 0, 1, 1);

	
	for (int i = 0; i < this->m_axises_.size(); i++)
	{
		qglviewer::Vec from(center.x, center.y, center.z);
		qglviewer::Vec to_p = from + this->m_axises_[i] * r * this->m_beishu_;

		glColor4fv(colors[i].data());
		if (this->m_axis_selected_ == i)
		{
			to_p = from + this->m_axises_[i] * r *this->m_beishu_ * 2;
			glColor4fv(colors[i].data());
		}

		qglviewer::Vec to(to_p.x, to_p.y, to_p.z);
		from = from + qglviewer::Vec(this->m_axises_[i].x*r*this->m_beishu_ / 2, this->m_axises_[i].y*r*this->m_beishu_ / 2, this->m_axises_[i].z*r*this->m_beishu_ / 2);
		this->get_shape()->glviewer()->drawArrow(from, to);
	}
}
bool Shape_Manipulator::double_click(QMouseEvent* e, int& activated)
{
	if ( ! (e->button() == Qt::LeftButton)  )
	{
		this->get_shape()->set_selected(false);
		activated = -1;
		return false;
	}
	if (this->m_shape_ == NULL)
	{
		this->get_shape()->set_selected(false);
		activated = -1;
		return false;
	}
	QPoint pos = e->pos();
	QPoint position_click = pos;

	QGLViewer* gl = this->m_shape_->glviewer();
	if (gl == NULL)
	{
		this->get_shape()->set_selected(false);
		activated = -1;
		return false;
	}
	Bound* bbb = this->get_shape()->getBoundbox();
	qglviewer::Vec up_vec = gl->camera()->upVector();
	const qglviewer::Vec from(bbb->centroid.x, bbb->centroid.y, bbb->centroid.z);
	const qglviewer::Vec to(bbb->centroid.x + up_vec.x * bbb->radius, bbb->centroid.y + up_vec.y * bbb->radius, bbb->centroid.z + up_vec.z * bbb->radius);

	qglviewer::Vec v_to = gl->camera()->projectedCoordinatesOf(to);
	qglviewer::Vec v = gl->camera()->projectedCoordinatesOf(from);

	float distance_threshold = std::pow((v.x - v_to.x), 2.0) + std::pow((v.y - v_to.y), 2.0);
	float distance_ = std::pow((position_click.x() - v.x), 2.0) + std::pow((position_click.y() - v.y), 2.0);


	if (distance_ < distance_threshold)
	{
		this->get_shape()->set_selected(true);
		activated = 0;
		return true;
	}
	else if (this->get_shape()->is_selected())
	{
		this->get_shape()->set_selected(false);
		activated = 0;
		return false;
	}
	else
	{
		this->get_shape()->set_selected(false);
		activated = -1;
		return false;
	}

};

int Shape_Manipulator::mouse_press(QMouseEvent* e)
{
	if ((e->button() == Qt::RightButton))
	{
		this->right_button_down_ = true;
	}
	else if (e->button() == Qt::LeftButton)
	{
		this->left_button_down_ = true;
	}
	if (this->m_shape_ == NULL)
	{
		this->get_shape()->set_selected(false);
		return false;
	}
	QPoint position_press = e->pos();
	Point2f p_click(position_press.x(), position_press.y());
	const std::vector<qglviewer::Vec> diretions = this->m_axises_;
	Bound* bbb = this->get_shape()->getBoundbox();
	const qglviewer::Vec center(bbb->centroid.x, bbb->centroid.y, bbb->centroid.z);
	QGLViewer* viewer = this->get_shape()->glviewer();
	float r = this->get_shape()->getBoundbox()->getRadius();

	for (unsigned int i = 0; i < diretions.size(); i++)
	{
		qglviewer::Vec direction = diretions[i];

		qglviewer::Vec from(center + direction * this->m_beishu_ / 2);
		qglviewer::Vec v_from = viewer->camera()->projectedCoordinatesOf(from);


		qglviewer::Vec	p_to = center + direction* r * this->m_beishu_;
		qglviewer::Vec to(p_to.x, p_to.y, p_to.z);
		qglviewer::Vec v_to = viewer->camera()->projectedCoordinatesOf(to);

		Segment2f seg(Point2f(v_from.x, v_from.y), Point2f(v_to.x, v_to.y));
		float distance = CGAL::squared_distance(seg, p_click);

		if (distance < 20)
		{
			this->m_axis_selected_ = i;
			this->start_pos_ = position_press;
			this->center_previous_ = this->center_;
			this->get_shape()->glviewer()->updateGL();
			return this->m_axis_selected_;
		}

	}

	return false;
};

int	 Shape_Manipulator::mouse_move(QMouseEvent *e, Vector3_f& v_ts)
{
	v_ts = Vector3_f(0, 0, 0);
	if (!(this->get_shape()->is_selected()))
	{
		return -1;
	}
	else if ((this->m_axis_selected_ < 0))
	{
		return -1;
	}
	else if (!(this->left_button_down_))
	{
		return -1;
	}
	QPoint pos_now = e->pos();

	qglviewer::Vec v_move(1, 0, 0);
	if (this->m_axis_selected_ >= 0 && this->m_axis_selected_ <= 2)
	{
		v_move = this->m_axises_[this->m_axis_selected_];
	}
	else
	{
		return -1;
	}
	
	QGLViewer* viewer = this->get_shape()->glviewer();
	qglviewer::Vec from(this->center_previous_.x(), this->center_previous_.y(), this->center_previous_.z());
	qglviewer::Vec v_from = viewer->camera()->projectedCoordinatesOf(from);
	Point2f p_from(v_from.x, v_from.y);

	Point3f p_to_tmp_3d = this->center_previous_ + Vector3_f(v_move.x, v_move.y, v_move.z);
	qglviewer::Vec to_tmp(p_to_tmp_3d.x(), p_to_tmp_3d.y(), p_to_tmp_3d.z());
	qglviewer::Vec v_to_tmp = viewer->camera()->projectedCoordinatesOf(to_tmp);

	Point2f p_to_tmp_2d(v_to_tmp.x, v_to_tmp.y);

	Line2f line_2d(p_from, p_to_tmp_2d);

	Point2f p_edit_from = line_2d.projection(Point2f(this->start_pos_.x(), this->start_pos_.y()));
	Point2f p_to_2d(pos_now.x(), pos_now.y());
	Point2f p_edit_to_2d = line_2d.projection(p_to_2d);

	Vector2_f v_move_2d(p_edit_from, p_edit_to_2d);
	Vector2_f  vector_line = line_2d.to_vector();

	Vector2_f  vector_unit(p_from, p_to_tmp_2d);


	float	beishu = (v_move_2d.squared_length() / vector_unit.squared_length());
	beishu = sqrt(beishu);

	if (v_move_2d*vector_line < 0)
	{
		beishu = -beishu;
	}

	Vector3_f ts(/*this->center_, this->center_previous_ + */Vector3_f(v_move.x, v_move.y, v_move.z)* beishu);
	v_ts = ts;

	this->start_pos_ = e->pos();
	return this->m_axis_selected_;
}

int	Shape_Manipulator::release(QMouseEvent *e)
{
	this->left_button_down_ = false;
	this->right_button_down_ = false;
	this->m_axis_selected_ = -1;
	this->get_shape()->glviewer()->updateGL();
	return this->m_axis_selected_;

}

bool Shape_Manipulator::wheel(QWheelEvent *e, Vector3_f& v_line, float& angle, Point3f& center_scale, float& scale, int& rotate_scale)
{
	angle = e->delta() / 8 * M_PI / 1800;
	rotate_scale = -1;
	if ((e->modifiers() == Qt::ControlModifier) && this->right_button_down_)
	{
		{
			rotate_scale = 1;
			bool scaled = false;
			if (!(this->get_shape()->is_selected()))
			{
				return scaled;
			}
			QPoint pos = e->pos();
			if (!(this->m_axis_selected_ >= 0))
			{
				return scaled;
			}
			CvPoint3D32f c =this->get_shape()->getBoundbox()->centroid;
			center_scale = Point3f(c.x, c.y, c.z);
			scale = 1 + angle;
			scaled = true;
			return scaled;
		}

	}
	else if ((e->modifiers() == Qt::AltModifier) && this->right_button_down_)
	{
		{
			rotate_scale = 2;
			bool scaled = false;

			center_scale = this->center_;
			v_line = Vector3_f(0, 0, 0);

			if (this->m_axis_selected_ >= 0 && this->m_axis_selected_ <= 2)
			{
				v_line = Vector3_f(this->m_axises_[this->m_axis_selected_].x, this->m_axises_[this->m_axis_selected_].y, this->m_axises_[this->m_axis_selected_].z);
			}
			CvPoint3D32f c = this->get_shape()->getBoundbox()->centroid;
			center_scale = Point3f(c.x, c.y, c.z);
			scale = 1 + angle;
			scaled = true;
			return scaled;
		}
	}
	else if (this->right_button_down_)
	{
		rotate_scale = 0;
		bool rotated = false;
		v_line = Vector3_f(0, 0, 0);
		if (this->m_axis_selected_ >= 0 && this->m_axis_selected_ <= 2)
		{
			v_line = Vector3_f(this->m_axises_[this->m_axis_selected_].x, this->m_axises_[this->m_axis_selected_].y, this->m_axises_[this->m_axis_selected_].z);
			rotated = true;
		}
		return rotated;
	}
	return false;
}
