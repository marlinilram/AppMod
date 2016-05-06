
#include "Texture_Viewer.h"
#include "Texture_Canvas.h"
#include "Bound.h"
#include <QKeyEvent>
#include "../Model/Model.h"
#include "../Model/Shape.h"
#include "../DispModule/Viewer/DispObject.h"
#include <QGLViewer/manipulatedFrame.h>
#include "PolygonMesh.h"
#include <QResizeEvent>
#include "PolygonMesh_Manipulator.h"
#include "viewer_selector.h"
Texture_Viewer::Texture_Viewer(QWidget *widget)
  : BasicViewer(widget)
{
  wireframe_ = false;
  show_trackball = false;
  play_lightball = false;
  this->init();
  this->m_left_button_down_ = false;
  this->m_right_button_down_ = false;
  m_edit_mode_ = -1;
  m_show_mesh_ = true;
 /* connect(this, SIGNAL(resizeEvent(QResizeEvent*)), this, SLOT(resize_happen(QResizeEvent*)));*/
}

Texture_Viewer::~Texture_Viewer()
{
	for (unsigned int i = 0; i < this->get_dispObjects().size();i++)
	{
		delete this->get_dispObjects()[i];
	}
}

void Texture_Viewer::resizeEvent(QResizeEvent* r)
{
	QGLViewer::resizeEvent(r);
	int w = r->size().width();
	int h = r->size().height();
	for (unsigned int i = 0; i < this->get_dispObjects().size(); i++)
	{
		dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[i])->setsize(w,h);
	}
	
};
void Texture_Viewer::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawCornerAxis();

  if (m_show_mesh_)
  for (int i = 0; i < dispObjects.size(); ++i)
  {
    glDisable(GL_LIGHTING);
    drawTrackBall();
    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
	else
	{
		dispObjects[i]->set_viewer(this);
	}
  }

  this->draw_points_under_mouse();
}

#include "color_table.h"
void Texture_Viewer::draw_points_under_mouse()
{
	std::shared_ptr<Model> m = dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel();
	if (m == NULL)
	{
		return;
	}
	LG::PolygonMesh* poly_mesh = m->getShape()->getPolygonMesh();
	int num_f = poly_mesh->n_faces();

	glColor3f(1.0, 0.0, 0.0);
	glPointSize(4);
	glBegin(GL_POINTS);
	for (int f_id = 0; f_id < num_f; f_id++)
	{
		if (! (this->m_faces_selected_[f_id]))
		{
			continue;
		}
		int num = 0;
		LG::Vec3 v_total(0, 0, 0);
		
		for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
		{
			LG::Vec3 p = poly_mesh->position(vfc);
			v_total = v_total + p;
			num++;
		}
		v_total = v_total / num;
		glVertex3f(v_total.x(), v_total.y(), v_total.z());
	}
	glEnd();

	
	for (int i = 0; i < m_boundaries_.size(); i++)
	{
		Colorf c = GLOBAL::color_from_table(255* i / m_boundaries_.size());
		glColor3fv(c.data());
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < m_boundaries_[i].size(); j++)
		{
			
			int num = 0;
			LG::Vec3 v_total(0, 0, 0);
			int f_id = m_boundaries_[i][j];
			for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
			{
				LG::Vec3 p = poly_mesh->position(vfc);
				v_total = v_total + p;
				num++;
			}
			v_total = v_total / num;
			glVertex3f(v_total.x(), v_total.y(), v_total.z());
		}
		glEnd();
	}
	



	

	this->startScreenCoordinatesSystem();
	glColor3f(1.0, 1.0, 0);
	if (this->m_left_button_down_)
	{
		glColor3f(0.0, 1.0, 1.);
	}
	glLineWidth(3);
	glBegin(GL_LINE_LOOP);
	for (unsigned int i = 0; i < this->m_points_for_delete_.size(); i++)
	{
		glVertex2i(m_points_for_delete_[i].x(), m_points_for_delete_[i].y());
	};
	glEnd();
	this->stopScreenCoordinatesSystem();
};

void Texture_Viewer::init()
{

  BasicViewer::init();

  // Set shader
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  //glEnable(GL_CULL_FACE);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_FLAT);

  glClearColor(1,1,1,0);
  setBackgroundColor(QColor(Qt::white));
  setForegroundColor(QColor(Qt::white));

  setSceneCenter(qglviewer::Vec(0, 0, 0));
  setSceneRadius(50);
  camera()->fitSphere(qglviewer::Vec(0, 0, 0), 5);
  camera()->setType(qglviewer::Camera::Type::PERSPECTIVE);
  camera()->setFlySpeed(0.5);
  setWheelandMouse();
}

void Texture_Viewer::drawTrackBall()
{
  if (!show_trackball)
  {
    return;
  }

	double v1,v2;
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
    v1 = ((dispObjects[0]->getBoundBox())->centroid).x + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).y + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.8,0.6,0.6);
		glVertex3f(v1,v2,((dispObjects[0]->getBoundBox())->centroid).z);
	}
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
		v1 = ((dispObjects[0]->getBoundBox())->centroid).y + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).z + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.6,0.8,0.6);
		glVertex3f(((dispObjects[0]->getBoundBox())->centroid).x,v1,v2);
	}
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
		v1 = ((dispObjects[0]->getBoundBox())->centroid).x + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).z + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.6,0.6,0.8);
		glVertex3f(v1,((dispObjects[0]->getBoundBox())->centroid).y,v2);
	}
	glEnd();
}

void Texture_Viewer::setWheelandMouse()
{
  /////////////////////////////////////////////////////
  //         Mouse bindings customization            //
  //     Changes standard action mouse bindings      //
  /////////////////////////////////////////////////////

  //
  setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);

  // Left and right buttons together make a camera zoom : emulates a mouse third button if needed.
  setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::LeftButton, CAMERA, ZOOM);

  setMouseBinding(Qt::ControlModifier | Qt::ShiftModifier, Qt::RightButton, SELECT);
  setWheelBinding(Qt::NoModifier, CAMERA, MOVE_FORWARD);
  setMouseBinding(Qt::AltModifier, Qt::LeftButton, CAMERA, TRANSLATE);

  // Add custom mouse bindings description (see mousePressEvent())
  setMouseBindingDescription(Qt::NoModifier, Qt::RightButton, "Opens a camera path context menu");

  setManipulatedFrame(new qglviewer::ManipulatedFrame());
}

void Texture_Viewer::toggleLightball()
{
  
}

void Texture_Viewer::updateBuffer()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
	  Texture_Canvas* texture_canvas = dynamic_cast<Texture_Canvas*>(dispObjects[i]);
	  if (texture_canvas)
    {
		texture_canvas->updateModelBuffer();
    }
  }

  doneCurrent();
}

void Texture_Viewer::updateColorBuffer()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
	  Texture_Canvas* texture_canvas = dynamic_cast<Texture_Canvas*>(dispObjects[i]);
	  if (texture_canvas)
    {
		texture_canvas->updateModelColorBuffer();
    }
  }

  doneCurrent();
}

void Texture_Viewer::resetCamera()
{
  // assume only one dispObjects MainCanvas exist here
	makeCurrent(); // in case of opengl context problem in restoreStateFromFile();
  if (dispObjects.empty())
  {
    return;
  }

  Bound* scene_bounds = dynamic_cast<Texture_Canvas*>(dispObjects[0])->getBoundBox();

  qglviewer::Vec scene_center;
  scene_center = qglviewer::Vec((scene_bounds->minX + scene_bounds->maxX) / 2,
    (scene_bounds->minY + scene_bounds->maxY) / 2,
    (scene_bounds->minZ + scene_bounds->maxZ) / 2);

  setSceneCenter(scene_center);

  float x_span = (scene_bounds->maxX - scene_bounds->minX) / 2;
  float y_span = (scene_bounds->maxY - scene_bounds->minY) / 2;
  float z_span = (scene_bounds->maxZ - scene_bounds->minZ) / 2;
  float scene_radius = sqrt(x_span * x_span + y_span * y_span + z_span * z_span);
  //scene_radius *= 1.5;

  setSceneRadius(scene_radius);
  camera()->fitSphere(scene_center, scene_radius);



  setStateFileName(QString((dynamic_cast<Texture_Canvas*>(dispObjects[0])->getFilePath()+"/camera_info.xml").c_str()));
  if (restoreStateFromFile())
    std::cout << "Load camera info successes...\n";
  else
    std::cout << "Load camera info failed...\n";
  doneCurrent();
  // set the scene in MainCanvasViewer
}

void Texture_Viewer::drawCornerAxis()
{
  int viewport[4];
  int scissor[4];

  // The viewport and the scissor are changed to fit the lower left
  // corner. Original values are saved.
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetIntegerv(GL_SCISSOR_BOX, scissor);

  // Axis viewport size, in pixels
  const int size = 150;
  glViewport(0, 0, size, size);
  glScissor(0, 0, size, size);

  // The Z-buffer is cleared to make the axis appear over the
  // original image.
  glClear(GL_DEPTH_BUFFER_BIT);

  // Tune for best line rendering
  glDisable(GL_LIGHTING);
  glLineWidth(3.0);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(-1, 1, -1, 1, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(camera()->orientation().inverse().matrix());

  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(1.0, 0.0, 0.0);

  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 1.0, 0.0);

  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 1.0);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glEnable(GL_LIGHTING);

  // The viewport and the scissor are restored.
  glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void Texture_Viewer::mousePressEvent(QMouseEvent* e)
{	
	if (this->m_edit_mode_ < 0)
	{
		QGLViewer::mousePressEvent(e);
		return;
	}
	else if (e->button()== Qt::LeftButton)
	{
		this->m_left_button_down_ = true;
	}
	else if (e->button() == Qt::RightButton)
	{
		m_right_button_down_ = true;
		m_points_for_delete_.clear();
	}

}
void Texture_Viewer::mouseDoubleClickEvent(QMouseEvent * event)
{
	if (this->m_edit_mode_ < 0)
	{
		QGLViewer::mouseDoubleClickEvent(event);
		return;
	}

};

void Texture_Viewer::mouseMoveEvent(QMouseEvent *e)
{
	if (this->m_edit_mode_ < 0)
	{
		QGLViewer::mouseMoveEvent(e);
		return;
	}
	if (this->m_left_button_down_ && dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel())
	{
		bool b = false;
		QPoint p = e->pos();
		this->m_points_for_delete_.push_back(p);
		this->updateGL();
// 		qglviewer::Vec v = this->camera()->pointUnderPixel(p, b);
// 		std::cout << b << " ";
// 		if (b )
// 		{
// 			this->m_points_ubder_mouse_.push_back(v);
// 			this->updateGL();
// 		}
// 		int x = p.x();
// 		int y = /*this->height() - */p.y();
// 		if (x < 0 || x >= this->width() || y < 0 || y >=this->height())
// 		{
// 			return;
// 		}
// 		std::shared_ptr<Model> m = dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel();
// 		//cv::imwrite("D:/test.jpg", m->getPrimitiveIDImg());
// 		int f_id = m->getPrimitiveIDImg().at<int>(y, x);
// 		std::cout << f_id << "\t";
// 		if (f_id  >= 0)
// 		{
// 			LG::PolygonMesh* poly_mesh = m->getShape()->getPolygonMesh();
// 			LG::Vec3 v_total(0, 0, 0);
// 			int num = 0;
// 			for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
// 			{
// 				LG::Vec3 v = poly_mesh->position(vfc);
// 				v_total = v_total + v;
// 				num++;
// 			}
// 			v_total = v_total / num;
// 			if (this->m_face_ids_.size() < 1 || this->m_face_ids_.back() != f_id)
// 			{
// 				this->m_face_ids_.push_back(f_id);
// 				this->m_points_ubder_mouse_.push_back(qglviewer::Vec(v_total.x(), v_total.y(), v_total.z()));
// 				this->updateGL();
// 			}
// 
// 		}
	}
	else if (this->m_right_button_down_)
	{
		QPoint p = e->pos();
		this->m_points_for_delete_.push_back(p);
		this->m_boundaries_.clear();
		this->updateGL();
	}
}
void Texture_Viewer::clear_selection()
{
	std::shared_ptr<Model> m = dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel();
	LG::PolygonMesh* poly_mesh = m->getShape()->getPolygonMesh();
	int num_f = poly_mesh->n_faces();
	this->m_faces_selected_.clear();
	this->m_faces_selected_.resize(num_f, false);
	this->m_boundaries_.clear();
};

const std::vector<std::vector<int>>& Texture_Viewer::get_boundaries()
{
	return this->m_boundaries_;
};
void Texture_Viewer::mouseReleaseEvent(QMouseEvent* e)
{
	
	if (this->m_edit_mode_ < 0)
	{
		QGLViewer::mouseReleaseEvent(e); return;
	}

// 	if (e->button() == Qt::LeftButton)
// 	{
// 
// 	}
	this->m_left_button_down_ = false;
	this->m_right_button_down_ = false;

	if (e->button() == Qt::RightButton)
	{
		this->m_right_button_down_ = false;
		
		std::shared_ptr<Model> m = dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel();
		LG::PolygonMesh* poly_mesh = m->getShape()->getPolygonMesh();
		int num_f = poly_mesh->n_faces();

		for (int f_id = 0; f_id < num_f; f_id++)
		{
			if ( !(this->m_faces_selected_[f_id]) )
			{
				continue;
			}
			int num = 0;
			LG::Vec3 v_total(0, 0, 0);
			for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
			{
				LG::Vec3 v = poly_mesh->position(vfc);
				v_total = v_total + v;
				num++;
			}
			v_total = v_total / num;
			qglviewer::Vec proc = this->camera()->projectedCoordinatesOf(qglviewer::Vec(v_total.x(), v_total.y(), v_total.z()));
			QPoint p_proc(proc.x, proc.y);

			if (p_proc.x() < 0 || p_proc.x() >= this->width() || p_proc.y() < 0 || p_proc.y() >= this->height())
			{
				continue;
			}

			
			if (Viewer_Selector::is_point_in_polygon(p_proc, m_points_for_delete_))
			{
				this->m_faces_selected_[f_id] = false;
			}

		}

		std::vector<std::vector<int>> bbb;
		PolygonMesh_Manipulator::boundary_find(poly_mesh, this->m_faces_selected_, bbb);
		this->m_boundaries_ = bbb;
		m_points_for_delete_.clear();
		this->updateGL();
	}
	else if (e->button() == Qt::LeftButton)
	{
		this->m_left_button_down_ = false;




		std::shared_ptr<Model> m = dynamic_cast<Texture_Canvas*>(this->get_dispObjects()[0])->getModel();
		LG::PolygonMesh* poly_mesh = m->getShape()->getPolygonMesh();
		int num_f = poly_mesh->n_faces();

		for (int f_id = 0; f_id < num_f; f_id++)
		{
			if (this->m_faces_selected_[f_id])
			{
				continue;
			}
			int num = 0;
			LG::Vec3 v_total(0, 0, 0);
			for (auto vfc : poly_mesh->vertices(LG::PolygonMesh::Face(f_id)))
			{
				LG::Vec3 v = poly_mesh->position(vfc);
				v_total = v_total + v;
				num++;
			}
			v_total = v_total / num;
			qglviewer::Vec proc = this->camera()->projectedCoordinatesOf(qglviewer::Vec(v_total.x(), v_total.y(), v_total.z()));
			QPoint p_proc(proc.x, proc.y);

			if (p_proc.x() < 0 || p_proc.x() >= this->width() || p_proc.y() < 0 || p_proc.y() >= this->height())
			{
				continue;
			}

			int f_i = m->getPrimitiveIDImg().at<int>(p_proc.y(), p_proc.x());

			if (f_id == f_i)
			{
				if (Viewer_Selector::is_point_in_polygon(p_proc, m_points_for_delete_))
				{
					this->m_faces_selected_[f_id] = true;
				}

			}
		}
		std::vector<std::vector<int>> bbb;
		PolygonMesh_Manipulator::boundary_find(poly_mesh, this->m_faces_selected_, bbb);
		this->m_boundaries_ = bbb;
		std::cout << bbb.size() << " bbb.size()";
		m_points_for_delete_.clear();
		this->updateGL();
	}

}

void Texture_Viewer::wheelEvent(QWheelEvent* e)
{
	//if (this->m_edit_mode_ < 0)
	{
		QGLViewer::wheelEvent(e); return;
	}
}

void Texture_Viewer::keyPressEvent(QKeyEvent *e)
{
		// Get event modifiers key
		const Qt::KeyboardModifiers modifiers = e->modifiers();

		// A simple switch on e->key() is not sufficient if we want to take state key into account.
		// With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
		// That's why we use imbricated if...else and a "handled" boolean.
		bool handled = false;
		if ((e->key() == Qt::Key_W) && (modifiers == Qt::NoButton))
		{
			wireframe_ = !wireframe_;
			if (wireframe_)
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			else
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			handled = true;
			updateGL();
		}
		else if ((e->key() == Qt::Key_R) && (modifiers == Qt::NoButton))
		{
			resetCamera();
			handled = true;
			updateGL();
		}


		else if ((e->key() == Qt::Key_Space))
		{
			m_show_mesh_ = !m_show_mesh_;
			handled = true;
			updateGL();
		}

		if (!handled)
		{
			QGLViewer::keyPressEvent(e);
		}
	
}

void Texture_Viewer::set_edit_mode(int b)
{
	this->m_edit_mode_ = b;
};
bool Texture_Viewer::get_edit_mode()
{
	return this->m_edit_mode_;
};

bool Texture_Viewer::draw_mesh_points()
{
	glBegin(GL_POINTS);
	//glColor4f(1, 1, 1, 1);
	glColor4f(1, 0, 0, 0);

	for (int i = 0; i < dispObjects.size(); ++i)
	{
		if (dispObjects[i]->getModel() == NULL)
		{
			continue;
		}
		const VertexList& vl = dispObjects[i]->getModel()->getShapeVertexList();
		for (unsigned int j = 0; j < vl.size(); j+=3)
		{
			glVertex3f(vl[j], vl[j+1], vl[j+2]);
		}
	}
	glEnd();
	return true;
};



