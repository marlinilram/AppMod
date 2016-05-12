#include "canvas_miniview.h"
#include "../qglviewer/vec.h"
#include "MainWindow_Texture.h"
#include "Texture_Viewer.h"
#include "QGLViewer/camera.h"
#include <QMouseEvent>
#include "QGLViewer/manipulatedCameraFrame.h"
#include "poly_mesh.h"
#include "vertex.h"
#include "facet.h"
#include <fstream>
#include "Facet_Group.h"
Canvas_Miniview::Canvas_Miniview(const QGLFormat& format, QWidget *parent, const QGLWidget* shareWidget)
{

	QGLViewer(format, parent, shareWidget);
	this->m_label_dragging_ = NULL;
	this->m_clicked_mouse_ = QPoint(0, 0);
	this->m_label_topleft_ = QPoint(0, 0);

	this->m_left_button_on_ = false;
	this->m_right_button_on_ = false;
	this->m_mesh_ = NULL;
	this->setStyleSheet("border: 1px groove gray;");
	this->m_mainWindow_ = NULL;
};
Canvas_Miniview::~Canvas_Miniview()
{
	
	clear();
};
void Canvas_Miniview::clear()
{
	if (this->m_label_dragging_)
	{
		delete this->m_label_dragging_;
		this->m_label_dragging_ = NULL;
	}
	if (this->m_mesh_)
	{
		delete this->m_mesh_;
		m_mesh_ = NULL;
	}
};
void Canvas_Miniview::init()
{

	setStateFileName("");

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//////////////////////////////////////////////////////////////////////////

	QColor bkgrd_color = Qt::white;
	setBackgroundColor(bkgrd_color);

	//////////////////////////////////////////////////////////////////////////

	this->camera()->frame()->setSpinningSensitivity(/*1.0f*/100.0f);
	setMouseTracking(true);

	GLfloat	light_position[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);


	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); /*GL_FALSE*/
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	glEnable(GL_LIGHT0);		// Enable Light 0
	glEnable(GL_LIGHTING);

	//////////////////////////////////////////////////////////////////////////

	/* to use facet color, the GL_COLOR_MATERIAL should be enabled */
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 128);

	/* to use material color, the GL_COLOR_MATERIAL should be disabled */
	//glDisable(GL_COLOR_MATERIAL);
	glewInit();
}
void Canvas_Miniview::draw()
{
	if (m_mesh_ != NULL)
	{
		m_mesh_->draw();
	}
}

void Canvas_Miniview::set_mainwindow(MainWindow_Texture* w)
{
	this->m_mainWindow_ = w;
};
void Canvas_Miniview::mouseDoubleClickEvent (QMouseEvent* event)
{

	{
		QGLViewer::mouseDoubleClickEvent(event);
	}
};

void Canvas_Miniview::mousePressEvent(QMouseEvent *event)
{
	
	if (event->button() == Qt::RightButton)
	{
		QImage img = this->grabFrameBuffer();
		if (this->m_label_dragging_ == NULL)
		{
			this->m_label_dragging_ = new QLabel();
			this->m_label_dragging_->setWindowFlags(Qt::FramelessWindowHint);
		}
		this->m_label_dragging_->setPixmap(QPixmap::fromImage(img));
		int w = this->width();
		int h = this->height();
		QPoint pos_global = this->mapToGlobal(QPoint(0, 0));
		this->m_clicked_mouse_ = event->globalPos();
		this->m_label_topleft_ = pos_global;
		int x = pos_global.x();
		int y = pos_global.y();
		this->m_label_dragging_->setGeometry(x, y, w, h);
		this->m_label_dragging_->show();
		this->m_right_button_on_ = true;

	}
	else
	{
		QGLViewer::mousePressEvent(event);
	}
};
void Canvas_Miniview::mouseMoveEvent(QMouseEvent *e)
{
	if (this->m_right_button_on_ && this->m_label_dragging_ != NULL)
	{
		QPoint pos = e->globalPos();
		int w = this->m_label_dragging_->width();
		int h = this->m_label_dragging_->height();

		int x = this->m_label_topleft_.x() + (pos.x() - this->m_clicked_mouse_.x());
		int y = this->m_label_topleft_.y() + (pos.y() - this->m_clicked_mouse_.y());
		this->m_label_dragging_->setGeometry(x, y, w, h);

	}
	else
	{
		QGLViewer::mouseMoveEvent(e);
	}
};

void Canvas_Miniview::fitScreen() 
{
	if (this->m_mesh_==NULL)
	{
		return;
	}
	

	Bbox3f box = m_mesh_->bbox();
	qglviewer::Vec  center( (box.xmin()+box.xmax())/2, (box.ymin()+box.ymax())/2, (box.zmin()+box.zmax())/2);

	qglviewer::Vec vmin(box.xmin(), box.ymin(), box.zmin());
	qglviewer::Vec vmax(box.xmax(), box.ymax(), box.zmax());

	vmin = center + (vmin - center) * 0.7;
	vmax = center + (vmax - center) * 0.7;

	setSceneBoundingBox(vmin, vmax);
	camera()->lookAt(sceneCenter());
	camera()->setUpVector(qglviewer::Vec(0, 0, 1));
	camera()->setViewDirection(qglviewer::Vec(0, 1, 0));
	showEntireScene();
} 

void Canvas_Miniview::mouseReleaseEvent(QMouseEvent *e)
{
	this->m_left_button_on_ = false;
	if (this->m_label_dragging_)
	{
		this->m_label_dragging_->hide();
	}

	if (this->m_right_button_on_ && this->m_label_dragging_ && this->m_mesh_!=NULL)
	{
		this->m_right_button_on_ = false;
// 		if (this->m_mainWindow_)
// 		{
// 			this->m_mainWindow_->load_obj(this->m_file_name_);
// 		}
		emit selected_obj(e->globalPos(), this->m_file_name_);
	}

	QGLViewer::mouseReleaseEvent(e);
	
};
bool Canvas_Miniview::do_read(const std::string& file_name, PolyMesh* mesh)
{
	std::ifstream input(file_name.c_str());
	Canvas_Miniview::do_read(input, mesh);
	return true;
};

void Canvas_Miniview::load_obj(QString obj_File)
{
	if (this->m_mesh_)
	{
		delete this->m_mesh_;
	}
	this->m_mesh_ = new PolyMesh;
	Canvas_Miniview::do_read(obj_File.toStdString(), m_mesh_);
	m_mesh_->set_color_by_normal();
	this->fitScreen();
	this->m_file_name_ = obj_File;
};
bool Canvas_Miniview::do_read(std::istream& input, PolyMesh* mesh)
{
	// Vertex index starts by 1 in obj format.
	while (!input.eof())
	{
		std::string line;
		getline(input, line);

		std::istringstream line_input(line);
		std::string keyword;
		line_input >> keyword;

		if (keyword == "v") { // read vertex coordinate
			Point3f v;
			line_input >> v;
			mesh->add_vertex(v);
		}
		else if (keyword == "vt") { // read texture coordinate of vertex
			Point2f t;
			line_input >> t;
			//tex_coords.push_back(t);
		}
		else if (keyword == "f") { // read facet's vertex indices
			std::vector<unsigned int> indices;
			while (!line_input.eof())
			{
				std::string s;
				line_input >> s;
				if (s.length() > 0) {
					std::istringstream face_input(s);
					int index;
					face_input >> index;
					indices.push_back(index - 1);

					char c;
					face_input >> c;
					if (c == '/') {
						face_input >> index;
						//tri.t_indices.push_back(index-1);
					}
				}
			}
			mesh->add_facet(indices);
		}
	}
	return true;
}
