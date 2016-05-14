#include "Texture_Viewer.h"
#include "Texture_Canvas.h"

#include "MainWindow_Texture.h"
#include "canvas_miniview.h"
#include "Texture_Label.h"
#include <QtWidgets/QHBoxLayout>
#include <QFileDialog>
#include <QDir>
#include "mini_texture.h"
#include "shape_item.h"
#include "shape_list.h"
#include "DispModuleHandler.h"
#include "TexSynHandler.h"
#include <QMessageBox>
#include "ParameterMgr.h"
#include "texture_points_corres.h"
#include "ShapeUtility.h"
#include "PolygonMesh.h"
#include "ParaShape.h"
MainWindow_Texture::MainWindow_Texture(QWidget * parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
{
	this->m_num_layer_texture_ = 5;
	this->m_num_each_layer_texture_ = 1;
	this->m_start_num_texture_ = 0;

	this->m_num_layer_objs_ = 5;
	this->m_num_each_layer_objs_ = 1;
	this->m_start_num_objs_ = 0;

	this->m_dlg_ = NULL;
	this->m_shape_list_ = NULL;
	this->setupUi(this);
	this->set_up_ui_texture();
	this->set_up_ui_objs();

	this->dockWidget_Texture_Brow->setGeometry(100, 100, 500, 500);
	this->m_viewer_ = NULL;
	this->set_up_viewer();
	this->m_mini_selected_ = NULL;
	this->tex_syn_handler.reset(new TexSynHandler);

	this->setWindowIcon(QIcon("icons/texture_ui.png"));
	this->connect_singal();
}
MainWindow_Texture::~MainWindow_Texture()
{
	if (this->m_dlg_ != NULL)
	{
		delete m_dlg_;
	}
	if (this->m_mini_selected_)
	{
		delete this->m_mini_selected_;
	}

	for (unsigned int i = 0; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i]->clear();
		this->m_image_labels_[i]->setParent(NULL);
		delete this->m_image_labels_[i];
	}

	for (unsigned int i = 0; i < this->m_miniviewers_.size(); i++)
	{
		this->m_miniviewers_[i]->setParent(NULL);
		delete this->m_miniviewers_[i];
	}
}

void MainWindow_Texture::selec_area( bool b)
{
	if (b)
	{
		this->m_viewer_->set_edit_mode(0);
		
	}
	else
	{
		this->m_viewer_->set_edit_mode(-1);
	}
	
};

void MainWindow_Texture::clear_select()
{
	this->m_viewer_->clear_selection();
};

void MainWindow_Texture::item_double_clicked(QListWidgetItem* it)
{
	if (this->m_mini_selected_ == NULL)
	{
		return;
	}
	ShapeItem* item = dynamic_cast<ShapeItem*>(it);
	MiniTexture* m = item->get_texture();
	this->m_shape_list_->set_texture(item, this->m_mini_selected_);
	if (m != NULL)
	{
		delete m;
	}
	this->m_mini_selected_->hide();
	connect(this->m_mini_selected_, SIGNAL(mask_selected_d0()), this, SLOT(mask_d0_select()));
	connect(this->m_mini_selected_, SIGNAL(mask_selected_d1()), this, SLOT(mask_d1_select()));

	this->m_mini_selected_ = NULL;
};

void MainWindow_Texture::texture_select(MiniTexture* minit)
{

	if (this->m_shape_list_ != NULL)
	{
		ShapeItem* item = dynamic_cast<ShapeItem*>(this->m_shape_list_->item(0));
		MiniTexture* m = item->get_texture();
		if (m != NULL)
		{
			m->setParent(NULL);
			delete m;
		}
		this->m_shape_list_->set_texture(item, minit);
	}

	this->m_mini_selected_ = minit;
	m_mini_selected_->setParent(this->m_viewer_);
	connect(this->m_mini_selected_, SIGNAL(mask_selected_d0()), this, SLOT(mask_d0_select()));
	connect(this->m_mini_selected_, SIGNAL(mask_selected_d1()), this, SLOT(mask_d1_select()));
	this->m_mini_selected_->setGeometry(this->m_viewer_->width() - m_mini_selected_->width(), this->m_viewer_->height() - m_mini_selected_->height(), m_mini_selected_->width(), m_mini_selected_->height());
	m_mini_selected_->show();
	this->m_viewer_->set_texture_now(m_mini_selected_);

};
void MainWindow_Texture::resizeEvent(QResizeEvent * event)
{
	QMainWindow::resizeEvent(event);
	if (m_mini_selected_)
	{
		this->m_mini_selected_->setGeometry(this->m_viewer_->width() - m_mini_selected_->width(), this->m_viewer_->height() - m_mini_selected_->height(), m_mini_selected_->width(), m_mini_selected_->height());

	}
	
};
void MainWindow_Texture::set_up_viewer()
{
	if (this->m_viewer_ == NULL)
	{
		this->m_viewer_ = new Texture_Viewer(this);
	}
	this->m_viewer_->resize(this->width(), this->height());
	//this->m_viewer_->setBackgroundColor(QColor(0,0,0));
	this->setCentralWidget(this->m_viewer_);
	this->m_viewer_->setAttribute(Qt::WA_MouseTracking);

	Texture_Canvas* tc = new Texture_Canvas();
	tc->setModel(NULL);

	

	this->m_viewer_->addDispObj(tc);
	tc->setsize(this->width(), this->height());
	this->m_viewer_->show();



};
void MainWindow_Texture::connect_singal()
{
	connect(actionSet_Texture_Dir, SIGNAL(triggered()), this, SLOT(set_texture_dir()));
	connect(actionSet_obj_dir, SIGNAL(triggered()), this, SLOT(set_objs_dir()));
	
	connect(actionShow_data_base, SIGNAL(triggered()), this, SLOT(show_data_base()));
	connect(actionData_dock, SIGNAL(triggered()), this, SLOT(set_data_dock()));
	connect(actionLoad_Obj, SIGNAL(triggered()), this, SLOT(load_obj()));

	connect(this->verticalScrollBar_Texture, SIGNAL(valueChanged(int)), this, SLOT(images_update(int)));
	connect(this->verticalScrollBar_Objs, SIGNAL(valueChanged(int)), this, SLOT(objs_update(int)));
	
    connect(actionRun_d1_synthesis, SIGNAL(triggered()), this, SLOT(run_d1_synthesis()));
	connect(actionRun_d0_synthesis, SIGNAL(triggered()), this, SLOT(run_d0_synthesis()));
	connect(actionRun_d0_all, SIGNAL(triggered()), this, SLOT(run_d0_synthesis_all()));
	actionRun_d0_all->setIcon(QIcon("icons/run_d0_all.jpg"));


	QActionGroup* toolActionGroup = new QActionGroup(this);

	toolActionGroup->addAction(actionSceneManipulation);
	actionSceneManipulation->setIcon(QIcon("icons/scene_manipulation.png"));

	toolActionGroup->addAction(actionArea_Select);
	actionArea_Select->setIcon(QIcon("icons/select_points.png"));

	toolActionGroup->addAction(actionArea_Select_Only_Visible);
	actionArea_Select_Only_Visible->setIcon(QIcon("icons/select_points_visible.png"));
	connect(toolActionGroup, SIGNAL(triggered(QAction*)), this, SLOT(operationModeChanged(QAction*)));
	
	connect(actionClear_Select, SIGNAL(triggered()), this, SLOT(clear_select()));

	connect(actionSelect_all_unselected, SIGNAL(triggered()), this->m_viewer_, SLOT(select_all_unselected()));
	connect(actionShow_line, SIGNAL(toggled(bool)), this->m_viewer_, SLOT(Show_line(bool)));
	connect(actionShow_mini_texture, SIGNAL(toggled(bool)), this->m_viewer_, SLOT(show_mini_texture(bool)));
};


void MainWindow_Texture::operationModeChanged(QAction* act)
{
	if (act == actionSceneManipulation)
	{
		this->m_viewer_->set_edit_mode(-1);
	}
	else if (act == actionArea_Select_Only_Visible)
	{
		this->m_viewer_->set_edit_mode(0);
	}
	else if (act == actionArea_Select)
	{
		this->m_viewer_->set_edit_mode(1);
	}
};
void MainWindow_Texture::load_obj(QString fileName_tmp)
{

//  	QString filter;
//  	filter = "obj file (*.obj)";
//  	QString sssss = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), fileName_tmp, filter);
// 	if (sssss.isEmpty())
// 	{
// 		return;
// 	}
// 	if (QMessageBox::question(this, "Open new Obj", "Open a new OBJ file?") == QMessageBox::No)
// 	{
// 		return;
// 	};



	QString fileName = fileName_tmp;

	if (fileName.isEmpty() == true) return;

	std::string model_file_path = fileName.toStdString();
	std::string model_file_name = model_file_path.substr(model_file_path.find_last_of('/') + 1);
	model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/'));
	std::shared_ptr<Model> m(new Model(model_file_path, model_file_name));

	static_cast<Texture_Canvas*>(this->m_viewer_->get_dispObjects()[0])->setModel(m);
	this->m_viewer_->clear_selection();
	this->m_viewer_->mark_points_out();

	this->tex_syn_handler->setSynthesisModel(m);
	this->shape_list_prepare();
	this->m_viewer_->resetCamera();

  this->m_viewer_->setGLActors(this->tex_syn_handler->getGLActors());
};

void MainWindow_Texture::load_obj()
{
	QString filter;
	filter = "obj file (*.obj)";

	QDir dir;
	QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
	if (fileName.isEmpty() == true) return;
	this->load_obj(fileName);
};
void MainWindow_Texture::shape_list_prepare()
{
	if (this->m_shape_list_ != NULL)
	{
		this->m_shape_list_->setParent(NULL);
		this->m_shape_list_->clear();
		delete this->m_shape_list_;
		this->m_shape_list_ = NULL;
	}
	std::shared_ptr<Model> m = this->m_viewer_->get_dispObjects()[0]->getModel();
	if (m == NULL)
	{  return;	}

	std::vector<Shape*> shapes;
	m->getShapeVector(shapes);
	if (shapes.size() < 1)
	{	return;	}

	if (this->m_shape_list_ == NULL)
	{
		this->m_shape_list_ = new ShapeList(NULL);
		connect(m_shape_list_, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(item_double_clicked(QListWidgetItem *)));
	}

	for (unsigned int i = 0; i < shapes.size(); i++)
	{
		ShapeItem* sp = new ShapeItem(this->m_shape_list_);
		sp->set_shape(shapes[i]);
		sp->setText("shape :: "+QString::number(i));
		QListWidgetItem* lw = new QListWidgetItem(this->m_shape_list_->texture_list());
		lw->setText("Texture not setted!");
		this->m_shape_list_->texture_list()->addItem(lw);
		this->m_shape_list_->addItem(sp);
		sp->setBackground(QBrush(QColor(255, 0, 0)));
		lw->setBackground(QBrush(QColor(255, 0, 0)));
	}
	this->m_shape_list_->setParent(this->dockWidget_Shape_Texture);
	this->m_shape_list_->setGeometry(10, 10, 200, 100);
	this->m_shape_list_->show();
};
void MainWindow_Texture::set_data_dock()
{
	if (this->m_dlg_ == NULL)
	{
		m_dlg_ = new Dialog_Database(0,0);
		m_dlg_->spinBox_Height->setValue(m_num_layer_texture_);
		m_dlg_->spinBox_Width->setValue(m_num_each_layer_texture_);
	}
	if (m_dlg_->exec())
	{
		int w = m_dlg_->spinBox_Width->value();
		int h = m_dlg_->spinBox_Height->value();

		this->m_num_layer_texture_ = h;
		this->m_num_each_layer_texture_ = w;

		this->m_start_num_texture_ = 0;
		QRect r = this->verticalLayout_Texture_Brow->geometry();
		this->verticalLayout_Texture_Brow->setGeometry(QRect(0, 0, this->dockWidget_Texture_Brow->width(), this->dockWidget_Texture_Brow->height()));
		QRect rr = this->verticalLayout_Texture_Brow->geometry();
		this->set_up_ui_texture();
		this->images_update(this->m_start_num_texture_);
	}
}
void MainWindow_Texture::set_up_ui_objs()
{
	for (unsigned int i = 0; i < this->m_miniviewers_.size(); i++)
	{
		this->m_miniviewers_[i]->clear();
		this->m_miniviewers_[i]->setParent(NULL);
		delete this->m_miniviewers_[i];
	}
	for (unsigned int i = 0; i < this->m_horizontal_layouts_objs_.size(); i++)
	{
		this->verticalLayout_Obj_Brow->removeItem(this->m_horizontal_layouts_objs_[i]);
		this->m_horizontal_layouts_objs_[i]->setParent(NULL);
		delete this->m_horizontal_layouts_objs_[i];
	}
	this->m_horizontal_layouts_objs_.clear();
	this->m_miniviewers_.clear();

	QGLFormat format = QGLFormat::defaultFormat();
	format.setSampleBuffers(true);
	format.setSamples(8);
	for (unsigned int i = 0; i < m_num_layer_objs_; i++)
	{
		QHBoxLayout *horizontalLayout = new QHBoxLayout(this);
		for (unsigned int j = 0; j < m_num_each_layer_objs_; j++)
		{
			Canvas_Miniview* l = new Canvas_Miniview(format, this, NULL);
			horizontalLayout->addWidget(l);
			this->m_miniviewers_.push_back(l);
			l->set_mainwindow(this);
			connect(l, SIGNAL(selected_obj(QPoint, QString)), this, SLOT(selected_obj(QPoint, QString)));
		}
		this->verticalLayout_Obj_Brow->addLayout(horizontalLayout);
	}
	this->verticalScrollBar_Objs->setSingleStep(m_num_layer_objs_ * m_num_each_layer_objs_);
	this->verticalScrollBar_Objs->setPageStep(m_num_layer_objs_ * m_num_each_layer_objs_);
};
void MainWindow_Texture::selected_obj(QPoint global_2D_pos, QString s)
{

	QPoint pixel = this->mapFromGlobal(global_2D_pos);
	QRect rc = this->m_viewer_->geometry();

	if (! rc.contains(pixel))
	{
		return;
	}
	this->load_obj(s);
	
};

void MainWindow_Texture::set_up_ui_texture()
{
	for (unsigned int i = 0; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i]->clear();
		this->m_image_labels_[i]->setParent(NULL);
		delete this->m_image_labels_[i];
	}
	for (unsigned int i = 0; i < this->m_horizontal_layouts_textures_.size(); i++)
	{
		this->verticalLayout_Texture_Brow->removeItem(this->m_horizontal_layouts_textures_[i]);
		this->m_horizontal_layouts_textures_[i]->setParent(NULL);
		delete this->m_horizontal_layouts_textures_[i];
	}
	this->m_horizontal_layouts_textures_.clear();
	this->m_image_labels_.clear();

	for (unsigned int i = 0; i < m_num_layer_texture_; i++)
	{
		QHBoxLayout *horizontalLayout = new QHBoxLayout(this);
		for (unsigned int j = 0; j < m_num_each_layer_texture_; j++)
		{
			Texture_Label* l = new Texture_Label(this);
 			horizontalLayout->addWidget(l);
			this->m_image_labels_.push_back(l);
			connect(l, SIGNAL(cut_selected(MiniTexture*)), this, SLOT(texture_select(MiniTexture*)));
		}
		this->verticalLayout_Texture_Brow->addLayout(horizontalLayout);
	}
	this->verticalScrollBar_Texture->setSingleStep(m_num_layer_texture_ * m_num_each_layer_texture_);
	this->verticalScrollBar_Texture->setPageStep(m_num_layer_texture_ * m_num_each_layer_texture_);
};
void MainWindow_Texture::show_data_base()
{
	this->dockWidget_Texture_Brow->show();
	this->dockWidget_Shape_Texture->show();
}
void MainWindow_Texture::set_objs_dir()
{
	QString file_folder = QFileDialog::getExistingDirectory(this, "set directory", ".");
	if (file_folder.isEmpty())
	{
		return;
	}
	this->m_objs_files_.clear();
	QDir d(file_folder);
	QStringList ds = d.entryList(QDir::Dirs | QDir::NoDot | QDir::NoDotDot, QDir::Name);

	for (unsigned int i = 0; i < ds.size(); i++)
	{
		QString dir_sub_s = file_folder + "/" + ds[i];
		QDir  dir_sub(dir_sub_s);

		QStringList filter;
		filter.push_back("*.obj");
		QStringList files = dir_sub.entryList(filter, QDir::Files, QDir::Name);
		if (files.size() > 0)
		{
			this->m_objs_files_.push_back(file_folder + "/" + ds[i] + "/" + files[0]);
		}
	};
	int max_num = this->m_objs_files_.size() - m_num_layer_objs_ * m_num_each_layer_objs_;
	if (max_num < 0)
	{
		max_num = 0;
	}
	this->verticalScrollBar_Objs->setMaximum(max_num);
	this->verticalScrollBar_Objs->setSingleStep(m_num_layer_objs_ * m_num_each_layer_objs_);
	this->verticalScrollBar_Objs->setPageStep(m_num_layer_objs_ * m_num_each_layer_objs_);
	this->verticalScrollBar_Objs->setValue(0);
	this->m_start_num_objs_ = 0;
	this->objs_update(this->m_start_num_objs_);
};


void MainWindow_Texture::set_texture_dir()
{
	QString file_folder = QFileDialog::getExistingDirectory(this, "set directory", ".");
	if (file_folder.isEmpty())
	{
		return;
	}
	this->m_texture_files_.clear();
	QDir d(file_folder); 
	QStringList ds = d.entryList(QDir::Dirs | QDir::NoDot | QDir::NoDotDot, QDir::Name);

	for (unsigned int i  =0; i < ds.size(); i++)
	{
		QString dir_sub_s = file_folder + "/" + ds[i];
		QDir  dir_sub(dir_sub_s);
		
		QStringList filter;
		filter.push_back("*.png");
		QStringList files = dir_sub.entryList(filter, QDir::Files, QDir::Name);
		if (files.size() > 0)
		{
			this->m_texture_files_.push_back(file_folder + "/" + ds[i] + "/" + files[0]);
		}
	};
	int max_num = this->m_texture_files_.size() - m_num_layer_texture_ * m_num_each_layer_texture_;
	if (max_num < 0)
	{
		max_num = 0;
	}
	this->verticalScrollBar_Texture->setMaximum(max_num);
	this->verticalScrollBar_Texture->setSingleStep(m_num_layer_texture_ * m_num_each_layer_texture_);
	this->verticalScrollBar_Texture->setPageStep(m_num_layer_texture_ * m_num_each_layer_texture_);
	this->verticalScrollBar_Texture->setValue(0);
	this->m_start_num_texture_ = 0;
	this->images_update(this->m_start_num_texture_);
};
void MainWindow_Texture::objs_update(int from)
{
	for (unsigned int i = 0; i < this->m_miniviewers_.size(); i++)
	{
		this->m_miniviewers_[i]->clear();
	}

	int index_from = 0;
	for (unsigned int i = from; (i < this->m_objs_files_.size()) && (i < from + m_num_layer_objs_ * m_num_each_layer_objs_); i++)
	{
		this->m_miniviewers_[i - from]->load_obj(this->m_objs_files_[i]);
		this->m_miniviewers_[i - from]->setEnabled(true);
		index_from++;
	}

	for (unsigned int i = index_from; i < this->m_miniviewers_.size(); i++)
	{
		this->m_miniviewers_[i - from]->setEnabled(false);
	}
};
void MainWindow_Texture::images_update(int from)
{
	for (unsigned int i = 0; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i]->clear();
	}

	int index_from = 0;
	for (unsigned int i = from; (i < this->m_texture_files_.size()) && (i < from + m_num_layer_texture_ * m_num_each_layer_texture_); i++)
	{
		this->m_image_labels_[i - from]->set_file(this->m_texture_files_[i]);
		this->m_image_labels_[i - from]->setEnabled(true);
		index_from++;
	}

	for (unsigned int i = index_from; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i - from]->setEnabled(false);
	}

};
void MainWindow_Texture::mask_d1_select()
{
// 	MiniTexture* mini = this->m_shape_list_->get_mini_texture(0);
// 	if (mini == NULL)
// 	{
// 		return;
// 	}
// 
// 	cv::Mat mask = mini->get_mask_d1();
// 	if (mask.cols < 100)
// 	{
// 		return;
// 	}
// 	IplImage iplImg = IplImage(mask);
// 	cvShowImage("mask", &iplImg);
// 
// 	this->tex_syn_handler->runD1Synthesis(this->m_shape_list_->getTexturePath(0));
};

void MainWindow_Texture::run_d1_synthesis()
{

//   MiniTexture* mini = this->m_shape_list_->get_mini_texture(0);
//   if (mini == NULL)
//   {
// 	  return;
//   }
// 
//   if (this->m_viewer_->get_boundaries().size() < 1)
//   {
// 	  QMessageBox::warning(this, "No target mask", "Please draw target mask first!");
// 	  return;
//   }
//   /* MiniTextureThread* minit = new MiniTextureThread(NULL, mini);*/
//   //minit->setParent(this);
//   mini->load_texture();
//   mini->set_mask_d1(cv::Mat(0, 0, CV_32FC1, 1));
//   mini->show_mesh_image_d1();
//   mini->hide();
//   mini->show();

  const std::vector<Texture_Mesh_Corres*>& tmsss = this->m_viewer_->get_textures_mesh_corres();
/*  std::string ssss;*/
  for (unsigned int i = 0; i < tmsss.size(); i++)
  {
    QString file_name = tmsss[i]->file_path();
    std::string std_file_path = file_name.toStdString().substr(0, file_name.toStdString().find_last_of('/'));
    cv::Mat mask_origin_source = tmsss[i]->get_mask_source();
    cv::Mat mask_target = tmsss[i]->get_mask_target();


    LG::GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:SrcAppOriginImageMask") = mask_origin_source.clone();
    LG::GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:TarAppMask") = mask_target.clone();
     this->tex_syn_handler->runD1Synthesis(std_file_path);
  }

  std::string ssss = this->apply_d1_displacement();

  QString meshlab = "C:&& cd C:/Program Files/VCG/MeshLab/&& start meshlab.exe  ";
  meshlab = meshlab + "\"" + QString::fromStdString(ssss) + "\" &";

  std::cout << meshlab.toStdString() << "\n";
  system(meshlab.toStdString().data());

};
void MainWindow_Texture::run_d0_synthesis_all()
{
	const std::vector<Texture_Mesh_Corres*>& tmsss = this->m_viewer_->get_textures_mesh_corres();


	std::string ssss;
	for (unsigned int i = 0; i < tmsss.size(); i++)
	{
		QString file_name = tmsss[i]->file_path();
		std::string std_file_path = file_name.toStdString().substr(0, file_name.toStdString().find_last_of('/'));
		cv::Mat mask_origin_source = tmsss[i]->get_mask_source();
		cv::Mat mask_target = tmsss[i]->get_mask_target();


		LG::GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:SrcAppOriginImageMask") = mask_origin_source.clone();
		LG::GlobalParameterMgr::GetInstance()->get_parameter<cv::Mat>("Synthesis:TarAppMask") = mask_target.clone();
		ssss = this->tex_syn_handler->runD0Synthesis(std_file_path);
	}


// 	 QString meshlab = "\"C:/Program Files (x86)/VCG/MeshLab/meshlab.exe\" " ;
// 	 meshlab = meshlab + "\"" + QString::fromStdString(ssss) + "\"";

	QString meshlab = "C:&& cd C:/Program Files/VCG/MeshLab/&& start meshlab.exe  ";
	meshlab = meshlab + "\"" + QString::fromStdString(ssss) + "\" &";

	   std::cout << meshlab.toStdString() << "\n";
	   system(meshlab.toStdString().data());
};

void MainWindow_Texture::mask_d0_select()
{

	MiniTexture* mini = this->m_shape_list_->get_mini_texture(0);

	//MiniTexture* mini = this->m_mini_selected_;
	if (mini == NULL)
	{
		return;
	}

	std::vector<int> face_selected = this->m_viewer_->face_selected();

	if (face_selected.size() < 1)
	{
		QMessageBox::warning(this, "No target mask", "Please draw target mask first!");
		return;
	}




	cv::Mat mask = mini->get_mask_d0();
	if (mask.cols < 100)
	{
		return;
	}

 //	std::string s = this->tex_syn_handler->runD0Synthesis(this->m_shape_list_->getTexturePath(0));

	Texture_Mesh_Corres* ts = new Texture_Mesh_Corres(this->m_viewer_);
	ts->set_data(
		this->m_viewer_->get_dispObjects()[0]->getModel()->getPolygonMesh(), 
		face_selected,
		mini->get_masked_image_D0(), 
		mini->get_file_name(), 
		this->m_viewer_,
		mask,
		tex_syn_handler.get()
		);
	ts->set_origin_image(mini->get_masked_image_D0());
/*	ts->set_mesh_image(mini->get_mesh_image_D0());*/
/*	ts->set_masked_image(mini->get_masked_image_D0());*/
	ts->show_origin_image();
	this->m_viewer_->add_textures_mesh_corre(ts);
	this->m_viewer_->mark_points();
	this->m_viewer_->clear_selection();
	ts->show();
	
// 	GLOBAL::m_selected_faces_ = this->m_viewer_->get_boundaries()[0];

};
void MainWindow_Texture::run_d0_synthesis()
{
// 	MiniTexture* mini = this->m_shape_list_->get_mini_texture(0);
// 	if (mini == NULL)
// 	{
// 		return;
// 	}
// 	
// 	if (this->m_viewer_->get_boundaries().size() < 1)
// 	{
// 		QMessageBox::warning(this, "No target mask", "Please draw target mask first!");
// 		return;
// 	}
// 
// 	mini->load_texture();
// 	mini->set_mask_d0(cv::Mat(0, 0, CV_32FC1, 1));
// 	mini->show_mesh_image_d0();
// 	mini->hide();
// 	mini->show();

}

std::string MainWindow_Texture::apply_d1_displacement()
{
  // merge all tmasss together
  const std::vector<Texture_Mesh_Corres*>& tmsss = this->m_viewer_->get_textures_mesh_corres();
  if (tmsss.empty())
  {
    std::cout << "No mask for target!" << std::endl;
    return "";
  }
  cv::Mat mask_target = tmsss[0]->get_mask_target().clone();
  for (int i = 0; i < mask_target.rows; ++i)
  {
    for (int j = 0; j < mask_target.cols; ++j)
    {
      float sum = mask_target.at<float>(i, j);
      for (size_t k = 1; k < tmsss.size(); ++k)
      {
        sum += tmsss[k]->get_mask_target().at<float>(i, j);
      }
      if (sum > 0.5)
      {
        mask_target.at<float>(i, j) = 1.0;
      }
    }
  }

  /*cv::imshow("mask together", mask_target);*/

  return this->tex_syn_handler->applyD1Displacement(mask_target);
}