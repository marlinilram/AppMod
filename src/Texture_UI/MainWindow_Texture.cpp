#include "MainWindow_Texture.h"
#include "Texture_Label.h"
#include <QtWidgets/QHBoxLayout>
#include <QFileDialog>
#include <QDir>
#include "Texture_Viewer.h"
#include "Texture_Canvas.h"
#include "mini_texture.h"
#include "shape_item.h"
#include "shape_list.h"
#include "DispModuleHandler.h"
#include "TexSynHandler.h"

MainWindow_Texture::MainWindow_Texture(QWidget * parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
{
	this->m_num_layer_ = 3;
	this->m_num_each_layer_ = 3;
	this->m_start_num_ = 0;
	this->m_dlg_ = NULL;
	this->m_shape_list_ = NULL;
	this->setupUi(this);
	this->set_up_ui_texture();
	this->connect_singal();
	this->dockWidget_Texture_Brow->setGeometry(100, 100, 500, 500);
	this->m_viewer_ = NULL;
	this->set_up_viewer();
	this->m_mini_selected_ = NULL;

  this->tex_syn_handler.reset(new TexSynHandler);
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
}
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
	this->m_mini_selected_ = NULL;
};
void MainWindow_Texture::texture_select(MiniTexture* minit)
{
	this->m_mini_selected_ = minit;
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
	this->m_viewer_->show();

	Texture_Canvas* tc = new Texture_Canvas();
	tc->setModel(NULL);
	this->m_viewer_->addDispObj(tc);
};
void MainWindow_Texture::connect_singal()
{
	connect(actionSet_Texture_Dir, SIGNAL(triggered()), this, SLOT(set_dir()));
	connect(actionShow_data_base, SIGNAL(triggered()), this, SLOT(show_data_base()));
	connect(actionData_dock, SIGNAL(triggered()), this, SLOT(set_data_dock()));
	connect(actionLoad_Obj, SIGNAL(triggered()), this, SLOT(load_obj()));

	connect(this->verticalScrollBar_Texture, SIGNAL(valueChanged(int)), this, SLOT(images_update(int)));
  connect(actionRun_d1_synthesis, SIGNAL(triggered()), this, SLOT(run_d1_synthesis()));
  connect(actionRun_d0_synthesis, SIGNAL(triggered()), this, SLOT(run_d0_synthesis()));
};

void MainWindow_Texture::load_obj()
{
	QString filter;
	filter = "obj file (*.obj)";

	QDir dir;
	QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
	if (fileName.isEmpty() == true) return;

	std::string model_file_path = fileName.toStdString();
	std::string model_file_name = model_file_path.substr(model_file_path.find_last_of('/') + 1);
	model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/'));
	std::shared_ptr<Model> m ( new Model(model_file_path, model_file_name) );

	this->m_viewer_->get_dispObjects()[0]->setModel(m);
	//this->m_viewer_->get_dispObjects()[0]->
	this->m_viewer_->get_dispObjects()[0]->updateModelBuffer();

  this->tex_syn_handler->setSynthesisModel(m);

	this->shape_list_prepare();
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
		m_dlg_->spinBox_Height->setValue(m_num_layer_);
		m_dlg_->spinBox_Width->setValue(m_num_each_layer_);
	}
	if (m_dlg_->exec())
	{
		int w = m_dlg_->spinBox_Width->value();
		int h = m_dlg_->spinBox_Height->value();

		this->m_num_layer_ = h;
		this->m_num_each_layer_ = w;

		this->m_start_num_ = 0;
		QRect r = this->verticalLayout_Texture_Brow->geometry();
		this->verticalLayout_Texture_Brow->setGeometry(QRect(0, 0, this->dockWidget_Texture_Brow->width(), this->dockWidget_Texture_Brow->height()));
		QRect rr = this->verticalLayout_Texture_Brow->geometry();
		this->set_up_ui_texture();
		this->images_update(this->m_start_num_);
	}
}

void MainWindow_Texture::set_up_ui_texture()
{
	for (unsigned int i = 0; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i]->clear();
		this->m_image_labels_[i]->setParent(NULL);
		delete this->m_image_labels_[i];
	}
	for (unsigned int i = 0; i < this->m_horizontal_layouts_.size(); i++)
	{
		this->verticalLayout_Texture_Brow->removeItem(this->m_horizontal_layouts_[i]); 
		this->m_horizontal_layouts_[i]->setParent(NULL);
		delete this->m_horizontal_layouts_[i];
	}
	this->m_horizontal_layouts_.clear();
	this->m_image_labels_.clear();

	for (unsigned int i = 0; i < m_num_layer_; i++)
	{
		QHBoxLayout *horizontalLayout = new QHBoxLayout(this);
		for (unsigned int j = 0; j < m_num_each_layer_; j++)
		{
			Texture_Label* l = new Texture_Label(this);
 			horizontalLayout->addWidget(l);
			this->m_image_labels_.push_back(l);
			connect(l, SIGNAL(cut_selected(MiniTexture*)), this, SLOT(texture_select(MiniTexture*)));
		}
		this->verticalLayout_Texture_Brow->addLayout(horizontalLayout);
	}
	this->verticalScrollBar_Texture->setSingleStep(m_num_layer_ * m_num_each_layer_);
	this->verticalScrollBar_Texture->setPageStep(m_num_layer_ * m_num_each_layer_);
};
void MainWindow_Texture::show_data_base()
{
	this->dockWidget_Texture_Brow->show();
	this->dockWidget_Shape_Texture->show();
}
void MainWindow_Texture::set_dir()
{
	QString file_folder = QFileDialog::getExistingDirectory(this, "set directory", ".");
	if (file_folder.isEmpty())
	{
		return;
	}
	this->m_files_.clear();
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
			this->m_files_.push_back(file_folder + "/" + ds[i] + "/" + files[0]);
		}
	};
	int max_num = this->m_files_.size() - m_num_layer_ * m_num_each_layer_;
	if (max_num < 0)
	{
		max_num = 0;
	}
	this->verticalScrollBar_Texture->setMaximum(max_num);
	this->verticalScrollBar_Texture->setSingleStep(m_num_layer_ * m_num_each_layer_);
	this->verticalScrollBar_Texture->setPageStep(m_num_layer_ * m_num_each_layer_);
	this->verticalScrollBar_Texture->setValue(0);
	this->m_start_num_ = 0;
	this->images_update(this->m_start_num_);
};

void MainWindow_Texture::images_update(int from)
{
	for (unsigned int i = 0; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i]->clear();
	}

	int index_from = 0;
	for (unsigned int i = from; (i < this->m_files_.size()) && (i < from + m_num_layer_ * m_num_each_layer_); i++)
	{
		this->m_image_labels_[i-from]->set_file(this->m_files_[i]);
		this->m_image_labels_[i - from]->setEnabled(true);
		index_from++;
	}

	for (unsigned int i = index_from; i < this->m_image_labels_.size(); i++)
	{
		this->m_image_labels_[i - from]->setEnabled(false);
	}

};

void MainWindow_Texture::run_d1_synthesis()
{
  this->tex_syn_handler->runD1Synthesis(this->m_shape_list_->getTexturePath(0));
};

void MainWindow_Texture::run_d0_synthesis()
{
  this->tex_syn_handler->runD0Synthesis(this->m_shape_list_->getTexturePath(0));
}