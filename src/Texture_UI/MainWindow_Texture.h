#ifndef MainWindow_Texture_H
#define MainWindow_Texture_H

#include <glew-1.11.0/include/GL/glew.h>
#include <QMainWindow>
#include "ui_MainWindow_Texture.h"
#include "Dialog_Database.h"
#include <memory>
#include <vector>
#include <string>
class QListWidgetItem;
class Texture_Label;
class QHBoxLayout;
class Texture_Viewer;
class MiniTexture;
class ShapeList;
class TexSynHandler;

class MainWindow_Texture : public QMainWindow, public Ui::MainWindow_Texture
{
    Q_OBJECT

public:
	MainWindow_Texture(QWidget * parent = 0, Qt::WindowFlags flags = 0);
	~MainWindow_Texture();

private:
	void connect_singal();
	void set_up_viewer();
	void shape_list_prepare();
private slots:
	void show_data_base();
	void set_dir();
	void images_update(int from);
	void set_data_dock();
	void load_obj();
	void texture_select(MiniTexture*);
	void item_double_clicked(QListWidgetItem *);
   
	void run_d1_synthesis();
	void run_d0_synthesis();
	void selec_area(bool);

//	QMainWindow* new_viewer_for_result_model(std::string file_path);
private:
	void set_up_ui_texture();
	int m_num_layer_;
	int m_num_each_layer_;
	std::vector<Texture_Label*> m_image_labels_;
	std::vector<QHBoxLayout*> m_horizontal_layouts_;

	std::vector<QString>		m_files_;
	int m_start_num_;
	Dialog_Database* m_dlg_;
	Texture_Viewer*	 m_viewer_;
	ShapeList* m_shape_list_;

	MiniTexture* m_mini_selected_;

   std::shared_ptr<TexSynHandler> tex_syn_handler;
   std::vector<QMainWindow*> m_viewer_for_result_;

};

#endif