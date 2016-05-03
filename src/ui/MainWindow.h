#ifndef MainWindow_H
#define MainWindow_H

#include <glew-1.11.0/include/GL/glew.h>
#include <QMainWindow>
#include <QGridLayout>
#include <QFileDialog>
#include <QDir>
#include <QGLContext>
#include <QThread>
#include "ui_MainWindow.h"

#include <memory>

class ParameterDock;
class DispModuleHandler;
class MainWindow_Texture;
class MainWindow : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MainWindow();
    ~MainWindow();

private slots:
    void loadModel();
    void snapShot();
    void fixCamera();
    void resetScreen();
    void updateGeometry();
    void refreshScreen();
	  void exportOBJ();
    void setOptParatoModel();
    void runAll();
    void renderTexture();
    void setVectorField();
	  void clearSelectedPoints();
	  void computeCameraPose();
	  void loadPoints();
    void deleteLastLine_Source();
    void deleteLastLine_Target();
    void showToolBox();
    void loadSynthesisTarget();
	void draw_feature(bool);
	void clear_drawn_feature();
	void show_texture_window(bool);
private:
	MainWindow_Texture *m_mainwindow_texture_;
    //Viewer *viewer;
    //Viewer *viewer_img;
    std::shared_ptr<ParameterDock> parameter_dock;

    std::shared_ptr<DispModuleHandler> disp_modules;


    //ImagePartAlg *img_part_alg;
    //QThread *img_part_alg_thread;

    //QGridLayout *main_grid_layout;

	//bool pointsSelect;
	//bool isCompute;

};

#endif