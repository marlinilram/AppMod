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

class MainCanvas;
class TrackballCanvas;
class FeatureGuided;

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
    void setFeatureRender(int state);

public:
    //Viewer *viewer;
    //Viewer *viewer_img;
    std::shared_ptr<MainCanvas> main_canvas;
    std::shared_ptr<TrackballCanvas> trackball_canvas;

    //ImagePartAlg *img_part_alg;
    //QThread *img_part_alg_thread;

    //QGridLayout *main_grid_layout;

	//bool pointsSelect;
	//bool isCompute;

};

#endif