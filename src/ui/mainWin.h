#ifndef MainWin_H
#define MainWin_H

#include <glew-1.11.0/include/GL/glew.h>
#include <QMainWindow>
#include <QGridLayout>
#include <QFileDialog>
#include <QDir>
#include <QGLContext>
#include <QThread>
#include "ui_mainWin.h"

class Viewer;
class Model;
class Coarse;
class Groundtruth;
class ImagePartAlg;
class FeatureGuided;

class MainWin : public QMainWindow, public Ui::MainWindow
{
    Q_OBJECT

public:
    MainWin();
    ~MainWin();

private slots:
    void loadModel();
    void snapShot();
    void loadS2ITransform();
    void fixCamera();
    void initLight();
    void checkVisibleVertices();
    void resetScreen();
    void updateLight();
    void computeNormal();
    void updateGeometry();
    void refreshScreen();
	void exportOBJ();
    void setOptParatoModel();
    void runAll();
    void renderTexture();
    void computeAll();
    void loadLightingBall();

signals:
    void callComputeInitLight(Coarse *, Viewer *);
    void callUpdateLight(Coarse *, Viewer *);
    void callComputeNormal(Coarse *, Viewer *);
	void callNLoptTest();
    void callRunWholeIter(Coarse *, Viewer *);
    void callComputeBRDFLightNormal(Coarse *, Viewer *);

public:
    //Viewer *viewer;
    //Viewer *viewer_img;
    Coarse *coarse_model;
    Groundtruth *gt_model;
    Model *lighting_ball;


    ImagePartAlg *img_part_alg;
    QThread *img_part_alg_thread;

    FeatureGuided* feature_guided;

    //QGridLayout *main_grid_layout;
};

#endif