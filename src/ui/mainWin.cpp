#include "Viewer.h"
#include "mainWin.h"
#include "Coarse.h"
#include "GroundTruth.h"
#include "ModelLight.h"
#include "I2SAlgorithms.h"

MainWin::MainWin()
{
    setupUi(this); 

    //viewer->makeCurrent();
    //viewer->setParent(centralwidget);
    //viewer_img->context()->create(viewer->context());

    //viewer = new Viewer(centralwidget);
    //viewer->setObjectName(QStringLiteral("viewer"));

    //viewer_img = new Viewer(centralwidget);
    //viewer_img->setObjectName(QStringLiteral("viewer_img"));
    ////viewer->setMinimumSize(QSize(800, 600));
    ////viewer->setMaximumSize(QSize(800, 600));

    //main_grid_layout = new QGridLayout(centralwidget);
    //setObjectName(QStringLiteral("main_grid_layout"));

    //main_grid_layout->addWidget(viewer, 0, 0, 1, 1);
    //main_grid_layout->addWidget(viewer_img, 0, 1, 1, 1);

    connect(action_Load_Model, SIGNAL(triggered()), this, SLOT(loadModel()));
    connect(action_Load_LightingBall, SIGNAL(triggered()), this, SLOT(loadLightingBall()));
    connect(action_Snap_Shot, SIGNAL(triggered()), this, SLOT(snapShot()));
    connect(action_Load_S2I_Transform, SIGNAL(triggered()), this, SLOT(loadS2ITransform()));
    connect(action_Fix_Camera, SIGNAL(triggered()), this, SLOT(fixCamera()));
    connect(action_Init_Light, SIGNAL(triggered()), this, SLOT(initLight()));
    connect(action_Check_Visible, SIGNAL(triggered()), this, SLOT(checkVisibleVertices()));
    connect(action_Reset_Screen, SIGNAL(triggered()), this, SLOT(resetScreen()));
    //connect(action_Update_Light, SIGNAL(triggered()), this, SLOT(updateLight()));
    connect(action_Compute_Normal, SIGNAL(triggered()), this, SLOT(computeNormal()));
    connect(action_Update_Geometry, SIGNAL(triggered()), this, SLOT(updateGeometry()));
	  connect(action_Export_OBJ, SIGNAL(triggered()), this, SLOT(exportOBJ()));
    connect(action_Render, SIGNAL(triggered()), this, SLOT(renderTexture()));
    connect(m_pushButton_set_para, SIGNAL(clicked()), this, SLOT(setOptParatoModel()));
    connect(m_pushButton_Run, SIGNAL(clicked()), this, SLOT(runAll()));
    connect(action_Compute_All, SIGNAL(triggered()), this, SLOT(computeAll()));

    this->show();

    coarse_model = nullptr;
    gt_model = nullptr;
    lighting_ball = nullptr;
    img_part_alg = new ImagePartAlg;
    img_part_alg_thread = new QThread;
    feature_guided = new FeatureGuided;

    connect(this, SIGNAL(callComputeInitLight(Coarse *, Viewer *)), img_part_alg, SLOT(computeInitLight(Coarse *, Viewer *)));
    connect(this, SIGNAL(callUpdateLight(Coarse *, Viewer *)), img_part_alg, SLOT(updateLight(Coarse *, Viewer *)));
    connect(this, SIGNAL(callComputeNormal(Coarse *, Viewer *)), img_part_alg, SLOT(computeNormal(Coarse *, Viewer *)));
    connect(this, SIGNAL(callRunWholeIter(Coarse *, Viewer *)), img_part_alg, SLOT(runWholeIter(Coarse *, Viewer *)));

    connect(this, SIGNAL(callComputeBRDFLightNormal(Coarse *, Viewer *)), img_part_alg, SLOT(solveRenderEqAll(Coarse *, Viewer *)));
    connect(img_part_alg, SIGNAL(refreshScreen()), this, SLOT(refreshScreen()));

    img_part_alg->moveToThread(img_part_alg_thread);

    img_part_alg_thread->start();
}

MainWin::~MainWin()
{
    delete img_part_alg;
    delete img_part_alg_thread;
    delete feature_guided;
}

void MainWin::loadModel()
{
    QString filter;
    filter = "obj file (*.obj)";

    QDir dir;
    QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
    if (fileName.isEmpty() == true) return;


    std::string model_file_path = fileName.toStdString();
    std::string model_file_name = model_file_path.substr(model_file_path.find_last_of('/') + 1);
    model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/'));
    int model_id = atoi(model_file_path.substr(model_file_path.find_last_of('/') + 1).c_str());
    model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/') + 1);

    if (coarse_model != nullptr) 
        delete coarse_model;
    coarse_model = new Coarse(model_id, model_file_path, model_file_name);

    viewer->getModel(coarse_model);
    viewer->resetCamera(coarse_model);
    coarse_model->setRenderer(viewer);
    viewer->setBackGroundImage(QString::fromStdString(
      model_file_path + std::to_string(model_id) + "/photo.png"));

    // make an output dir
    char time_postfix[50];
    time_t current_time = time(NULL);
    strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    std::string outptu_file_path = coarse_model->getDataPath() + "/output" + time_postfix;
    dir.mkpath(QString(outptu_file_path.c_str()));
    coarse_model->setOutputPath(outptu_file_path);

    setOptParatoModel();

    //coarse_model->exportPtRenderInfo(233);
    //coarse_model->drawFaceNormal();

    //if (gt_model == nullptr)
    //{
    //    gt_model = new Groundtruth(coarse_model);
    //    viewer_img->getModel(gt_model);
    //    gt_model->setRenderer(viewer_img);
    //}
    //coarse_model->setGtModelPtr(gt_model);
}

void MainWin::loadLightingBall()
{
    //QString filter;
    //filter = "obj file (*.obj)";

    //QDir dir;
    //QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
    //if (fileName.isEmpty() == true) return;


    //std::string model_file_path = fileName.toStdString();
    //std::string model_file_name = model_file_path.substr(model_file_path.find_last_of('/') + 1);
    //model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/'));
    //int model_id = atoi(model_file_path.substr(model_file_path.find_last_of('/') + 1).c_str());
    //model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/') + 1);

    //if (lighting_ball != nullptr) 
    //    delete lighting_ball;
    //lighting_ball = new Model(model_id, model_file_path, model_file_name);

    ////viewer->getModel(coarse_model);
    //lighting_ball->setRenderer(viewer_img);

    //lighting_ball->getModelLightObj()->getOutsideLight() = coarse_model->getLightRec();
    //lighting_ball->getModelLightObj()->getOutsideSampleMatrix() = coarse_model->getModelLightObj()->getSampleMatrix();

    //lighting_ball->computeBrightness();
    //viewer_img->getModel(lighting_ball);

}

void MainWin::exportOBJ()
{
	coarse_model->exportOBJ();

    //coarse_model->setInit();
}

void MainWin::snapShot()
{
    if (coarse_model)
        viewer->getSnapShot(coarse_model, true);

}

void MainWin::loadS2ITransform()
{
    if (coarse_model)
        coarse_model->loadS2ITransform();
}

void MainWin::fixCamera()
{
    viewer->fixCamera();
}

void MainWin::initLight()
{
    if (coarse_model)
    {
        emit callComputeInitLight(coarse_model, viewer);

        // update coarse_model's rho and color
        // pass new color to viewer to display

    }
}

void MainWin::checkVisibleVertices()
{
    viewer->checkVisibleVertices(coarse_model);
}

void MainWin::resetScreen()
{
	//emit callNLoptTest();

    viewer->resetScreen();
    viewer->getModel(coarse_model);
    viewer->setShowModel(true);
}

void MainWin::updateLight()
{
    //if (coarse_model)
    //{
    //    emit callUpdateLight(coarse_model, viewer);

    //    // update coarse_model's rho and color
    //    // pass new color to viewer to display

    //}

	//Eigen::VectorXf v_test(coarse_model->getModelLightObj()->getNumSamples());
	//viewer->checkVertexVisbs(0, coarse_model, v_test);

	//std::ofstream f_v(coarse_model->getDataPath() + "/first_visb_test.mat");
	//if (f_v)
	//{
	//	f_v << v_test;

	//	f_v.close();
	//}

	coarse_model->updateVertexRho();
    renderTexture();
}

void MainWin::computeNormal()
{
    if (coarse_model)
    {
        emit callComputeNormal(coarse_model, viewer);
    }
}

void MainWin::updateGeometry()
{
    if (coarse_model)
    {
        GeometryPartAlg geoAlg;
        geoAlg.updateWithExNormal(coarse_model);

        coarse_model->computeLight();
        viewer->getModel(coarse_model);
        this->refreshScreen();
    }
}

void MainWin::refreshScreen()
{
    viewer->UpdateGLOutside();
}

void MainWin::setOptParatoModel()
{
    int int_paras[3];
    
    int_paras[0] = m_spinBox_iter_num->value();
    int_paras[1] = m_spinBox_deform_iter->value();
    int_paras[2] = m_spinBox_cluster_num->value();


    double double_paras[13];

    double_paras[0] = m_spinBox_BRDF_Light_sfs->value();
    double_paras[1] = m_spinBox_Light_Reg->value();
    double_paras[2] = m_spinBox_cluster_smooth->value();
    double_paras[3] = m_spinBox_norm_sfs->value();
    double_paras[4] = m_spinBox_norm_smooth->value();
    double_paras[5] = m_spinBox_norm_normalized->value();
    double_paras[6] = m_spinBox_k_strech->value();
    double_paras[7] = m_spinBox_k_bend->value();
    double_paras[8] = m_spinBox_deform_normal->value();
    double_paras[9] = m_spinBox_vertical_move->value();
    double_paras[10] = m_spinBox_rho_smooth->value();
    double_paras[11] = m_spinBox_rho_s_r->value();
    double_paras[12] = m_spinBox_norm_prior->value();

    if (coarse_model)
        coarse_model->getParaObjPtr()->setOptParameter(3, int_paras, 13, double_paras);
}

void MainWin::runAll()
{
    if (coarse_model)
    {
        emit callRunWholeIter(coarse_model, viewer);
    }
}

void MainWin::renderTexture()
{
    viewer->resetScreen();
    viewer->getModelWithTexture(coarse_model, coarse_model->getRhoImg());
    viewer->setShowModel(true);
    viewer->UpdateGLOutside();

    viewer->setSnapshotFormat("PNG");

    char time_postfix[50];
    time_t current_time = time(NULL);
    strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    std::string file_time_postfix = time_postfix;

    viewer->saveSnapshot(QString((coarse_model->getOutputPath()+"/ren_img" + file_time_postfix + ".png").c_str()));
}

void MainWin::computeAll()
{
    //if (coarse_model)
    //{
    //    emit callComputeBRDFLightNormal(coarse_model, viewer);
    //}

    // test tele-reg

  //tele2d *teleRegister = new tele2d( 50, 0.02,1 ) ;

  //CURVES curves ;
  //std::vector<std::vector<int>> group ;
  //std::vector<int2> endps ;

	
  //teleRegister->load_Curves( "curves.txt", curves, group, endps );


  //teleRegister->init( curves, group, endps  ) ;


  ////// Uncomment the 3 lines below, you can directly run the registration and save the result.
  ////teleRegister->runRegister() ;
  ////teleRegister->outputResCurves( "rescurves.txt") ;
  ////return 0;



  //teleRegister->setInputField() ; // only for visualization

  QString filter;
  filter = "image file (*.png)";

  QDir dir;
  QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
  if (fileName.isEmpty() == true) return;


  std::string fileSource = fileName.toStdString();
  std::string fileTarget = fileSource.substr(0, fileSource.find_last_of('/') + 1) + "featureP.png";
  feature_guided->initImages(fileSource, fileTarget);
  feature_guided->initRegister();
  feature_guided->initVisualization(viewer_img);
}