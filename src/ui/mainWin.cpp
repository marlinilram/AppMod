#include "mainWin.h"


MainWin::MainWin()
{
    setupUi(this); 

    //viewer->makeCurrent();
    //viewer->setParent(centralwidget);
    viewer_img->context()->create(viewer->context());

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
    connect(action_Snap_Shot, SIGNAL(triggered()), this, SLOT(snapShot()));
    connect(action_Load_S2I_Transform, SIGNAL(triggered()), this, SLOT(loadS2ITransform()));
    connect(action_Fix_Camera, SIGNAL(triggered()), this, SLOT(fixCamera()));
    connect(action_Init_Light, SIGNAL(triggered()), this, SLOT(initLight()));
    connect(action_Check_Visible, SIGNAL(triggered()), this, SLOT(checkVisibleVertices()));
    connect(action_Reset_Screen, SIGNAL(triggered()), this, SLOT(resetScreen()));
    connect(action_Update_Light, SIGNAL(triggered()), this, SLOT(updateLight()));
    connect(action_Compute_Normal, SIGNAL(triggered()), this, SLOT(computeNormal()));
    connect(action_Update_Geometry, SIGNAL(triggered()), this, SLOT(updateGeometry()));
	connect(action_Export_OBJ, SIGNAL(triggered()), this, SLOT(exportOBJ()));
    connect(action_Render, SIGNAL(triggered()), this, SLOT(renderTexture()));
    connect(m_pushButton_set_para, SIGNAL(clicked()), this, SLOT(setOptParatoModel()));
    connect(m_pushButton_Run, SIGNAL(clicked()), this, SLOT(runAll()));

    this->show();

    coarse_model = nullptr;
    img_part_alg = new ImagePartAlg;
    img_part_alg_thread = new QThread;

    connect(this, SIGNAL(callComputeInitLight(Coarse *, Viewer *)), img_part_alg, SLOT(computeInitLight(Coarse *, Viewer *)));
    connect(this, SIGNAL(callUpdateLight(Coarse *, Viewer *)), img_part_alg, SLOT(updateLight(Coarse *, Viewer *)));
    connect(this, SIGNAL(callComputeNormal(Coarse *, Viewer *)), img_part_alg, SLOT(computeNormal(Coarse *, Viewer *)));
    connect(this, SIGNAL(callRunWholeIter(Coarse *, Viewer *)), img_part_alg, SLOT(runWholeIter(Coarse *, Viewer *)));
	connect(this, SIGNAL(callNLoptTest()), img_part_alg, SLOT(testNLopt()));
    connect(img_part_alg, SIGNAL(refreshScreen()), this, SLOT(refreshScreen()));

    img_part_alg->moveToThread(img_part_alg_thread);

    img_part_alg_thread->start();
}

MainWin::~MainWin()
{
    delete img_part_alg;
    delete img_part_alg_thread;
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
    coarse_model->setRenderer(viewer);

    viewer_img->getModel(coarse_model);

}

void MainWin::exportOBJ()
{
	coarse_model->exportOBJ();
}

void MainWin::snapShot()
{
    if (coarse_model)
        viewer->getSnapShot(coarse_model);

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
        geoAlg.updateGeometry(coarse_model);
    }
}

void MainWin::refreshScreen()
{
    viewer->updateGL();
}

void MainWin::setOptParatoModel()
{
    int num_iter = m_spinBox_iter_num->value();
    double other_paras[6];

    other_paras[0] = m_spinBox_BRDF_Light_sfs->value();
    other_paras[1] = m_spinBox_Light_Reg->value();
    other_paras[2] = m_spinBox_cluster_smooth->value();
    other_paras[3] = m_spinBox_norm_sfs->value();
    other_paras[4] = m_spinBox_norm_smooth->value();
    other_paras[5] = m_spinBox_norm_normalized->value();

    coarse_model->setOptParameter(num_iter, 6, other_paras);
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
}