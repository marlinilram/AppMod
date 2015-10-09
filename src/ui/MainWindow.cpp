#include "MainWindow.h"
#include "ParameterDock.h"
#include "DispModuleHandler.h"
#include "Model.h"

//#include "camera_pose.h"

MainWindow::MainWindow()
{
    setupUi(this); 

    connect(action_Load_Model, SIGNAL(triggered()), this, SLOT(loadModel()));
    connect(action_Snap_Shot, SIGNAL(triggered()), this, SLOT(snapShot()));
    connect(action_Fix_Camera, SIGNAL(triggered()), this, SLOT(fixCamera()));
    connect(action_Reset_Screen, SIGNAL(triggered()), this, SLOT(resetScreen()));
    connect(action_Update_Geometry, SIGNAL(triggered()), this, SLOT(updateGeometry()));
    connect(action_Export_OBJ, SIGNAL(triggered()), this, SLOT(exportOBJ()));
    connect(action_Render, SIGNAL(triggered()), this, SLOT(renderTexture()));
    connect(action_Vector_Field, SIGNAL(triggered()), this, SLOT(setVectorField()));
    connect(action_Load_2D_3D_points, SIGNAL(triggered()), this, SLOT(loadPoints()));
    connect(action_Delete_Last_Line_Of_Source, SIGNAL(triggered()), this, SLOT(deleteLastLine_Source()));
    connect(action_Delete_Last_Line_Of_Target, SIGNAL(triggered()), this, SLOT(deleteLastLine_Target()));

    disp_modules.reset(new DispModuleHandler(centralwidget));
    this->setCentralWidget(centralwidget);

    parameter_dock.reset(new ParameterDock);
    parameter_dock->setFixedWidth(250);
    this->addDockWidget(Qt::LeftDockWidgetArea, parameter_dock.get());
    parameter_dock->setDispModules(disp_modules);

    this->show();

	//pointsSelect = false;
	//isCompute = false;
}

MainWindow::~MainWindow()
{
}

void MainWindow::loadModel()
{
    QString filter;
    filter = "obj file (*.obj)";

    QDir dir;
    QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
    if (fileName.isEmpty() == true) return;

    std::string model_file_path = fileName.toStdString();
    std::string model_file_name = model_file_path.substr(model_file_path.find_last_of('/') + 1);
    model_file_path = model_file_path.substr(0, model_file_path.find_last_of('/'));

    std::shared_ptr<Model> share_model(new Model(model_file_path, model_file_name));

    disp_modules->loadModel(share_model, model_file_path);

    setOptParatoModel();

    this->snapShot();

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

void MainWindow::exportOBJ()
{
	//coarse_model->exportOBJ();

    //coarse_model->setInit();
  disp_modules->exportOBJ();
}

void MainWindow::snapShot()
{
    //if (coarse_model)
    //    viewer->getSnapShot(coarse_model, true);
  disp_modules->snapShot();
}

void MainWindow::fixCamera()
{
 //   viewer->fixCamera();
}

void MainWindow::resetScreen()
{
	//emit callNLoptTest();

    //viewer->resetScreen();
    //viewer->getModel(coarse_model);
    //viewer->setShowModel(true);
}

void MainWindow::updateGeometry()
{
    //if (coarse_model)
    //{
    //    GeometryPartAlg geoAlg;
    //    geoAlg.updateWithExNormal(coarse_model);

    //    coarse_model->computeLight();
    //    viewer->getModel(coarse_model);
    //    this->refreshScreen();
    //}
  disp_modules->updateGeometry();
}

void MainWindow::refreshScreen()
{
   // viewer->UpdateGLOutside();
}

void MainWindow::setOptParatoModel()
{
    //int int_paras[3];
    //
    //int_paras[0] = m_spinBox_iter_num->value();
    //int_paras[1] = m_spinBox_deform_iter->value();
    //int_paras[2] = m_spinBox_cluster_num->value();


    //double double_paras[13];

    //double_paras[0] = m_spinBox_BRDF_Light_sfs->value();
    //double_paras[1] = m_spinBox_Light_Reg->value();
    //double_paras[2] = m_spinBox_cluster_smooth->value();
    //double_paras[3] = m_spinBox_norm_sfs->value();
    //double_paras[4] = m_spinBox_norm_smooth->value();
    //double_paras[5] = m_spinBox_norm_normalized->value();
    //double_paras[6] = m_spinBox_k_strech->value();
    //double_paras[7] = m_spinBox_k_bend->value();
    //double_paras[8] = m_spinBox_deform_normal->value();
    //double_paras[9] = m_spinBox_vertical_move->value();
    //double_paras[10] = m_spinBox_rho_smooth->value();
    //double_paras[11] = m_spinBox_rho_s_r->value();
    //double_paras[12] = m_spinBox_norm_prior->value();

    //if (coarse_model)
    //    coarse_model->getParaObjPtr()->setOptParameter(3, int_paras, 13, double_paras);
}

void MainWindow::runAll()
{
    //if (coarse_model)
    //{
    //    emit callRunWholeIter(coarse_model, viewer);
    //}
}

void MainWindow::renderTexture()
{
    //viewer->resetScreen();
    //viewer->getModelWithTexture(coarse_model, coarse_model->getRhoImg());
    //viewer->setShowModel(true);
    //viewer->UpdateGLOutside();

    //viewer->setSnapshotFormat("PNG");

    //char time_postfix[50];
    //time_t current_time = time(NULL);
    //strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    //std::string file_time_postfix = time_postfix;

    //viewer->saveSnapshot(QString((coarse_model->getOutputPath()+"/ren_img" + file_time_postfix + ".png").c_str()));
}

void MainWindow::setVectorField()
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

  //QString filter;
  //filter = "image file (*.png)";

  //QDir dir;
  //QString fileName = QFileDialog::getOpenFileName(this, QString(tr("Open Obj File")), dir.absolutePath(), filter);
  //if (fileName.isEmpty() == true) return;

  disp_modules->initFeatureModel();

  //std::string fileSource = fileName.toStdString();
  //std::string fileTarget = fileSource.substr(0, fileSource.find_last_of('/') + 1) + "featurePP.png";
  //feature_guided->initImages(fileSource, fileTarget);
  //feature_guided->initRegister();
  //feature_guided->initVisualization(viewer_img);

  //proj_opt.updateShape(feature_guided, coarse_model);
}

void MainWindow::computeCameraPose()
{
	//coarse_model->getSelectedPoints();
	//if(coarse_model->imgpts->size() != viewer->objpts.size())
	//	cout << "The number of selected image points is not equal to the number of corresponding 3D points." << endl;
	//else if(coarse_model->imgpts->size() != 8 || viewer->objpts.size() != 8)
	//	cout << "The number of selected points is not equal to 8." << endl;
	//else
	//{
	//	CameraPose cp;
	//	cp.getCameraPose(*(coarse_model->imgpts),viewer->objpts);

	//	//GLdouble mvm[16];
	//	//Eigen::Matrix4d camera_pose;
	//	//camera_pose << cp.rotation.at<double>(0,0), cp.rotation.at<double>(0,1), cp.rotation.at<double>(0,2), cp.translation.at<double>(0,0),
	//	//	           cp.rotation.at<double>(1,0), cp.rotation.at<double>(1,1), cp.rotation.at<double>(1,2), cp.translation.at<double>(1,0),
	//	//			   cp.rotation.at<double>(2,0), cp.rotation.at<double>(2,1), cp.rotation.at<double>(2,2), cp.translation.at<double>(2,0),
	//	//			   0, 0, 0, 1;
	//	//Eigen::Matrix4d camera_pose_inv = camera_pose.inverse();

	//	//mvm[0] = camera_pose_inv(0, 0);
	//	//mvm[1] = camera_pose_inv(0, 1);
	//	//mvm[2] = camera_pose_inv(0, 2);
	//	//mvm[3] = camera_pose_inv(0, 3);
	//	//mvm[4] = camera_pose_inv(1, 0);
	//	//mvm[5] = camera_pose_inv(1, 1);
	//	//mvm[6] = camera_pose_inv(1, 2);
	//	//mvm[7] = camera_pose_inv(1, 3);
	//	//mvm[8] = camera_pose_inv(2, 0);
	//	//mvm[9] = camera_pose_inv(2, 1);
	//	//mvm[10] = camera_pose_inv(2, 2);
	//	//mvm[11] = camera_pose_inv(2, 3);
	//	//mvm[12] = camera_pose_inv(3, 0);
	//	//mvm[13] = camera_pose_inv(3, 1);
	//	//mvm[14] = camera_pose_inv(3, 2);
	//	//mvm[15] = camera_pose_inv(3, 3);
	//	//viewer->camera()->setFromModelViewMatrix(mvm);
	//	/*mvm[0] = 0;
	//	mvm[1] = 0;
	//	mvm[2] = -1;
	//	mvm[3] = 10;
	//	mvm[4] = 0;
	//	mvm[5] = 1;
	//	mvm[6] = 0;
	//	mvm[7] = 0;
	//	mvm[8] = 1;
	//	mvm[9] = 0;
	//	mvm[10] = 0;
	//	mvm[11] = 0;
	//	mvm[12] = 0;
	//	mvm[13] = 0;
	//	mvm[14] = 0;
	//	mvm[15] = 1;
	//	viewer->camera()->setFromModelViewMatrix(mvm);*/
	//	/*GLint viewport[4];
	//	viewer->camera()->getViewport(viewport);
	//	Mat vp = Mat_<double>::zeros(4,1);
	//	vp.at<double>(0,0) = viewport[0];
	//	vp.at<double>(1,0) = viewport[1];
	//	vp.at<double>(2,0) = viewport[2];
	//	vp.at<double>(3,0) = viewport[3];
	//	std::cout << " The viewport is " << vp << std::endl;*/
	//	
	//	qreal pm[12];
	//	pm[0] = cp.projectionMatrix.at<double>(0,0);
	//	pm[1] = cp.projectionMatrix.at<double>(0,1);
	//	pm[2] = cp.projectionMatrix.at<double>(0,2);
	//	pm[3] = cp.projectionMatrix.at<double>(0,3);
	//	pm[4] = cp.projectionMatrix.at<double>(1,0);
	//	pm[5] = cp.projectionMatrix.at<double>(1,1);
	//	pm[6] = cp.projectionMatrix.at<double>(1,2);
	//	pm[7] = cp.projectionMatrix.at<double>(1,3);
	//	pm[8] = cp.projectionMatrix.at<double>(2,0);
	//	pm[9] = cp.projectionMatrix.at<double>(2,1);
	//	pm[10] = cp.projectionMatrix.at<double>(2,2);
	//	pm[11] = cp.projectionMatrix.at<double>(2,3);

	//	viewer->camera()->setFromProjectionMatrix(pm);


	//	//viewer->camera()->position();
	//	//std::cout<<"camera orie: "<<viewer->camera()->orientation()<<"\n";
	//	//std::cout<<"clipping: " << viewer->camera()->zNear()<<"\t"<<viewer->camera()->zFar()<<"\n";

	//	cout << "The rotation matrix is " << endl << cp.rotation << endl;
	//	cout << "The translation vector is " << endl << cp.translation << endl;
	//	cout << "The projection Matrix is " << endl << cp.projectionMatrix << endl;
	//}
}

void MainWindow::clearSelectedPoints()
{
	//std::cout << "Clear all selected points." << endl;
	//coarse_model->getSelectedPoints();
	//while(!(coarse_model->imgpts->empty()))
	//	coarse_model->imgpts->pop_back();
	//while(!(viewer->objpts.empty()))
	//	viewer->objpts.pop_back();
}

void MainWindow::loadPoints()
{
	//while(!(viewer->pts2d.empty()))
	//	viewer->pts2d.pop_back();
	//while(!(viewer->pts3d.empty()))
	//	viewer->pts3d.pop_back();
	//coarse_model->getSelectedPoints();
	//for(std::vector<CvPoint2D32f>::iterator it = coarse_model->imgpts->begin();it != coarse_model->imgpts->end();it ++)
	//	viewer->pts2d.push_back(*it);
	//for(std::vector<CvPoint3D32f>::iterator it = viewer->objpts.begin();it != viewer->objpts.end();it ++)
	//	viewer->pts3d.push_back(*it);

 // ProjOptimize proj_opt;
 // proj_opt.updateShape(feature_guided, coarse_model);
}

void MainWindow::deleteLastLine_Source()
{
  disp_modules->deleteLastCrspLine_Source();
}

void MainWindow::deleteLastLine_Target()
{
  disp_modules->deleteLastCrspLine_Target();
}