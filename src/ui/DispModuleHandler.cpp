#include "DispModuleHandler.h"
#include "MainCanvasViewer.h"
#include "TrackballViewer.h"
#include "VectorFieldViewer.h"
#include "SynthesisViewer.h"
#include "MainCanvas.h"
#include "TrackballCanvas.h"
#include "VectorFieldCanvas.h"
#include "SynthesisCanvas.h"
#include "ParameterMgr.h"

#include "Model.h"
#include "FeatureGuided.h"

#include "AlgHandler.h"

#include <QWidget>
#include <QGridLayout>
#include <QDockWidget>
#include <QMainWindow>

DispModuleHandler::DispModuleHandler(QWidget* parent)
{
  //QGridLayout *gridLayout_3;
  //gridLayout_3 = new QGridLayout(parent);
  //gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));

  QDockWidget* main_canvas_dock = new QDockWidget(parent);
  main_canvas_dock->setWindowTitle("centerDock");
  main_canvas_dock->setAllowedAreas(Qt::BottomDockWidgetArea);
  main_canvas_dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  main_canvas_dock->setMinimumHeight(500);
  main_canvas_dock->setMinimumWidth(500);
  QMainWindow* main_window = dynamic_cast<QMainWindow*>(parent);
  main_window->setCentralWidget(main_canvas_dock);

  QDockWidget* trackball_dock = new QDockWidget(parent);
  QDockWidget* synthesis_dock = new QDockWidget(parent);
  QDockWidget* src_vector_dock = new QDockWidget(parent);
  QDockWidget* tar_vector_dock = new QDockWidget(parent);

  trackball_dock->setAllowedAreas(Qt::BottomDockWidgetArea);
  synthesis_dock->setAllowedAreas(Qt::BottomDockWidgetArea);
  src_vector_dock->setAllowedAreas(Qt::BottomDockWidgetArea);
  tar_vector_dock->setAllowedAreas(Qt::BottomDockWidgetArea);

  main_window->addDockWidget(Qt::BottomDockWidgetArea, trackball_dock);
  main_window->addDockWidget(Qt::BottomDockWidgetArea, synthesis_dock);
  main_window->addDockWidget(Qt::BottomDockWidgetArea, src_vector_dock);
  main_window->addDockWidget(Qt::BottomDockWidgetArea, tar_vector_dock);


  main_canvas_viewer.reset(new MainCanvasViewer(main_canvas_dock));
  main_canvas_viewer->setObjectName(QStringLiteral("main_canvas_viewer"));
  //gridLayout_3->addWidget(main_canvas_viewer.get(), 0, 0, 2, 2);
  main_canvas_dock->setWidget(main_canvas_viewer.get());
  
  trackball_viewer.reset(new TrackballViewer(parent));
  trackball_viewer->setObjectName(QStringLiteral("trackball_viewer"));
  trackball_dock->setWidget(trackball_viewer.get());

  synthesis_viewer.reset(new SynthesisViewer(parent));
  synthesis_viewer->setObjectName(QStringLiteral("synthesis_viewer"));
  synthesis_dock->setWidget(synthesis_viewer.get());

  source_vector_viewer.reset(new VectorFieldViewer(parent));
  source_vector_viewer->setObjectName(QStringLiteral("src_vec_field_viewer"));
  src_vector_dock->setWidget(source_vector_viewer.get());
  target_vector_viewer.reset(new VectorFieldViewer(parent));
  target_vector_viewer->setObjectName(QStringLiteral("trg_vec_field_viewer"));
  tar_vector_dock->setWidget(target_vector_viewer.get());

  main_canvas.reset(new MainCanvas);
  trackball_canvas.reset(new TrackballCanvas);
  trackball_viewer->setMainCanvasViewer(main_canvas_viewer);
  trackball_viewer->setSourceVectorViewer(source_vector_viewer);
  trackball_viewer->setTargetVectorViewer(target_vector_viewer);

  synthesis_canvas.reset(new SynthesisCanvas);

  source_vector_canvas.reset(new VectorFieldCanvas);
  target_vector_canvas.reset(new VectorFieldCanvas);

  source_vector_canvas->setRenderMode(VectorField::SOURCE_MODE);
  target_vector_canvas->setRenderMode(VectorField::TARGET_MODE);

  alg_handler.reset(new AlgHandler);
}

void DispModuleHandler::loadModel(std::shared_ptr<Model> model, std::string model_file_path)
{
  cur_file_path = model_file_path;
  trackball_canvas->setModel(model);
  trackball_viewer->deleteDispObj(trackball_canvas.get());
  trackball_viewer->addDispObj(trackball_canvas.get());
  trackball_viewer->resetCamera();

  main_canvas->setModel(model);
  main_canvas_viewer->deleteDispObj(main_canvas.get());
  main_canvas_viewer->addDispObj(main_canvas.get());
  main_canvas_viewer->setBackgroundImage(QString::fromStdString(model_file_path + "/photo.png"));
  main_canvas_viewer->updateGLOutside();

  LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform") = Matrix4f::Identity();

  alg_handler->setShapeModel(model);
}

void DispModuleHandler::loadSynthesisTarget(std::shared_ptr<Model> model, std::string model_file_path)
{
  synthesis_canvas->setModel(model);
  synthesis_viewer->deleteDispObj(synthesis_canvas.get());
  synthesis_viewer->addDispObj(synthesis_canvas.get());
  synthesis_viewer->resetCamera();

  alg_handler->setSynthesisModel(model);
}

void DispModuleHandler::exportOBJ()
{
  trackball_canvas->getModel()->exportOBJ(0);
}

void DispModuleHandler::snapShot()
{
  main_canvas_viewer->getSnapShot();
}

void DispModuleHandler::updateGeometry()
{
  alg_handler->doProjOptimize();
  main_canvas_viewer->setGLActors(alg_handler->getGLActors());
  trackball_viewer->setGLActors(alg_handler->getGLActors());

  updateCanvas();
  source_vector_viewer->updateSourceField();
  source_vector_viewer->updateScalarFieldTexture();
  target_vector_viewer->updateScalarFieldTexture();
}

void DispModuleHandler::initFeatureModel()
{
  if (trackball_canvas->getModel())
  {
    std::shared_ptr<FeatureGuided> share_feature_model(new FeatureGuided(trackball_canvas->getModel(), trackball_canvas->getModel()->getDataPath()));

    source_vector_canvas->setFeatureModel(share_feature_model);
    source_vector_viewer->deleteDispObj(source_vector_canvas.get());
    source_vector_viewer->addDispObj(source_vector_canvas.get());
    //source_vector_viewer->updateGLOutside();

    target_vector_canvas->setFeatureModel(share_feature_model);
    target_vector_viewer->deleteDispObj(target_vector_canvas.get());
    target_vector_viewer->addDispObj(target_vector_canvas.get());
    //target_vector_viewer->updateGLOutside();

    // TODO: bug here, every time reset the feature model
    // we need to reset the projection optimization
    alg_handler->setFeatureModel(share_feature_model);
  }
}

void DispModuleHandler::updateCanvas()
{
  trackball_viewer->updateBuffer();
  trackball_viewer->updateGLOutside();

  main_canvas_viewer->updateBuffer();
  main_canvas_viewer->updateGLOutside();
}

void DispModuleHandler::setEdgeThreshold(int val)
{
  main_canvas->setEdgeThreshold(float(val) / 100.0);
  main_canvas_viewer->updateGLOutside();
}

void DispModuleHandler::setUseFlat(int state)
{
  main_canvas->setUseFlat(state);
  main_canvas_viewer->updateGLOutside();
}

void DispModuleHandler::showCrspLines(int state)
{
  source_vector_viewer->isDrawAllLines(bool(state));
  target_vector_viewer->isDrawAllLines(bool(state));

  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
}

void DispModuleHandler::showProjCrsp(int state)
{
  main_canvas_viewer->setIsDrawActors(bool(state));
  trackball_viewer->setIsDrawActors(bool(state));
  synthesis_viewer->setIsDrawActors(bool(state));

  main_canvas_viewer->updateGLOutside();
  trackball_viewer->updateGLOutside();
  synthesis_viewer->updateGLOutside();
}

void DispModuleHandler::deleteLastCrspLine_Source()
{
  source_vector_viewer->deleteLastLine();
}

void DispModuleHandler::deleteLastCrspLine_Target()
{
  target_vector_viewer->deleteLastLine();
}

void DispModuleHandler::setVectorFieldViewerPara(std::vector<bool>& checkStates)
{
  source_vector_viewer->setDispPara(checkStates);
  target_vector_viewer->setDispPara(checkStates);

  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
}

void DispModuleHandler::toggleVectorFieldMode(int state)
{
  // it's a QComboBox here
  if (state == 1)
  {
    source_vector_viewer->setInteractionMode(VectorField::SELECT_POINT);
    connect(source_vector_viewer.get(), SIGNAL(triggeredInteractiveCrsp()), this, SLOT(updateGeometryInteractive()));
  }
  else if (state == 0)
  {
    source_vector_viewer->setInteractionMode(VectorField::DRAW_CRSP_LINE);
    disconnect(source_vector_viewer.get(), SIGNAL(triggeredInteractiveCrsp()), this, SLOT(updateGeometryInteractive()));
  }
  else if (state == 2)
  {
    source_vector_viewer->setInteractionMode(VectorField::CORRECT_CRSP);
    disconnect(source_vector_viewer.get(), SIGNAL(triggeredInteractiveCrsp()), this, SLOT(updateGeometryInteractive()));
  }
  else if (state == 3 || state == 4)
  {
    source_vector_viewer->setInteractionMode(VectorField::DELETE_TARGET_CURVES);
    disconnect(source_vector_viewer.get(), SIGNAL(triggeredInteractiveCrsp()), this, SLOT(updateGeometryInteractive()));

    switch (state)
    {
    case 3:
      LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("LFeature:delete_interactive_reverse") = false;
      break;
    case 4:
      LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("LFeature:delete_interactive_reverse") = true;
      break;
    default:
      break;
    }
  }
  else if (state == 5)
  {
    source_vector_viewer->setInteractionMode(VectorField::ADD_TARGET_CURVES);
    disconnect(source_vector_viewer.get(), SIGNAL(triggeredInteractiveCrsp()), this, SLOT(updateGeometryInteractive()));
  }
}

void DispModuleHandler::updateGeometryInteractive()
{
  alg_handler->doInteractiveProjOptimize();
  main_canvas_viewer->setGLActors(alg_handler->getGLActors());
  trackball_viewer->setGLActors(alg_handler->getGLActors());

  updateCanvas();
  source_vector_viewer->updateSourceField();
  source_vector_viewer->updateScalarFieldTexture();
  target_vector_viewer->updateScalarFieldTexture();
}

void DispModuleHandler::showBackgroundImage(int state)
{
  main_canvas_viewer->setShowBackground(state);

  main_canvas_viewer->updateGLOutside();
}

void DispModuleHandler::runNormalTransfer()
{
  alg_handler->doNormalTransfer();
  trackball_viewer->setGLActors(alg_handler->getGLActors());
  trackball_viewer->updateGLOutside();
  main_canvas_viewer->setReflectanceImage(QString::fromStdString(cur_file_path + "/reflectance.png"));
  updateCanvas();
}

void DispModuleHandler::runNormalCompute()
{
  alg_handler->doNormalCompute();
  trackball_viewer->setGLActors(alg_handler->getGLActors());
  updateCanvas();
  //cv::Mat normal = alg_handler->getDecompImg()->getNormal();
  ////need to reset others
  //cv::Mat normal_normalized = cv::Mat(normal.size().height,normal.size().width,CV_32FC3);
  //for(int i = 0; i < normal.size().height; i ++)
  //{
  //  for(int j = 0; j < normal.size().width; j ++)
  //  {
  //    normal_normalized.at<cv::Vec3f>(i,j)[0] = (normal.at<cv::Vec3f>(i,j)[0] + 1) / 2;
  //    normal_normalized.at<cv::Vec3f>(i,j)[1] = (normal.at<cv::Vec3f>(i,j)[1] + 1) / 2;
  //    normal_normalized.at<cv::Vec3f>(i,j)[2] = (normal.at<cv::Vec3f>(i,j)[2] + 1) / 2;
  //  }
  //}
  //cv::imshow("Normal_(0 ~ 1)",normal_normalized);
  ////cv::imshow("Normal_(-1 ~ 1)",normal);
}

void DispModuleHandler::runLFRegRigid()
{
  //alg_handler->doDetailSynthesis();

  // now used to test PorjICP
  //source_vector_viewer->updateSourceField(2);
  //alg_handler->doProjICP();
  //updateCanvas();
  //source_vector_viewer->updateGLOutside();

  // new trial from 11/10/2015 for large feature reg
  alg_handler->doLargeFeatureReg();
  trackball_viewer->updateCamera();
  trackball_viewer->updateGLOutside();
  main_canvas_viewer->updateGLOutside();
  source_vector_viewer->updateSourceField(2);
  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
}

void DispModuleHandler::toggleMainViewMode(int state)
{
  // it's a QComboBox here
  if (state == 1)
  {
    main_canvas_viewer->setInteractionMode(MainViewer::TAG_PLANE);
    main_canvas_viewer->clearPreviousInteractionInfo();
    main_canvas_viewer->updateBuffer();
    main_canvas_viewer->updateGLOutside();

  }
  else if (state == 0)
  {
    main_canvas_viewer->setInteractionMode(MainViewer::STATIC);
    main_canvas_viewer->clearPreviousInteractionInfo();
    main_canvas_viewer->updateBuffer();
    main_canvas_viewer->updateGLOutside();
  }
}

void DispModuleHandler::resetCamera()
{
  trackball_viewer->resetCamera();
}

// set distance attenuation in scalar field computation
void DispModuleHandler::setSFieldDistAttenuation()
{
  // coefficient range [0, 10]
  target_vector_viewer->updateSourceField(1);
  target_vector_viewer->updateScalarFieldTexture();
}

void DispModuleHandler::setShowTrackball()
{
  int state = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowTrackball");
  trackball_viewer->setShowTrackball((state == 0 ? false : true));
  trackball_viewer->updateGLOutside();
}

void DispModuleHandler::setSFieldPara(int set_type)
{
  target_vector_viewer->updateSourceField(set_type);
  target_vector_viewer->updateScalarFieldTexture();
  source_vector_viewer->updateGLOutside();
}

void DispModuleHandler::setMainCanvasRenderMode()
{
  trackball_viewer->updateCamera();
  trackball_viewer->updateGLOutside();
  source_vector_viewer->updateSourceField(2);
  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
  main_canvas_viewer->updateCanvasRenderMode();
  main_canvas_viewer->updateGLOutside();
}

void DispModuleHandler::runLFRegNonRigid()
{
  alg_handler->doLargeFeatureReg(1);
  updateCanvas();
  source_vector_viewer->updateSourceField(2);
  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
  std::cout << "bug test.\n";
  /*trackball_viewer->setGLActors(alg_handler->getGLActors());
  updateCanvas();*/
}
void DispModuleHandler::changeToLightball()
{
  trackball_viewer->toggleLightball();
  updateCanvas();
}
void DispModuleHandler::doSynthesis()
{
  alg_handler->doDetailSynthesis();
  trackball_viewer->setGLActors(alg_handler->getGLActors());
  main_canvas_viewer->setSynthesisReflectance();
  updateCanvas();
}

void DispModuleHandler::updateShapeCrest()
{
  trackball_viewer->updateShapeCrest();
  main_canvas_viewer->updateGLOutside();
  source_vector_viewer->updateSourceField(0);
  source_vector_viewer->updateGLOutside();
  target_vector_viewer->updateGLOutside();
}

void DispModuleHandler::updateTargetCurves()
{
  //source_vector_viewer->updateSourceField(5);
  source_vector_viewer->updateGLOutside();
}

void DispModuleHandler::testApplyDisplacement()
{
  alg_handler->testApplyDisplacement();
  trackball_viewer->setGLActors(alg_handler->getGLActors());
  synthesis_viewer->setGLActors(alg_handler->getSynGLActors());
  //main_canvas_viewer->setSynthesisReflectance();
  updateCanvas();
}

void DispModuleHandler::runApplyDisplacement()
{
  alg_handler->runApplyDisplacement();
}

void DispModuleHandler::loadDetailMap()
{
  alg_handler->loadDetailMap();
}

void DispModuleHandler::updateSField(int type)
{
  source_vector_viewer->updateSourceField(type);
  source_vector_viewer->updateGLOutside();
}