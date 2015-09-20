#include "MainCanvasViewer.h"
#include "MainCanvas.h"
#include "Bound.h"

MainCanvasViewer::MainCanvasViewer(QWidget *widget)
  : BasicViewer(widget)
{

}

MainCanvasViewer::~MainCanvasViewer()
{

}

void MainCanvasViewer::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (int i = 0; i < dispObjects.size(); ++i)
  {
    startScreenCoordinatesSystem(true);
    dynamic_cast<MainCanvas*>(dispObjects[i])->drawBackground(height(), width());
    stopScreenCoordinatesSystem();

    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
  }
}

void MainCanvasViewer::init()
{
  std::cout << "Initialize Main Canvas Viewer.\n";
  BasicViewer::init();

  // Set shader
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  //glEnable(GL_CULL_FACE);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_FLAT);

  qglClearColor(QColor(Qt::white));
  setBackgroundColor(QColor(Qt::white));
  setForegroundColor(QColor(Qt::white));

  setSceneCenter(qglviewer::Vec(0, 0, 0));
  setSceneRadius(50);
  camera()->fitSphere(qglviewer::Vec(0, 0, 0), 5);
  camera()->setType(qglviewer::Camera::Type::PERSPECTIVE);

  // forbid interaction in this viewer
  clearMouseBindings();
}

void MainCanvasViewer::resetCamera()
{
  // assume only one dispObjects MainCanvas exist here
  Bound* scene_bounds = dynamic_cast<MainCanvas*>(dispObjects[0])->getBoundBox();

  qglviewer::Vec scene_center;
  scene_center = qglviewer::Vec((scene_bounds->minX + scene_bounds->maxX) / 2,
    (scene_bounds->minY + scene_bounds->maxY) / 2,
    (scene_bounds->minZ + scene_bounds->maxZ) / 2);

  setSceneCenter(scene_center);

  float x_span = (scene_bounds->maxX - scene_bounds->minX) / 2;
  float y_span = (scene_bounds->maxY - scene_bounds->minY) / 2;
  float z_span = (scene_bounds->maxZ - scene_bounds->minZ) / 2;
  float scene_radius = x_span>y_span ? (x_span > z_span ? x_span : z_span) : (y_span > z_span ? y_span : z_span);
  scene_radius *= 1.5;

  setSceneRadius(scene_radius);
  camera()->fitSphere(scene_center, scene_radius);



  setStateFileName(QString((dynamic_cast<MainCanvas*>(dispObjects[0])->getFilePath()+"/camera_info.xml").c_str()));
  if (restoreStateFromFile())
    std::cout << "Load camera info successes...\n";
}

void MainCanvasViewer::getSnapShot()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[i]);
    if (main_canvas)
    {
      main_canvas->drawInfo(QPaintDevice::height(), QPaintDevice::width());
    }
  }
  
  doneCurrent();
}

void MainCanvasViewer::setBackgroundImage(QString fname)
{
  makeCurrent();

  QImage img(fname);

  if (img.isNull())
  {
    qWarning("Unable to load file, unsupported file format");
    return;
  }

  this->setMinimumHeight(img.height());
  this->setMinimumWidth(img.width());
  this->setMaximumHeight(img.height());
  this->setMaximumWidth(img.width());

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[i]);
    if (main_canvas)
    {
      main_canvas->setBackgroundImage(fname);
    }
  }

  doneCurrent();
}

void MainCanvasViewer::updateBuffer()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[i]);
    if (main_canvas)
    {
      main_canvas->updateModelBuffer();
    }
  }

  doneCurrent();
}