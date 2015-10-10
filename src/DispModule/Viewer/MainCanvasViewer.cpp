#include "MainCanvasViewer.h"
#include "MainCanvas.h"
#include "Bound.h"

MainCanvasViewer::MainCanvasViewer(QWidget *widget)
  : BasicViewer(widget)
{
  is_draw_actors = false;
}

MainCanvasViewer::~MainCanvasViewer()
{

}

void MainCanvasViewer::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (int i = 0; i < dispObjects.size(); ++i)
  {
    //startScreenCoordinatesSystem(true);
    dynamic_cast<MainCanvas*>(dispObjects[i])->drawBackground();
    //stopScreenCoordinatesSystem();

    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
  }

  if (is_draw_actors)
  {
    glClear(GL_DEPTH_BUFFER_BIT);
    drawActors();
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
  camera()->setFlySpeed(0.5);
  // forbid interaction in this viewer
  clearMouseBindings();
}

void MainCanvasViewer::getSnapShot()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[i]);
    if (main_canvas)
    {
      main_canvas->drawInfo();

      // get camera info, matrix is column major
      GLfloat modelview[16];
      GLfloat projection[16];
      GLint viewport[4];
      camera()->getModelViewMatrix(modelview);
      camera()->getProjectionMatrix(projection);
      camera()->getViewport(viewport);
      main_canvas->passCameraInfo(modelview, projection, viewport);
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

void MainCanvasViewer::drawActors()
{
  for (decltype(actors.size()) i = 0; i < actors.size(); ++i)
  {
    actors[i].draw();
  }
}

void MainCanvasViewer::setGLActors(std::vector<GLActor>& actors)
{
  this->actors = actors;
}

void MainCanvasViewer::syncCameraToModel()
{
  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[i]);
    if (main_canvas)
    {
      // get camera info, matrix is column major
      GLfloat modelview[16];
      GLfloat projection[16];
      GLint viewport[4];
      camera()->getModelViewMatrix(modelview);
      camera()->getProjectionMatrix(projection);
      camera()->getViewport(viewport);
      main_canvas->passCameraInfo(modelview, projection, viewport);
    }
  }
}