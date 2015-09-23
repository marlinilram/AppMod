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
    //startScreenCoordinatesSystem(true);
    dynamic_cast<MainCanvas*>(dispObjects[i])->drawBackground();
    //stopScreenCoordinatesSystem();

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

float MainCanvasViewer::sampleDensity()
{
// return the pixel length of average edge length of the model
  makeCurrent();

  MainCanvas* main_canvas = dynamic_cast<MainCanvas*>(dispObjects[0]);
  if (!main_canvas)
  {
    std::cout << "Dynamic cast to MainCanvas failed.\n";

    doneCurrent();
    return 0.0;
  }

  float avg_edge_length = main_canvas->getModelAvgEdgeLength();

  GLint Viewport[4];
  GLdouble Projection[16];
  // Precomputation begin
  glGetIntegerv(GL_VIEWPORT , Viewport);
  glGetDoublev (GL_PROJECTION_MATRIX, Projection);
  // Precomputation end
  GLdouble v[4], vs[4];
  v[0]=avg_edge_length; v[1]=0.0; v[2]=0.0; v[3]=1.0;
  vs[0]=Projection[0 ]*v[0] + Projection[4 ]*v[1] + Projection[8 ]*v[2] + Projection[12 ]*v[3];
  vs[1]=Projection[1 ]*v[0] + Projection[5 ]*v[1] + Projection[9 ]*v[2] + Projection[13 ]*v[3];
  vs[2]=Projection[2 ]*v[0] + Projection[6 ]*v[1] + Projection[10]*v[2] + Projection[14 ]*v[3];
  vs[3]=Projection[3 ]*v[0] + Projection[7 ]*v[1] + Projection[11]*v[2] + Projection[15 ]*v[3];
  vs[0] /= vs[3];
  vs[1] /= vs[3];
  vs[2] /= vs[3];
  vs[0] = vs[0] * 0.5 + 0.5;
  vs[1] = vs[1] * 0.5 + 0.5;
  vs[2] = vs[2] * 0.5 + 0.5;
  vs[0] = vs[0] * Viewport[2] + Viewport[0];
  vs[1] = vs[1] * Viewport[3] + Viewport[1];

  GLdouble ori[2] = { 0.5, 0.5 };

  doneCurrent();
  return sqrt((vs[0] - ori[0]) * (vs[0] - ori[0]) + (vs[1] - ori[1]) * (vs[1] - ori[1]));
}