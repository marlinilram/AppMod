#include "SynthesisViewer.h"
#include "SynthesisCanvas.h"

#include "GLActor.h"
#include "Bound.h"
#include "Model.h"
#include "ParameterMgr.h"

SynthesisViewer::SynthesisViewer(QWidget* widget)
  : BasicViewer(widget)
{
  is_draw_actors = false;
}

SynthesisViewer::~SynthesisViewer()
{

}

void SynthesisViewer::draw()
{
  // do nothing here
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  for (int i = 0; i < dispObjects.size(); ++i)
  {
    glDisable(GL_LIGHTING);
    //drawCrestLines();
    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
  }

  if (is_draw_actors)
  {
    //glClear(GL_DEPTH_BUFFER_BIT);
    drawActors();
  }
}

void SynthesisViewer::init()
{
  std::cout << "Initialize Synthesis Viewer.\n";

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
}

void SynthesisViewer::resetCamera()
{
  // assume only one dispObjects MainCanvas exist here
  if (dispObjects.empty())
  {
    return;
  }

  Bound* scene_bounds = dynamic_cast<SynthesisCanvas*>(dispObjects[0])->getBoundBox();

  qglviewer::Vec scene_center;
  scene_center = qglviewer::Vec((scene_bounds->minX + scene_bounds->maxX) / 2,
    (scene_bounds->minY + scene_bounds->maxY) / 2,
    (scene_bounds->minZ + scene_bounds->maxZ) / 2);

  setSceneCenter(scene_center);

  float x_span = (scene_bounds->maxX - scene_bounds->minX) / 2;
  float y_span = (scene_bounds->maxY - scene_bounds->minY) / 2;
  float z_span = (scene_bounds->maxZ - scene_bounds->minZ) / 2;
  float scene_radius = sqrt(x_span * x_span + y_span * y_span + z_span * z_span);
  //scene_radius *= 1.5;

  setSceneRadius(scene_radius);
  camera()->fitSphere(scene_center, scene_radius);



  setStateFileName(QString((dynamic_cast<SynthesisCanvas*>(dispObjects[0])->getFilePath()+"/camera_info.xml").c_str()));
  if (restoreStateFromFile())
    std::cout << "Load camera info successes...\n";
  else
    std::cout << "Load camera info failed...\n";
}

void SynthesisViewer::setGLActors(std::vector<GLActor>& actors)
{
  this->actors = actors;
}

void SynthesisViewer::drawActors()
{
  for (decltype(actors.size()) i = 0; i < actors.size(); ++i)
  {
    actors[i].draw();
  }
}