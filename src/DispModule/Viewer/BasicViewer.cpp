#include "BasicViewer.h"
#include "DispObject.h"

BasicViewer::BasicViewer()
{
  QGLViewer::QGLViewer();
}

BasicViewer::~BasicViewer()
{

}

BasicViewer::BasicViewer(QWidget *widget)
  : QGLViewer(widget)
{

}

void BasicViewer::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (int i = 0; i < dispObjects.size(); ++i)
  {
    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
  }
}

void BasicViewer::addDispObj(DispObject* disp_obj)
{
  makeCurrent();

  dispObjects.push_back(disp_obj);
  dispObjects.back()->setGLProperty();

  doneCurrent();
}

void BasicViewer::deleteDispObj(DispObject* disp_obj)
{
  std::vector<DispObject *>::iterator iter;
  iter = std::find(dispObjects.begin(), dispObjects.end(), disp_obj);
  if (iter !=dispObjects.end())
  {
    dispObjects.erase(iter);
    std::cout << "deleted a display object.\n";
  }
  else
  {
    std::cout << "Display object not found.\n";
  }
}

void BasicViewer::init()
{
  // Restore previous viewer state.
}
