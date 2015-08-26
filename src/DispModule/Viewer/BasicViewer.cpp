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
  dispObjects.push_back(disp_obj);
}

void BasicViewer::init()
{
  // Restore previous viewer state.
}
