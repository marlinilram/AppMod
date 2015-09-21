#include "VectorFieldViewer.h"
#include "VectorFieldCanvas.h"

VectorFieldViewer::VectorFieldViewer(QWidget* widget)
  : BasicViewer(widget)
{

}

VectorFieldViewer::~VectorFieldViewer()
{

}

void VectorFieldViewer::draw()
{
  BasicViewer::draw();
}

void VectorFieldViewer::init()
{
  BasicViewer::init();

  //set camera
  camera()->setType(qglviewer::Camera::Type::ORTHOGRAPHIC);
  camera()->setPosition(qglviewer::Vec(0.5, 0.5, 1.0));

  clearMouseBindings();
  setWheelBinding(Qt::NoModifier, CAMERA, MOVE_FORWARD);
}

void VectorFieldViewer::updateSourceVectorField()
{
  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    VectorFieldCanvas* canvas = dynamic_cast<VectorFieldCanvas*>(dispObjects[i]);
    if (canvas)
    {
      canvas->updateSourceVectorField();
    }
  }

  updateGLOutside();
}