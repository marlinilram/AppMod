#ifndef VectorFieldViewer_H
#define VectorFieldViewer_H

#include "BasicViewer.h"

#include <memory>

class VectorFieldViewer : public BasicViewer
{
public:
  VectorFieldViewer(QWidget* widget);
  ~VectorFieldViewer();

  void updateSourceVectorField();

protected:
  virtual void draw();
  virtual void init();
};
#endif // !VectorFieldViewer_H
