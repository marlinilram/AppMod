#ifndef SynthesisViewer_H
#define SynthesisViewer_H

#include "BasicViewer.h"

#include <memory>

class SynthesisViewer : public BasicViewer
{
public:
  SynthesisViewer(QWidget* widget);
  ~SynthesisViewer();

  void resetCamera();

protected:
  virtual void draw();
  virtual void init();
};

#endif // !SynthesisViewer_H
