#ifndef MainCanvasViewer_H
#define MainCanvasViewer_H

#include "BasicViewer.h"
#include <QString>

class MainCanvasViewer : public BasicViewer
{
public:
  MainCanvasViewer(QWidget *widget);
  ~MainCanvasViewer();

  void getSnapShot();
  void setBackgroundImage(QString fname);
  void updateBuffer();
  float sampleDensity();

protected:
  virtual void draw();
  virtual void init();
};

#endif