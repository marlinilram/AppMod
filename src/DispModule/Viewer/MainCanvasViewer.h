#ifndef MainCanvasViewer_H
#define MainCanvasViewer_H

#include "BasicViewer.h"
#include <QString>

class MainCanvasViewer : public BasicViewer
{
public:
  MainCanvasViewer(QWidget *widget);
  ~MainCanvasViewer();

  void resetCamera();
  void getSnapShot();
  void setBackgroundImage(QString fname);
  void updateBuffer();

protected:
  virtual void draw();
  virtual void init();
};

#endif