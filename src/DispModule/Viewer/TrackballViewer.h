#ifndef TrackballViewer_H
#define TrackballViewer_H

#include "BasicViewer.h"

#include <memory>

class MainCanvasViewer;
class QMouseEvent;

class TrackballViewer : public BasicViewer
{
public:
  TrackballViewer(QWidget *widget);
  ~TrackballViewer();

  void setMainCanvasViewer(std::shared_ptr<MainCanvasViewer> viewer);
  void updateBuffer();
  void resetCamera();

protected:
  virtual void draw();
  virtual void init();

private:
  void drawCornerAxis();
  void setWheelandMouse();
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);

private:
  bool sync_camera;

private:
  std::shared_ptr<MainCanvasViewer> main_canvas_viewer;
};

#endif