#ifndef TrackballViewer_H
#define TrackballViewer_H

#include "BasicViewer.h"

#include <memory>

class MainCanvasViewer;
class VectorFieldViewer;
class QMouseEvent;

class TrackballViewer : public BasicViewer
{
public:
  TrackballViewer(QWidget *widget);
  ~TrackballViewer();

  void setMainCanvasViewer(std::shared_ptr<MainCanvasViewer> viewer);
  void setSourceVectorViewer(std::shared_ptr<VectorFieldViewer> viewer);
  void updateBuffer();
  void resetCamera();
  
protected:
  virtual void draw();
  virtual void init();
  void drawTrackBall();

private:
  void drawCornerAxis();
  void setWheelandMouse();
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  void syncCamera();
  void wheelEvent(QWheelEvent* e);

private:
  bool sync_camera;

private:
  std::shared_ptr<MainCanvasViewer> main_canvas_viewer;
  std::shared_ptr<VectorFieldViewer> source_vector_viewer;
};

#endif