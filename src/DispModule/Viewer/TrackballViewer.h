#ifndef TrackballViewer_H
#define TrackballViewer_H

#include "BasicViewer.h"

#include <memory>

class MainCanvasViewer;
class VectorFieldViewer;
class QMouseEvent;
class QKeyEvent;

class TrackballViewer : public BasicViewer
{
public:
  TrackballViewer(QWidget *widget);
  ~TrackballViewer();

  void setMainCanvasViewer(std::shared_ptr<MainCanvasViewer> viewer);
  void setSourceVectorViewer(std::shared_ptr<VectorFieldViewer> viewer);
  void setTargetVectorViewer(std::shared_ptr<VectorFieldViewer> viewer);
  void updateBuffer();
  void updateColorBuffer();
  void resetCamera();
  void toggleLightball();

  void setGLActors(std::vector<GLActor>& actors);
  inline void setIsDrawActors(bool state) { is_draw_actors = state; };
  inline void setShowTrackball(bool state) { show_trackball = state; };
  
protected:
  virtual void draw();
  virtual void init();
  void drawTrackBall();
  void drawActors();
  //void drawCrestLines();

private:
  void drawCornerAxis();
  void setWheelandMouse();
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  void syncCamera(int sync_type = 1);
  void wheelEvent(QWheelEvent* e);
  void keyPressEvent(QKeyEvent *e);

private:
  bool sync_camera;
  bool wireframe_;
  bool show_trackball;
  bool play_lightball;

  std::vector<GLActor> actors;
  bool is_draw_actors;


private:
  std::shared_ptr<MainCanvasViewer> main_canvas_viewer;
  std::shared_ptr<VectorFieldViewer> source_vector_viewer;
  std::shared_ptr<VectorFieldViewer> target_vector_viewer;
};

#endif