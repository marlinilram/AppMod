#ifndef Texture_Viewer_H
#define Texture_Viewer_H

#include "BasicViewer.h"

#include <memory>

class MainCanvasViewer;
class VectorFieldViewer;
class QMouseEvent;
class QKeyEvent;

class Texture_Viewer : public BasicViewer
{
public:
	Texture_Viewer(QWidget *widget);
	~Texture_Viewer();

  void updateBuffer();
  void updateColorBuffer();
  void toggleLightball();
  void resetCamera();
  inline void setShowTrackball(bool state) { show_trackball = state; };

  void updateCamera();
  
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
  void mouseDoubleClickEvent(QMouseEvent * event);
  void wheelEvent(QWheelEvent* e);
  void keyPressEvent(QKeyEvent *e);

private:

  bool wireframe_;
  bool show_trackball;
  bool play_lightball;

private:

};

#endif