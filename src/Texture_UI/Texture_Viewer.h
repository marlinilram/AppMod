#ifndef Texture_Viewer_H
#define Texture_Viewer_H

#include "BasicViewer.h"
#include <opencv2/core/core.hpp>
#include <memory>

class MainCanvasViewer;
class VectorFieldViewer;
class QMouseEvent;
class QKeyEvent;
class QResizeEvent;
class Texture_Viewer : public BasicViewer
{
	Q_OBJECT
public:
	Texture_Viewer(QWidget *widget);
	~Texture_Viewer();

  void updateBuffer();
  void updateColorBuffer();
  void toggleLightball();
  void resetCamera();
  inline void setShowTrackball(bool state) { show_trackball = state; };

  void updateCamera();
  void clear_selection();
  void set_edit_mode(int);
  bool get_edit_mode();
  bool draw_mesh_points();
  const std::vector<std::vector<int>>& get_boundaries();

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
  void draw_points_under_mouse();
  void resizeEvent(QResizeEvent*);


private:

  bool wireframe_;
  bool show_trackball;
  bool play_lightball;

  bool m_left_button_down_;
  bool m_right_button_down_;
  int  m_edit_mode_;


  std::vector<bool>   m_faces_selected_;
  std::vector<std::vector<int>>   m_boundaries_;;
  std::vector<QPoint> m_points_for_delete_;
  int k_start_;
  bool m_show_mesh_;
private:

};

#endif