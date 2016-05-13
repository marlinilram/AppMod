#ifndef Texture_Viewer_H
#define Texture_Viewer_H

#include "BasicViewer.h"
#include <opencv2/core/core.hpp>
#include <memory>
#include <QImage>
class Texture_Mesh_Corres;
class MainCanvasViewer;
class VectorFieldViewer;
class QMouseEvent;
class QKeyEvent;
class MiniTexture;
class GLActor;

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
  QImage snapshot_bounding_box();
  const std::vector<Texture_Mesh_Corres*>& get_textures_mesh_corres();
  void add_textures_mesh_corre(Texture_Mesh_Corres*);
  const std::vector<bool>& get_face_selected(){ return this->m_faces_selected_; };
  void mark_points_out();
  void mark_points();
  void set_texture_now(MiniTexture* m);
  std::vector<int> face_selected();

  void setGLActors(std::vector<GLActor>& actors);

public slots:
	void delete_textures_mesh_corre(Texture_Mesh_Corres*);
	void select_all_unselected();
	void Show_line(bool);
	void show_mini_texture(bool);
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
  void moveEvent(QMoveEvent * event);
  void keyPressEvent(QKeyEvent *e);
  void draw_points_under_mouse();
  void resizeEvent(QResizeEvent*);

  void releaseEvent_no_text_mesh_corres(QMouseEvent *e);
  void releaseEvent_with_text_mesh_corres(QMouseEvent *e);


private:

  bool wireframe_;
  bool show_trackball;
  bool play_lightball;

  bool m_left_button_down_;
  bool m_right_button_down_;
  int  m_edit_mode_;

  std::vector<GLActor> actors;
  bool draw_actors;

  std::vector<bool>   m_faces_selected_;
  std::vector<bool>   m_faces_selected_all_;
  std::vector<std::vector<int>>   m_boundaries_;;
  std::vector<QPoint> m_points_for_delete_;
  std::vector<Texture_Mesh_Corres*>	m_textures_mesh_corres_;
  int k_start_;
  bool m_show_mesh_;
  MiniTexture* m_texture_now_;
  bool m_show_line_;
private:

};

#endif