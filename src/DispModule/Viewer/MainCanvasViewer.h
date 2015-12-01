#ifndef MainCanvasViewer_H
#define MainCanvasViewer_H

#include "BasicViewer.h"
#include "GLActor.h"
#include <QString>

namespace MainViewer
{
  enum INTERACTIONMODE
  {
    STATIC = 0,
    TAG_PLANE = 1
  };
};

class MainCanvasViewer : public BasicViewer
{
public:
  MainCanvasViewer(QWidget *widget);
  ~MainCanvasViewer();

  void getSnapShot();
  void setBackgroundImage(QString fname);
  void setReflectanceImage(QString fname);
  void updateBuffer();
  void updateColorBuffer();
  void setGLActors(std::vector<GLActor>& actors);
  inline void setIsDrawActors(bool state) { is_draw_actors = state; };
  void syncCameraToModel();
  inline void setShowBackground(int state) { show_background = (state == 0) ? false : true; };
  inline void setInteractionMode(MainViewer::INTERACTIONMODE mode) { interaction_mode = mode; };
  void clearPreviousInteractionInfo();
  void updateCanvasRenderMode();

private:
  void mouseReleaseEvent(QMouseEvent *e);
  void keyPressEvent(QKeyEvent *e);

private:
  std::vector<GLActor> actors;

  bool is_draw_actors;
  bool show_background;
  bool show_wireframe;

  MainViewer::INTERACTIONMODE interaction_mode;

protected:
  virtual void draw();
  virtual void init();
  void drawActors();
};

#endif