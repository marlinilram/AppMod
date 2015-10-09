#ifndef MainCanvasViewer_H
#define MainCanvasViewer_H

#include "BasicViewer.h"
#include "GLActor.h"
#include <QString>

class MainCanvasViewer : public BasicViewer
{
public:
  MainCanvasViewer(QWidget *widget);
  ~MainCanvasViewer();

  void getSnapShot();
  void setBackgroundImage(QString fname);
  void updateBuffer();
  void setGLActors(std::vector<GLActor>& actors);
  inline void setIsDrawActors(bool state) { is_draw_actors = state; };

private:
  std::vector<GLActor> actors;

  bool is_draw_actors;

protected:
  virtual void draw();
  virtual void init();
  void drawActors();
};

#endif