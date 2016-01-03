#ifndef SynthesisViewer_H
#define SynthesisViewer_H

#include "BasicViewer.h"

#include <memory>

class GLActor;
class QKeyEvent;

class SynthesisViewer : public BasicViewer
{
public:
  SynthesisViewer(QWidget* widget);
  ~SynthesisViewer();

  void resetCamera();

  void setGLActors(std::vector<GLActor>& actors);
  inline void setIsDrawActors(bool state) { is_draw_actors = state; };

protected:
  virtual void draw();
  virtual void init();
  void drawActors();

private:
  void keyPressEvent(QKeyEvent *e);

private:
  std::vector<GLActor> actors;
  bool is_draw_actors;
  bool is_draw_objects;
  bool wireframe_;
};

#endif // !SynthesisViewer_H
