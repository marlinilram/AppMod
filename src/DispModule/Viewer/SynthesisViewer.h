#ifndef SynthesisViewer_H
#define SynthesisViewer_H

#include "BasicViewer.h"

#include <memory>

class GLActor;

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
  std::vector<GLActor> actors;
  bool is_draw_actors;
};

#endif // !SynthesisViewer_H
