#ifndef VectorFieldViewer_H
#define VectorFieldViewer_H

#include "BasicViewer.h"
#include "BasicHeader.h"
#include <memory>

class VectorFieldViewer : public BasicViewer
{
public:
  VectorFieldViewer(QWidget* widget);
  ~VectorFieldViewer();

  void updateSourceVectorField();
  void setConstrainedPoints();
  void deleteLastLine();

protected:
  virtual void draw();
  virtual void init();
  void drawLines();
  //virtual void postSelection(const QPoint& point);

private:
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  
  bool drawLine;
  std::vector<Vector2f> line;
  std::vector<std::vector<Vector2f>> lines;
};
#endif // !VectorFieldViewer_H
