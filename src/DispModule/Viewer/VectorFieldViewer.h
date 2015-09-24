#ifndef VectorFieldViewer_H
#define VectorFieldViewer_H

#include "BasicViewer.h"
#include "BasicHeader.h"
#include "BasicDataType.h"
#include <memory>

class VectorFieldViewer : public BasicViewer
{
public:
  VectorFieldViewer(QWidget* widget);
  ~VectorFieldViewer();

  void updateSourceVectorField();
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
  std::vector<double2> line;
};
#endif // !VectorFieldViewer_H
