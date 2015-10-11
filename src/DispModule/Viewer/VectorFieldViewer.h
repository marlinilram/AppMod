#ifndef VectorFieldViewer_H
#define VectorFieldViewer_H

#include "BasicViewer.h"
#include "BasicHeader.h"
#include "BasicDataType.h"
#include <memory>

namespace VectorField
{
  enum INTERACTIONMODE
  {
    DRAW_CRSP_LINE = 0,
    SELECT_POINT = 1
  };
};

class VectorFieldViewer : public BasicViewer
{
  Q_OBJECT

public:
  VectorFieldViewer(QWidget* widget);
  ~VectorFieldViewer();

  void updateSourceField();
  void updateScalarFieldTexture();
  void deleteLastLine();
  void isDrawAllLines(bool allLines);
  void setDispPara(std::vector<bool>& states);
  bool inDrawLineMode();
  bool inSelectPointMode();
  inline void setInteractionMode(VectorField::INTERACTIONMODE mode) { interaction_mode = mode; };

signals:
  void triggeredInteractiveCrsp();

protected:
  virtual void draw();
  virtual void init();
  void drawLine();
  void drawAllLines();
  //virtual void postSelection(const QPoint& point);

private:
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);

  bool is_drawLine;
  bool is_drawAllLines;
  std::vector<double2> line;

  int selected_v_id;
  double user_start[2];
  double user_end[2];

  VectorField::INTERACTIONMODE interaction_mode;
};
#endif // !VectorFieldViewer_H
