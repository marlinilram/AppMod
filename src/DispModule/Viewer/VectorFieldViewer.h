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
    SELECT_POINT = 1,
    CORRECT_CRSP = 2,
    DELETE_TARGET_CURVES = 3,
    ADD_TARGET_CURVES = 4
  };
};

class VectorFieldViewer : public BasicViewer
{
  Q_OBJECT

public:
  VectorFieldViewer(QWidget* widget);
  ~VectorFieldViewer();

  void updateSourceField(int update_type = 0);
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
  void drawSelectedLine();
  //virtual void postSelection(const QPoint& point);

private:
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);

  bool is_drawLine;
  bool is_drawAllLines;
  bool delete_target_mode;
  std::vector<double2> line;
  std::vector<double2> selected_line;

  int selected_v_id;
  double user_start[2];
  double user_end[2];

  Matrix4f proj_mat_out;

  VectorField::INTERACTIONMODE interaction_mode;
};
#endif // !VectorFieldViewer_H
