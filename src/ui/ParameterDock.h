#ifndef ParameterDock_H
#define ParameterDock_H

#include <QDockWidget>
#include "ui_ParameterDock.h"

#include <memory>

class DispModuleHandler;

class ParameterDock : public QDockWidget, public Ui::ParameterDock
{
  Q_OBJECT;

public:
  ParameterDock();
  ~ParameterDock();

  void setDispModules(std::shared_ptr<DispModuleHandler> modules);
  void initFeatureRender();
  void setInitPara();

private slots:
  void setFeatureRender(int state);
  void setEdgeThreshold(int val);
  void setUseFlat(int state);
  void showAllLines(int state);
  void showProjCrsp(int state);
  void setInteractiveProjAlign(int state);
  void setInteractiveMainView(int state);
  void showBackgroundImage(int state);
  void runNormalTransfer();
  void runNormalCompute();
  void runDetailSynthesis();
  void setDistAttenuation(int val);
  void setShowTrackball(int state);
  void changeToLightball(int state);
  void doSynthesis();

  void setSFieldRad(double val);
  void setSFieldExpa(double val);
  void setSFieldExpb(double val);
  void setSFieldParaw(double val);
  void setSFieldWinCenter(int val);
  void setSFieldWinWidth(int val);
  void setSFieldType(int val);
  void setMainRenderMode(int state);

  void setLFRegMethod(int state);
  void runLFRegNonRigid();

private:
  std::shared_ptr<DispModuleHandler> disp_modules;
};

#endif // !ParameterDock_H
