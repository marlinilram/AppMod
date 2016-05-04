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
  void runLFRegRigid();
  void setDistAttenuation(int val);
  void setShowTrackball(int state);
  void changeToLightball(int state);
  void doSynthesis();

  void setSFieldRad(double val);
  void setSFieldExpa(double val);
  void setSFieldExpb(double val);
  void setSFieldExpc(double val);
  void setSFieldExpd(double val);
  void setSFieldExpe(double val);
  void setSFieldParaw(double val);
  void setSFieldWinCenter(int val);
  void setSFieldWinWidth(int val);
  void setSFieldType(int val);
  void setMainRenderMode(int state);
  void setLamdARAP(double val);
  void setLamdFlat(double val);
  void setLamdData(double val);
  void setLamdSField(double val);
  void setLamdSymmetry(double val);
  void setUseSymmetry(int state);
  void setSourceCurvesThreshold(double val);
  void setSourceCurvesConnectThreshold(double val);
  void setShowColorCrest(int state);
  void setTargetCurvesThreshold(double val);
  void setSynthesisScale(double val);
  void setNRing(double val);

  void setSlotsSynParamters();
  void setSynResolution(int val);
  void setSynPryLevels(int val);
  void setSynPatchSize(int val);
  void setSynMaxIter(int val);
  void setSynRandSize(int val);
  void setSynOcc(double val);
  void setSynBiasRate(double val);
  void setSynBetaCenter(double val);
  void setSynBetaMult(double val);


  void setLFRegMethod(int state);
  void runLFRegNonRigid();
  void testApplyDisplacement();
  void runApplyDisplacement();
  void runLoadDetailMap();

  void setSynGeometryTransferParaMap(int state);

  void setCrspTypeMode(int type_mode);

  // 5/1/2016
  void useExtFeatureLine(int state);
  void setExtNRing(int val);
  void runDebugAlg();

private:
  std::shared_ptr<DispModuleHandler> disp_modules;
};

#endif // !ParameterDock_H
