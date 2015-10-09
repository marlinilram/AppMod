#include "ParameterDock.h"
#include "DispModuleHandler.h"

ParameterDock::ParameterDock()
{
  setupUi(this);

  connect(edgeThresholdSlider, SIGNAL(valueChanged(int)), this, SLOT(setEdgeThreshold(int)));
  connect(flatCheckbox, SIGNAL(stateChanged(int)), this, SLOT(setUseFlat(int)));
  connect(Show_All_Lines_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(showAllLines(int)));
  connect(Show_ProjCrsp_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(showProjCrsp(int)));

  // set feature render mode
  QList<QCheckBox*> checkBox_FeatureRenderMode = groupBox_2->findChildren<QCheckBox*>();
  for (int i = 0; i < checkBox_FeatureRenderMode.size(); ++i)
  {
    connect(checkBox_FeatureRenderMode.at(i), SIGNAL(stateChanged(int)), this, SLOT(setFeatureRender(int)));
  }
}

ParameterDock::~ParameterDock()
{

}

void ParameterDock::setDispModules(std::shared_ptr<DispModuleHandler> modules)
{
  disp_modules = modules;
}

void ParameterDock::setFeatureRender(int state)
{
  //QList<QCheckBox*> checkBox_FeatureRenderMode = groupBox_2->findChildren<QCheckBox*>();
  //std::vector<bool> checkStates;
  //for (int i = 0; i < checkBox_FeatureRenderMode.size(); ++i)
  //{
  //  checkStates.push_back(checkBox_FeatureRenderMode.at(i)->checkState());
  //}
  //feature_guided->setVissualizationPara(checkStates);
}

void ParameterDock::setEdgeThreshold(int val)
{
  disp_modules->setEdgeThreshold(val);
}

void ParameterDock::setUseFlat(int state)
{
  disp_modules->setUseFlat(state);
}

void ParameterDock::showAllLines(int state)
{
  disp_modules->showCrspLines(state);
}

void ParameterDock::showProjCrsp(int state)
{
  disp_modules->showProjCrsp(state);
}