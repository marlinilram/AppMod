#include "ParameterDock.h"
#include "DispModuleHandler.h"
#include "ParameterMgr.h"

ParameterDock::ParameterDock()
{
  setupUi(this);

  setInitPara();

  connect(edgeThresholdSlider, SIGNAL(valueChanged(int)), this, SLOT(setEdgeThreshold(int)));
  connect(flatCheckbox, SIGNAL(stateChanged(int)), this, SLOT(setUseFlat(int)));
  connect(Show_All_Lines_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(showAllLines(int)));
  connect(Show_ProjCrsp_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(showProjCrsp(int)));
  connect(Proj_Interact_Mode_ComboBox, SIGNAL(currentIndexChanged(int)), SLOT(setInteractiveProjAlign(int)));
  connect(Show_Image_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(showBackgroundImage(int)));
  connect(NormalTransfer_PushButton, SIGNAL(clicked()), this, SLOT(runNormalTransfer()));
  connect(NormalCompute_PushButton, SIGNAL(clicked()), this, SLOT(runNormalCompute()));
  connect(DetailSynthesis_PushButton, SIGNAL(clicked()), this, SLOT(runDetailSynthesis()));
  connect(Main_Interact_Mode_ComboBox, SIGNAL(currentIndexChanged(int)), SLOT(setInteractiveMainView(int)));
  connect(Show_Trackball_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(setShowTrackball(int)));
  connect(SField_rad_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldRad(double)));
  connect(SField_a_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldExpa(double)));
  connect(SField_b_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldExpb(double)));
  connect(SField_w_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldParaw(double)));
  connect(SField_WinCenter_Slider, SIGNAL(valueChanged(int)), this, SLOT(setSFieldWinCenter(int)));
  connect(SField_WinWidth_Slider, SIGNAL(valueChanged(int)), this, SLOT(setSFieldWinWidth(int)));
  connect(Main_Render_Mode_ComboBox, SIGNAL(currentIndexChanged(int)), SLOT(setMainRenderMode(int)));
  connect(LFReg_Method_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(setLFRegMethod(int)));
  connect(SField_Type_ComboBox, SIGNAL(currentIndexChanged(int)), SLOT(setSFieldType(int)));
  connect(LFReg_NonRigid_PushButton, SIGNAL(clicked()), this, SLOT(runLFRegNonRigid()));
  connect(Show_Light_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(changeToLightball(int)));

  // set feature render mode
  QList<QCheckBox*> checkBox_FeatureRenderMode = FeatureViewGroupBox->findChildren<QCheckBox*>();
  for (int i = 0; i < checkBox_FeatureRenderMode.size(); ++i)
  {
    connect(checkBox_FeatureRenderMode.at(i), SIGNAL(stateChanged(int)), this, SLOT(setFeatureRender(int)));
  }
}

ParameterDock::~ParameterDock()
{

}

void ParameterDock::setInitPara()
{

}

void ParameterDock::setDispModules(std::shared_ptr<DispModuleHandler> modules)
{
  disp_modules = modules;

  // init the checkBox list in vector field canvas
  //this->setFeatureRender(1);
}

void ParameterDock::setFeatureRender(int state)
{
  QList<QCheckBox*> checkBox_FeatureRenderMode = FeatureViewGroupBox->findChildren<QCheckBox*>();
  std::vector<bool> checkStates(checkBox_FeatureRenderMode.size());
  for (int i = 0; i < checkBox_FeatureRenderMode.size(); ++i)
  {
    std::string name = checkBox_FeatureRenderMode.at(i)->objectName().toStdString();
    int list_id = std::stoi(name.substr(name.find_last_of('_') + 1));
    checkStates[list_id] = checkBox_FeatureRenderMode.at(i)->checkState();
  }
  disp_modules->setVectorFieldViewerPara(checkStates);
}

void ParameterDock::initFeatureRender()
{
  setFeatureRender(1);
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

void ParameterDock::setInteractiveProjAlign(int state)
{
  disp_modules->toggleVectorFieldMode(state);
}

void ParameterDock::showBackgroundImage(int state)
{
  disp_modules->showBackgroundImage(state);
}

void ParameterDock::runNormalTransfer()
{
  disp_modules->runNormalTransfer();
}

void ParameterDock::runNormalCompute()
{
  disp_modules->runNormalCompute();
}

void ParameterDock::runDetailSynthesis()
{
  disp_modules->runDetailSynthesis();
}

void ParameterDock::setInteractiveMainView(int state)
{
  disp_modules->toggleMainViewMode(state);
}

void ParameterDock::setDistAttenuation(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<float>("SField:DistAttenuation") = double(val) / 99;
  disp_modules->setSFieldDistAttenuation();
}

void ParameterDock::setShowTrackball(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowTrackball") = state;
  disp_modules->setShowTrackball();
}

void ParameterDock::setSFieldRad(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:rad") = val;
  disp_modules->setSFieldPara();
}

void ParameterDock::setSFieldExpa(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:a") = val;
  disp_modules->setSFieldPara(4);
}

void ParameterDock::setSFieldExpb(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:b") = val;
  disp_modules->setSFieldPara(4);
}

void ParameterDock::setSFieldParaw(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:w") = val;
  disp_modules->setSFieldPara();
}

void ParameterDock::setSFieldWinCenter(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinCenter") = double(val) / 100;
  disp_modules->setSFieldPara(3);
}

void ParameterDock::setSFieldWinWidth(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:WinWidth") = double(val) / 100;
  disp_modules->setSFieldPara(3);
}

void ParameterDock::setMainRenderMode(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:renderWithTransform") = state;
  disp_modules->setMainCanvasRenderMode();
}

void ParameterDock::setLFRegMethod(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:registerMethod") = state;
}

void ParameterDock::setSFieldType(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("SField:Type") = val;
  disp_modules->setSFieldPara(1);
}

void ParameterDock::runLFRegNonRigid()
{
  disp_modules->runLFRegNonRigid();
}

void ParameterDock::changeToLightball(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") = state;
  disp_modules->changeToLightball();
}