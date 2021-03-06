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
  connect(LFReg_Rigid_PushButton, SIGNAL(clicked()), this, SLOT(runLFRegRigid()));
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
  connect(Synthesis_PushButton, SIGNAL(clicked()), this, SLOT(doSynthesis()));;
  connect(lamd_ARAP_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLamdARAP(double)));
  connect(lamd_flat_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLamdFlat(double)));
  connect(lamd_data_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLamdData(double)));
  connect(lamd_SField_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setLamdSField(double)));
  connect(source_curves_t1_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSourceCurvesThreshold(double)));
  connect(source_curves_t3_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSourceCurvesConnectThreshold(double)));
  connect(target_curves_t2_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setTargetCurvesThreshold(double)));
  connect(SField_c_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldExpc(double)));
  connect(SField_d_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldExpd(double)));
  connect(SField_e_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSFieldExpe(double)));
  connect(TestApplyDisplacement_PushButton, SIGNAL(clicked()), this, SLOT(testApplyDisplacement()));
  connect(RunApplyDisplacement_pushButton, SIGNAL(clicked()), this, SLOT(runApplyDisplacement()));
  connect(Synthesis_Scale_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSynthesisScale(double)));
  connect(Crsp_Type_ComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setCrspTypeMode(int)));
  connect(RunLoadDetailMap_PushButton, SIGNAL(clicked()), this, SLOT(runLoadDetailMap()));
  connect(Show_Color_Crest_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(setShowColorCrest(int)));
  connect(N_Ring_DoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setNRing(double)));
  connect(checkBox_Use_Ext_Feature_line, SIGNAL(stateChanged(int)), this, SLOT(useExtFeatureLine(int)));
  connect(spinBox_Ext_N_Ring, SIGNAL(valueChanged(int)), this, SLOT(setExtNRing(int)));
  connect(pushButton_Run_Debug_Alg, SIGNAL(clicked()), this, SLOT(runDebugAlg()));
  connect(doubleSpinBox_Lamd_Symm, SIGNAL(valueChanged(double)), this, SLOT(setLamdSymmetry(double)));
  connect(checkBox_Use_Symm, SIGNAL(stateChanged(int)), this, SLOT(setUseSymmetry(int)));
  // set feature render mode
  QList<QCheckBox*> checkBox_FeatureRenderMode = FeatureViewGroupBox->findChildren<QCheckBox*>();
  for (int i = 0; i < checkBox_FeatureRenderMode.size(); ++i)
  {
    connect(checkBox_FeatureRenderMode.at(i), SIGNAL(stateChanged(int)), this, SLOT(setFeatureRender(int)));
  }

  this->setSlotsSynParamters();
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

void ParameterDock::runLFRegRigid()
{
  disp_modules->runLFRegRigid();
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

void ParameterDock::setSFieldExpc(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:c") = val;
  disp_modules->setSFieldPara(4);
}

void ParameterDock::setSFieldExpd(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:d") = val;
  disp_modules->setSFieldPara(4);
}

void ParameterDock::setSFieldExpe(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("SField:e") = val;
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
  //LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:renderWithTransform") = state;
  //disp_modules->setMainCanvasRenderMode();
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

void ParameterDock::doSynthesis()
{
  disp_modules->doSynthesis();
}

void ParameterDock::setLamdARAP(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("LFeature:lamd_ARAP") = val;
}

void ParameterDock::setLamdFlat(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("LFeature:lamd_flat") = val;
}

void ParameterDock::setLamdData(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("LFeature:lamd_data") = val;
}

void ParameterDock::setLamdSField(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("LFeature:lamd_SField") = val;
}

void ParameterDock::setSourceCurvesThreshold(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("ShapCrest:source_curves_threshhold") = val;
  disp_modules->updateShapeCrest();
}

void ParameterDock::setSourceCurvesConnectThreshold(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("ShapCrest:source_curves_conntect_threshhold") = val;
  disp_modules->updateShapeCrest();
}

void ParameterDock::setTargetCurvesThreshold(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("FeatureGuided:target_curves_threshhold") = val;
  disp_modules->updateTargetCurves();
}

void ParameterDock::testApplyDisplacement()
{
  disp_modules->testApplyDisplacement();
}

void ParameterDock::setSynthesisScale(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:scale") = val;
}

void ParameterDock::runApplyDisplacement()
{
  disp_modules->runApplyDisplacement();
}

void ParameterDock::runLoadDetailMap()
{
  disp_modules->loadDetailMap();
}

void ParameterDock::setCrspTypeMode(int type_mode)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("SField:crsp_type") = type_mode;
  disp_modules->updateSField(6);
}

void ParameterDock::setNRing(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:n_ring") = int(val);
}

void ParameterDock::setShowColorCrest(int state)
{
  if (state != 0) LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("ShapCrest:source_curves_show_color") = true;
  else LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("ShapCrest:source_curves_show_color") = false;
}

void ParameterDock::setSlotsSynParamters()
{
  connect(Synthesis_Res_spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSynResolution(int)));
  connect(Synthesis_PryLevels_spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSynPryLevels(int)));
  connect(Synthesis_PatchSize_spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSynPatchSize(int)));
  connect(Synthesis_MaxIter_spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSynMaxIter(int)));
  connect(Synthesis_RandSize_spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSynRandSize(int)));

  connect(Synthesis_occ_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSynOcc(double)));
  connect(Synthesis_biasrate_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSynBiasRate(double)));
  connect(Synthesis_betacenter_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSynBetaCenter(double)));
  connect(Synthesis_betamultdoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setSynBetaMult(double)));

  connect(Synthesis_GeometryTransfer_ParaMap_checkBox, SIGNAL(stateChanged(int)), this, SLOT(setSynGeometryTransferParaMap(int)));
}

void ParameterDock::setSynResolution(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:resolution") = val;
;
}

void ParameterDock::setSynPryLevels(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:pry_levels") = val;

}

void ParameterDock::setSynPatchSize(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:patch_size") = val;
}

void ParameterDock::setSynMaxIter(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:max_iter") = val;
}

void ParameterDock::setSynRandSize(int val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("Synthesis:rand_size") = val;
}

void ParameterDock::setSynOcc(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:occ") = val;
}

void ParameterDock::setSynBiasRate(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:bias_rate") = val;
}

void ParameterDock::setSynBetaCenter(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_center") = val;
}

void ParameterDock::setSynBetaMult(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("Synthesis:beta_mult") = val;
}

void ParameterDock::setSynGeometryTransferParaMap(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("Synthesis:geo_transfer_use_para_map") = ((state == 0) ? false : true);
}

void ParameterDock::useExtFeatureLine(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:Use_Ext_Feature_Line") = state;
  disp_modules->useExtFeatureLine();
}

void ParameterDock::setExtNRing(int val)
{
  // set the n ring region for visible feature line discrimination
  LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:Vis_Ext_Feature_Line_N_Ring") = val;
}

void ParameterDock::runDebugAlg()
{
  disp_modules->runDebugAlg();
}

void ParameterDock::setLamdSymmetry(double val)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<double>("LFeature:lamd_symm") = val;
}

void ParameterDock::setUseSymmetry(int state)
{
  LG::GlobalParameterMgr::GetInstance()->get_parameter<bool>("LFeature:use_symm") = ((state == 0) ? false : true);
}