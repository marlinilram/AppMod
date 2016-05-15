#ifndef PARAINIT_H
#define PARAINIT_H

#include "ParameterMgr.h"

#include "BasicHeader.h"

void InitGlobalParameter()
{
  LG::GlobalParameterMgr::GetInstance()->add_parameter<float>("SField:DistAttenuation", 0.0f);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("TrackballView:ShowTrackball", 1);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:rad", 0.04);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:a", 0.7);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:b", 0.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:w", 0.1);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:c", 2.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:d", 5.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:e", 2.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("SField:crsp_type", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:WinWidth", 1.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:WinCenter", 0.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("SField:Type", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<Matrix4f>("LFeature:rigidTransform", Matrix4f::Identity());
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:renderWithTransform", 0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:registerMethod", 11);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("LFeature:lamd_ARAP", 10.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("LFeature:lamd_flat", 0.01);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("LFeature:lamd_data", 0.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("LFeature:lamd_SField", 0.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("LFeature:lamd_symm", 5.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("LFeature:use_symm", true);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("LFeature:delete_interactive_reverse", false);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:Use_Ext_Feature_Line", 1);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:Vis_Ext_Feature_Line_N_Ring", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("SnapShot:SaveToFile", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("TrackballView:ShowLightball", 0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<Matrix4f>("Lightball:cameraTransform", Matrix4f::Identity());

  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("ShapCrest:source_curves_threshhold", 0.75);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("ShapCrest:source_curves_conntect_threshhold", -0.85);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("ShapCrest:source_curves_show_color", false);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("FeatureGuided:target_curves_threshhold", 0.5);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("Synthesis:scale", 0.07);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("Synthesis:is_wait", true);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:n_ring", 0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:resolution", 1024);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:pry_levels", 5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:patch_size", 10);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:max_iter", 5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("Synthesis:rand_size", 5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("Synthesis:occ", 0.0002);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("Synthesis:bias_rate", 0.1);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("Synthesis:beta_center", 0.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("Synthesis:beta_mult", 5.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("Synthesis:geo_transfer_use_para_map", false);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<cv::Mat>("Synthesis:SrcAppMask");
  LG::GlobalParameterMgr::GetInstance()->add_parameter<cv::Mat>("Synthesis:SrcAppOriginImageMask");
  LG::GlobalParameterMgr::GetInstance()->add_parameter<cv::Mat>("Synthesis:TarAppMask");
  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("ShapeManipulator:Axis", true);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<bool>("DebugOutput:ShowRefineCrspTime", true);
}

#endif