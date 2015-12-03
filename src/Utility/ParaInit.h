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
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:b", 0.3);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:w", 0.1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:WinWidth", 1.0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<double>("SField:WinCenter", 0.5);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("SField:Type", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<Matrix4f>("LFeature:rigidTransform", Matrix4f::Identity());
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:renderWithTransform", 0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("LFeature:registerMethod", 11);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("SnapShot:SaveToFile", 1);

  LG::GlobalParameterMgr::GetInstance()->add_parameter<int>("TrackballView:ShowLightball", 0);
  LG::GlobalParameterMgr::GetInstance()->add_parameter<Matrix4f>("Lightball:cameraTransform", Matrix4f::Identity());
}

#endif