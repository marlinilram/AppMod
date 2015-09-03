#ifndef FeatureGuidedVis_H
#define FeatureGuidedVis_H

#include "BasicViewer.h"
#include "DispObject.h"

class FeatureGuided;

class FeatureGuidedVis : public DispObject
{
public:
  FeatureGuidedVis();
  virtual ~FeatureGuidedVis();

  virtual bool display();
  virtual Bound* getBoundBox();
  virtual void setGLProperty();

  void init(FeatureGuided* init_data_ptr);

  bool displayVectorField();
  bool displayTargetVectorField();
  bool displayTargetCurves();
  bool displaySourceCurves();
  bool displayScalarField();
  bool displayFittedCurves();
  bool displayHistMatchPts();

  void setScalarField();

protected:
  FeatureGuided* data_ptr;
  GLubyte bgmap[800][800][3];
  float alpha;
  float alpha_bridging;
  int display_step;

  float u_max;
  float v_max;
  float ratio;

private:
  FeatureGuidedVis(const FeatureGuidedVis&);
  void operator = (const FeatureGuidedVis&);
};

#endif