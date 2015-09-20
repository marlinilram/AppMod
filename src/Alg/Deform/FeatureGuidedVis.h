#ifndef FeatureGuidedVis_H
#define FeatureGuidedVis_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <memory>
#include <vector>

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
  void setVisualizationParas(std::vector<bool>& paras);

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

  std::unique_ptr<Bound> bound;

  std::vector<bool> vis_paras;

private:
  FeatureGuidedVis(const FeatureGuidedVis&);
  void operator = (const FeatureGuidedVis&);
};

#endif