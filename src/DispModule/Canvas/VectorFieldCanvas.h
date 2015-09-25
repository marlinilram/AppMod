#ifndef FeatureGuidedVis_H
#define FeatureGuidedVis_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"
#include "BasicHeader.h"
#include "BasicDataType.h"


#include <memory>
#include <vector>

namespace VectorField
{
enum RENDERMODE
{
  TARGET_MODE = 0,
  SOURCE_MODE = 1
};
};

class FeatureGuided;
class FeatureLine;

class VectorFieldCanvas : public DispObject
{
public:
  VectorFieldCanvas();
  virtual ~VectorFieldCanvas();

  virtual bool display();
  virtual Bound* getBoundBox();
  virtual void setGLProperty();

  void setFeatureModel(std::shared_ptr<FeatureGuided> model);
  void setVisualizationParas(std::vector<bool>& paras);

  bool displayVectorField();
  bool displayTargetVectorField();
  bool displayTargetCurves();
  bool displaySourceCurves();
  bool displayScalarField();
  bool displayFittedCurves();
  bool displayHistMatchPts();

  void setScalarField();
  void updateSourceVectorField();

  void setRenderMode(VectorField::RENDERMODE mode) { render_mode = mode; };

  void addConstrainedLines(std::vector<double2>& line);
  void deleteLastLine();
  std::shared_ptr<FeatureLine> getFeatureLine(); 

protected:
  std::shared_ptr<FeatureGuided> feature_model;
  GLubyte bgmap[800][800][3];
  float alpha;
  float alpha_bridging;
  int display_step;

  float u_max;
  float v_max;
  float ratio;

  std::unique_ptr<Bound> bound;

  std::vector<bool> vis_paras;

  VectorField::RENDERMODE render_mode;

private:
  VectorFieldCanvas(const VectorFieldCanvas&);
  void operator = (const VectorFieldCanvas&);
};

#endif