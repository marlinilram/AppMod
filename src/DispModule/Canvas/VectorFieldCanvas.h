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
  bool displaySourceCrspList();
  bool displayTargetCrspList();
  bool displayUserCrsp();
  bool displayAllCurvesPoints();

  void setScalarField();
  void setFBO();
  void updateSourceField(int update_type = 0);

  void setRenderMode(VectorField::RENDERMODE mode) { render_mode = mode; };

  void addConstrainedLines(std::vector<double2>& line, std::vector<double2>& selectedLine);
  void deleteTargetCurves(std::vector<double2>& line, int& w, int& h, Matrix4f& proj_mat_out);
  void deleteLastLine();
  std::shared_ptr<FeatureLine> getFeatureLine(); 
  void setConstrainedPair(double start[2], double end[2]);
  void getTargetCurves(CURVES& target_curves);

protected:
  std::shared_ptr<FeatureGuided> feature_model;
  GLubyte bgmap[800][800][3];
  float alpha;
  float alpha_bridging;
  int display_step;

  float u_max;
  float v_max;
  float ratio;

  GLuint offscr_color;
  GLuint offscr_depth;
  GLuint offscr_fbo;

  int width, height;

  std::unique_ptr<Bound> bound;

  std::vector<bool> vis_paras;

  VectorField::RENDERMODE render_mode;

  int crsp_oder;

private:
  VectorFieldCanvas(const VectorFieldCanvas&);
  void operator = (const VectorFieldCanvas&);
};

#endif