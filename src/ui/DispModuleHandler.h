#ifndef DispModuleHandler_H
#define DispModuleHandler_H

#include <memory>
#include <string>
#include <vector>
#include <QObject>

class QWidget;
class MainCanvas;
class MainCanvasViewer;
class TrackballCanvas;
class TrackballViewer;
class VectorFieldCanvas;
class VectorFieldViewer;
class SynthesisCanvas;
class SynthesisViewer;
class AlgHandler;
class Model;

class DispModuleHandler : public QObject
{
  Q_OBJECT

public:
  DispModuleHandler() {};
  ~DispModuleHandler() {};

  DispModuleHandler(QWidget* parent);

  void loadModel(std::shared_ptr<Model> model, std::string model_file_path);
  void exportOBJ();
  void snapShot();
  void updateGeometry();
  void initFeatureModel();
  void updateCanvas();
  void setEdgeThreshold(int val);
  void setUseFlat(int state);
  void showCrspLines(int state);
  void showProjCrsp(int state);
  void deleteLastCrspLine_Source();
  void deleteLastCrspLine_Target();
  void setVectorFieldViewerPara(std::vector<bool>& checkStates);
  void toggleVectorFieldMode(int state);
  void toggleMainViewMode(int state);
  void showBackgroundImage(int state);
  void runNormalTransfer();
  void runNormalCompute();
  void runDetailSynthesis();
  void resetCamera();
  void setSFieldDistAttenuation();
  void setShowTrackball();
  void setSFieldPara(int set_type = 1);
  void setMainCanvasRenderMode();
  void runLFRegNonRigid();
  void changeToLightball();
  void doSynthesis();
  void updateShapeCrest();
  void updateTargetCurves();
  void loadSynthesisTarget(std::shared_ptr<Model> model, std::string model_file_path);

public slots:
  void updateGeometryInteractive();

public:
  std::shared_ptr<MainCanvas> main_canvas;
  std::shared_ptr<MainCanvasViewer> main_canvas_viewer;

  std::shared_ptr<TrackballCanvas> trackball_canvas;
  std::shared_ptr<TrackballViewer> trackball_viewer;

  std::shared_ptr<VectorFieldCanvas> source_vector_canvas, target_vector_canvas;
  std::shared_ptr<VectorFieldViewer> source_vector_viewer, target_vector_viewer;

  std::shared_ptr<SynthesisCanvas> synthesis_canvas;
  std::shared_ptr<SynthesisViewer> synthesis_viewer;

  std::shared_ptr<AlgHandler> alg_handler;

  std::string cur_file_path;

private:
  DispModuleHandler(const DispModuleHandler&);
  void operator = (const DispModuleHandler&);
};
#endif // !DispModuleHandler_H
