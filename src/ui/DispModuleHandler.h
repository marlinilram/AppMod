#ifndef DispModuleHandler_H
#define DispModuleHandler_H

#include <memory>
#include <string>

class QWidget;
class MainCanvas;
class MainCanvasViewer;
class TrackballCanvas;
class TrackballViewer;
class VectorFieldCanvas;
class VectorFieldViewer;
class AlgHandler;
class Model;

class DispModuleHandler
{
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

public:
  std::shared_ptr<MainCanvas> main_canvas;
  std::shared_ptr<MainCanvasViewer> main_canvas_viewer;

  std::shared_ptr<TrackballCanvas> trackball_canvas;
  std::shared_ptr<TrackballViewer> trackball_viewer;

  std::shared_ptr<VectorFieldCanvas> source_vector_canvas, target_vector_canvas;
  std::shared_ptr<VectorFieldViewer> source_vector_viewer, target_vector_viewer;

  std::shared_ptr<AlgHandler> alg_handler;

private:
  DispModuleHandler(const DispModuleHandler&);
  void operator = (const DispModuleHandler&);
};
#endif // !DispModuleHandler_H
