#ifndef DetailSynthesis_H
#define DetailSynthesis_H

#include "BasicHeader.h"

#include <cv.h>
#include <memory>

class Model;
class MeshParameterization;
class SynthesisTool;
class Shape;
class ParaShape;
class CurveGuidedVectorField;
class GLActor;
class KevinVectorField;
namespace LG {
class PolygonMesh;
}

class DetailSynthesis
{
public:
  DetailSynthesis();
  ~DetailSynthesis();

  void testMeshPara(std::shared_ptr<Model> model);
  void computeDisplacementMap(std::shared_ptr<Model> model); //old one; useless
  void prepareFeatureMap(std::shared_ptr<Model> model);
  void prepareDetailMap(std::shared_ptr<Model> model);
  void applyDisplacementMap(STLVectori vertex_set, std::shared_ptr<Shape> cut_shape, std::shared_ptr<Model> model, cv::Mat disp_map);
  void startDetailSynthesis(std::shared_ptr<Model> model);
  void computeVectorField(std::shared_ptr<Model> model);
  void getDrawableActors(std::vector<GLActor>& actors);
  void testShapePlane(std::shared_ptr<Model> model);
  void mergeSynthesis(ParaShape* para_shape, std::shared_ptr<Model> model);
  void patchSynthesis(std::shared_ptr<Model> model);

  // transfer
  void doTransfer(std::shared_ptr<Model> src_model, std::shared_ptr<Model> tar_model);
  void test(std::shared_ptr<Model> model);

private:
  void computeFeatureMap(ParaShape* para_shape, std::vector<std::vector<float> >& feature_list);
  void computeDetailMap(ParaShape* para_shape, std::vector<cv::Mat>& detail_image, std::shared_ptr<Model> model, std::set<int>& visible_faces);
  void computeDisplacementMap(ParaShape* para_shape, LG::PolygonMesh* displacement_mesh, std::shared_ptr<Model> model, std::set<int>& visible_faces);

private:
  std::shared_ptr<MeshParameterization> mesh_para;
  std::shared_ptr<SynthesisTool>        syn_tool;
  std::shared_ptr<CurveGuidedVectorField> curve_guided_vector_field;
  std::shared_ptr<KevinVectorField> kevin_vector_field;
  std::vector<GLActor> actors;
  int resolution;
  double normalize_max;

private:
  DetailSynthesis(const DetailSynthesis&);
  void operator = (const DetailSynthesis&);
};
#endif // !DetailSynthesis_H