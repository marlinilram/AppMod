#ifndef DetailSynthesis_H
#define DetailSynthesis_H

#include "BasicHeader.h"

#include <cv.h>
#include <memory>

class Model;
class MeshParameterization;
class SynthesisTool;
class Shape;
class CurveGuidedVectorField;
class GLActor;
class KevinVectorField;

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

private:
  void computeFeatureMap(std::vector<cv::Mat>& feature_map, std::vector<std::vector<float> >& feature_list, bool is_src);
  void computeDetailMap(std::vector<cv::Mat>& detail_map, std::vector<cv::Mat>& detail_image, std::shared_ptr<Model> model);
  void computeDisplacementMap(cv::Mat& displacement_map, cv::Mat& displacement_image, std::shared_ptr<Model> model);

private:
  std::shared_ptr<MeshParameterization> mesh_para;
  std::shared_ptr<SynthesisTool>        syn_tool;
  std::shared_ptr<CurveGuidedVectorField> curve_guided_vector_field;
  std::shared_ptr<KevinVectorField> kevin_vector_field;
  cv::Mat displacement_map;
  std::vector<cv::Mat> src_feature_map;
  std::vector<cv::Mat> tar_feature_map;
  std::vector<cv::Mat> src_detail_map; // tar_detail_map is taken from the synthesis result
  std::vector<GLActor> actors;
  int resolution;

private:
  DetailSynthesis(const DetailSynthesis&);
  void operator = (const DetailSynthesis&);
};
#endif // !DetailSynthesis_H