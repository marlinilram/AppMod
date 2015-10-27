#ifndef DetailSynthesis_H
#define DetailSynthesis_H

#include <cv.h>
#include <memory>

class Model;
class MeshParameterization;

class DetailSynthesis
{
public:
  DetailSynthesis();
  ~DetailSynthesis();

  void testMeshPara(std::shared_ptr<Model> model);
  void computeDisplacementMap(std::shared_ptr<Model> model);

private:
  std::shared_ptr<MeshParameterization> mesh_para;
  cv::Mat displacement_map;

private:
  DetailSynthesis(const DetailSynthesis&);
  void operator = (const DetailSynthesis&);
};
#endif // !DetailSynthesis_H
