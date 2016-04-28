#ifndef GeometryTransfer_H
#define GeometryTransfer_H

#include <memory>
#include <vector>

class Model;

class GeometryTransfer
{
public:
  GeometryTransfer();
  ~GeometryTransfer();

  void prepareSampleVertex(std::shared_ptr<Model> tar_model, std::vector<int>& v_ids);
  std::string transferDeformation(std::shared_ptr<Model> tar_model, const std::vector<int>& v_ids, const std::vector<float>& v_list, float lamd_move = 5.0, bool use_arap = true);

  void debugDeformation(std::shared_ptr<Model> tar_model);

private:
  GeometryTransfer(const GeometryTransfer&);
  void operator = (const GeometryTransfer&);
};

#endif // !GeometryTransfer_H
