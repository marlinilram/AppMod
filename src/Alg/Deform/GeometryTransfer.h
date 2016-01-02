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
  void transferDeformation(std::shared_ptr<Model> tar_model, const std::vector<int>& v_ids, const std::vector<float>& v_list);

private:
  GeometryTransfer(const GeometryTransfer&);
  void operator = (const GeometryTransfer&);
};

#endif // !GeometryTransfer_H
