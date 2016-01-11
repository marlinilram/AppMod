#ifndef NormalTransfer_H
#define NormalTransfer_H

#include <memory>
#include <vector>
#include <string>
#include <set>
#include "GLActor.h"

#include "BasicHeader.h"

class Model;
class Solver;
class ARAP;
class NormalConstraint;

class NormalTransfer
{
public:
  NormalTransfer();
  ~NormalTransfer();

  void prepareNewNormal(std::shared_ptr<Model> model, std::string normal_file_name);
  void getDrawableActors(std::vector<GLActor>& actors);
  void visibleFaceInNormalMap(std::shared_ptr<Model> model, std::string normal_file_name, std::set<int>& face_in_normal);
  void visibleVertexInNormalMap(std::shared_ptr<Model> model, std::string normal_file_name, STLVectori& vertex_in_normal);

private:
  std::vector<GLActor> actors;

  std::shared_ptr<Solver> solver;
  std::shared_ptr<ARAP> arap; // or use FMS here
  std::shared_ptr<NormalConstraint> normal_constraint;

private:
  NormalTransfer(const NormalTransfer&);
  void operator = (const NormalTransfer&);
};

#endif // !NormalTransfer_H
