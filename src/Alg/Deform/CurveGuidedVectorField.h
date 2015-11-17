#ifndef CurveGuidedVectorField_H
#define CurveGuidedVectorField_H

#include "BasicHeader.h"
#include <memory>

class Model;
class ARAP;
class GLActor;

class CurveGuidedVectorField
{
public:
  CurveGuidedVectorField();
  ~CurveGuidedVectorField();

  void computeVectorField(std::shared_ptr<Model> model);
  void projectVectorField(std::shared_ptr<Model> model, float low, float high);
  void getDrawableActors(std::vector<GLActor>& actors);

private:
  std::vector<Vector3f> vector_field;
  std::shared_ptr<ARAP> arap;
  std::vector<GLActor> actors;
};

#endif

