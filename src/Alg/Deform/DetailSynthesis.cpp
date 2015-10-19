#include "DetailSynthesis.h"
#include "Model.h"
#include "MeshParameterization.h"

DetailSynthesis::DetailSynthesis()
{

}

DetailSynthesis::~DetailSynthesis()
{

}

void DetailSynthesis::testMeshPara(std::shared_ptr<Model> model)
{
  mesh_para.reset(new MeshParameterization);

  mesh_para->doMeshParameterization(model);
}