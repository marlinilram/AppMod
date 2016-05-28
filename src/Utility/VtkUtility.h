#ifndef VtkUtility_H

#include "BasicHeader.h"
#include "PolygonMesh.h"

namespace VtkUtility
{
  void getCurvature(LG::PolygonMesh* mesh);

  STLVectori getConnectedComponent(LG::PolygonMesh* mesh, LG::PolygonMesh::Face face);
  void cutMesh(LG::PolygonMesh* mesh, std::vector<STLVectori>& components);
}

#endif // !VtkUtility_H
