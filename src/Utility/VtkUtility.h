#ifndef VtkUtility_H

#include "BasicHeader.h"

namespace LG {
  class PolygonMesh;
}


namespace VtkUtility
{
  void getCurvature(LG::PolygonMesh* mesh);
}

#endif // !VtkUtility_H
