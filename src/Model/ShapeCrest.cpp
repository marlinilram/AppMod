#include "ShapeCrest.h"
#include "BasicHeader.h"
#include "Shape.h"

#include <set>

ShapeCrest::ShapeCrest()
{

}

ShapeCrest::~ShapeCrest()
{
  std::cout << "Deleted a ShapeCrest.\n";
}

void ShapeCrest::setShape(std::shared_ptr<Shape> in_shape)
{
  shape = in_shape;
  buildCandidates();
}

void ShapeCrest::buildCandidates()
{
  // search all edges with Dihedral angle larger than a threshold
  const NormalList& face_normals = shape->getFaceNormal();
  const FaceList& face_list = shape->getFaceList();
  const STLVectori& edge_connectivity = shape->getEdgeConnectivity();
  std::vector<bool> visited_edge(edge_connectivity.size(), false);
  STLVectorf edge_dihedral(edge_connectivity.size(), -1.0f);

  for (size_t i = 0; i < edge_connectivity.size(); ++i)
  {
    if (!visited_edge[i])
    {
      if (edge_connectivity[i] == -1)
      {
        visited_edge[i] = true;
      }
      else
      {
        // not a boundary edge
        int f_0 = i / 3;
        int f_1 = edge_connectivity[i] / 3;

        Vector3f f0_normal;
        f0_normal << face_normals[3 * f_0 + 0],
                     face_normals[3 * f_0 + 1],
                     face_normals[3 * f_0 + 2];

        Vector3f f1_normal;
        f1_normal << face_normals[3 * f_1 + 0],
                     face_normals[3 * f_1 + 1],
                     face_normals[3 * f_1 + 2];

        float dihedral_cos = f0_normal.dot(f1_normal);

        edge_dihedral[i] = dihedral_cos;
        edge_dihedral[edge_connectivity[i]] = dihedral_cos;
        visited_edge[i] = true;
        visited_edge[edge_connectivity[i]] = true;
      }
    }
  }
  
  std::set<int> candidates;
  crest_edges.clear();
  int inner_index[6] = {0, 1, 1, 2, 2, 0};
  for (size_t i = 0; i < edge_dihedral.size(); ++i)
  {
    if (edge_dihedral[i] < 0.3)
    {
      if (candidates.find(edge_connectivity[i]) == candidates.end())
      {
        candidates.insert(i);

        int face_id = i / 3;
        int inner_id = i % 3;
        int v0 = face_list[3 * face_id + inner_index[2 * inner_id + 0]];
        int v1 = face_list[3 * face_id + inner_index[2 * inner_id + 1]];
        crest_edges.push_back(Edge(v0, v1));
      }
    }
  }
}

const std::vector<Edge>& ShapeCrest::getCrestEdge()
{
  return crest_edges;
}