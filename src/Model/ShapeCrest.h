#ifndef ShapeCrest_H
#define ShapeCrest_H

#include "BasicHeader.h"
#include <memory>
#include <set>

class Shape;

class ShapeCrest
{
public:
  ShapeCrest();
  ~ShapeCrest();

  void setShape(std::shared_ptr<Shape> in_shape);
  const std::vector<Edge>& getCrestEdge();
  const std::vector<STLVectori>& getCrestLine();
  const std::vector<STLVectori>& getVisbleCrestLine();
  
  void buildCandidates();
  void mergeCandidates();
  bool connectable(int v_start, int v_ori_n, int v_cur_n);
  void computeVisible(std::set<int>& vis_faces);

public:
  std::vector<Edge> crest_edges;
  std::vector<STLVectori> crest_lines;

  std::vector<Edge> visible_edges;
  std::vector<STLVectori> visible_lines;

  STLVectorf edge_dihedral; // store dihedral angle for all edges
  std::set<int> candidates; // store edge id of crest_edges

private:
  std::shared_ptr<Shape> shape;

private:
  ShapeCrest(const ShapeCrest&);
  void operator = (const ShapeCrest&);
};

#endif // !ShapeCrest_H
