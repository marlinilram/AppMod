#ifndef ShapeCrest_H
#define ShapeCrest_H

#include "BasicHeader.h"
#include "CrestCode.h"
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
  std::vector<std::vector<Vector3f>>& getCrestLinesPoints(){ return crestLinesPoints; };
  std::vector<std::vector<int>>& getCrestLinesPointsId(){ return crestLinesPointsId; };
  void computeCrestLinesPoints();

public:
  std::vector<Edge> crest_edges;
  std::vector<STLVectori> crest_lines;

  std::vector<Edge> visible_edges;
  std::vector<STLVectori> visible_lines;

  STLVectorf edge_dihedral; // store dihedral angle for all edges
  std::set<int> candidates; // store edge id of crest_edges

private:
  std::shared_ptr<Shape> shape;
  std::shared_ptr<CrestCode> crest;
  std::vector<std::vector<int>> crestLinesPointsId;
  std::vector<std::vector<Vector3f>> crestLinesPoints;

private:
  ShapeCrest(const ShapeCrest&);
  void operator = (const ShapeCrest&);
};

#endif // !ShapeCrest_H
