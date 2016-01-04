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
  const std::vector<STLVectori>& getCrestCodeLine();
  
  void buildCandidates();
  void mergeCandidates(std::vector<Edge>& vis_edges, std::vector<std::vector<int>>& vis_lines);
  bool connectable(int v_start, int v_ori_n, int v_cur_n);
  void computeVisible(std::set<int>& vis_faces);
  void buildEdgeLineMapper();

  void setCrestCode(std::shared_ptr<CrestCode> in_crestCode);
  /*std::vector<std::vector<Vector3f>>& getCrestLinesPoints(){ return crestLinesPoints; };
  std::vector<std::vector<int>>& getCrestLinesPointsId(){ return crestLinesPointsId; };
  void computeCrestLinesPoints();*/
  void generateEdgesFromCrestCode();

public:
  std::vector<Edge> crest_edges;
  std::vector<STLVectori> crest_lines;
  std::map<std::pair<int, int>, int> edge_line_mapper; //global mapper
  std::map<int, int> visible_global_mapper;
  std::map<int, std::vector<int>> global_visible_mapper;

  std::vector<STLVectori> crestCode_lines;

  //std::vector<Edge> visible_edges;
  std::map<int, std::vector<Edge>> visible_edges;
  std::vector<STLVectori> visible_lines;

  STLVectorf edge_dihedral; // store dihedral angle for all edges
  std::set<int> candidates; // store edge id of crest_edges

private:
  std::shared_ptr<Shape> shape;
  std::shared_ptr<CrestCode> crestCode;
  /*std::vector<std::vector<int>> crestLinesPointsId;
  std::vector<std::vector<Vector3f>> crestLinesPoints;*/

private:
  ShapeCrest(const ShapeCrest&);
  void operator = (const ShapeCrest&);
};

#endif // !ShapeCrest_H
