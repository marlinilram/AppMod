#include "ShapeCrest.h"
#include "BasicHeader.h"
#include "Shape.h"
#include "CrestCode.h"
#include "KDTreeWrapper.h"

#include <set>
#include <fstream>

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
  mergeCandidates(crest_edges, crest_lines);
  buildEdgeLineMapper();
  //crestCode.reset(new CrestCode);
  //crestCode->setShape(in_shape);
  //crestCode_lines = crestCode->getCrestLines();
}

const std::vector<Edge>& ShapeCrest::getCrestEdge()
{
  return crest_edges;
}

const std::vector<STLVectori>& ShapeCrest::getCrestLine()
{
  return crest_lines;
}

const std::vector<STLVectori>& ShapeCrest::getCrestCodeLine()
{
  return crestCode_lines;
}

const std::vector<STLVectori>& ShapeCrest::getVisbleCrestLine()
{
  return visible_lines;
}

void ShapeCrest::buildCandidates()
{
  // search all edges with Dihedral angle larger than a threshold
  const NormalList& face_normals = shape->getFaceNormal();
  const FaceList& face_list = shape->getFaceList();
  const STLVectori& edge_connectivity = shape->getEdgeConnectivity();
  std::vector<bool> visited_edge(edge_connectivity.size(), false);
  edge_dihedral.resize(edge_connectivity.size(), -1.0f);

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
  
  candidates.clear();
  crest_edges.clear();
  int inner_index[6] = {0, 1, 1, 2, 2, 0};
  for (size_t i = 0; i < edge_dihedral.size(); ++i)
  {
    if (edge_dihedral[i] < 0.75)
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

void ShapeCrest::mergeCandidates(std::vector<Edge> vis_edges, std::vector<std::vector<int>>& vis_lines)
{
  for (size_t i = 0; i < vis_edges.size(); ++i)
  {
    STLVectori temp_edges;
    temp_edges.push_back(vis_edges[i].first);
    temp_edges.push_back(vis_edges[i].second);
    vis_lines.push_back(temp_edges);
  }
  // merge connected edge
  int tag = 0;
  size_t i = 0;
  while (i < vis_lines.size())
  {
    int start = vis_lines[i][0];
    int end   = vis_lines[i][vis_lines[i].size() - 1];

    for (size_t j = i + 1; j < vis_lines.size(); ++j)
    {
      int cur_start = vis_lines[j][0];
      int cur_end   = vis_lines[j][vis_lines[j].size() - 1];

      // four types
      if (start == cur_start)
      {
        int start_n = vis_lines[i][1]; // the next v_id from start
        int cur_start_n = vis_lines[j][1]; // the next v_id from start
        if (connectable(start, start_n, cur_start_n))
        {
          std::reverse(vis_lines[j].begin(), vis_lines[j].end());
          vis_lines[i].insert(vis_lines[i].begin(), vis_lines[j].begin(), vis_lines[j].end() - 1);
          vis_lines.erase(vis_lines.begin() + j);
          tag = 1;
          break;
        }
      }
      else if (start == cur_end)
      {
        int start_n = vis_lines[i][1]; // the next v_id from start
        int cur_end_p = vis_lines[j][vis_lines[j].size() - 2];
        if (connectable(start, start_n, cur_end_p))
        {
          vis_lines[i].insert(vis_lines[i].begin(), vis_lines[j].begin(), vis_lines[j].end() - 1);
          vis_lines.erase(vis_lines.begin() + j);
          tag = 1;
          break;
        }
      }
      else if (end == cur_start)
      {
        int end_p = vis_lines[i][vis_lines[i].size() - 2];
        int cur_start_n = vis_lines[j][1]; // the next v_id from start
        if (connectable(end, end_p, cur_start_n))
        {
          vis_lines[i].insert(vis_lines[i].end(), vis_lines[j].begin() + 1, vis_lines[j].end());
          vis_lines.erase(vis_lines.begin() + j);
          tag = 1;
          break;
        }
      }
      else if (end == cur_end)
      {
        int end_p = vis_lines[i][vis_lines[i].size() - 2];
        int cur_end_p = vis_lines[j][vis_lines[j].size() - 2];
        if (connectable(end, end_p, cur_end_p))
        {
          std::reverse(vis_lines[j].begin(), vis_lines[j].end());
          vis_lines[i].insert(vis_lines[i].end(), vis_lines[j].begin() + 1, vis_lines[j].end());
          vis_lines.erase(vis_lines.begin() + j);
          tag = 1;
          break;
        }
      }
    }

    if (tag == 1)
    {
      tag = 0;
    }
    else
    {
      ++i;
    }
  }
}

bool ShapeCrest::connectable(int v_start, int v_ori_n, int v_cur_n)
{
  // test if the two connected (physically) edge should be connected
  // as a crest line
  // Only if the cosine of between them is less than -0.75
  const VertexList& vertex_list = shape->getVertexList();

  Vector3f dir_0;
  dir_0 << vertex_list[3 * v_ori_n + 0] - vertex_list[3 * v_start + 0],
           vertex_list[3 * v_ori_n + 1] - vertex_list[3 * v_start + 1],
           vertex_list[3 * v_ori_n + 2] - vertex_list[3 * v_start + 2];

  Vector3f dir_1;
  dir_1 << vertex_list[3 * v_cur_n + 0] - vertex_list[3 * v_start + 0],
           vertex_list[3 * v_cur_n + 1] - vertex_list[3 * v_start + 1],
           vertex_list[3 * v_cur_n + 2] - vertex_list[3 * v_start + 2];

  float cur_cos = dir_0.dot(dir_1) / dir_0.norm() / dir_1.norm();
  if (cur_cos < -0.75)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ShapeCrest::computeVisible(std::set<int>& vis_faces)
{
  /*std::vector<Edge> crest_edges_cache = crest_edges;
  std::vector<STLVectori> crest_lines_cache = crest_lines;*/
  const STLVectori& edge_connectivity = shape->getEdgeConnectivity();

  /*crest_edges.clear();
  crest_lines.clear();*/
  int inner_index[6] = {0, 1, 1, 2, 2, 0};
  std::set<int>::iterator it;
  size_t i = 0;
  std::map<int, std::vector<Edge> >::iterator visible_edges_it;
  visible_edges.clear();
  visible_lines.clear();
  for (it = candidates.begin(); it != candidates.end(); ++it)
  {
    int f_0 = (*it) / 3;
    int f_1 = edge_connectivity[(*it)] / 3;

    if (vis_faces.find(f_0) != vis_faces.end() || vis_faces.find(f_1) != vis_faces.end())
    {
      int curve_id = edge_line_mapper[crest_edges[i]];
      visible_edges_it = visible_edges.find(curve_id);
      if (visible_edges_it != visible_edges.end())
      {
        visible_edges[curve_id].push_back(crest_edges[i]);
      }
      else
      {
        visible_edges[curve_id] = std::vector<Edge>();
        visible_edges[curve_id].push_back(crest_edges[i]);
      }
      
      /*crest_edges.push_back(crest_edges_cache[i]);*/
    }

    ++i;
  }
  visible_global_mapper.clear();
  global_visible_mapper.clear();
  std::vector<std::vector<int>> temp_visible_lines;
  int vis_line_count = 0;
  for(auto i : visible_edges)
  {
    temp_visible_lines.clear();
    mergeCandidates(i.second, temp_visible_lines);
    for(size_t j = 0; j < temp_visible_lines.size(); j ++)
    {
      visible_lines.push_back(temp_visible_lines[j]);
      visible_global_mapper[vis_line_count] = i.first;
      global_visible_mapper[i.first].push_back(vis_line_count);
      vis_line_count ++;
    }
  }
  //mergeCandidates();

  /*visible_edges = crest_edges;
  visible_lines = crest_lines;*/

  /*crest_edges = crest_edges_cache;
  crest_lines = crest_lines_cache;*/

  // build the mapper for visible crest lines to the whole crest lines
  /*visible_global_mapper.clear();
  for(size_t i = 0; i < visible_lines.size(); i ++)
  {
    visible_global_mapper[i] = edge_line_mapper[std::pair<int, int>(visible_lines[i][0], visible_lines[i][1])];
  }*/
  //std::cout << "Updating visible_line_mapper finished.\n " ;
  return;
}


//void ShapeCrest::computeCrestLinesPoints()
//{
//  crest.reset(new CrestCode);
//  crest->exportInputFile(shape);
//  std::vector<std::vector<Vector3f>> CLPoints;
//  crest->getCrestLinesPoints(CLPoints);
//  int id;
//  std::vector<float> pt;
//  pt.resize(3);
//  crestLinesPointsId.resize(CLPoints.size());
//  crestLinesPoints.resize(CLPoints.size());
//
//  for(size_t i = 0 ; i < CLPoints.size() ; i ++)
//  {
//    for(size_t j = 0 ; j < CLPoints[i].size() ; j ++)
//    {
//      pt[0] = CLPoints[i][j].x();
//      pt[1] = CLPoints[i][j].y();
//      pt[2] = CLPoints[i][j].z();
//      (shape->getKDTree())->nearestPt(pt,id);
//      crestLinesPointsId[i].push_back(id);
//      Vector3f point;
//      point << (shape->getVertexList())[id * 3],(shape->getVertexList())[id * 3 + 1],(shape->getVertexList())[id * 3 + 2];
//      crestLinesPoints[i].push_back(point);
//      //crestLinesPoints[i].push_back(CLPoints[i][j]);
//    }
//  }
//  *size_t sum = 0;
//  for(size_t i = 0; i < crestLinesPoints.size(); i ++)
//    sum += crestLinesPoints[i].size();*/
//}

void ShapeCrest::buildEdgeLineMapper()
{
  edge_line_mapper.clear();
  for(size_t i = 0; i < crest_lines.size(); i ++)
  {
    for(size_t j = 0; j < crest_lines[i].size() - 1; j ++)
    {
      int start = crest_lines[i][j];
      int end = crest_lines[i][j + 1];
      edge_line_mapper[std::pair<int, int>(start, end)] = i;
      edge_line_mapper[std::pair<int, int>(end, start)] = i;
    }
  }

}