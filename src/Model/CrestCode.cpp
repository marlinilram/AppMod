#include "CrestCode.h"
#include "BasicHeader.h"
#include "Shape.h"
#include "KDTreeWrapper.h"
#include "fstream"
#include <set>

CrestCode::CrestCode()
{

}

CrestCode::~CrestCode()
{

}

void CrestCode::setShape(std::shared_ptr<Shape> in_shape)
{
  shape = in_shape;
  exportInputFile();
  computeCrestLines();
}

void CrestCode::exportInputFile()
{
  FILE *fp,*fp_ply2;
  fp = fopen("input.txt","w+");
  fp_ply2 = fopen("model.ply2","w+");
  size_t vertexNum = (shape->getVertexList()).size();
  size_t faceNum = (shape->getFaceList()).size();
  int neighborhoodSize = 1;                            //need to be reset!
  fprintf(fp,"%d\n",vertexNum / 3);
  fprintf(fp,"%d\n",faceNum / 3);
  fprintf(fp,"%d\n",neighborhoodSize);
  fprintf(fp,"%d\n",1);

  fprintf(fp_ply2,"%d\n",vertexNum / 3);
  fprintf(fp_ply2,"%d\n",faceNum / 3);

  for(size_t i = 0 ; i < vertexNum ; i ++)
  {
    fprintf(fp,"%f\n",(shape->getVertexList())[i]);
    fprintf(fp_ply2,"%f\n",(shape->getVertexList())[i]);
  }
  for(size_t j = 0 ; j < faceNum ; j ++)
  {
    fprintf(fp,"%d\n",(shape->getFaceList())[j]);
    if(j % 3 == 0)
    {
      fprintf(fp_ply2,"%d\n",3);
    }
    fprintf(fp_ply2,"%d\n",(shape->getFaceList())[j]);
  }
  fclose(fp);
  fclose(fp_ply2);
}

void CrestCode::mergeCrestEdges()
{
  for (size_t i = 0; i < crest_edges.size(); ++i)
  {
    STLVectori temp_edges;
    temp_edges.push_back(crest_edges[i].first);
    temp_edges.push_back(crest_edges[i].second);
    crest_lines.push_back(temp_edges);
  }
  // merge connected edge
  int tag = 0;
  size_t i = 0;
  while (i < crest_lines.size())
  {
    int start = crest_lines[i][0];
    int end   = crest_lines[i][crest_lines[i].size() - 1];

    for (size_t j = i + 1; j < crest_lines.size(); ++j)
    {
      int cur_start = crest_lines[j][0];
      int cur_end   = crest_lines[j][crest_lines[j].size() - 1];

      // four types
      if (start == cur_start)
      {
        int start_n = crest_lines[i][1]; // the next v_id from start
        int cur_start_n = crest_lines[j][1]; // the next v_id from start
        std::reverse(crest_lines[i].begin(), crest_lines[i].end());
        crest_lines[i].insert(crest_lines[i].end(), crest_lines[j].begin() + 1, crest_lines[j].end());
        crest_lines.erase(crest_lines.begin() + j);
        tag = 1;
        break;
      }
      else if (start == cur_end)
      {
        int start_n = crest_lines[i][1]; // the next v_id from start
        int cur_end_p = crest_lines[j][crest_lines[j].size() - 2];
        crest_lines[i].insert(crest_lines[i].begin(), crest_lines[j].begin(), crest_lines[j].end() - 1);
        crest_lines.erase(crest_lines.begin() + j);
        tag = 1;
        break;
      }
      else if (end == cur_start)
      {
        int end_p = crest_lines[i][crest_lines[i].size() - 2];
        int cur_start_n = crest_lines[j][1]; // the next v_id from start
        crest_lines[i].insert(crest_lines[i].end(), crest_lines[j].begin() + 1, crest_lines[j].end());
        crest_lines.erase(crest_lines.begin() + j);
        tag = 1;
        break;
      }
      else if (end == cur_end)
      {
        int end_p = crest_lines[i][crest_lines[i].size() - 2];
        int cur_end_p = crest_lines[j][crest_lines[j].size() - 2];
        std::reverse(crest_lines[j].begin(), crest_lines[j].end());
        crest_lines[i].insert(crest_lines[i].end(), crest_lines[j].begin() + 1, crest_lines[j].end());
        crest_lines.erase(crest_lines.begin() + j);
        tag = 1;
        break;
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

void CrestCode::computeCrestLines()
{
  system("Crest.exe input.txt output.txt");
  FILE *fp1,*fp2;
  fp1 = fopen("ravines.txt","r");
  fp2 = fopen("ridges.txt","r");
  int vertexNum_ravines,vertexNum_ridges,crestEdgesNum_ravines,crestEdgesNum_ridges,crestLinesNum_ravines,crestLinesNum_ridges;
  Vector3f point;
  float x,y,z;
  int id;
  std::vector<Vector3f> crestLinesPoints;
  fscanf(fp1,"%d%d%d",&vertexNum_ravines,&crestEdgesNum_ravines,&crestLinesNum_ravines);
  fscanf(fp2,"%d%d%d",&vertexNum_ridges,&crestEdgesNum_ridges,&crestLinesNum_ridges);
  //crest_lines.resize(crestLinesNum_ravines + crestLinesNum_ridges);
  for(int i = 0; i < vertexNum_ravines; i ++)
  {
    fscanf(fp1,"%f%f%f%d",&x,&y,&z,&id);
    point << x,y,z;
    crestLinesPoints.push_back(point);
    //crest_lines[id].push_back(i);
  }
  for(int i = 0; i < vertexNum_ridges; i ++)
  {
    fscanf(fp2,"%f%f%f%d",&x,&y,&z,&id);
    point << x,y,z;
    crestLinesPoints.push_back(point);
    //crest_lines[id + crestLinesNum_ravines].push_back(i + vertexNum_ravines);
  }
  /*size_t sum = 0;
  for(size_t i = 0; i < crestLinesPoints.size(); i ++)
    sum += crestLinesPoints[i].size();*/
  float ridgeness, sphericalness , cyclideness;
  Vector3f attributes;
  for(int i = 0; i < crestLinesNum_ravines; i ++)
  {
    fscanf(fp1,"%f%f%f",&ridgeness,&sphericalness,&cyclideness);
    attributes << ridgeness,sphericalness,cyclideness;
    crest_lines_attributes.push_back(attributes);
  }
  for(int i = 0; i < crestLinesNum_ridges; i ++)
  {
    fscanf(fp2,"%f%f%f",&ridgeness,&sphericalness,&cyclideness);
    attributes << ridgeness,sphericalness,cyclideness;
    crest_lines_attributes.push_back(attributes);
  }
  int endPoint1,endPoint2,triangleId;
  Edge e;
  for(int i = 0; i < crestEdgesNum_ravines; i ++)
  {
    fscanf(fp1,"%d%d%d",&endPoint1,&endPoint2,&triangleId);
    e.first = endPoint1;
    e.second = endPoint2;
    crest_edges.push_back(e);
  }
  for(int i = 0; i < crestEdgesNum_ridges; i ++)
  {
    fscanf(fp2,"%d%d%d",&endPoint1,&endPoint2,&triangleId);
    e.first = endPoint1 + vertexNum_ravines;
    e.second = endPoint2 + vertexNum_ravines;
    crest_edges.push_back(e);
  }

  fclose(fp1);
  fclose(fp2);

  mergeCrestEdges();
  /*size_t sum = 0;
  for(size_t i = 0; i < crest_lines.size(); i ++)
    sum += crest_lines[i].size();*/
  std::vector<float> pt;
  pt.resize(3);
  std::vector<STLVectori> tmp_crestLines;
  tmp_crestLines.resize(crest_lines.size());
  int pt_id;
  for(size_t i = 0; i < crest_lines.size(); i ++)
  {
    for(size_t j = 0; j < crest_lines[i].size(); j++)
    {
      pt[0] = crestLinesPoints[crest_lines[i][j]].x();
      pt[1] = crestLinesPoints[crest_lines[i][j]].y();
      pt[2] = crestLinesPoints[crest_lines[i][j]].z();
      (shape->getKDTree())->nearestPt(pt,pt_id);
      if(j != 0)
      {
        int endId = tmp_crestLines[i][tmp_crestLines[i].size() - 1];
        if(endId != pt_id)
        {
          tmp_crestLines[i].push_back(pt_id);
        }
      }
      else
      {
        tmp_crestLines[i].push_back(pt_id);
      }
    }
  }
  crest_lines = tmp_crestLines;
}

std::vector<STLVectori>& CrestCode::getCrestLines()
{
  std::set<length_id> candidate;
  for(size_t i = 0; i < crest_lines.size(); i ++)
  {
    length_id temp;
    temp.id = i;
    temp.length = crest_lines[i].size();
    candidate.insert(temp);
  }
  int best_candidate = 10;
  std::vector<STLVectori> temp_crest_lines = crest_lines;
  crest_lines.clear();
  std::set<length_id>::const_iterator it = candidate.begin();
  for(int i = 0; i < best_candidate; i ++)
  {
    crest_lines.push_back(temp_crest_lines[it->id]);
    it ++;
  }
  return crest_lines;
}