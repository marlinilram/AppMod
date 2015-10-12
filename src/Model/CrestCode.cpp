#include "CrestCode.h"
#include "BasicHeader.h"
#include "Shape.h"
#include "fstream"

CrestCode::CrestCode()
{

}

CrestCode::~CrestCode()
{

}

void CrestCode::exportInputFile(std::shared_ptr<Shape> shape)
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

void CrestCode::getCrestLinesPoints(std::vector<std::vector<Vector3f>>& crestLinesPoints)
{
  system("Crest.exe input.txt output.txt");
  FILE *fp1,*fp2;
  fp1 = fopen("ravines.txt","r");
  fp2 = fopen("ridges.txt","r");
  int vertexNum_ravines,vertexNum_ridges,crestEdgesNum_ravines,crestEdgesNum_ridges,crestLinesNum_ravines,crestLinesNum_ridges;
  Vector3f point;
  float x,y,z;
  int id;
  fscanf(fp1,"%d%d%d",&vertexNum_ravines,&crestEdgesNum_ravines,&crestLinesNum_ravines);
  fscanf(fp2,"%d%d%d",&vertexNum_ridges,&crestEdgesNum_ridges,&crestLinesNum_ridges);
  crestLinesPoints.resize(crestLinesNum_ravines + crestLinesNum_ridges);
  for(int i = 0 ; i < vertexNum_ravines ; i ++)
  {
    fscanf(fp1,"%f%f%f%d",&x,&y,&z,&id);
    point << x,y,z;
    crestLinesPoints[id].push_back(point);
  }
  for(int i = 0 ; i < vertexNum_ridges ; i ++)
  {
    fscanf(fp2,"%f%f%f%d",&x,&y,&z,&id);
    point << x,y,z;
    crestLinesPoints[id + crestLinesNum_ravines].push_back(point);
  }
  /*size_t sum = 0;
  for(size_t i = 0; i < crestLinesPoints.size(); i ++)
    sum += crestLinesPoints[i].size();*/

  fclose(fp1);
  fclose(fp2);
}


