#include "DetailSynthesis.h"
#include "Model.h"
#include "MeshParameterization.h"
#include "KDTreeWrapper.h"
#include "Shape.h"
#include "BasicHeader.h"
#include "ShapeUtility.h"
#include "obj_writer.h"


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

void DetailSynthesis::computeDisplacementMap(std::shared_ptr<Model> model)
{
  int width = 1000, height = 1000;
  displacement_map = cv::Mat(height, width, CV_32FC1);
  std::shared_ptr<KDTreeWrapper> kdTree = mesh_para->kdTree_UV;
  AdjList adjFaces_list = mesh_para->cut_shape->getVertexShareFaces();
  for(int x = 0; x < height; x ++)
  {
    for(int y = 0; y < width; y ++)
    {
      int pt_id;
      std::vector<float> pt;
      pt.resize(2);
      pt[0] = float(x) / height;
      pt[1] = float(y) / width;
      kdTree->nearestPt(pt,pt_id);
      std::vector<int> adjFaces = adjFaces_list[pt_id];
      int face_id;
      float point[3];
      point[0] = pt[0];
      point[1] = pt[1];
      point[2] = 0;
      float lambda[3];
      int id1_before,id2_before,id3_before;
      for(size_t i = 0; i < adjFaces.size(); i ++)
      {
        float l[3];
        int v1_id,v2_id,v3_id;
        float v1[3],v2[3],v3[3];
        v1_id = (mesh_para->cut_shape->getFaceList())[3 * adjFaces[i]];
        v2_id = (mesh_para->cut_shape->getFaceList())[3 * adjFaces[i] + 1];
        v3_id = (mesh_para->cut_shape->getFaceList())[3 * adjFaces[i] + 2];
        v1[0] = (mesh_para->cut_shape->getUVCoord())[2 * v1_id];
        v1[1] = (mesh_para->cut_shape->getUVCoord())[2 * v1_id + 1];
        v1[2] = 0;
        v2[0] = (mesh_para->cut_shape->getUVCoord())[2 * v2_id];
        v2[1] = (mesh_para->cut_shape->getUVCoord())[2 * v2_id + 1];
        v2[2] = 0;
        v3[0] = (mesh_para->cut_shape->getUVCoord())[2 * v3_id];
        v3[1] = (mesh_para->cut_shape->getUVCoord())[2 * v3_id + 1];
        v3[2] = 0;
        ShapeUtility::computeBaryCentreCoord(point,v1,v2,v3,l);
        if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
        {
          face_id = i;
          lambda[0] = l[0];
          lambda[1] = l[1];
          lambda[2] = l[2];
          id1_before = v1_id;
          id2_before = v2_id;
          id3_before = v3_id;
        }
      }
      float v1_worldCoord_before[3],v2_worldCoord_before[3],v3_worldCoord_before[3];
      v1_worldCoord_before[0] = (mesh_para->cut_shape->getVertexList())[3 * id1_before];
      v1_worldCoord_before[1] = (mesh_para->cut_shape->getVertexList())[3 * id1_before + 1];
      v1_worldCoord_before[2] = (mesh_para->cut_shape->getVertexList())[3 * id1_before + 2];
      v2_worldCoord_before[0] = (mesh_para->cut_shape->getVertexList())[3 * id2_before];
      v2_worldCoord_before[1] = (mesh_para->cut_shape->getVertexList())[3 * id2_before + 1];
      v2_worldCoord_before[2] = (mesh_para->cut_shape->getVertexList())[3 * id2_before + 2];
      v3_worldCoord_before[0] = (mesh_para->cut_shape->getVertexList())[3 * id3_before];
      v3_worldCoord_before[1] = (mesh_para->cut_shape->getVertexList())[3 * id3_before + 1];
      v3_worldCoord_before[2] = (mesh_para->cut_shape->getVertexList())[3 * id3_before + 2];
      float pt_worldCoord_before[3];
      pt_worldCoord_before[0] = lambda[0] * v1_worldCoord_before[0] + lambda[1] * v2_worldCoord_before[0] + lambda[2] * v3_worldCoord_before[0];
      pt_worldCoord_before[1] = lambda[0] * v1_worldCoord_before[1] + lambda[1] * v2_worldCoord_before[1] + lambda[2] * v3_worldCoord_before[1];
      pt_worldCoord_before[2] = lambda[0] * v1_worldCoord_before[2] + lambda[1] * v2_worldCoord_before[2] + lambda[2] * v3_worldCoord_before[2];
      int id1_after,id2_after,id3_after;
      id1_after = mesh_para->vertex_set[id1_before];
      id2_after = mesh_para->vertex_set[id2_before];
      id3_after = mesh_para->vertex_set[id3_before];
      float v1_worldCoord_after[3],v2_worldCoord_after[3],v3_worldCoord_after[3];
      v1_worldCoord_after[0] = (model->getShapeVertexList())[3 * id1_after];
      v1_worldCoord_after[1] = (model->getShapeVertexList())[3 * id1_after + 1];
      v1_worldCoord_after[2] = (model->getShapeVertexList())[3 * id1_after + 2];
      v2_worldCoord_after[0] = (model->getShapeVertexList())[3 * id2_after];
      v2_worldCoord_after[1] = (model->getShapeVertexList())[3 * id2_after + 1];
      v2_worldCoord_after[2] = (model->getShapeVertexList())[3 * id2_after + 2];
      v3_worldCoord_after[0] = (model->getShapeVertexList())[3 * id3_after];
      v3_worldCoord_after[1] = (model->getShapeVertexList())[3 * id3_after + 1];
      v3_worldCoord_after[2] = (model->getShapeVertexList())[3 * id3_after + 2];
      float pt_worldCoord_after[3];
      pt_worldCoord_after[0] = lambda[0] * v1_worldCoord_after[0] + lambda[1] * v2_worldCoord_after[0] + lambda[2] * v3_worldCoord_after[0];
      pt_worldCoord_after[1] = lambda[0] * v1_worldCoord_after[1] + lambda[1] * v2_worldCoord_after[1] + lambda[2] * v3_worldCoord_after[1];
      pt_worldCoord_after[2] = lambda[0] * v1_worldCoord_after[2] + lambda[1] * v2_worldCoord_after[2] + lambda[2] * v3_worldCoord_after[2];
      float v1_normal_original_mesh[3],v2_normal_original_mesh[3],v3_normal_original_mesh[3];
      v1_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * id1_after];
      v1_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * id1_after + 1];
      v1_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * id1_after + 2];
      v2_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * id2_after];
      v2_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * id2_after + 1];
      v2_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * id2_after + 2];
      v3_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * id3_after];
      v3_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * id3_after + 1];
      v3_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * id3_after + 2];
      float pt_normal_original_mesh[3];
      pt_normal_original_mesh[0] = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      pt_normal_original_mesh[1] = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      pt_normal_original_mesh[2] = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];
      float pt_difference[3];
      pt_difference[0] = pt_worldCoord_after[0] - pt_worldCoord_before[0];
      pt_difference[1] = pt_worldCoord_after[1] - pt_worldCoord_before[1];
      pt_difference[2] = pt_worldCoord_after[2] - pt_worldCoord_before[2];
      displacement_map.at<float>(x,y) = pt_difference[0] * pt_normal_original_mesh[0] + pt_difference[1] * pt_normal_original_mesh[1] + pt_difference[2] * pt_normal_original_mesh[2];
    }
  }
  // show the displacement_map
  /*double min,max;
  cv::minMaxLoc(displacement_map,&min,&max);
  cv::imshow("displacement_map",(displacement_map - min) / (max - min));*/

  // check whether the displacement_map is computed correctedly
  //std::cout << "UV_coord size is :" << mesh_para->cut_shape->getUVCoord().size() << std::endl;
  VertexList vertex_check;
  /*vertex_check.resize(model->getShapeVertexList().size());*/
  vertex_check =  mesh_para->vertex_original_mesh;
  for(int i = 0; i < mesh_para->vertex_set.size(); i ++)
  {
    float pt_check[3];
    pt_check[0] = vertex_check[3 * mesh_para->vertex_set[i]];
    pt_check[1] = vertex_check[3 * mesh_para->vertex_set[i] + 1];
    pt_check[2] = vertex_check[3 * mesh_para->vertex_set[i] + 2];
    float normal_check[3];
    normal_check[0] = mesh_para->normal_original_mesh[3 * mesh_para->vertex_set[i]];
    normal_check[1] = mesh_para->normal_original_mesh[3 * mesh_para->vertex_set[i] + 1];
    normal_check[2] = mesh_para->normal_original_mesh[3 * mesh_para->vertex_set[i] + 2];
    float displacement;
    float U,V;
    U = (mesh_para->cut_shape->getUVCoord())[2 * i];
    V = (mesh_para->cut_shape->getUVCoord())[2 * i + 1];
    //std::cout << "U : " << U << " , " << "V : " << V << std::endl;
    int img_x,img_y;
    img_x = int(U * (float)height);
    img_y = int(V * (float)width);
    if(img_x == height)
    {
      img_x --;
    }
    if(img_y == width)
    {
      img_y --;    
    }
    displacement = displacement_map.at<float>(img_x,img_y);
    vertex_check[3 * mesh_para->vertex_set[i]] = pt_check[0] + normal_check[0] * displacement;
    vertex_check[3 * mesh_para->vertex_set[i] + 1] = pt_check[1] + normal_check[1] * displacement;
    vertex_check[3 * mesh_para->vertex_set[i] + 2] = pt_check[2] + normal_check[2] * displacement;
  }

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::shape_t obj_shape;
  obj_shape.mesh.positions = vertex_check;
  obj_shape.mesh.indices = model->getShapeFaceList();
  //obj_shape.mesh.texcoords = model->getShapeUVCoord();
  shapes.push_back(obj_shape);

  std::string output_name = model->getOutputPath() + "/check" + ".obj";
  WriteObj(output_name, shapes, materials);
}



        