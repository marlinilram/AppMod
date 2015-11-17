#include "DetailSynthesis.h"
#include "Model.h"
#include "MeshParameterization.h"
#include "KDTreeWrapper.h"
#include "Shape.h"
#include "BasicHeader.h"
#include "ShapeUtility.h"
#include "obj_writer.h"
#include "SynthesisTool.h"
#include "CurveGuidedVectorField.h"
#include "GLActor.h"

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
  resolution = 500;
  displacement_map = cv::Mat(resolution, resolution, CV_32FC1);
  std::shared_ptr<KDTreeWrapper> kdTree = mesh_para->kdTree_UV;
  AdjList adjFaces_list = mesh_para->cut_shape->getVertexShareFaces();
  for(int x = 0; x < resolution; x ++)
  {
    for(int y = 0; y < resolution; y ++)
    {
      int pt_id;
      std::vector<float> pt;
      pt.resize(2);
      pt[0] = float(x) / resolution;
      pt[1] = float(y) / resolution;
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
  applyDisplacementMap(mesh_para->vertex_set, mesh_para->cut_shape, model, displacement_map);
}

void DetailSynthesis::computeFeatureMap(std::vector<cv::Mat>& feature_map, bool is_src)
{
  resolution = 500;
  for(int i = 0; i < 3; i ++)
  {
    feature_map.push_back(cv::Mat(resolution, resolution, CV_32FC1));
  }
  /*feature_map = cv::Mat(resolution, resolution, CV_32FC3);*/
  /*int dims[3] = { resolution, resolution, 3};
  feature_map.create(3, dims, CV_32F);*/
  std::shared_ptr<Shape> shape;
  std::shared_ptr<KDTreeWrapper> kdTree;
  STLVectori v_set;
  if(is_src)
  {
    shape = mesh_para->cut_shape;
    kdTree = mesh_para->kdTree_UV;
    v_set = mesh_para->vertex_set;
  }
  else
  {
    shape = mesh_para->cut_shape_hidden;
    kdTree = mesh_para->kdTree_UV_hidden;
    v_set = mesh_para->vertex_set_hidden;
  }
  AdjList adjFaces_list = shape->getVertexShareFaces();
  for(int x = 0; x < resolution; x ++)
  {
    for(int y = 0; y < resolution; y ++)
    {
      int pt_id;
      std::vector<float> pt;
      pt.resize(2);
      pt[0] = float(x) / resolution;
      pt[1] = float(y) / resolution;
      kdTree->nearestPt(pt,pt_id);
      std::vector<int> adjFaces = adjFaces_list[pt_id];
      int face_id;
      float point[3];
      point[0] = pt[0];
      point[1] = pt[1];
      point[2] = 0;
      float lambda[3];
      int id1,id2,id3;
      for(size_t i = 0; i < adjFaces.size(); i ++)
      {
        float l[3];
        int v1_id,v2_id,v3_id;
        float v1[3],v2[3],v3[3];
        v1_id = (shape->getFaceList())[3 * adjFaces[i]];
        v2_id = (shape->getFaceList())[3 * adjFaces[i] + 1];
        v3_id = (shape->getFaceList())[3 * adjFaces[i] + 2];
        v1[0] = (shape->getUVCoord())[2 * v1_id];
        v1[1] = (shape->getUVCoord())[2 * v1_id + 1];
        v1[2] = 0;
        v2[0] = (shape->getUVCoord())[2 * v2_id];
        v2[1] = (shape->getUVCoord())[2 * v2_id + 1];
        v2[2] = 0;
        v3[0] = (shape->getUVCoord())[2 * v3_id];
        v3[1] = (shape->getUVCoord())[2 * v3_id + 1];
        v3[2] = 0;
        ShapeUtility::computeBaryCentreCoord(point,v1,v2,v3,l);
        if(l[0] >= 0 && l[1] >= 0 && l[2] >= 0)
        {
          face_id = i;
          lambda[0] = l[0];
          lambda[1] = l[1];
          lambda[2] = l[2];
          id1 = v1_id;
          id2 = v2_id;
          id3 = v3_id;
        }
      }
      
      float v1_normal_original_mesh[3],v2_normal_original_mesh[3],v3_normal_original_mesh[3];
      v1_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id1]];
      v1_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id1] + 1];
      v1_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id1] + 2];
      v2_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id2]];
      v2_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id2] + 1];
      v2_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id2] + 2];
      v3_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id3]];
      v3_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id3] + 1];
      v3_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id3] + 2];

      /*feature_map.at<float>(x,y,0) = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map.at<float>(x,y,1) = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map.at<float>(x,y,2) = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];*/
      /*feature_map.at<cv::Vec3f>(x,y)[0] = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map.at<cv::Vec3f>(x,y)[1] = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map.at<cv::Vec3f>(x,y)[2] = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];*/
      feature_map[0].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map[1].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map[2].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];
    }
  }
}

void DetailSynthesis::applyDisplacementMap(STLVectori vertex_set, std::shared_ptr<Shape> cut_shape, std::shared_ptr<Model> model, cv::Mat disp_map)
{
  VertexList vertex_check;
  vertex_check =  mesh_para->vertex_original_mesh;
  for(int i = 0; i < vertex_set.size(); i ++)
  {
    float pt_check[3];
    pt_check[0] = vertex_check[3 * vertex_set[i]];
    pt_check[1] = vertex_check[3 * vertex_set[i] + 1];
    pt_check[2] = vertex_check[3 * vertex_set[i] + 2];
    float normal_check[3];
    normal_check[0] = mesh_para->normal_original_mesh[3 * vertex_set[i]];
    normal_check[1] = mesh_para->normal_original_mesh[3 * vertex_set[i] + 1];
    normal_check[2] = mesh_para->normal_original_mesh[3 * vertex_set[i] + 2];
    float displacement;
    float U,V;
    U = (cut_shape->getUVCoord())[2 * i];
    V = (cut_shape->getUVCoord())[2 * i + 1];
    //std::cout << "U : " << U << " , " << "V : " << V << std::endl;
    int img_x,img_y;
    img_x = int(U * (float)resolution);
    img_y = int(V * (float)resolution);
    if(img_x == resolution)
    {
      img_x --;
    }
    if(img_y == resolution)
    {
      img_y --;    
    }
    displacement = disp_map.at<float>(img_x,img_y);
    vertex_check[3 * vertex_set[i]] = pt_check[0] + normal_check[0] * displacement;
    vertex_check[3 * vertex_set[i] + 1] = pt_check[1] + normal_check[1] * displacement;
    vertex_check[3 * vertex_set[i] + 2] = pt_check[2] + normal_check[2] * displacement;
  }

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::shape_t obj_shape;
  obj_shape.mesh.positions = vertex_check;
  obj_shape.mesh.indices = model->getShapeFaceList();
  //obj_shape.mesh.texcoords = model->getShapeUVCoord();
  shapes.push_back(obj_shape);

  char time_postfix[50];
  time_t current_time = time(NULL);
  strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
  std::string file_time_postfix = time_postfix;

  std::string output_name = model->getOutputPath() + "/detail_synthesis" + file_time_postfix + ".obj";
  WriteObj(output_name, shapes, materials);
}


void DetailSynthesis::startDetailSynthesis(std::shared_ptr<Model> model)
{
  /*cv::FileStorage fs(model->getDataPath() + "/displacement_map.xml", cv::FileStorage::WRITE);
  fs << "displacement_map" << displacement_map;*/
  /*cv::FileStorage fs(model->getDataPath() + "/displacement_map.xml", cv::FileStorage::READ);
  cv::Mat disp_mat;
  fs["displacement_map"] >> disp_mat;*/

  /*computeDisplacementMap(model);
  computeFeatureMap(src_feature_map, TRUE);
  computeFeatureMap(tar_feature_map, FALSE);
  syn_tool.reset(new SynthesisTool);
  syn_tool->init(src_feature_map, tar_feature_map, displacement_map);
  syn_tool->doSynthesis();
  cv::Mat tar_detail = (syn_tool->getTargetDetail())[0];
  applyDisplacementMap(mesh_para->vertex_set_hidden, mesh_para->cut_shape_hidden, model, tar_detail);*/

  cv::Mat inputImage;
  inputImage = cv::imread("sample06.png");
  /*cv::Mat grayImage = cv::Mat(inputImage.size().height,inputImage.size().width,CV_32FC1);
  cv::cvtColor(inputImage,grayImage,CV_BGR2GRAY);*/
  inputImage.convertTo(inputImage,CV_32FC1);
  double min,max;
  cv::minMaxLoc(inputImage,&min,&max);
  cv::imshow("input", (inputImage - min) / (max - min));
  syn_tool.reset(new SynthesisTool);
  syn_tool->doImageSynthesis(inputImage);
  cv::Mat tar_detail = (syn_tool->getTargetDetail())[0];
  cv::minMaxLoc(tar_detail,&min,&max);
  cv::imshow("result", (tar_detail - min) / (max - min));
}

void DetailSynthesis::computeVectorField(std::shared_ptr<Model> model)
{
  curve_guided_vector_field.reset(new CurveGuidedVectorField);
  curve_guided_vector_field->computeVectorField(model);
  actors.clear();
  std::vector<GLActor> temp_actors;
  curve_guided_vector_field->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void DetailSynthesis::getDrawableActors(std::vector<GLActor>& actors)
{
  actors = this->actors;
}