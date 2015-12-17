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
#include "KevinVectorField.h"
#include "PolygonMesh.h"
#include "YMLHandler.h"

using namespace LG;

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


void DetailSynthesis::prepareFeatureMap(std::shared_ptr<Model> model)
{
  // compute feature
  // 1. normalized height
  // 2. surface normal 
  // 3. multi-scale solid angle curvature, not implemented yet
  // 4. directional occlusion

  ShapeUtility::computeSymmetry(model);
  ShapeUtility::computeNormalizedHeight(model);
  ShapeUtility::computeDirectionalOcclusion(model);

  PolygonMesh* poly_mesh = model->getPolygonMesh();
  PolygonMesh::Vertex_attribute<Scalar> normalized_height = poly_mesh->vertex_attribute<Scalar>("v:NormalizedHeight");
  PolygonMesh::Vertex_attribute<STLVectorf> directional_occlusion = poly_mesh->vertex_attribute<STLVectorf>("v:DirectionalOcclusion");
  PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
  PolygonMesh::Vertex_attribute<Vec3> v_symmetry = poly_mesh->vertex_attribute<Vec3>("v:symmetry");

  std::vector<std::vector<float> > vertex_feature_list(poly_mesh->n_vertices(), std::vector<float>());
  for (auto vit : poly_mesh->vertices())
  {
    vertex_feature_list[vit.idx()].push_back(normalized_height[vit]);
    vertex_feature_list[vit.idx()].push_back(v_symmetry[vit][0]);
    vertex_feature_list[vit.idx()].push_back(v_symmetry[vit][1]);
    vertex_feature_list[vit.idx()].push_back(v_symmetry[vit][2]);
    //vertex_feature_list[vit.idx()].push_back(v_normals[vit][0]);
    //vertex_feature_list[vit.idx()].push_back(v_normals[vit][1]);
    //vertex_feature_list[vit.idx()].push_back(v_normals[vit][2]);
    for (size_t i = 0; i < directional_occlusion[vit].size(); ++i)
    {
      vertex_feature_list[vit.idx()].push_back(directional_occlusion[vit][i]/directional_occlusion[vit].size());
    }
  }

  computeFeatureMap(src_feature_map, vertex_feature_list, TRUE);
  computeFeatureMap(tar_feature_map, vertex_feature_list, FALSE);

  // save feature map to file
  for (size_t i = 0; i < src_feature_map.size(); ++i)
  {
    YMLHandler::saveToFile(model->getOutputPath(), std::string("src_feature_") + std::to_string(i) + ".yml", src_feature_map[i]);
    YMLHandler::saveToFile(model->getOutputPath(), std::string("tar_feature_") + std::to_string(i) + ".yml", tar_feature_map[i]);

    YMLHandler::saveToMat(model->getOutputPath(), std::string("src_feature_") + std::to_string(i) + ".mat", src_feature_map[i]);
    YMLHandler::saveToMat(model->getOutputPath(), std::string("tar_feature_") + std::to_string(i) + ".mat", tar_feature_map[i]);
  }
}

void DetailSynthesis::computeFeatureMap(std::vector<cv::Mat>& feature_map, std::vector<std::vector<float> >& feature_list, bool is_src)
{
  resolution = 512;
  int dim_feature = feature_list[0].size();
  feature_map.clear();
  for(int i = 0; i < dim_feature; i ++)
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
      kdTree->nearestPt(pt,pt_id); // pt has been modified
      std::vector<int> adjFaces = adjFaces_list[pt_id];
      int face_id;
      float point[3];
      point[0] = float(x) / resolution;//pt[0];
      point[1] = float(y) / resolution;;//pt[1];
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
        l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
        l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
        l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
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
      
      // put feature into feature map from feature_list
      for (int i = 0; i < dim_feature; ++i)
      {
        feature_map[i].at<float>(resolution - y - 1,x) = lambda[0] * feature_list[v_set[id1]][i] + lambda[1] * feature_list[v_set[id2]][i] + lambda[2] * feature_list[v_set[id3]][i];
      }


      /*float v1_normal_original_mesh[3],v2_normal_original_mesh[3],v3_normal_original_mesh[3];
      v1_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id1]];
      v1_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id1] + 1];
      v1_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id1] + 2];
      v2_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id2]];
      v2_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id2] + 1];
      v2_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id2] + 2];
      v3_normal_original_mesh[0] = mesh_para->normal_original_mesh[3 * v_set[id3]];
      v3_normal_original_mesh[1] = mesh_para->normal_original_mesh[3 * v_set[id3] + 1];
      v3_normal_original_mesh[2] = mesh_para->normal_original_mesh[3 * v_set[id3] + 2];*/

      /*feature_map.at<float>(x,y,0) = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map.at<float>(x,y,1) = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map.at<float>(x,y,2) = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];*/
      /*feature_map.at<cv::Vec3f>(x,y)[0] = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map.at<cv::Vec3f>(x,y)[1] = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map.at<cv::Vec3f>(x,y)[2] = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];*/
      /*feature_map[0].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[0] + lambda[1] * v2_normal_original_mesh[0] + lambda[2] * v3_normal_original_mesh[0];
      feature_map[1].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[1] + lambda[1] * v2_normal_original_mesh[1] + lambda[2] * v3_normal_original_mesh[1];
      feature_map[2].at<float>(x,y) = lambda[0] * v1_normal_original_mesh[2] + lambda[1] * v2_normal_original_mesh[2] + lambda[2] * v3_normal_original_mesh[2];*/
    }
  }
}

void DetailSynthesis::prepareDetailMap(std::shared_ptr<Model> model)
{
  // Prepare the detail map for synthesis
  // 1. details include displacement and reflectance, both are cv::Mat ?
  // 2. need to convert them to the parameterized domain
  //   1) for each pixel in parameterized image, found its original position by barycentric interpolation
  //   2) project it onto image plane and take corresponding value in original details image
  //   3) store the value into the parameterized image

  cv::FileStorage fs(model->getDataPath() + "/reflectance.xml", cv::FileStorage::READ);
  cv::Mat detail_reflectance_mat;
  fs["reflectance"] >> detail_reflectance_mat;

  std::vector<cv::Mat> detail_image(3);
  cv::split(detail_reflectance_mat, &detail_image[0]);
  // dilate the detail map in case of black
  for (int i = 0; i < 3; ++i)
  {
    ShapeUtility::dilateImage(detail_image[i], 15);
  }
  //cv::merge(&detail_image[0], 3, detail_reflectance_mat);
  //imshow("dilate souroce", detail_reflectance_mat);
  computeDetailMap(src_detail_map, detail_image, model);

  // save detail map to file
  for (size_t i = 0; i < src_detail_map.size(); ++i)
  {
    YMLHandler::saveToFile(model->getOutputPath(), std::string("src_detail_") + std::to_string(i) + ".yml", src_detail_map[i]);
    YMLHandler::saveToMat(model->getOutputPath(), std::string("src_detail_") + std::to_string(i) + ".mat", src_detail_map[i]);
  }
}

void DetailSynthesis::computeDetailMap(std::vector<cv::Mat>& detail_map, std::vector<cv::Mat>& detail_image, std::shared_ptr<Model> model)
{
  resolution = 512;
  int dim_detail = detail_image.size();
  detail_map.clear();
  for(int i = 0; i < dim_detail; i ++)
  {
    detail_map.push_back(cv::Mat(resolution, resolution, CV_32FC1));
  }

  PolygonMesh* poly_mesh = model->getPolygonMesh();
  std::shared_ptr<Shape> shape = mesh_para->cut_shape;
  std::shared_ptr<KDTreeWrapper> kdTree = mesh_para->kdTree_UV;
  STLVectori v_set = mesh_para->vertex_set;
  
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
      kdTree->nearestPt(pt,pt_id); // pt has been modified
      std::vector<int> adjFaces = adjFaces_list[pt_id];
      int face_id;
      float point[3];
      point[0] = float(x) / resolution;//pt[0];
      point[1] = float(y) / resolution;;//pt[1];
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
        l[0] = (fabs(l[0]) < 1e-4) ? 0 : l[0];
        l[1] = (fabs(l[1]) < 1e-4) ? 0 : l[1];
        l[2] = (fabs(l[2]) < 1e-4) ? 0 : l[2];
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
      
      Vector3f pos = lambda[0] * poly_mesh->position(PolygonMesh::Vertex(v_set[id1]))
                   + lambda[1] * poly_mesh->position(PolygonMesh::Vertex(v_set[id2]))
                   + lambda[2] * poly_mesh->position(PolygonMesh::Vertex(v_set[id3]));
      float winx, winy;
      model->getProjectPt(pos.data(), winx, winy); // start from left upper corner
      winy = winy < 0 ? 0 : (winy >= detail_image[0].rows ? detail_image[0].rows - 1 : winy);
      winx = winx < 0 ? 0 : (winx >= detail_image[0].cols ? detail_image[0].cols - 1 : winx);
      // put detail into detail map from detail image
      for (int i = 0; i < dim_detail; ++i)
      {
        detail_map[i].at<float>(resolution - y - 1,x) = detail_image[i].at<float>(winy, winx);
      }
    }
  }
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
  prepareFeatureMap(model);
  prepareDetailMap(model);

  syn_tool.reset(new SynthesisTool);
  syn_tool->init(src_feature_map, tar_feature_map, src_detail_map);
  syn_tool->doSynthesis();

  // map the synthesis to model color
  resolution = 512;
  std::vector<std::vector<cv::Mat> >& detail_result = syn_tool->getTargetDetail();
  detail_result[0][0]; // R
  detail_result[1][0]; // G
  detail_result[2][0]; // B

  PolygonMesh* poly_mesh = model->getPolygonMesh();
  PolygonMesh::Vertex_attribute<int> syn_texture_tag = poly_mesh->vertex_attribute<int>("v:syn_texture_tag");

  cv::Mat tar_detail;
  std::vector<cv::Mat> output_detail;
  output_detail.push_back(syn_tool->getTargetDetail()[2][0].clone());
  output_detail.push_back(syn_tool->getTargetDetail()[1][0].clone());
  output_detail.push_back(syn_tool->getTargetDetail()[0][0].clone());
  cv::merge(&output_detail[0], 3, tar_detail);
  double min,max;
  cv::minMaxLoc(tar_detail,&min,&max);
  tar_detail = tar_detail;// / max;
  model->getSynRImg() = tar_detail.clone();
  cv::imshow("result", (tar_detail) / (max));

  std::shared_ptr<Shape> shape = mesh_para->cut_shape_hidden;
  STLVectori v_set = mesh_para->vertex_set_hidden;

  STLVectorf uv_list = model->getShapeUVCoord();
  STLVectorf color_list = model->getShapeColorList();
  const STLVectorf& cut_uv_list_hidden = shape->getUVCoord();
  for (size_t i = 0; i < cut_uv_list_hidden.size() / 2; ++i)
  {
    int winx = cut_uv_list_hidden[2 * i + 0] * resolution;
    int winy = cut_uv_list_hidden[2 * i + 1] * resolution;

    winy = winy < 0 ? 0 : (winy >= detail_result[0][0].rows ? detail_result[0][0].rows - 1 : winy);
    winx = winx < 0 ? 0 : (winx >= detail_result[0][0].cols ? detail_result[0][0].cols - 1 : winx);

    float new_color[3];
    new_color[0] = detail_result[0][0].at<float>(resolution - 1 - winy, winx);
    new_color[1] = detail_result[1][0].at<float>(resolution - 1 - winy, winx);
    new_color[2] = detail_result[2][0].at<float>(resolution - 1 - winy, winx);

    // get vertex id in original model
    color_list[3 * v_set[i] + 0] = new_color[0] / max;
    color_list[3 * v_set[i] + 1] = new_color[1] / max;
    color_list[3 * v_set[i] + 2] = new_color[2] / max;

    // put uv for syn texture
    uv_list[2 * v_set[i] + 0] = cut_uv_list_hidden[2 * i + 0];
    uv_list[2 * v_set[i] + 1] = cut_uv_list_hidden[2 * i + 1];

    // put uv tag for syn texture
    syn_texture_tag[PolygonMesh::Vertex(v_set[i])] = 1;
  }

  cv::Mat src_detail;
  std::vector<cv::Mat> seen_detail;
  seen_detail.push_back(src_detail_map[2].clone());
  seen_detail.push_back(src_detail_map[1].clone());
  seen_detail.push_back(src_detail_map[0].clone());
  cv::merge(&seen_detail[0], 3, src_detail);
  cv::minMaxLoc(src_detail,&min,&max);
  src_detail = src_detail;// / max;
  model->getOriRImg() = src_detail.clone();
  cv::imshow("source", (src_detail));

  shape = mesh_para->cut_shape;
  v_set = mesh_para->vertex_set;

  const STLVectorf& cut_uv_list = shape->getUVCoord();
  for (size_t i = 0; i < cut_uv_list.size() / 2; ++i)
  {
    int winx = cut_uv_list[2 * i + 0] * resolution;
    int winy = cut_uv_list[2 * i + 1] * resolution;

    winy = winy < 0 ? 0 : (winy >= src_detail_map[0].rows ? src_detail_map[0].rows - 1 : winy);
    winx = winx < 0 ? 0 : (winx >= src_detail_map[0].cols ? src_detail_map[0].cols - 1 : winx);

    float new_color[3];
    new_color[0] = src_detail_map[0].at<float>(resolution - 1 - winy, winx);
    new_color[1] = src_detail_map[1].at<float>(resolution - 1 - winy, winx);
    new_color[2] = src_detail_map[2].at<float>(resolution - 1 - winy, winx);

    // get vertex id in original model
    color_list[3 * v_set[i] + 0] = new_color[0] / max;
    color_list[3 * v_set[i] + 1] = new_color[1] / max;
    color_list[3 * v_set[i] + 2] = new_color[2] / max;

    // put uv for original texture
    uv_list[2 * v_set[i] + 0] = cut_uv_list[2 * i + 0];
    uv_list[2 * v_set[i] + 1] = cut_uv_list[2 * i + 1];

    // put uv tag for original texture
    syn_texture_tag[PolygonMesh::Vertex(v_set[i])] = 0;
  }

  model->updateColorList(color_list);
  model->updateUVCoord(uv_list);


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

  // test image synthesis
  //cv::Mat inputImage;
  //cv::imread("sample09-low.png").convertTo(inputImage, CV_32FC3);
  //inputImage = inputImage / 255.0;
  ///*cv::Mat grayImage = cv::Mat(inputImage.size().height,inputImage.size().width,CV_32FC1);
  //cv::cvtColor(inputImage,grayImage,CV_BGR2GRAY);
  //inputImage.convertTo(inputImage,CV_32FC1);*/
  //double min,max;
  //cv::minMaxLoc(inputImage,&min,&max);
  //cv::imshow("input", (inputImage - min) / (max - min));
  //syn_tool.reset(new SynthesisTool);
  //std::vector<cv::Mat> detail_image(3);
  //cv::split(inputImage, &detail_image[0]);
  //syn_tool->doImageSynthesis(detail_image);
  //cv::Mat tar_detail;
  //std::vector<cv::Mat> output_detail;
  //output_detail.push_back(syn_tool->getTargetDetail()[0][0].clone());
  //output_detail.push_back(syn_tool->getTargetDetail()[1][0].clone());
  //output_detail.push_back(syn_tool->getTargetDetail()[2][0].clone());
  //cv::merge(&output_detail[0], 3, tar_detail);
  //cv::minMaxLoc(tar_detail,&min,&max);
  //cv::imshow("result", (tar_detail - min) / (max - min));
}

void DetailSynthesis::computeVectorField(std::shared_ptr<Model> model)
{
  /*curve_guided_vector_field.reset(new CurveGuidedVectorField);
  curve_guided_vector_field->computeVectorField(model);*/
  kevin_vector_field.reset(new KevinVectorField);
  kevin_vector_field->init(model);
  kevin_vector_field->compute_s_hvf();
  actors.clear();
  std::vector<GLActor> temp_actors;
  //curve_guided_vector_field->getDrawableActors(temp_actors);
  kevin_vector_field->getDrawableActors(temp_actors);
  for (size_t i = 0; i < temp_actors.size(); ++i)
  {
    actors.push_back(temp_actors[i]);
  }
}

void DetailSynthesis::getDrawableActors(std::vector<GLActor>& actors)
{
  actors = this->actors;
}