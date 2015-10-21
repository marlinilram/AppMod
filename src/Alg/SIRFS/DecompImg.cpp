#include "DecompImg.h"
#include "Model.h"

#include "Solver.h"
#include "ARAP.h"
#include "NormalConstraint.h"

DecompImg::DecompImg()
{
  actors.push_back(GLActor(ML_POINT, 5.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 1.0f));
}

DecompImg::~DecompImg()
{

}

//void DecompImg::setModel(std::shared_ptr<Model> model)
//{
//  this->model = model;
//}

void DecompImg::computeNormal(std::shared_ptr<Model> model)
{
  float lambda = 0.1;
  cv::Mat normal_from_mesh = model->getNImg();
  normal_from_shading = cv::Mat(normal_from_mesh.size().height,normal_from_mesh.size().width,CV_32FC3);
  cv::Mat shading;
  cv::FileStorage fs(model->getDataPath() + "/shading.xml", cv::FileStorage::READ);
  fs["shading"] >> shading;
  /*cv::imread(model->getDataPath() + "/d22.png").convertTo(shading, CV_32FC3);*/
  cv::imshow("shading_img", shading);
  //cv::imshow("normal", normal_from_mesh);
  cv::Mat light;
  cv::Mat s = cv::Mat(normal_from_mesh.size().height * normal_from_mesh.size().width, 1, CV_32FC1);
  cv::Mat n = cv::Mat(normal_from_mesh.size().height * normal_from_mesh.size().width, 3, CV_32FC1);
  int count = 0;
  for(int i = 0; i < normal_from_mesh.size().height; i ++)
  {
    for(int j = 0; j < normal_from_mesh.size().width; j ++)
    {
      cv::Vec3f tmp = shading.at<cv::Vec3f>(i,j);
      s.at<float>(count,0) = (tmp[0] + tmp[1] + tmp[2]) / 3;
      n.at<float>(count,0) = normal_from_mesh.at<cv::Vec3f>(i,j)[0] * 2 - 1;
      n.at<float>(count,1) = normal_from_mesh.at<cv::Vec3f>(i,j)[1] * 2 - 1;
      n.at<float>(count,2) = normal_from_mesh.at<cv::Vec3f>(i,j)[2] * 2 - 1;
      count ++;
    }
  }
  cv::Mat n_transpose = n.t();
  cv::Mat inv,mul;
  mul = n_transpose * n;
  inv = mul.inv();
  light = inv * n_transpose * s;
  cv::Mat light_transpose = light.t();
  cv::Mat tmp;
  tmp = lambda * cv::Mat::eye(3,3,CV_32FC1);
  tmp += light * light_transpose;
  cv::Mat tmp_inv = tmp.inv();
  cv::Mat normal_from_shader_pixel;
  cv::Mat normal_from_mesh_pixel = cv::Mat(3,1,CV_32FC1);
  for(int i = 0; i < normal_from_mesh.size().height; i ++)
  {
    for(int j = 0; j < normal_from_mesh.size().width; j ++)
    {
      cv::Vec3f n_tmp = normal_from_mesh.at<cv::Vec3f>(i,j);
      normal_from_mesh_pixel.at<float>(0,0) = n_tmp[0] * 2 - 1;
      normal_from_mesh_pixel.at<float>(1,0) = n_tmp[1] * 2 - 1;
      normal_from_mesh_pixel.at<float>(2,0) = n_tmp[2] * 2 - 1;
      cv::Vec3f s_tmp = shading.at<cv::Vec3f>(i,j);
      normal_from_shader_pixel = tmp_inv * ((s_tmp[0] + s_tmp[1] + s_tmp[2]) / 3 * light + lambda * normal_from_mesh_pixel);
      float sum = sqrt(pow(normal_from_shader_pixel.at<float>(0,0),2) + pow(normal_from_shader_pixel.at<float>(1,0),2) + pow(normal_from_shader_pixel.at<float>(2,0),2));
      normal_from_shading.at<cv::Vec3f>(i,j)[0] = normal_from_shader_pixel.at<float>(0,0) / sum;
      normal_from_shading.at<cv::Vec3f>(i,j)[1] = normal_from_shader_pixel.at<float>(1,0) / sum;
      normal_from_shading.at<cv::Vec3f>(i,j)[2] = normal_from_shader_pixel.at<float>(2,0) / sum;
    }
  }

  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  int *primitive_id_ptr = (int *)primitive_id_img.data;

  std::map<int, Vector3f> normal_map;
  std::map<int, Vector3f>::iterator iter_map;

  for (int i = 0; i < primitive_id_img.rows; ++i)
  {
    for (int j = 0; j < primitive_id_img.cols; ++j)
    {
      int face_id = primitive_id_ptr[i * primitive_id_img.cols + j];
      if (face_id >= 0)
      {
        int x = j;
        int y = i;
        cv::Vec3f normal_in_photo = normal_from_shading.at<cv::Vec3f>(y, x);
        Vector3f new_normal;
        new_normal << normal_in_photo[2],
                      normal_in_photo[1],
                      normal_in_photo[0];
        //model->getUnprojectVec(new_normal);

        iter_map = normal_map.find(face_id);
        if (iter_map != normal_map.end())
        {
          iter_map->second += new_normal;
        }
        else
        {
          normal_map[face_id] = new_normal;
        }
      }
    }
  }

  std::vector<int> faces_in_photo;
  std::vector<Vector3f> faces_new_normal;
  for (iter_map = normal_map.begin(); iter_map != normal_map.end(); ++iter_map)
  {
    faces_in_photo.push_back(iter_map->first);
    faces_new_normal.push_back(iter_map->second.normalized());

    Vector3f start;
    model->getShapeFaceCenter(iter_map->first, start.data());
    Vector3f end = iter_map->second.normalized();
    end = start + 0.1*end;
    actors[2].addElement(start[0], start[1], start[2], 1.0, 0.0, 0.0);
    actors[2].addElement(end[0], end[1], end[2], 1.0, 0.0, 0.0);
  }
  NormalList new_normals;// = model->getShapeFaceNormal();
  for (size_t i = 0; i < faces_in_photo.size(); ++i)
  {
    int f_id = faces_in_photo[i];
    //new_normals[3 * f_id + 0] = faces_new_normal[i](0);
    //new_normals[3 * f_id + 1] = faces_new_normal[i](1);
    //new_normals[3 * f_id + 2] = faces_new_normal[i](2);
    new_normals.push_back(faces_new_normal[i](0));
    new_normals.push_back(faces_new_normal[i](1));
    new_normals.push_back(faces_new_normal[i](2));
  }

  FaceList face_list = model->getShapeFaceList();
  VertexList vertex_list = model->getShapeVertexList();
  NormalList normal_list = model->getShapeNormalList();
  AdjList vertex_shared_faces = model->getShapeVertexShareFaces();
  AdjList adj_list = model->getShapeVertexAdjList();


  std::shared_ptr<Solver> solver(new Solver);
  std::shared_ptr<ARAP> arap(new ARAP);
  std::shared_ptr<NormalConstraint> normal_constraint(new NormalConstraint);
  // add constraints to solver
  solver->addConstraint(normal_constraint);
    
  // add constraints to solver
  solver->addConstraint(arap);


  solver->problem_size = vertex_list.size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());

  arap->setSolver(solver);
  arap->initConstraint(vertex_list, face_list,adj_list);
  arap->setLamdARAP(5.0f);

  normal_constraint->setSolver(solver);
  // use partial normal
  normal_constraint->initMatrix(face_list, vertex_list, vertex_shared_faces, normal_list, new_normals, faces_in_photo);
  //normal_constraint->initMatrix(face_list, vertex_list, vertex_shared_faces, normal_list, new_normals);
  normal_constraint->setLamdNormal(3.0f);
  normal_constraint->setLamdVMove(5.0f);

  solver->initCholesky();
  int max_iter = 20;
  int cur_iter = 0;
  do 
  {
    solver->runOneStep();
    ++cur_iter;

    std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

  } while (cur_iter < max_iter);

  std::vector<float> new_vertex_list(solver->P_Opt.data(), solver->P_Opt.data() + solver->P_Opt.rows() * solver->P_Opt.cols());

  model->updateShape(new_vertex_list);

  std::cout << "Update geometry finished...\n";
}

void DecompImg::getDrawableActors(std::vector<GLActor>& actors)
{
   actors = this->actors;
}