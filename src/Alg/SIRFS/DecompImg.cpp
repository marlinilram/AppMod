#include "DecompImg.h"
#include "Model.h"
DecompImg::DecompImg()
{
  actors.push_back(GLActor(ML_POINT, 5.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 1.0f));
}

DecompImg::~DecompImg()
{

}

void DecompImg::setModel(std::shared_ptr<Model> model)
{
  this->model = model;
}

void DecompImg::computeNormal()
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


}
