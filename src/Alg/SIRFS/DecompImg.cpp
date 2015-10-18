#include "DecompImg.h"
#include "Model.h"
DecompImg::DecompImg()
{

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
  float lambda = 0.5;
  cv::Mat normal_from_mesh = model->getNormalImg();
  normal_from_shader = cv::Mat(normal_from_mesh.size().height,normal_from_mesh.size().width,CV_32FC3);
  cv::Mat shader;
  //set shader
  //......
  cv::Mat light = cv::Mat(3,1,CV_32FC1);
  //set light
  //......
  cv::Mat light_transpose;
  cvTranspose(&light,&light_transpose);
  cv::Mat tmp;
  tmp = lambda * cv::Mat::eye(3,3,CV_32FC1);
  tmp += light * light_transpose;
  cv::Mat normal_from_shader_pixel;
  cv::Mat normal_from_mesh_pixel = cv::Mat(3,1,CV_32FC1);
  for(int i = 0; i < normal_from_mesh.size().height; i ++)
  {
    for(int j = 0; j < normal_from_mesh.size().width; j ++)
    {
      normal_from_mesh_pixel.at<float>(0,0) = normal_from_mesh.at<cv::Vec3f>(i,j)[0];
      normal_from_mesh_pixel.at<float>(1,0) = normal_from_mesh.at<cv::Vec3f>(i,j)[1];
      normal_from_mesh_pixel.at<float>(2,0) = normal_from_mesh.at<cv::Vec3f>(i,j)[2];
      normal_from_shader_pixel = tmp * (shader.at<float>(i,j) * light + lambda * normal_from_mesh_pixel);
      normal_from_shader.at<cv::Vec3f>(i,j)[0] = normal_from_shader_pixel.at<float>(0,0);
      normal_from_shader.at<cv::Vec3f>(i,j)[1] = normal_from_shader_pixel.at<float>(1,0);
      normal_from_shader.at<cv::Vec3f>(i,j)[2] = normal_from_shader_pixel.at<float>(2,0);
    }
  }
}
