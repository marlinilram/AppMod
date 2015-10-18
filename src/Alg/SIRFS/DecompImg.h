#ifndef DecompImg_H
#define DecompImg_H
#include<cv.h>
#include <memory>

class Model;

class DecompImg
{
public:
  DecompImg();
  ~DecompImg();

  void setModel(std::shared_ptr<Model> model);
  void computeNormal();
  inline cv::Mat& getNormal(){ return normal_from_shader; };
private:
  std::shared_ptr<Model> model;
  cv::Mat normal_from_shader;
  
private:
  DecompImg(const DecompImg&);
  void operator = (const DecompImg&);
};
#endif // !DecompImg_H
