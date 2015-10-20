#ifndef DecompImg_H
#define DecompImg_H

#include <memory>
#include <cv.h>

#include "GLActor.h"

class Model;

class DecompImg
{
public:
  DecompImg();
  ~DecompImg();

  void setModel(std::shared_ptr<Model> model);
  void computeNormal();
  inline cv::Mat& getNormal(){ return normal_from_shading; };
private:
  std::shared_ptr<Model> model;
  std::vector<GLActor> actors;
  cv::Mat normal_from_shading;
  
private:
  DecompImg(const DecompImg&);
  void operator = (const DecompImg&);
};
#endif // !DecompImg_H
