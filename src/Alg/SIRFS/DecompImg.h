#ifndef DecompImg_H
#define DecompImg_H

#include <memory>

class Model;

class DecompImg
{
public:
  DecompImg();
  ~DecompImg();

  void setModel(std::shared_ptr<Model> model);

private:
  std::shared_ptr<Model> model;
  
private:
  DecompImg(const DecompImg&);
  void operator = (const DecompImg&);
};
#endif // !DecompImg_H
