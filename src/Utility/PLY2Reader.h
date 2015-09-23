#ifndef PLY2Reader_H
#define PLY2Reader_H

#include <memory>
#include <string>

class Shape;

class PLY2Reader
{
public:
  PLY2Reader() {};
  ~PLY2Reader() {};

  static bool loadPLY2Mesh(std::shared_ptr<Shape> shape, std::string fname);

private:
  PLY2Reader(const PLY2Reader&);
  void operator = (PLY2Reader);
};

#endif // !PLY2Reader_H
