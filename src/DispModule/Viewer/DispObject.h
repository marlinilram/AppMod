#ifndef DispObject_H
#define DispObject_H

// this is interface for all object which can be displayed in BasicViewer
class Bound;

class DispObject
{
public:
  DispObject();
  virtual ~DispObject();

  virtual bool display() = 0;
  virtual void setGLProperty() = 0;
  virtual Bound* getBoundBox() = 0;

private:
  DispObject(const DispObject&);
  void operator = (const DispObject&);
};

#endif