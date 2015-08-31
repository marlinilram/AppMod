#ifndef DispObject_H
#define DispObject_H

// this is interface for all object which can be displayed in BasicViewer
#include "Bound.h"

class DispObject
{
public:
  DispObject();
  virtual ~DispObject();

  virtual bool display() = 0;
  virtual void setGLProperty() = 0;
  virtual Bound* getBoundBox() = 0;


protected:
  Bound bound;

private:
  DispObject(const DispObject&);
  void operator = (const DispObject&);
};

#endif