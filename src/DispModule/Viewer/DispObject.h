#ifndef DispObject_H
#define DispObject_H

#include "Model.h"
// this is interface for all object which can be displayed in BasicViewer
class Bound;
class QGLViewer;

class DispObject
{
public:
  DispObject();
  virtual ~DispObject();

  virtual bool display() = 0;
  virtual void setGLProperty() = 0;
  virtual Bound* getBoundBox() = 0;
  virtual QGLViewer* viewer();
  virtual void set_viewer(QGLViewer*);
  virtual void updateModelBuffer() = 0;
  void setModel(std::shared_ptr<Model> shared_model);
  std::shared_ptr<Model> getModel();
private:
  DispObject(const DispObject&);
  void operator = (const DispObject&);
  QGLViewer* m_viewer_;
  std::shared_ptr<Model> m_model_;

};

#endif