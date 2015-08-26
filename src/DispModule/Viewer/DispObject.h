#ifndef DispObject_H
#define DispObject_H

// this is interface for all object which can be displayed in BasicViewer

class DispObject
{
public:
  DispObject();
  virtual ~DispObject();

  virtual bool display() = 0;

private:
  DispObject(const DispObject&);
  void operator = (const DispObject&);
};

#endif