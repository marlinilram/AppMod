#ifndef TrackballCanvas_H
#define TrackballCanvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <memory>
#include <string>

class Model;
class QGLShaderProgram;
class QGLBuffer;

class TrackballCanvas : public DispObject
{
public:
  TrackballCanvas();
  ~TrackballCanvas();

  void setModel(std::shared_ptr<Model> shared_model);

  virtual bool display();
  virtual void setGLProperty();
  virtual Bound* getBoundBox();

  void setShaderProgram();
  void drawModel();
  void updateModelBuffer();

  std::string getFilePath();

private:
  std::shared_ptr<Model> model;

  std::unique_ptr<QGLShaderProgram> basic_shader;
  std::unique_ptr<QGLBuffer> vertex_buffer;
  std::unique_ptr<QGLBuffer> face_buffer;
  std::unique_ptr<QGLBuffer> normal_buffer;
  std::unique_ptr<QGLBuffer> color_buffer;

  GLenum num_vertex;
  GLenum num_face;

  int render_mode;

private:
  TrackballCanvas(const TrackballCanvas&);
  void operator = (const TrackballCanvas&);
};

#endif