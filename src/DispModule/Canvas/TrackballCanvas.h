#ifndef TrackballCanvas_H
#define TrackballCanvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <memory>
#include <string>
#include <vector>
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
  void drawPrimitiveID();
  void updateModelBuffer();
  void updateModelColorBuffer();

  std::string getFilePath();
  void setsize(int w, int h);
  cv::Mat& get_primitive(){ return this->primitive_ID; };
  virtual QGLViewer* viewer();
  virtual void set_viewer(QGLViewer*);
  void setFBO();
private:

  std::unique_ptr<QGLShaderProgram> basic_shader;
  std::unique_ptr<QGLBuffer> vertex_buffer;
  std::unique_ptr<QGLBuffer> face_buffer;
  std::unique_ptr<QGLBuffer> normal_buffer;
  std::unique_ptr<QGLBuffer> color_buffer;

  GLenum num_vertex;
  GLenum num_face;

  int render_mode;
  int use_flat;

  GLuint offscr_color;
  GLuint offscr_depth;
  GLuint offscr_fbo;
  int width, height;
private:
  cv::Mat primitive_ID;
  TrackballCanvas(const TrackballCanvas&);
  void operator = (const TrackballCanvas&);
};

#endif