#ifndef SynthesisCanvas_H
#define SynthesisCanvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <memory>
#include <string>
#include <QImage>

class Model;
class QGLShaderProgram;
class QGLBuffer;

class SynthesisCanvas : public DispObject
{
public:
  SynthesisCanvas();
  ~SynthesisCanvas();

  void setModel(std::shared_ptr<Model> shared_model);
  std::shared_ptr<Model> getModel() { return model; };

  virtual bool display();
  virtual void setGLProperty();
  virtual Bound* getBoundBox();

  void setFBO();
  void setShaderProgram();
  void drawModel();
  void drawPrimitiveID();
  void updateModelBuffer();
  void updateModelColorBuffer();
  std::string getFilePath();

  void setTextureImage(QImage& glImg, GLuint& texture);
  void setSynthesisReflectance();

private:
  std::shared_ptr<Model> model;

  std::unique_ptr<QGLShaderProgram> basic_shader;
  std::unique_ptr<QGLBuffer> vertex_buffer;
  std::unique_ptr<QGLBuffer> face_buffer;
  std::unique_ptr<QGLBuffer> normal_buffer;
  std::unique_ptr<QGLBuffer> color_buffer;
  std::unique_ptr<QGLBuffer> uv_buffer;

  GLuint synthesis_reflect_texture;

  GLenum num_vertex;
  GLenum num_face;

  int render_mode;

  GLuint offscr_color;
  GLuint offscr_depth;
  GLuint offscr_fbo;
  int width, height;

private:
  SynthesisCanvas(const SynthesisCanvas&);
  void operator = (const SynthesisCanvas&);
};

#endif // !SynthesisCanvas_H
