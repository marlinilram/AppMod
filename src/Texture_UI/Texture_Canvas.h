#ifndef Texture_Canvas_H
#define Texture_Canvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <memory>
#include <string>
#include <vector>
class Model;
class QGLShaderProgram;
class QGLBuffer;

class Texture_Canvas : public DispObject
{
public:
	Texture_Canvas();
	~Texture_Canvas();

  void setModel(std::shared_ptr<Model> shared_model);

  virtual bool display();
  virtual void setGLProperty();
  virtual Bound* getBoundBox();

  void setShaderProgram();
  void drawModel();
  void updateModelBuffer();
  void updateModelColorBuffer();

  std::string getFilePath();

  virtual QGLViewer* viewer();
  virtual void set_viewer(QGLViewer*);

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
  bool has_data_;
private:
	Texture_Canvas(const Texture_Canvas&);
	void operator = (const Texture_Canvas&);
	
};

#endif