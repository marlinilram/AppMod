#ifndef MainCanvas_H
#define MainCanvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <string>
#include <memory>
#include <QString>

class Model;
class QGLShaderProgram;
class QGLBuffer;

// need to deal with background image
// and edge detection shader
// only deal with drawing and rendering here
// interaction will be processed in the Viewer class
class MainCanvas : public DispObject
{
public:
  MainCanvas();
  ~MainCanvas();

  virtual bool display();
  void drawBackground();
  void drawInfo();
  virtual void setGLProperty();
  virtual Bound* getBoundBox();

  void setModel(std::shared_ptr<Model> model);
  void setModel(std::string path, std::string name);
  void updateModelBuffer();
  void setShaderProgram();
  void drawModel();
  void drawModelEdge();
  void sketchShader();
  void setFBO();
  void setSketchFBO();

  void setBackgroundImage(QString fname);

  std::string getFilePath();

  inline void setEdgeThreshold(float value) { edge_threshold = value * 2.0; };
  inline void setUseFlat(int state) { use_flat = (state == 0) ? 0 : 1; };

private:
  std::shared_ptr<Model> model;

  std::unique_ptr<QGLShaderProgram> basic_shader;
  std::unique_ptr<QGLShaderProgram> edge_detect_shader;
  std::unique_ptr<QGLShaderProgram> sketch_shader;

  std::unique_ptr<QGLBuffer> vertex_buffer;
  std::unique_ptr<QGLBuffer> face_buffer;
  std::unique_ptr<QGLBuffer> normal_buffer;
  std::unique_ptr<QGLBuffer> color_buffer;
  std::unique_ptr<QGLBuffer> sketch_vertex_buffer;

  GLuint offscr_color;
  GLuint offscr_depth;
  GLuint offscr_fbo;

  GLuint sketch_fbo;
  GLuint sketch_texture;
  GLuint sketch_depth;

  GLuint background_texture;

  GLenum num_vertex;
  GLenum num_face;

  int width, height;
  float ratio, u_max, v_max;
  bool show_background_img;

  bool save_to_file;
  bool wireframe_, flatShading_;
  bool show_model;
  int render_mode;

  float edge_threshold;
  int use_flat;

private:
  MainCanvas(const MainCanvas&);
  void operator = (const MainCanvas&);
};

#endif