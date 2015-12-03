#ifndef MainCanvas_H
#define MainCanvas_H

#include <glew-1.11.0/include/GL/glew.h>
#include "DispObject.h"

#include <string>
#include <memory>
#include <QString>
#include <QImage>

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
  void drawInfo(double z_scale = 1);
  void passCameraInfo(GLfloat modelview[16], GLfloat projection[16], GLint viewport[4]);
  virtual void setGLProperty();
  virtual Bound* getBoundBox();

  void setModel(std::shared_ptr<Model> model);
  void setModel(std::string path, std::string name);
  void updateModelBuffer();
  void updateModelColorBuffer();
  void setShaderProgram();
  void drawModel();
  void drawModelEdge();
  void drawShapeCrest();
  void drawPrimitiveImg();
  void updateVisibleEdge();
  void sketchShader();
  void renderNImage();
  void nmsShader();
  void setFBO();
  void setSketchFBO();

  void setBackgroundImage(QString fname);
  void setReflectanceImage(QString fname);
  void setTextureImage(QImage& glImg, GLuint& texture);

  std::string getFilePath();

  inline void setEdgeThreshold(float value) { edge_threshold = value; };
  inline void setUseFlat(int state) { use_flat = (state == 0) ? 0 : 1; };
  inline void setShowBackground(int state) { show_background_img = (state == 0) ? false : true; };
  void setCanvasRenderMode();

  // pass interaction info
  void passTagPlanePos(int x, int y);
  void clearInteractionInfo();

private:
  std::shared_ptr<Model> model;

  std::unique_ptr<QGLShaderProgram> basic_shader;
  std::unique_ptr<QGLShaderProgram> edge_detect_shader;
  std::unique_ptr<QGLShaderProgram> sketch_shader;
  std::unique_ptr<QGLShaderProgram> lf_update_shader;

  std::unique_ptr<QGLBuffer> vertex_buffer;
  std::unique_ptr<QGLBuffer> face_buffer;
  std::unique_ptr<QGLBuffer> normal_buffer;
  std::unique_ptr<QGLBuffer> color_buffer;
  std::unique_ptr<QGLBuffer> sketch_vertex_buffer;
  std::unique_ptr<QGLBuffer> vertex_crest_buffer;
  std::unique_ptr<QGLBuffer> uv_buffer;

  GLuint offscr_color;
  GLuint offscr_depth;
  GLuint offscr_fbo;

  GLuint sketch_fbo;
  GLuint sketch_texture;
  GLuint sketch_depth;

  GLuint nms_fbo;
  GLuint nms_texture;
  GLuint nms_depth;

  GLuint background_texture;
  GLuint reflect_texture;

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

  int render_with_model_transform;

private:
  MainCanvas(const MainCanvas&);
  void operator = (const MainCanvas&);
};

#endif