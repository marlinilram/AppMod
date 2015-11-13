#include "MainCanvas.h"
#include "Model.h"

#include "YMLHandler.h"
#include "Colormap.h"
#include "CurvesUtility.h"
#include "ParameterMgr.h"
#include <QGLShader>
#include <QGLBuffer>

MainCanvas::MainCanvas()
  : show_background_img(false), show_model(true), wireframe_(false), flatShading_(true), save_to_file(true)
{
  render_mode = 2;
  edge_threshold = 0.75;
  use_flat = 1;

  render_with_model_transform = 0;
}

MainCanvas::~MainCanvas()
{

}

bool MainCanvas::display()
{
  drawPrimitiveImg();

  if (use_flat)
  {
    int render_mode_cache = render_mode;
    render_mode = 4;
    //drawModel();
    drawShapeCrest();
    render_mode = render_mode_cache;
  }
  else
  {
    int render_mode_cache = render_mode;
    render_mode = 3;
    glDisable(GL_LIGHTING);
    //glDepthFunc(GL_ALWAYS);
    drawModel();
    //glDepthFunc(GL_LEQUAL);
    render_mode = render_mode_cache;
  }
  return true;
}

void MainCanvas::setGLProperty()
{
  setShaderProgram();
  updateModelBuffer();
}

Bound* MainCanvas::getBoundBox()
{
  return model->getBoundBox();
}

void MainCanvas::setShaderProgram()
{
  vertex_buffer.reset(new QGLBuffer);
  vertex_buffer->create();
  face_buffer.reset(new QGLBuffer(QGLBuffer::IndexBuffer));
  face_buffer->create();
  color_buffer.reset(new QGLBuffer);
  color_buffer->create();
  normal_buffer.reset(new QGLBuffer);
  normal_buffer->create();
  vertex_crest_buffer.reset(new QGLBuffer);
  vertex_crest_buffer->create();

  basic_shader.reset(new QGLShaderProgram);
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/fragmentShader.frag");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/vertexShader.vert");
  basic_shader->link();

  edge_detect_shader.reset(new QGLShaderProgram);
  edge_detect_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/edge.frag");
  edge_detect_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/edge.vert");
  edge_detect_shader->link();

  sketch_shader.reset(new QGLShaderProgram);
  sketch_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/sketch.frag");
  sketch_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/sketch.vert");
  sketch_shader->link();

  lf_update_shader.reset(new QGLShaderProgram);
  lf_update_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/lf_update.frag");
  lf_update_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/lf_update.vert");
  lf_update_shader->link();
}

void MainCanvas::setModel(std::shared_ptr<Model> model)
{
  this->model = model;
}

void MainCanvas::setModel(std::string path, std::string name)
{
  model.reset(new Model(path, name));
}

void MainCanvas::updateModelBuffer()
{
  const VertexList& vertex_list = model->getShapeVertexList();
  const FaceList&   face_list   = model->getShapeFaceList();
  const NormalList& normal_list = model->getShapeNormalList();
  const STLVectorf& color_list  = model->getShapeColorList();
  const Edges&      crest_edge = model->getShapeCrestEdge();
  std::vector<GLfloat> v_crest(vertex_list.size() / 3, 0.0);
  for (size_t i = 0; i < crest_edge.size(); ++i)
  {
    v_crest[crest_edge[i].first] = 1.0f;
    v_crest[crest_edge[i].second] = 1.0f;
  }

  num_vertex = GLenum(vertex_list.size() / 3);
  num_face   = GLenum(face_list.size() / 3);

  vertex_buffer->bind();
  vertex_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  vertex_buffer->write(0, &vertex_list[0], num_vertex * 3 * sizeof(GLfloat));
  vertex_buffer->release();

  face_buffer->bind();
  face_buffer->allocate(num_face * 3 * sizeof(GLuint));
  face_buffer->write(0, &face_list[0], num_face * 3 * sizeof(GLuint));
  face_buffer->release();

  color_buffer->bind();
  color_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  color_buffer->write(0, &color_list[0], num_vertex * 3 * sizeof(GLfloat));
  color_buffer->release();

  normal_buffer->bind();
  normal_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  normal_buffer->write(0, &normal_list[0], num_vertex * 3 * sizeof(GLfloat));
  normal_buffer->release();

  vertex_crest_buffer->bind();
  vertex_crest_buffer->allocate(num_vertex * 1 * sizeof(GLfloat));
  vertex_crest_buffer->write(0, &v_crest[0], num_vertex * 1 * sizeof(GLfloat));

  GLenum error_code = glGetError();
  if (error_code != 0)
  {
    std::cout<<"MainCanvas: GL Error in getting model. Error Code: " << error_code << "\n";
  }
}

void MainCanvas::setBackgroundImage(QString fname)
{
  QImage img(fname);

  if (img.isNull())
  {
    qWarning("Unable to load file, unsupported file format");
    return;
  }

  qWarning("Load %s, %dx%d pixels", fname.toLatin1().constData(), img.width(), img.height());

  // 1E-3 needed. Just try with width=128 and see !
  int newWidth  = img.width(); //1<<(int)(1+log(img.width() -1+1E-3) / log(2.0));
  int newHeight = img.height(); //1<<(int)(1+log(img.height()-1+1E-3) / log(2.0));

  u_max = img.width()  / (float)newWidth;
  v_max = img.height() / (float)newHeight;

  if ((img.width()!=newWidth) || (img.height()!=newHeight))
  {
    qWarning("Image size set to %dx%d pixels", newWidth, newHeight);
    img = img.copy(0, 0, newWidth, newHeight);
  }

  ratio = newWidth / float(newHeight);

  QImage glImg = QGLWidget::convertToGLFormat(img);  // flipped 32bit RGBA

  // Bind the img texture...
  // Enable GL textures  
  glBindTexture(GL_TEXTURE_2D, 0);
  glGenTextures(1, &background_texture);
  glBindTexture(GL_TEXTURE_2D, background_texture);

  if(glIsTexture(background_texture)) std::cout<<"gen background texture ok..\n";
  else std::cout << "gen background texture failed...\n";

  // Nice texture coordinate interpolation
  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D(GL_TEXTURE_2D, 0, 4, glImg.width(), glImg.height(), 0,
    GL_RGBA, GL_UNSIGNED_BYTE, glImg.bits());

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  if (glGetError() != 0)
  {
    std::cout<<"GL Error in setting background image\n";
  }

  show_background_img = true;

  // set FBO based on the screen (backgraound image) size
  height = img.height();
  width = img.width();

  setFBO();
  setSketchFBO();
}

void MainCanvas::setFBO()
{
  glGenFramebuffers(1, &offscr_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

  glGenRenderbuffers(1, &offscr_color);
  glBindRenderbuffer(GL_RENDERBUFFER, offscr_color);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA32F, width, height);

  glGenRenderbuffers(1, &offscr_depth);
  glBindRenderbuffer(GL_RENDERBUFFER, offscr_depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, width, height);

  // attach color and depth textures to fbo
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, offscr_color);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, offscr_depth);

  static const GLenum draw_buffers[] = { GL_COLOR_ATTACHMENT0 };
  glDrawBuffers(1, draw_buffers);

  GLenum framebuffer_status;
  framebuffer_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (framebuffer_status != GL_FRAMEBUFFER_COMPLETE)
    std::cout << "set offscreen frame buffer object failed\n";

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MainCanvas::setSketchFBO()
{
  // set sketch fbo
  glGenFramebuffers(1, &sketch_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, sketch_fbo);

  glGenTextures(1, &sketch_texture);
  glBindTexture(GL_TEXTURE_2D, sketch_texture);

  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0,
    GL_RGBA, GL_FLOAT, 0);

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  glGenRenderbuffers(1, &sketch_depth);
  glBindRenderbuffer(GL_RENDERBUFFER, sketch_depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, sketch_depth);

  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, sketch_texture, 0);

  // Set the list of draw buffers.
  GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
  glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

  // Always check that our framebuffer is ok
  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::cout << "set sketch frame buffer object failed\n";

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // set non maximum surpression fbo
  glGenFramebuffers(1, &nms_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, nms_fbo);

  glGenTextures(1, &nms_texture);
  glBindTexture(GL_TEXTURE_2D, nms_texture);

  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0,
    GL_RGBA, GL_FLOAT, 0);

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  glGenRenderbuffers(1, &nms_depth);
  glBindRenderbuffer(GL_RENDERBUFFER, nms_depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, nms_depth);

  glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, nms_texture, 0);

  // Set the list of draw buffers.
  GLenum DrawBufferss[1] = {GL_COLOR_ATTACHMENT0};
  glDrawBuffers(1, DrawBufferss); // "1" is the size of DrawBuffers

  // Always check that our framebuffer is ok
  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::cout << "set sketch frame buffer object failed\n";

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  static const GLfloat g_quad_vertex_buffer_data[] = { 
    -1.0f, -1.0f, 0.0f,
    1.0f, -1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f,
    1.0f, -1.0f, 0.0f,
    1.0f,  1.0f, 0.0f,
  };

  sketch_vertex_buffer.reset(new QGLBuffer);
  sketch_vertex_buffer->create();
  sketch_vertex_buffer->bind();
  sketch_vertex_buffer->allocate(18 * sizeof(GLfloat));
  sketch_vertex_buffer->write(0, g_quad_vertex_buffer_data, 18 * sizeof(GLfloat));
  sketch_vertex_buffer->release();
}

void MainCanvas::drawModel()
{
  if (!show_model)
  {
    return;
  }

  basic_shader->bind();

  basic_shader->setUniformValue("fMeshSize", GLfloat(num_face));
  basic_shader->setUniformValue("L", QVector3D(-0.4082, -0.4082, 0.8165));
  basic_shader->setUniformValue("renderMode", GLint(render_mode));
  basic_shader->setUniformValue("threshold", GLfloat(edge_threshold));

  color_buffer->bind();
  basic_shader->setAttributeBuffer("color", GL_FLOAT, 0, 3, 0);
  basic_shader->enableAttributeArray("color");
  color_buffer->release();

  vertex_buffer->bind();
  //shaderProgram->setAttributeArray("vertex", vertices.constData());
  basic_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  basic_shader->enableAttributeArray("vertex");
  vertex_buffer->release();

  normal_buffer->bind();
  basic_shader->setAttributeBuffer("normal", GL_FLOAT, 0, 3, 0);
  basic_shader->enableAttributeArray("normal");
  normal_buffer->release();

  vertex_crest_buffer->bind();
  basic_shader->setAttributeBuffer("vCrestTag", GL_FLOAT, 0, 1, 0);
  basic_shader->enableAttributeArray("vCrestTag");
  vertex_crest_buffer->release();

  //glDrawElements(GL_TRIANGLES, 3 * num_faces, GL_UNSIGNED_INT, faces);

  //glDrawArrays(GL_TRIANGLES, 0, vertices.size());
  if (render_mode == 2)
  {
    //glDepthFunc(GL_ALWAYS);
    //glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glEnable(GL_CULL_FACE);
    //glCullFace(GL_FRONT);
  }

  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();
  
  //glDisable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  //glDepthFunc(GL_LESS);
  //glEnable(GL_DEPTH_TEST);


  basic_shader->disableAttributeArray("vertex");
  basic_shader->disableAttributeArray("color");
  basic_shader->disableAttributeArray("normal");
  basic_shader->release();

}

void MainCanvas::drawShapeCrest()
{
  const VertexList& vertex_list = model->getShapeVertexList();
  const std::vector<Edge>& crest_edge = model->getShapeCrestEdge();
  const std::vector<STLVectori>& crest_lines = model->getShapeVisbleCrestLine();
  const NormalList& vertex_normal = model->getShapeNormalList();

  Matrix4f model_transform = Matrix4f::Identity();
  if (render_with_model_transform != 0)
  {
    model_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  }

  //glClear(GL_DEPTH_BUFFER_BIT);
  glLineWidth(3);
  glDisable(GL_LIGHTING);
  glDepthFunc(GL_LEQUAL);

  //for (size_t i = 0; i < crest_edge.size(); ++i)
  //{
  //  GLfloat v_0[3];
  //  v_0[0] = vertex_list[3 * crest_edge[i].first + 0] + edge_threshold * vertex_normal[3 * crest_edge[i].first + 0];
  //  v_0[1] = vertex_list[3 * crest_edge[i].first + 1] + edge_threshold * vertex_normal[3 * crest_edge[i].first + 1];
  //  v_0[2] = vertex_list[3 * crest_edge[i].first + 2] + edge_threshold * vertex_normal[3 * crest_edge[i].first + 2];

  //  GLfloat v_1[3];
  //  v_1[0] = vertex_list[3 * crest_edge[i].second + 0] + edge_threshold * vertex_normal[3 * crest_edge[i].second + 0];
  //  v_1[1] = vertex_list[3 * crest_edge[i].second + 1] + edge_threshold * vertex_normal[3 * crest_edge[i].second + 1];
  //  v_1[2] = vertex_list[3 * crest_edge[i].second + 2] + edge_threshold * vertex_normal[3 * crest_edge[i].second + 2];

  //  glVertex3f(v_0[0], v_0[1], v_0[2]);
  //  glVertex3f(v_1[0], v_1[1], v_1[2]);
  //}

  for (size_t i = 0; i < crest_lines.size(); ++i)
  {
    //QColor color = 
    //  qtJetColor(double(i)/crest_lines.size());
    //glColor4f( color.redF(), color.greenF(), color.blueF(), 0.1f );
    glColor4f(0.0f, 1.0f, 0.0f, 0.0f);
    glBegin(GL_LINE_STRIP);

    for (size_t j = 0; j < crest_lines[i].size(); ++j)
    {
      Vector4f v;
      v[0] = vertex_list[3 * crest_lines[i][j] + 0];
      v[1] = vertex_list[3 * crest_lines[i][j] + 1];
      v[2] = vertex_list[3 * crest_lines[i][j] + 2];
      v[3] = 1.0f;
      v = model_transform * v;
      v = v / v[3];

      glVertex3f(v[0], v[1], v[2]);
    }

    glEnd();
  }

  //glReadBuffer(GL_COLOR_ATTACHMENT0);
  //cv::Mat &edge_image = model->getEdgeImg();
  //edge_image.create(height, width, CV_32FC1);
  //glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, (float*)edge_image.data);
  //cv::flip(edge_image, edge_image, 0);
  //edge_image = 1 - edge_image;
}

void MainCanvas::drawModelEdge()
{
  if (!show_model)
  {
    return;
  }

  edge_detect_shader->bind();

  edge_detect_shader->setUniformValue("fMeshSize", GLfloat(num_face));
  edge_detect_shader->setUniformValue("L", QVector3D(-0.4082, -0.4082, 0.8165));
  edge_detect_shader->setUniformValue("renderMode", GLint(render_mode));
  edge_detect_shader->setUniformValue("use_flat", GLint(use_flat));

  color_buffer->bind();
  edge_detect_shader->setAttributeBuffer("color", GL_FLOAT, 0, 3, 0);
  edge_detect_shader->enableAttributeArray("color");
  color_buffer->release();

  vertex_buffer->bind();
  //shaderProgram->setAttributeArray("vertex", vertices.constData());
  edge_detect_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  edge_detect_shader->enableAttributeArray("vertex");
  vertex_buffer->release();

  normal_buffer->bind();
  edge_detect_shader->setAttributeBuffer("normal", GL_FLOAT, 0, 3, 0);
  edge_detect_shader->enableAttributeArray("normal");
  normal_buffer->release();

  //glDrawElements(GL_TRIANGLES, 3 * num_faces, GL_UNSIGNED_INT, faces);

  //glDrawArrays(GL_TRIANGLES, 0, vertices.size());
  //if (render_mode == 2)
  //{
  //  glEnable(GL_BLEND);
  //  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //  //glEnable(GL_CULL_FACE);
  //  //glCullFace(GL_FRONT);
  //}

  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  edge_detect_shader->disableAttributeArray("vertex");
  edge_detect_shader->disableAttributeArray("color");
  edge_detect_shader->disableAttributeArray("normal");
  edge_detect_shader->release();
}

void MainCanvas::sketchShader()
{
  glBindFramebuffer(GL_FRAMEBUFFER, sketch_fbo);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawModelEdge();

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  cv::Mat z_img;
  z_img.create(height, width, CV_32FC1);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, (float*)z_img.data);
  float min_depth = std::numeric_limits<float>::max();
  float max_depth = std::numeric_limits<float>::min();
  float *ptr = (float*)(z_img.data);
  for(int j = 0;j < z_img.rows;j++){
    for(int i = 0;i < z_img.cols;i++){
      float value = *ptr;
      if (value > max_depth && value < 1.0) max_depth = value;
      if (value < min_depth && value > 0.0) min_depth = value;
      ++ptr;
    }
  }

//  std:: cout << "mindepth: " << min_depth << "\tmaxdepth: " << max_depth << "\n";

  cv::Mat r_img;
  r_img.create(height, width, CV_32FC3);
  glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, (float*)r_img.data);
  cv::flip(r_img, r_img, 0);

  // if we can the smallest z and largest z
  // rescale in the shader

  glBindFramebuffer(GL_FRAMEBUFFER, nms_fbo);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  sketch_shader->bind();

  // step 1 for edge detection
  // compute gradient in x, y direction, magnitute and direction
  sketch_shader->setUniformValue("sketch_texture", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, sketch_texture);
  glActiveTexture(0);

  sketch_shader->setUniformValue("width", GLfloat(width));
  sketch_shader->setUniformValue("height", GLfloat(height));
  sketch_shader->setUniformValue("isBackground", GLint(0));
  sketch_shader->setUniformValue("threshold", GLfloat(edge_threshold));
  sketch_shader->setUniformValue("minDepth", GLfloat(min_depth));
  sketch_shader->setUniformValue("maxDepth", GLfloat(max_depth));

  // 1rst attribute buffer : vertices
  sketch_vertex_buffer->bind();
  sketch_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  sketch_shader->enableAttributeArray("vertex");
  // Draw the triangles !
  glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles
  sketch_vertex_buffer->release();

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  cv::Mat mag_img;
  mag_img.create(height, width, CV_32FC3);
  glReadPixels(0, 0, width, height, GL_RGB, GL_FLOAT, (float*)mag_img.data);
  cv::flip(mag_img, mag_img, 0);

  // step 2 for edge detection
  // non maximum surpression
  sketch_shader->disableAttributeArray("vertex");
  sketch_shader->release();

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  sketch_shader->bind();

  // Bind our texture in Texture Unit 0
  sketch_shader->setUniformValue("sketch_texture", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, nms_texture);
  glActiveTexture(0);

  sketch_shader->setUniformValue("isBackground", GLint(2));
  sketch_shader->setUniformValue("width", GLfloat(width));
  sketch_shader->setUniformValue("height", GLfloat(height));
  sketch_shader->setUniformValue("threshold", GLfloat(edge_threshold));

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // 1rst attribute buffer : vertices
  sketch_vertex_buffer->bind();
  sketch_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  sketch_shader->enableAttributeArray("vertex");
  // Draw the triangles !
  glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles
  sketch_vertex_buffer->release();

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  cv::Mat &edge_image = model->getEdgeImg();
  edge_image.create(height, width, CV_32FC1);
  glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, (float*)edge_image.data);
  cv::flip(edge_image, edge_image, 0);

  cv::Mat pre_edge_image;
  pre_edge_image.create(height, width, CV_32FC3);
  glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, (float*)pre_edge_image.data);
  cv::flip(pre_edge_image, pre_edge_image, 0);

  std::string data_path = model->getOutputPath();
  cv::imwrite(data_path + "/pre_edge_image.png", pre_edge_image*255);

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  sketch_shader->disableAttributeArray("vertex");
  sketch_shader->release();
  // Set our "renderedTexture" sampler to user Texture Unit 0
  ////std::cout << glGetError()<<"\n";
  //glBindTexture(GL_TEXTURE_2D, 0);
  //glGenTextures(1, &text_ogl);
  ////std::cout << glGetError()<<"\n";

  //glBindTexture(GL_TEXTURE_2D, text_ogl);

  //if(glIsTexture(text_ogl)) std::cout<<"gen texture ok..\n";
  //else std::cout << "gen texture failed...\n";

  ////std::vector<float> data(3*rho_img.rows*rho_img.cols);
  //std::vector<float> data;
  //for (int i = 0; i < rho_img.rows; ++i)
  //{
  //  for (int j = 0; j < rho_img.cols; ++j)
  //  {
  //    data.push_back(rho_img.at<cv::Vec3f>(rho_img.rows-1-i, j)[2]);
  //    data.push_back(rho_img.at<cv::Vec3f>(rho_img.rows-1-i, j)[1]);
  //    data.push_back(rho_img.at<cv::Vec3f>(rho_img.rows-1-i, j)[0]);
  //  }
  //}
  ////std::cout << "test";
  ////glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB32F, rho_img.cols, rho_img.rows);
  //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, rho_img.cols, rho_img.rows, 0, GL_RGB, GL_FLOAT, &data[0]);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

void MainCanvas::drawBackground()
{
  if (!show_background_img)
  {
    return;
  }

  sketch_shader->bind();

  // Bind our texture in Texture Unit 0
  sketch_shader->setUniformValue("sketch_texture", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, background_texture);
  //glActiveTexture(0); // it leads to glerror 1280 which I don't know why

  sketch_shader->setUniformValue("isBackground", GLint(1));

  // 1rst attribute buffer : vertices
  sketch_vertex_buffer->bind();
  sketch_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  sketch_shader->enableAttributeArray("vertex");
  // Draw the triangles !
  glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles
  sketch_vertex_buffer->release();

  sketch_shader->disableAttributeArray("vertex");
  sketch_shader->release();

  //glDisable(GL_LIGHTING);
  //glEnable(GL_TEXTURE_2D);
  //glColor3f(1,1,1);


  //// Draws the background quad
  //glNormal3f(0.0, 0.0, 1.0);
  //glBegin(GL_QUADS);
  //glTexCoord2f(0.0,   1.0-v_max);	glVertex2i(0,0);
  //glTexCoord2f(0.0,   1.0);		glVertex2i(0,height);
  //glTexCoord2f(u_max, 1.0);		glVertex2i(width,height);
  //glTexCoord2f(u_max, 1.0-v_max);	glVertex2i(width,0);
  //glEnd();


  //// Depth clear is not absolutely needed. An other option would have been to draw the
  //// QUAD with a 0.999 z value (z ranges in [0, 1[ with startScreenCoordinatesSystem()).
  //glDisable(GL_TEXTURE_2D);
  glClear(GL_DEPTH_BUFFER_BIT);
  //glEnable(GL_LIGHTING);

  //std::cout<<"u_max: "<<u_max<<"\tv_max: "<<v_max<<"\theight: "<<height()<<"\twidth: "<<width()<<"\n";
}

std::string MainCanvas::getFilePath()
{
  return model->getDataPath();
}

void MainCanvas::drawInfo()
{
  float *primitive_buffer = new float[height*width];
  cv::Mat &r_img = model->getRImg();
  r_img.create(height, width, CV_32FC3);
  cv::Mat &z_img = model->getZImg();
  z_img.create(height, width, CV_32FC1);
  cv::Mat &mask_rimg = model->getRMask();

  int render_mode_cache = render_mode;
  render_mode = 1;

  bool show_background_img_cache = show_background_img;
  show_background_img = false;

  glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawModel();

  glReadBuffer(GL_COLOR_ATTACHMENT0);

  glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, primitive_buffer);
  glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, (float*)r_img.data);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, (float*)z_img.data);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  render_mode = render_mode_cache;
  show_background_img = show_background_img_cache;

  cv::Mat primitive_ID_img(height, width, CV_32FC1, primitive_buffer);
  cv::flip(primitive_ID_img, primitive_ID_img, 0);
  //cv::Mat z_img(height, width, CV_32FC1, z_buffer);
  cv::flip(z_img, z_img, 0);
  //cv::Mat r_img(height, width, CV_32FC3, image_buffer);
  cv::flip(r_img, r_img, 0);

  cv::Mat &primitive_ID = model->getPrimitiveIDImg();
  primitive_ID.create(height, width, CV_32S);
  primitive_ID.setTo(cv::Scalar(-1));

  for (int j = 0; j < width; ++j)
  {
    for (int i = 0; i < height; ++i)
    {
      float fPrimitive = primitive_ID_img.at<float>(i, j);

      fPrimitive = fPrimitive*num_face;
      int iPrimitive = (int)(fPrimitive < 0 ? (fPrimitive - 0.5) : (fPrimitive + 0.5));

      //if (iPrimitive == 169)
      //{
      //    test.at<float>(i, j) = 1;
      //}
      //else test.at<float>(i, j) = 0;
      if (iPrimitive < (int)num_face) primitive_ID.at<int>(i, j) = iPrimitive;
      //{
      //if (iPrimitive == 168 || iPrimitive == 169 || iPrimitive == 170)
      //    a += std::to_string(i) + " " + std::to_string(j) + " " + std::to_string(iPrimitive) + "\n";
      //}
    }
    //LOG::Instance()->OutputMisc(a.c_str());
    //cv::imwrite("primitiveImg.png", primitive_ID_img * 255);
  }

  //model->passRenderImgInfo(z_img, primitive_ID, r_img);
  cv::Mat mask_temp = (primitive_ID >= 0);
  mask_temp.convertTo(mask_rimg, CV_8UC1);

  if (save_to_file)
  {
    std::string data_path = model->getOutputPath();
    cv::imwrite(data_path + "/r_img.png", r_img*255);
    cv::imwrite(data_path + "/z_img.png", z_img*255);
    cv::imwrite(data_path + "/primitive_img.png", primitive_ID_img*255);

    YMLHandler::saveToFile(data_path, std::string("rendered.yml"), r_img);
    //YMLHandler::saveToFile(data_path, std::string("primitive.yml"), primitive_ID);
  }

  delete primitive_buffer;
}


void MainCanvas::renderNImage()
{
  // draw eye normal to offscreen buffer for intrinsic image decomposition
  int render_mode_cache = render_mode;
  render_mode = 1;

  bool show_background_img_cache = show_background_img;
  show_background_img = false;

  glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawModel();

  glReadBuffer(GL_COLOR_ATTACHMENT0);

  //glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, primitive_buffer);
  //glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, (float*)r_img.data);
  //glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, (float*)z_img.data);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  render_mode = render_mode_cache;
  show_background_img = show_background_img_cache;
}

void MainCanvas::passCameraInfo(GLfloat modelview[16], GLfloat projection[16], GLint viewport[4])
{
  model->passCameraPara(modelview, projection, viewport);
}

void MainCanvas::drawPrimitiveImg()
{
  int render_mode_cache = render_mode;
  render_mode = 1;

  bool show_background_img_cache = show_background_img;
  show_background_img = false;

  float *primitive_buffer = new float[height*width];
  cv::Mat &z_img = model->getZImg();
  z_img.create(height, width, CV_32FC1);

  cv::Mat &n_img = model->getNImg();
  n_img.create(height, width, CV_32FC3);

  glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawModel();
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, primitive_buffer);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, (float*)z_img.data);
  glReadPixels(0, 0, width, height, GL_BGR, GL_FLOAT, (float*)n_img.data);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  render_mode = render_mode_cache;
  show_background_img = show_background_img_cache;

  cv::Mat primitive_ID_img(height, width, CV_32FC1, primitive_buffer);
  cv::flip(primitive_ID_img, primitive_ID_img, 0);
  cv::flip(z_img, z_img, 0);
  cv::flip(n_img, n_img, 0);

  cv::Mat &primitive_ID = model->getPrimitiveIDImg();
  primitive_ID.create(height, width, CV_32S);
  primitive_ID.setTo(cv::Scalar(-1));
  std::set<int> vis_faces;

  for (int j = 0; j < width; ++j)
  {
    for (int i = 0; i < height; ++i)
    {
      float fPrimitive = primitive_ID_img.at<float>(i, j);

      fPrimitive = fPrimitive*num_face;
      int iPrimitive = (int)(fPrimitive < 0 ? (fPrimitive - 0.5) : (fPrimitive + 0.5));

      if (iPrimitive < (int)num_face) 
      {
        primitive_ID.at<int>(i, j) = iPrimitive;
        if (vis_faces.find(iPrimitive) == vis_faces.end())
        {
          vis_faces.insert(iPrimitive);
        }
      }
    }
  }
  
  CurvesUtility::getBoundaryImg(model->getEdgeImg(), primitive_ID);
  model->computeShapeCrestVisible(vis_faces);
  delete primitive_buffer;
}

void MainCanvas::passTagPlanePos(int x, int y)
{
  model->addTaggedPlane(x, y);
}

void MainCanvas::clearInteractionInfo()
{
  // clear the tag plane
  model->clearTaggedPlanes();
}

void MainCanvas::updateVisibleEdge()
{
  float *primitive_buffer = new float[height*width];

  glBindFramebuffer(GL_FRAMEBUFFER, offscr_fbo);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  lf_update_shader->bind();

  lf_update_shader->setUniformValue("fMeshSize", GLfloat(num_face));
  Matrix4f& tmp_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform");
  lf_update_shader->setUniformValue("rigidTransform", QMatrix4x4(tmp_transform.data()).transposed());

  vertex_buffer->bind();
  //shaderProgram->setAttributeArray("vertex", vertices.constData());
  lf_update_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
  lf_update_shader->enableAttributeArray("vertex");
  vertex_buffer->release();

  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  lf_update_shader->disableAttributeArray("vertex");
  lf_update_shader->release();

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, width, height, GL_ALPHA, GL_FLOAT, primitive_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  std::set<int> vis_faces;
  for (int i = 0; i < width * height; ++i)
  {
    float fPrimitive = primitive_buffer[i] * num_face;
    int iPrimitive = (int)(fPrimitive < 0 ? (fPrimitive - 0.5) : (fPrimitive + 0.5));
    if (iPrimitive < (int)num_face) 
    {
      if (vis_faces.find(iPrimitive) == vis_faces.end())
      {
        vis_faces.insert(iPrimitive);
      }
    }
  }

  model->computeShapeCrestVisible(vis_faces);
  delete primitive_buffer;
}

void MainCanvas::setCanvasRenderMode()
{
  render_with_model_transform = LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("LFeature:renderWithTransform");
}