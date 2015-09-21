#include "MainCanvas.h"
#include "Model.h"
#include "Shape.h"
#include "BasicHeader.h"
#include "YMLHandler.h"
#include <QGLShader>
#include <QGLBuffer>

MainCanvas::MainCanvas()
  : show_background_img(false), show_model(true), wireframe_(false), flatShading_(true), save_to_file(true)
{
  render_mode = 2;
}

MainCanvas::~MainCanvas()
{

}

bool MainCanvas::display()
{
  drawModel();
  return true;
}

void MainCanvas::setGLProperty()
{
  updateModelBuffer();
  setShaderProgram();
}

Bound* MainCanvas::getBoundBox()
{
  return model->getBoundBox();
}

void MainCanvas::setShaderProgram()
{
  basic_shader.reset(new QGLShaderProgram);
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/fragmentShader.fsh");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/vertexShader.vsh");
  basic_shader->link();

  edge_detect_shader.reset(new QGLShaderProgram);
  edge_detect_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/edge.fsh");
  edge_detect_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/edge.vsh");
  edge_detect_shader->link();
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
  const VertexList& vertex_list = model->getShape()->getVertexList();
  const FaceList&   face_list   = model->getShape()->getFaceList();
  const NormalList& normal_list = model->getShape()->getNormalList();
  const STLVectorf& color_list  = model->getShape()->getColorList();

  num_vertex = GLenum(vertex_list.size() / 3);
  num_face   = GLenum(face_list.size() / 3);

  vertex_buffer.reset(new QGLBuffer);
  vertex_buffer->create();
  vertex_buffer->bind();
  vertex_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  vertex_buffer->write(0, &vertex_list[0], num_vertex * 3 * sizeof(GLfloat));
  vertex_buffer->release();

  face_buffer.reset(new QGLBuffer(QGLBuffer::IndexBuffer));
  face_buffer->create();
  face_buffer->bind();
  face_buffer->allocate(num_face * 3 * sizeof(GLuint));
  face_buffer->write(0, &face_list[0], num_face * 3 * sizeof(GLuint));
  face_buffer->release();

  color_buffer.reset(new QGLBuffer);
  color_buffer->create();
  color_buffer->bind();
  color_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  color_buffer->write(0, &color_list[0], num_vertex * 3 * sizeof(GLfloat));
  color_buffer->release();

  normal_buffer.reset(new QGLBuffer);
  normal_buffer->create();
  normal_buffer->bind();
  normal_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  normal_buffer->write(0, &normal_list[0], num_vertex * 3 * sizeof(GLfloat));
  normal_buffer->release();

  if (glGetError() != 0)
  {
    std::cout<<"GL Error in getting model\n";
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
  int newWidth  = 1<<(int)(1+log(img.width() -1+1E-3) / log(2.0));
  int newHeight = 1<<(int)(1+log(img.height()-1+1E-3) / log(2.0));

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
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  // Nice texture coordinate interpolation
  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D(GL_TEXTURE_2D, 0, 4, glImg.width(), glImg.height(), 0,
    GL_RGBA, GL_UNSIGNED_BYTE, glImg.bits());

  if (glGetError() != 0)
  {
    std::cout<<"GL Error in setting background image\n";
  }

  show_background_img = true;

  // set FBO based on the screen (backgraound image) size
  int height = img.height();
  int width = img.width();

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
    std::cout << "set frame buffer object failed\n";

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
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

  //glDrawElements(GL_TRIANGLES, 3 * num_faces, GL_UNSIGNED_INT, faces);

  //glDrawArrays(GL_TRIANGLES, 0, vertices.size());
  if (render_mode == 2)
  {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glEnable(GL_CULL_FACE);
    //glCullFace(GL_FRONT);
  }

  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  basic_shader->disableAttributeArray("vertex");
  basic_shader->disableAttributeArray("color");
  basic_shader->release();
}

void MainCanvas::drawBackground(int height, int width)
{
  if (!show_background_img)
  {
    return;
  }

  //glDisable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glColor3f(1,1,1);


  // Draws the background quad
  glNormal3f(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0,   1.0-v_max);	glVertex2i(0,0);
  glTexCoord2f(0.0,   1.0);		glVertex2i(0,height);
  glTexCoord2f(u_max, 1.0);		glVertex2i(width,height);
  glTexCoord2f(u_max, 1.0-v_max);	glVertex2i(width,0);
  glEnd();


  // Depth clear is not absolutely needed. An other option would have been to draw the
  // QUAD with a 0.999 z value (z ranges in [0, 1[ with startScreenCoordinatesSystem()).
  glClear(GL_DEPTH_BUFFER_BIT);
  glDisable(GL_TEXTURE_2D);
  //glEnable(GL_LIGHTING);

  //std::cout<<"u_max: "<<u_max<<"\tv_max: "<<v_max<<"\theight: "<<height()<<"\twidth: "<<width()<<"\n";
}

std::string MainCanvas::getFilePath()
{
  return model->getDataPath();
}

void MainCanvas::drawInfo(int height, int width)
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