#include "TrackballCanvas.h"
#include "Model.h"

#include <QGLShader>
#include <QGLBuffer>


TrackballCanvas::TrackballCanvas()
{
  render_mode = 1;
}

TrackballCanvas::~TrackballCanvas()
{

}

bool TrackballCanvas::display()
{
  drawModel();
  return true;
}

void TrackballCanvas::setGLProperty()
{
  updateModelBuffer();
  setShaderProgram();
}

Bound* TrackballCanvas::getBoundBox()
{
  return model->getBoundBox();
}

void TrackballCanvas::setModel(std::shared_ptr<Model> shared_model)
{
  model = shared_model;
}

void TrackballCanvas::setShaderProgram()
{
  basic_shader.reset(new QGLShaderProgram);
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/fragmentShader.frag");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/vertexShader.vert");
  basic_shader->link();
}

void TrackballCanvas::drawModel()
{
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

void TrackballCanvas::updateModelBuffer()
{
  const VertexList& vertex_list = model->getShapeVertexList();
  const FaceList&   face_list   = model->getShapeFaceList();
  const NormalList& normal_list = model->getShapeNormalList();
  const STLVectorf& color_list  = model->getShapeColorList();

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
    std::cout<<"TrackballCanvas: GL Error in getting model\n";
  }
}

std::string TrackballCanvas::getFilePath()
{
  return model->getDataPath();
}