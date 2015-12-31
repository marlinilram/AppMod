#include "TrackballCanvas.h"
#include "Model.h"

#include "PolygonMesh.h"
#include "ParameterMgr.h"

#include <QGLShader>
#include <QGLBuffer>


TrackballCanvas::TrackballCanvas()
{
  render_mode = 3;
  use_flat = 1;
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
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/edge.frag");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/edge.vert");
  basic_shader->link();
}

void TrackballCanvas::drawModel()
{
  basic_shader->bind();

  basic_shader->setUniformValue("fMeshSize", GLfloat(num_face));
  basic_shader->setUniformValue("L", QVector3D(-0.4082, -0.4082, 0.8165));
  basic_shader->setUniformValue("renderMode", GLint(render_mode));
  basic_shader->setUniformValue("use_flat", GLint(use_flat));

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

  // test to draw only visible line
  // use fill to populate z-buffer
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  //face_buffer->bind();
  //glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  //face_buffer->release();

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  basic_shader->disableAttributeArray("vertex");
  basic_shader->disableAttributeArray("color");
  basic_shader->release();
}

void TrackballCanvas::updateModelBuffer()
{
  //const VertexList& vertex_list = model->getShapeVertexList();
  //const FaceList&   face_list   = model->getShapeFaceList();
  //const NormalList& normal_list = model->getShapeNormalList();
  //const STLVectorf& color_list  = model->getShapeColorList();
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0)
  {
    use_flat = 1;
  }
  else 
  {
    use_flat = 0;
  }

  LG::PolygonMesh* poly_mesh;
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0)
  {
    poly_mesh = model->getPolygonMesh();
  }
  else
  {
    poly_mesh = model->getLightPolygonMesh();
  }
  LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_normals = poly_mesh->vertex_attribute<LG::Vec3>("v:normal");
  LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_colors = poly_mesh->vertex_attribute<LG::Vec3>("v:colors");
  VertexList vertex_list;
  NormalList normal_list;
  STLVectorf color_list;
  FaceList face_list;
  for (auto vit : poly_mesh->vertices())
  {
    LG::Vec3 pt = poly_mesh->position(vit);
    LG::Vec3 n = v_normals[vit];
    LG::Vec3 c = v_colors[vit];
    for (int i = 0; i < 3; ++i)
    {
      vertex_list.push_back(pt[i]);
      normal_list.push_back(n[i]);
      color_list.push_back(c[i]);
    }
  }
  for (auto fit : poly_mesh->faces())
  {
    for (auto vfc : poly_mesh->vertices(fit))
    {
      face_list.push_back(size_t(vfc.idx()));
    }
  }

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

void TrackballCanvas::updateModelColorBuffer()
{
  LG::PolygonMesh* poly_mesh;
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0)
  {
    poly_mesh = model->getPolygonMesh();
  }
  else
  {
    poly_mesh = model->getLightPolygonMesh();
  }
  LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_colors = poly_mesh->vertex_attribute<LG::Vec3>("v:colors");
  STLVectorf color_list;
  for (auto vit : poly_mesh->vertices())
  {
    LG::Vec3 c = v_colors[vit];
    for (int i = 0; i < 3; ++i)
    {
      color_list.push_back(c[i]);
    }
  }

  color_buffer->bind();
  color_buffer->allocate(num_vertex * 3 * sizeof(GLfloat));
  color_buffer->write(0, &color_list[0], num_vertex * 3 * sizeof(GLfloat));
  color_buffer->release();

  GLenum error_code = glGetError();
  if (error_code != 0)
  {
    std::cout<<"TrackballCanvas: GL Error in getting model. Error Code: " << error_code << "\n";
  }
}


std::string TrackballCanvas::getFilePath()
{
  return model->getDataPath();
}