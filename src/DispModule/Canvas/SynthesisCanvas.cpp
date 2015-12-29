#include "SynthesisCanvas.h"
#include "Model.h"

#include "PolygonMesh.h"
#include "ParameterMgr.h"

#include <QGLShader>
#include <QGLBuffer>

SynthesisCanvas::SynthesisCanvas()
{
  render_mode = 5;
}

SynthesisCanvas::~SynthesisCanvas()
{

}

bool SynthesisCanvas::display()
{
  drawModel();
  return true;
}

void SynthesisCanvas::setGLProperty()
{
  updateModelBuffer();
  setShaderProgram();
}

Bound* SynthesisCanvas::getBoundBox()
{
  return model->getBoundBox();
}

void SynthesisCanvas::setModel(std::shared_ptr<Model> shared_model)
{
  model = shared_model;
}

void SynthesisCanvas::setShaderProgram()
{
  basic_shader.reset(new QGLShaderProgram);
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/edge.frag");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/edge.vert");
  basic_shader->link();
}

void SynthesisCanvas::drawModel()
{
  basic_shader->bind();

  basic_shader->setUniformValue("fMeshSize", GLfloat(num_face));
  basic_shader->setUniformValue("L", QVector3D(-0.4082, -0.4082, 0.8165));
  basic_shader->setUniformValue("renderMode", GLint(render_mode));
  //basic_shader->setUniformValue("use_flat", GLint(use_flat));

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
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  face_buffer->bind();
  glDrawElements(GL_TRIANGLES, num_face * 3, GL_UNSIGNED_INT, (void*)0);
  face_buffer->release();

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  basic_shader->disableAttributeArray("vertex");
  basic_shader->disableAttributeArray("color");
  basic_shader->release();
}

void SynthesisCanvas::updateModelBuffer()
{
  LG::PolygonMesh* poly_mesh = model->getPolygonMesh();
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
    std::cout<<"SynthesisCanvas: GL Error in getting model\n";
  }
}

void SynthesisCanvas::updateModelColorBuffer()
{
  LG::PolygonMesh* poly_mesh = model->getPolygonMesh();
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
    std::cout<<"SynthesisCanvas: GL Error in getting model. Error Code: " << error_code << "\n";
  }
}

std::string SynthesisCanvas::getFilePath()
{
  return model->getDataPath();
}

void SynthesisCanvas::setTextureImage(QImage& glImg, GLuint& texture)
{
  // Bind the img texture...
  // Enable GL textures  
  glBindTexture(GL_TEXTURE_2D, 0);
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);

  if(glIsTexture(texture)) std::cout<<"gen texture ok..\n";
  else std::cout << "gen texture failed...\n";

  // Nice texture coordinate interpolation
  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D(GL_TEXTURE_2D, 0, 4, glImg.width(), glImg.height(), 0,
    GL_RGBA, GL_UNSIGNED_BYTE, glImg.bits());

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT );

  if (glGetError() != 0)
  {
    std::cout<<"GL Error in setting background image\n";
  }
}

void SynthesisCanvas::setSynthesisReflectance()
{
  cv::Mat temp = model->getSynRImg() * 255;
  cv::Mat ref_img;
  temp.convertTo(ref_img, CV_8UC3);
  cv::cvtColor(ref_img, ref_img, CV_BGR2RGB);
  QImage syn_ref((const uchar *) ref_img.data, ref_img.cols, ref_img.rows, ref_img.step, QImage::Format_RGB888);
  QImage gl_syn_ref = QGLWidget::convertToGLFormat(syn_ref);
  setTextureImage(gl_syn_ref, synthesis_reflect_texture);
  syn_ref.save(QString::fromStdString(model->getDataPath() + "/syn_ref.png"));
}