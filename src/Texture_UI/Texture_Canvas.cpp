#include "Texture_Canvas.h"
#include "Model.h"

#include "PolygonMesh.h"
#include "ParameterMgr.h"
#include "Shape.h"
#include <QGLShader>
#include <QGLBuffer>


Texture_Canvas::Texture_Canvas()
{
  render_mode = 3;
  use_flat = 1;
  has_data_ = false;
}

Texture_Canvas::~Texture_Canvas()
{

}

bool Texture_Canvas::display()
{
  drawModel();
  return true;
}
QGLViewer* Texture_Canvas::viewer()
{
	return DispObject::viewer();
};
void Texture_Canvas::set_viewer(QGLViewer* gl)
{
	DispObject::set_viewer(gl);

	std::shared_ptr<Model> mm = this->getModel();
	if (mm == NULL)
	{
		return;
	}

	std::vector<Shape*> shapes;
	mm->getShapeVector(shapes);
	for (unsigned int i = 0; i < shapes.size(); i++)
	{
		shapes[i]->set_glviewer(gl);
		shapes[i]->set_show_mani(true);
		shapes[i]->draw_manipulator();
	}
};
void Texture_Canvas::setGLProperty()
{
  updateModelBuffer();
  setShaderProgram();
}

Bound* Texture_Canvas::getBoundBox()
{
  return this->getModel()->getBoundBox();
}

void Texture_Canvas::setModel(std::shared_ptr<Model> shared_model)
{
	if (shared_model == NULL)
	{
		return;
	}
	DispObject::setModel(shared_model);
	shared_model->set_dis_obj(this);
}

void Texture_Canvas::setShaderProgram()
{
  basic_shader.reset(new QGLShaderProgram);
  basic_shader->addShaderFromSourceFile(QGLShader::Fragment, "shader/edge.frag");
  basic_shader->addShaderFromSourceFile(QGLShader::Vertex,   "shader/edge.vert");
  basic_shader->link();
}

void Texture_Canvas::drawModel()
{
if (this->getModel() == NULL)
{
	return;
}
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

void Texture_Canvas::updateModelBuffer()
{
  //const VertexList& vertex_list = model->getShapeVertexList();
  //const FaceList&   face_list   = model->getShapeFaceList();
  //const NormalList& normal_list = model->getShapeNormalList();
  //const STLVectorf& color_list  = model->getShapeColorList();
//   if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("View:ShowLightball") == 0)
//   {
//     use_flat = 1;
//   }
//   else 
//   {
//     use_flat = 0;
//   }
  if (this->getModel() == NULL)
  {
	  return;
  }
  use_flat = 1;
  std::vector<LG::PolygonMesh*> poly_meshes;
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0)
  {
    this->getModel()->getPolygonMeshVector(poly_meshes);
  }
  else
  {
	  poly_meshes.push_back(this->getModel()->getLightPolygonMesh());
  }

  VertexList vertex_list;
  NormalList normal_list;
  STLVectorf color_list;
  FaceList face_list;
  int vert_accu = 0;
  for (size_t i = 0; i < poly_meshes.size(); i++)
  {
    LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_normals = poly_meshes[i]->vertex_attribute<LG::Vec3>("v:normal");
    LG::PolygonMesh::Vertex_attribute<LG::Vec3> v_colors = poly_meshes[i]->vertex_attribute<LG::Vec3>("v:colors");
    for (auto vit : poly_meshes[i]->vertices())
    {
      LG::Vec3 pt = poly_meshes[i]->position(vit);
      LG::Vec3 n = v_normals[vit];
      LG::Vec3 c = v_colors[vit];
      for (int i = 0; i < 3; ++i)
      {
        vertex_list.push_back(pt[i]);
        normal_list.push_back(n[i]);
        color_list.push_back(c[i]);
      }
    }
    for (auto fit : poly_meshes[i]->faces())
    {
      for (auto vfc : poly_meshes[i]->vertices(fit))
      {
        face_list.push_back(size_t(vfc.idx() + vert_accu));
      }
    }
    vert_accu += poly_meshes[i]->n_vertices();
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

void Texture_Canvas::updateModelColorBuffer()
{
  LG::PolygonMesh* poly_mesh;
  if (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0)
  {
	  poly_mesh = this->getModel()->getPolygonMesh();
  }
  else
  {
	  poly_mesh = this->getModel()->getLightPolygonMesh();
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


std::string Texture_Canvas::getFilePath()
{
	return this->getModel()->getDataPath();
}