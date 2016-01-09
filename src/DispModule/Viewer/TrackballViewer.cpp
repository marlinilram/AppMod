#include "MainCanvasViewer.h"
#include "TrackballViewer.h"
#include "VectorFieldViewer.h"
#include "TrackballCanvas.h"
#include "Bound.h"
#include <QKeyEvent>
#include "Model.h"
#include "ShapeCrest.h"
#include "ParameterMgr.h"
#include <QGLViewer/manipulatedFrame.h>

TrackballViewer::TrackballViewer(QWidget *widget)
  : BasicViewer(widget), sync_camera(false)
{
  wireframe_ = false;
  is_draw_actors = true;
  show_trackball = false;
  play_lightball = false;
  is_draw_actors = false;
}

TrackballViewer::~TrackballViewer()
{

}

void TrackballViewer::setMainCanvasViewer(std::shared_ptr<MainCanvasViewer> viewer)
{
  main_canvas_viewer = viewer;
}

void TrackballViewer::setSourceVectorViewer(std::shared_ptr<VectorFieldViewer> viewer)
{
  source_vector_viewer = viewer;
}

void TrackballViewer::setTargetVectorViewer(std::shared_ptr<VectorFieldViewer> viewer)
{
  target_vector_viewer = viewer;
}

void TrackballViewer::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  for (int i = 0; i < dispObjects.size(); ++i)
  {
    glDisable(GL_LIGHTING);
    drawTrackBall();
    if (!dispObjects[i]->display())
    {
      std::cerr<<"Error when drawing object " << i << ".\n";
    }
  }


  if (is_draw_actors)
  {
    //glClear(GL_DEPTH_BUFFER_BIT);
    drawActors();
  }
  drawCornerAxis();
}

void TrackballViewer::init()
{
  std::cout << "Initialize Trackball Viewer.\n";

  BasicViewer::init();

  // Set shader
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  //glEnable(GL_CULL_FACE);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_FLAT);

  qglClearColor(QColor(Qt::white));
  setBackgroundColor(QColor(Qt::white));
  setForegroundColor(QColor(Qt::white));

  setSceneCenter(qglviewer::Vec(0, 0, 0));
  setSceneRadius(50);
  camera()->fitSphere(qglviewer::Vec(0, 0, 0), 5);
  camera()->setType(qglviewer::Camera::Type::PERSPECTIVE);
  camera()->setFlySpeed(0.5);
  setWheelandMouse();
}

void TrackballViewer::drawTrackBall()
{
  if (!show_trackball)
  {
    return;
  }

	//double radius = 1.5;
	double v1,v2;
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
    v1 = ((dispObjects[0]->getBoundBox())->centroid).x + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).y + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.8,0.6,0.6);
		glVertex3f(v1,v2,((dispObjects[0]->getBoundBox())->centroid).z);
	}
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
		v1 = ((dispObjects[0]->getBoundBox())->centroid).y + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).z + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.6,0.8,0.6);
		glVertex3f(((dispObjects[0]->getBoundBox())->centroid).x,v1,v2);
	}
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(double i = 0;i <= 2 * M_PI;i += (M_PI / 180))
	{
		v1 = ((dispObjects[0]->getBoundBox())->centroid).x + ((dispObjects[0]->getBoundBox())->getRadius()) * cos(i);
		v2 = ((dispObjects[0]->getBoundBox())->centroid).z + ((dispObjects[0]->getBoundBox())->getRadius()) * sin(i);
		glColor3f(0.6,0.6,0.8);
		glVertex3f(v1,((dispObjects[0]->getBoundBox())->centroid).y,v2);
	}
	glEnd();
}

void TrackballViewer::setWheelandMouse()
{
  /////////////////////////////////////////////////////
  //         Mouse bindings customization            //
  //     Changes standard action mouse bindings      //
  /////////////////////////////////////////////////////

  //
  setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);

  // Left and right buttons together make a camera zoom : emulates a mouse third button if needed.
  setMouseBinding(Qt::Key_Z, Qt::NoModifier, Qt::LeftButton, CAMERA, ZOOM);

  setMouseBinding(Qt::ControlModifier | Qt::ShiftModifier, Qt::RightButton, SELECT);
  setWheelBinding(Qt::NoModifier, CAMERA, MOVE_FORWARD);
  setMouseBinding(Qt::AltModifier, Qt::LeftButton, CAMERA, TRANSLATE);

  // Add custom mouse bindings description (see mousePressEvent())
  setMouseBindingDescription(Qt::NoModifier, Qt::RightButton, "Opens a camera path context menu");

  setManipulatedFrame(new qglviewer::ManipulatedFrame());
}

void TrackballViewer::toggleLightball()
{
  play_lightball = (LG::GlobalParameterMgr::GetInstance()->get_parameter<int>("TrackballView:ShowLightball") == 0 ? false : true);
  if (play_lightball)
  {
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, FRAME, ROTATE);
  }
  else
  {
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, CAMERA, ROTATE);
  }
}

void TrackballViewer::updateBuffer()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    TrackballCanvas* trackball_canvas = dynamic_cast<TrackballCanvas*>(dispObjects[i]);
    if (trackball_canvas)
    {
      trackball_canvas->updateModelBuffer();
    }
  }

  doneCurrent();
}

void TrackballViewer::updateColorBuffer()
{
  makeCurrent();

  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    TrackballCanvas* trackball_canvas = dynamic_cast<TrackballCanvas*>(dispObjects[i]);
    if (trackball_canvas)
    {
      trackball_canvas->updateModelColorBuffer();
    }
  }

  doneCurrent();
}

void TrackballViewer::resetCamera()
{
  // assume only one dispObjects MainCanvas exist here
  if (dispObjects.empty())
  {
    return;
  }

  Bound* scene_bounds = dynamic_cast<TrackballCanvas*>(dispObjects[0])->getBoundBox();

  qglviewer::Vec scene_center;
  scene_center = qglviewer::Vec((scene_bounds->minX + scene_bounds->maxX) / 2,
    (scene_bounds->minY + scene_bounds->maxY) / 2,
    (scene_bounds->minZ + scene_bounds->maxZ) / 2);

  setSceneCenter(scene_center);

  float x_span = (scene_bounds->maxX - scene_bounds->minX) / 2;
  float y_span = (scene_bounds->maxY - scene_bounds->minY) / 2;
  float z_span = (scene_bounds->maxZ - scene_bounds->minZ) / 2;
  float scene_radius = sqrt(x_span * x_span + y_span * y_span + z_span * z_span);
  //scene_radius *= 1.5;

  setSceneRadius(scene_radius);
  camera()->fitSphere(scene_center, scene_radius);



  setStateFileName(QString((dynamic_cast<TrackballCanvas*>(dispObjects[0])->getFilePath()+"/camera_info.xml").c_str()));
  if (restoreStateFromFile())
    std::cout << "Load camera info successes...\n";
  else
    std::cout << "Load camera info failed...\n";

  // set the scene in MainCanvasViewer
  syncCamera();
}

void TrackballViewer::drawCornerAxis()
{
  int viewport[4];
  int scissor[4];

  // The viewport and the scissor are changed to fit the lower left
  // corner. Original values are saved.
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetIntegerv(GL_SCISSOR_BOX, scissor);

  // Axis viewport size, in pixels
  const int size = 150;
  glViewport(0, 0, size, size);
  glScissor(0, 0, size, size);

  // The Z-buffer is cleared to make the axis appear over the
  // original image.
  glClear(GL_DEPTH_BUFFER_BIT);

  // Tune for best line rendering
  glDisable(GL_LIGHTING);
  glLineWidth(3.0);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(-1, 1, -1, 1, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(camera()->orientation().inverse().matrix());

  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(1.0, 0.0, 0.0);

  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 1.0, 0.0);

  glColor3f(0.0, 0.0, 1.0);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0.0, 1.0);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glEnable(GL_LIGHTING);

  // The viewport and the scissor are restored.
  glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void TrackballViewer::mousePressEvent(QMouseEvent* e)
{
  //if ((e->button() == Qt::RightButton) && (e->modifiers() == Qt::NoButton))
  //{
  //  QMenu menu(this);
  //  menu.addAction("Camera positions");
  //  menu.addSeparator();
  //  QMap<QAction*, int> menuMap;

  //  bool atLeastOne = false;
  //  // We only test the 20 first indexes. This is a limitation.
  //  for (unsigned short i = 0; i < 20; ++i)
  //    if (camera()->keyFrameInterpolator(i))
  //    {
  //      atLeastOne = true;
  //      QString text;
  //      if (camera()->keyFrameInterpolator(i)->numberOfKeyFrames() == 1)
  //        text = "Position " + QString::number(i);
  //      else
  //        text = "Path " + QString::number(i);

  //      menuMap[menu.addAction(text)] = i;
  //    }

  //    if (!atLeastOne)
  //    {
  //      menu.addAction("No position defined");
  //      menu.addAction("Use to Alt+Fx to define one");
  //    }

  //    QAction* action = menu.exec(e->globalPos());

  //    if (atLeastOne && action)
  //      camera()->playPath(menuMap[action]);
  //}
  //else
  //if (!lightball_mode)
  {
    QGLViewer::mousePressEvent(e);
    sync_camera = true;
  }
  //else if (lightball_mode)
  {

  }
}

void TrackballViewer::mouseMoveEvent(QMouseEvent *e)
{
  QGLViewer::mouseMoveEvent(e);
  if(sync_camera)
  {
    syncCamera(0); // mouse move event don't sync the vector field viewer to decrease lagecy
  }
}

void TrackballViewer::mouseReleaseEvent(QMouseEvent* e)
{
  QGLViewer::mouseReleaseEvent(e);
  if(sync_camera)
  {
    syncCamera();
  }
  sync_camera = false;
}

void TrackballViewer::wheelEvent(QWheelEvent* e)
{
  if (e->modifiers() ==  Qt::ShiftModifier)
  {
    qreal cur_fov = camera()->fieldOfView();
    qreal cur_fd = camera()->focusDistance();
    std::cout << "Current FOV: " << cur_fov << "\t";
    if (e->delta() > 0)
    {
      camera()->setFieldOfView(cur_fov + 0.0175);
    }
    else
    {
      camera()->setFieldOfView(cur_fov - 0.0175);
    }
    //std::cout << "angle delta: " << e->delta() << std::endl;
    // set FOV
    camera()->setFocusDistance(cur_fd);
    syncCamera(0);
    std::cout << camera()->position()[0] << " " << camera()->position()[1] << " "  << camera()->position()[2] << std::endl;
    updateGLOutside();
    GLdouble m[16];
    camera()->getProjectionMatrix(m);
    Eigen::Map<Eigen::Matrix4d>proj(m, 4, 4);
    std::cout<< proj << std::endl;
  }
  else
  {
    QGLViewer::wheelEvent(e);
    syncCamera();
  }
}

void TrackballViewer::syncCamera(int sync_type)
{
  if(main_canvas_viewer)
  {
    if (!play_lightball)
    {
      GLdouble m[16];
      camera()->getModelViewMatrix(m);
      main_canvas_viewer->camera()->setFromModelViewMatrix(m);
      main_canvas_viewer->setSceneCenter(sceneCenter());
      main_canvas_viewer->setSceneRadius(sceneRadius());
      main_canvas_viewer->camera()->setZClippingCoefficient(camera()->zClippingCoefficient());
      main_canvas_viewer->camera()->setFieldOfView(camera()->fieldOfView());
      main_canvas_viewer->updateGLOutside();
      main_canvas_viewer->syncCameraToModel();
    }
    else
    {
      //GLfloat m[16];
      //camera()->getModelViewMatrix(m);
      Eigen::Map<const Eigen::Matrix4d>temp(manipulatedFrame()->matrix(), 4, 4);
      LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("Lightball:cameraTransform") = temp.cast<float>();
      main_canvas_viewer->updateColorBuffer(); 
      main_canvas_viewer->updateGLOutside();
      updateColorBuffer();
      updateGLOutside();
    }

    //std::cout << "Trackball: znear " << camera()->zNear() << "\tzfar " << camera()->zFar() << "\tfocal " << camera()->focusDistance() << "\tradius " << camera()->sceneRadius() << "\n";
    //std::cout << "Trackball scene center: " << camera()->sceneCenter().x << " " << camera()->sceneCenter().y << " " << camera()->sceneCenter().z <<"\n";
    //std::cout << "Maincanvas: znear " << main_canvas_viewer->camera()->zNear() << "\tzfar " << main_canvas_viewer->camera()->zFar() << "\tfocal " << main_canvas_viewer->camera()->focusDistance() << "\tradius " << main_canvas_viewer->camera()->sceneRadius()  << "\n";
    //std::cout << "Maincanvas scene center: " << main_canvas_viewer->camera()->sceneCenter().x << " " << main_canvas_viewer->camera()->sceneCenter().y << " " << main_canvas_viewer->camera()->sceneCenter().z <<"\n";
  }

  if (sync_type == 1)
  {
    if (source_vector_viewer)
    {
      source_vector_viewer->updateSourceField();
      source_vector_viewer->updateScalarFieldTexture();
    }
    if (target_vector_viewer)
    {
      target_vector_viewer->updateScalarFieldTexture();
    }
  }
}

void TrackballViewer::keyPressEvent(QKeyEvent *e)
{
  // Get event modifiers key
  const Qt::KeyboardModifiers modifiers = e->modifiers();

  // A simple switch on e->key() is not sufficient if we want to take state key into account.
  // With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
  // That's why we use imbricated if...else and a "handled" boolean.
  bool handled = false;
  if ((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton))
  {
    wireframe_ = !wireframe_;
    if (wireframe_)
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    handled = true;
    updateGL();
  }
  else if ((e->key()==Qt::Key_R) && (modifiers==Qt::NoButton))
  {
    resetCamera();
    handled = true;
    updateGL();
  }

  if (!handled)
  {
    QGLViewer::keyPressEvent(e);
  }
}

void TrackballViewer::drawActors()
{
  for (decltype(actors.size()) i = 0; i < actors.size(); ++i)
  {
    actors[i].draw();
  }
}

void TrackballViewer::setGLActors(std::vector<GLActor>& actors)
{
  this->actors = actors;
  // don't show the line only show points
}

//void TrackballViewer::drawCrestLines()
//{
//  glClear(GL_DEPTH_BUFFER_BIT);
//  for (size_t i = 0; i < dispObjects.size(); ++i)
//  {
//    TrackballCanvas* trackball_canvas = dynamic_cast<TrackballCanvas*>(dispObjects[i]);
//    if (trackball_canvas)
//    {
//      std::vector<std::vector<Vector3f>> crestLinesPoints = trackball_canvas->getModel()->getShapeCrest()->getCrestLinesPoints();
//      for(size_t i = 0; i < 10; i ++)
//      {
//        glPointSize(2.0f);
//        glBegin(GL_POINTS);
//        for(size_t j = 0; j < crestLinesPoints[i].size(); j ++)
//        {
//          glColor3f(0,0,0);
//          glVertex3f(crestLinesPoints[i][j].x(),crestLinesPoints[i][j].y(),crestLinesPoints[i][j].z());
//        }
//        glEnd();
//      }
//    }
//  }
//}

void TrackballViewer::updateShapeCrest()
{
  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    TrackballCanvas* trackball_canvas = dynamic_cast<TrackballCanvas*>(dispObjects[i]);
    if (trackball_canvas)
    {
      trackball_canvas->getModel()->updateShapeCrest();
    }
  }
}

void TrackballViewer::updateCamera()
{
  GLdouble m[16];
  camera()->getModelViewMatrix(m);
  Eigen::Map<Eigen::Matrix4d>model_view(m, 4, 4);

  model_view = model_view * LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform").cast<double>();
  LG::GlobalParameterMgr::GetInstance()->get_parameter<Matrix4f>("LFeature:rigidTransform") = Matrix4f::Identity();
  camera()->setFromModelViewMatrix(model_view.data());
  syncCamera();
}