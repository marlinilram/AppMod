#include "VectorFieldViewer.h"
#include "VectorFieldCanvas.h"
#include <qevent.h>
#include "BasicHeader.h"
#include "FeatureGuided.h"
//#include <cv.h>
//#include <eigen\dense>

VectorFieldViewer::VectorFieldViewer(QWidget* widget)
  : BasicViewer(widget)
{

}

VectorFieldViewer::~VectorFieldViewer()
{

}

void VectorFieldViewer::draw()
{
  BasicViewer::draw();
  drawLines();
}

void VectorFieldViewer::init()
{
  BasicViewer::init();

  //set camera
  camera()->setType(qglviewer::Camera::Type::ORTHOGRAPHIC);
  camera()->setPosition(qglviewer::Vec(0.5, 0.5, 1.0));

  //clearMouseBindings();
  setWheelBinding(Qt::NoModifier, CAMERA, MOVE_FORWARD);
}

//void VectorFieldViewer::postSelection(const QPoint& point)
//{
//   //std::cout << "The selected point is :" << point.x() <<" , " << point.y() << std::endl;
//   qreal src[3],res[3];
//   src[0] = point.x();
//   src[1] = point.y();
//   src[2] = 0.5;
//   camera()->getUnprojectedCoordinatesOf(src,res);
//
//   std::cout << "The UnprojectedCoordinates of the selected point is:" << res[0] << "," << res[1] << "," << res[2] << std::endl;
//   /*GLdouble mvpm[16],mvm[16];
//   camera()->getModelViewProjectionMatrix(mvpm);
//   camera()->getProjectionMatrix(mvm);
//   std::cout << "The ModelViewMatrix is : " << std::endl 
//     << mvm[0] << "," << mvm[1] << "," << mvm[2] << "," << mvm[3] << std::endl
//     << mvm[4] << "," << mvm[5] << "," << mvm[6] << "," << mvm[7] << std::endl
//     << mvm[8] << "," << mvm[9] << "," << mvm[10] << "," << mvm[11] << std::endl
//     << mvm[12] << "," << mvm[13] << "," << mvm[14] << "," << mvm[15] << std::endl;
//   Eigen::MatrixXd mvpm_mat(4,4);
//   mvpm_mat << mvpm[0],mvpm[1],mvpm[2],mvpm[3],
//     mvpm[4],mvpm[5],mvpm[6],mvpm[7],
//     mvpm[8],mvpm[9],mvpm[10],mvpm[11],
//     mvpm[12],mvpm[13],mvpm[14],mvpm[15];
//   Eigen::MatrixXd topLeftPoint(4,1),topLeftPoint_projected(4,1);
//   topLeftPoint << 0,1,0,1;
//   topLeftPoint_projected = mvpm_mat * topLeftPoint;
//   double topLeft_x = topLeftPoint_projected(0,0);
//   double topLeft_y = topLeftPoint_projected(1,0);
//   std::cout << "The projected top left point is :" << topLeft_x << "," << topLeft_y << "," << topLeftPoint_projected(2,0) << "," << topLeftPoint_projected(3,0) << std::endl;
//   std::cout << "The camera position is :" << camera()->position().x << "," << camera()->position().y << "," << camera()->position().z << std::endl;*/
//   
//}

void VectorFieldViewer::updateSourceVectorField()
{
  for (size_t i = 0; i < dispObjects.size(); ++i)
  {
    VectorFieldCanvas* canvas = dynamic_cast<VectorFieldCanvas*>(dispObjects[i]);
    if (canvas)
    {
      canvas->updateSourceVectorField();
    }
  }

  updateGLOutside();
}


void VectorFieldViewer::mousePressEvent(QMouseEvent *e)
{
  drawLine = true;
  Vector2f point;
  qreal src[3],res[3];
  src[0] = e->x();
  src[1] = e->y();
  src[2] = 0.5;
  camera()->getUnprojectedCoordinatesOf(src,res);
  point[0] = res[0];
  point[1] = res[1];
  line.push_back(point);
  //std::cout << "The UnprojectedCoordinates of the selected point is :" << point[0] << "," << point[1] << std::endl;
}

void VectorFieldViewer::mouseMoveEvent(QMouseEvent *e)
{
  if(drawLine)
  {
    if(!line.empty())
    {
      Vector2f previousPoint = line[line.size() - 1];
      Vector2f currentPoint;
      qreal src[3],res[3];
      src[0] = e->x();
      src[1] = e->y();
      src[2] = 0.5;
      camera()->getUnprojectedCoordinatesOf(src,res);
      currentPoint[0] = res[0];
      currentPoint[1] = res[1];
      if(sqrt(pow(currentPoint[0] - previousPoint[0],2) + pow(currentPoint[1] - previousPoint[1],2)) > 0.01)
        line.push_back(currentPoint);
      updateGLOutside();
    }
  }
}

void VectorFieldViewer::mouseReleaseEvent(QMouseEvent *e)
{
  std::cout << "The points of the drawline are :" << std::endl;
  for(int i = 0;i < line.size();i ++)
    std::cout << line[i][0] << "," << line[i][1] << std::endl;
  drawLine = false;
  lines.push_back(line);
  while(!line.empty())
    line.pop_back();
}

void VectorFieldViewer::drawLines()
{
  if(line.size() > 1)
  {
    glBegin(GL_LINE_STRIP);
    for(int i = 0;i  < line.size();i ++)
    {
      glColor3f(1,1,1);
      glVertex3f(line[i][0],line[i][1],0);
    }
    glEnd();
  } 
}

void VectorFieldViewer::setConstrainedPoints()
{
  VectorFieldCanvas* canvas = dynamic_cast<VectorFieldCanvas*>(dispObjects[0]);
  canvas->constrainedLines = std::shared_ptr<std::vector<std::vector<Vector2f>>>(&lines);
  canvas->setConstrainedPoints();
}

void VectorFieldViewer::deleteLastLine()
{
  VectorFieldCanvas* canvas = dynamic_cast<VectorFieldCanvas*>(dispObjects[0]);
  if(!lines.empty())
  {
    lines.pop_back();
    std::cout << "Delete the last constrained line from vector field." << std::endl;
  }
  else
  {
    std::cout << "There is no constrained line can be deleted!" << std::endl;
  }
  /*std::cout << "The number of constrained line of source_vector_field_lines is :" << ((canvas->feature_model->source_vector_field_lines).get())->size() << std::endl;
  std::cout << "The number of constrained line of target_vector_field_lines is :" << ((canvas->feature_model->target_vector_field_lines).get())->size() << std::endl;*/
}