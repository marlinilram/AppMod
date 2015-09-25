#include "VectorFieldCanvas.h"
#include "FeatureGuided.h"
#include "FeatureLine.h"
#include "tele2d.h"
#include "Colormap.h"
#include "Bound.h"

#include "opencv2/contrib/contrib.hpp"

#define __E  2.718
#define __H  0.6

double3 dis2color( double dis ){

  dis = std::min( std::max(dis, 0.0 ), 1.0 ) ;
  double dis1 = 1.0+ (log10((dis*0.99+0.01))/2) ;

  dis1 = dis ;

  double x = pow( __E, -( dis1 - 0.0 ) * ( dis1 - 0.0 )/(0.4 * 0.4) ) * 255;
  double y = pow( __E, -( dis1 - 0.5 ) * ( dis1 - 0.5 )/(0.7 * 0.7) ) * 255;
  double z = pow( __E, -( dis1 - 1.0 ) * ( dis1 - 1.0 )/(1.0 * 1.0) ) * 255;

  x = std::min( std::max(x, 0.0 ), 255.0 ) ;
  y = std::min( std::max(y, 0.0 ), 255.0 ) ;
  z = std::min( std::max(z, 0.0 ), 255.0 ) ;

  return double3( x, y, z);
}

VectorFieldCanvas::VectorFieldCanvas()
{
  alpha = 1.0;
  alpha_bridging = 0.3;
  display_step = 5;

  u_max = 1.0;
  v_max = 1.0;
  ratio = 1.0;

  vis_paras.resize(7, true);
}

VectorFieldCanvas::~VectorFieldCanvas()
{

}

void VectorFieldCanvas::setFeatureModel(std::shared_ptr<FeatureGuided> model)
{
  feature_model = model;
}

bool VectorFieldCanvas::display()
{
  if (render_mode == VectorField::SOURCE_MODE)
  {
    if (vis_paras[0])
    {
      this->displayVectorField();
    }
    if (vis_paras[2])
    {
      this->displaySourceCurves();
    }
  }
  else if (render_mode == VectorField::TARGET_MODE)
  {
    if (vis_paras[1])
    {
      this->displayTargetVectorField();
    }

    if (vis_paras[3])
    {
      this->displayTargetCurves();
    }
  }

  //if (vis_paras[5])
  //{
  //  this->displayFittedCurves();
  //}
  //if (vis_paras[6])
  //{
  //  this->displayHistMatchPts();
  //}
  return true;
}

bool VectorFieldCanvas::displayVectorField()
{
  std::shared_ptr<tele2d> teleRegister = this->feature_model->GetTeleRegister();
  display_step = teleRegister->resolution / 20;

  // draw scalar field
  //if( teleRegister->osculatingCircles.size()){

  //  for( int i =0; i<800 ; ++i ){
  //    for( int j=0; j<800; ++j ){
  //      double h = 0.1 ;
  //      double3 field_color = dis2color( teleRegister->dis[i][j] ) ;
  //      bgmap[j][i][0] = field_color.x ;
  //      bgmap[j][i][1] = field_color.y ;
  //      bgmap[j][i][2] = field_color.z ;
  //    }
  //  }
  //  if( 1 ){
  //    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 ) ;
  //    glRasterPos2i( 0, 0 ) ;
  //    glDrawPixels( 800, 800, GL_RGB,  GL_UNSIGNED_BYTE, bgmap ) ;
  //  }	

  //}

  //glLineWidth(3) ;


  // draw vector field
  glPointSize(1) ;
  glLineWidth(2) ;
  std::vector<double2>  vector_field = teleRegister->vector_field ;
  int resolution = teleRegister->resolution ;
  if(  vector_field.size() == resolution*resolution ){


    for( int i=0; i<resolution;  i += display_step ){
      for( int j=0; j<resolution; j += display_step ){

        glColor3f( 0.5, 0.5, 0.5 ) ;

        double len = 4.0  * resolution / 100;

        if( j%2 == 0){

          float x = ((double)i+0.5)/(double)resolution ;
          float y = ((double)j+0.5)/(double)resolution ;


          double2 direction = vector_field[i+j*resolution] ;



          double2 p1 = double2( x-0.5*len*direction.x/resolution, y-0.5*len*direction.y/resolution) ;
          double2 p2 = double2( x+0.5*len*direction.x/resolution, y+0.5*len*direction.y/resolution) ;

          double2 dir = p2-p1 ;

          double2 p3 = p1 + dir * 0.5 + double2(-dir.y*0.15, dir.x*0.15) ;
          double2 p4 =  p1 + dir * 0.5 + double2(dir.y*0.15, -dir.x*0.15) ;

          double2 base = p1 + dir * 0.7 ;
          glBegin( GL_LINES );
          glVertex3f( p1.x,p1.y, 0 ) ;
          glVertex3f(base.x, base.y, 0 ) ;
          glEnd() ;

          glBegin(GL_TRIANGLES);
          glVertex3f( p2.x,p2.y, 0 ) ;
          glVertex3f( p3.x,p3.y, 0 ) ;
          glVertex3f( p4.x,p4.y, 0 ) ;
          glEnd();

        }else{

          if( i == resolution-1 )
            continue ;

          float x = ((double)i+1.0)/(double)resolution ;
          float y = ((double)j+0.5)/(double)resolution ;
          double2 direction = vector_field[i+j*resolution] ;
          double2 p1 = double2( x-0.5*len*direction.x/resolution, y-0.5*len*direction.y/resolution) ;
          double2 p2 = double2( x+0.5*len*direction.x/resolution, y+0.5*len*direction.y/resolution) ;

          double2 dir = p2-p1 ;

          double2 p3 = p1 + dir * 0.5 + double2(-dir.y*0.15, dir.x*0.15) ;
          double2 p4 =  p1 + dir * 0.5 + double2(dir.y*0.15, -dir.x*0.15) ;

          double2 base = p1 + dir * 0.7 ;
          glBegin( GL_LINES );
          glVertex3f( p1.x,p1.y, 0 ) ;
          glVertex3f(base.x, base.y, 0 ) ;
          glEnd() ;

          glBegin(GL_TRIANGLES);
          glVertex3f( p2.x,p2.y, 0 ) ;
          glVertex3f( p3.x,p3.y, 0 ) ;
          glVertex3f( p4.x,p4.y, 0 ) ;
          glEnd();

        }


      }
    }
  }

  return true;
}

bool VectorFieldCanvas::displayTargetVectorField()
{
  std::shared_ptr<tele2d> teleRegister = this->feature_model->GetTargetTeleRegister();
  display_step = teleRegister->resolution / 20;

  // draw vector field
  glPointSize(1) ;
  glLineWidth(2) ;
  std::vector<double2>  vector_field = teleRegister->vector_field ;
  int resolution = teleRegister->resolution ;
  if(  vector_field.size() == resolution*resolution ){


    for( int i=0; i<resolution;  i += display_step ){
      for( int j=0; j<resolution; j += display_step ){

        glColor3f( 0.5, 0.5, 0.5 ) ;

        double len = 4.0  * resolution / 100;

        if( j%2 == 0){

          float x = ((double)i+0.5)/(double)resolution ;
          float y = ((double)j+0.5)/(double)resolution ;


          double2 direction = vector_field[i+j*resolution] ;



          double2 p1 = double2( x-0.5*len*direction.x/resolution, y-0.5*len*direction.y/resolution) ;
          double2 p2 = double2( x+0.5*len*direction.x/resolution, y+0.5*len*direction.y/resolution) ;

          double2 dir = p2-p1 ;

          double2 p3 = p1 + dir * 0.5 + double2(-dir.y*0.15, dir.x*0.15) ;
          double2 p4 =  p1 + dir * 0.5 + double2(dir.y*0.15, -dir.x*0.15) ;

          double2 base = p1 + dir * 0.7 ;
          glBegin( GL_LINES );
          glVertex3f( p1.x,p1.y, 0 ) ;
          glVertex3f(base.x, base.y, 0 ) ;
          glEnd() ;

          if (i == 0 && j == 90)
          {
            std::cout << "x: " << p1.x << " y: " << p1.y << "\n";
          }

          glBegin(GL_TRIANGLES);
          glVertex3f( p2.x,p2.y, 0 ) ;
          glVertex3f( p3.x,p3.y, 0 ) ;
          glVertex3f( p4.x,p4.y, 0 ) ;
          glEnd();

        }else{

          if( i == resolution-1 )
            continue ;

          float x = ((double)i+1.0)/(double)resolution ;
          float y = ((double)j+0.5)/(double)resolution ;
          double2 direction = vector_field[i+j*resolution] ;
          double2 p1 = double2( x-0.5*len*direction.x/resolution, y-0.5*len*direction.y/resolution) ;
          double2 p2 = double2( x+0.5*len*direction.x/resolution, y+0.5*len*direction.y/resolution) ;

          double2 dir = p2-p1 ;

          double2 p3 = p1 + dir * 0.5 + double2(-dir.y*0.15, dir.x*0.15) ;
          double2 p4 =  p1 + dir * 0.5 + double2(dir.y*0.15, -dir.x*0.15) ;

          double2 base = p1 + dir * 0.7 ;
          glBegin( GL_LINES );
          glVertex3f( p1.x,p1.y, 0 ) ;
          glVertex3f(base.x, base.y, 0 ) ;
          glEnd() ;

          glBegin(GL_TRIANGLES);
          glVertex3f( p2.x,p2.y, 0 ) ;
          glVertex3f( p3.x,p3.y, 0 ) ;
          glVertex3f( p4.x,p4.y, 0 ) ;
          glEnd();

        }


      }
    }
  }
  return true;
}

bool VectorFieldCanvas::displayTargetCurves()
{
  std::shared_ptr<tele2d> teleRegister = this->feature_model->GetTeleRegister();
  display_step = teleRegister->resolution / 40;

  CURVES target_curves;
  feature_model->NormalizedTargetCurves(target_curves);
  glPointSize(2) ;
  glLineWidth(1);
  glColor4f( 148.0/225.0, 178.0/225.0, 53.0/225.0, alpha ) ;
  for (int i = 0; i < target_curves.size(); ++i)
  {
    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < target_curves[i].size(); ++j)
    {
      double2 pos = target_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
      glEnd();
  }
  return true;
}

bool VectorFieldCanvas::displaySourceCurves()
{
  std::shared_ptr<tele2d> teleRegister = this->feature_model->GetTeleRegister();
  display_step = teleRegister->resolution / 40;

  CURVES source_curves;
  this->feature_model->NormalizedSourceCurves(source_curves);
  glPointSize(2) ;
  glLineWidth(1);
  glColor4f( 1.0f, 0.0f, 1.0f, alpha ) ;
  for (int i = 0; i < source_curves.size(); ++i)
  {
    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < source_curves[i].size(); ++j)
    {
      double2 pos = source_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
    glEnd();
  }
  return true;
}

bool VectorFieldCanvas::displayFittedCurves()
{
  CURVES fitted_curves;
  this->feature_model->GetFittedCurves(fitted_curves);
  glPointSize(2) ;
  glLineWidth(5);
  //glClear(GL_DEPTH_BUFFER_BIT);
  glColor4f( 0.0f, 1.0f, 0.75f, alpha ) ;
  for (int i = 0; i < fitted_curves.size(); ++i)
  {
    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < fitted_curves[i].size(); ++j)
    {
      double2 pos = fitted_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
    glEnd();
  }
  return true;
}

bool VectorFieldCanvas::displayHistMatchPts()
{
  CURVES hist_curves;
  this->feature_model->FindHistMatchCrsp(hist_curves);
  glPointSize(2) ;
  glLineWidth(5);
  //glClear(GL_DEPTH_BUFFER_BIT);
  glColor4f( 0.75f, 1.0f, 0.0f, alpha ) ;
  for (int i = 0; i < hist_curves.size(); ++i)
  {
    glBegin(GL_LINE_STRIP);
    for (int j = 0; j < hist_curves[i].size(); ++j)
    {
      double2 pos = hist_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
    glEnd();
  }
  return true;
}

Bound* VectorFieldCanvas::getBoundBox()
{
  int resolution = this->feature_model->GetTeleRegister()->resolution;

  this->bound->minX = 0;
  this->bound->maxX = resolution;
  this->bound->minY = 0;
  this->bound->maxY = resolution;
  this->bound->minZ = 0.0;
  this->bound->maxZ = 0.1;
  return this->bound.get();
}

bool VectorFieldCanvas::displayScalarField()
{

  glEnable(GL_TEXTURE_2D);
  glColor3f(1,1,1);

  glNormal3f(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 1.0 - v_max);   glVertex2i(0, 0);
  glTexCoord2f(0.0, 1.0);           glVertex2i(0, 1);
  glTexCoord2f(u_max, 1.0);         glVertex2i(1, 1);
  glTexCoord2f(u_max, 1.0 - v_max); glVertex2i(1, 0);
  glEnd();

  glClear(GL_DEPTH_BUFFER_BIT);
  glDisable(GL_TEXTURE_2D);

  return true;
}

void VectorFieldCanvas::setScalarField()
{
  GLubyte* bgmap = new GLubyte[800*800*3];
  float* dmap = new float[800 * 800];

  std::vector<float> query(2, 0.0);
  kdtree::KDTreeResultVector result;
  std::shared_ptr<kdtree::KDTree> edge_KDTree = feature_model->getSourceKDTree();
  float max_dist = std::numeric_limits<float>::min();
  double2 normalized_translate(0.0, 0.0);
  double normalized_scale = 0.0;
  this->feature_model->GetSourceNormalizePara(normalized_translate, normalized_scale);

  for (int i = 0; i < 800; ++i)
  {
    for (int j = 0; j < 800; ++j)
    {
      query[0] = ((j / 800.0) - 0.5) / normalized_scale + 0.5 - normalized_translate.x;
      query[1] = ((i / 800.0) - 0.5) / normalized_scale + 0.5 - normalized_translate.y;
      edge_KDTree->n_nearest(query, 1, result);
      dmap[j + i * 800] = result[0].dis;
      if (result[0].dis > max_dist)
      {
        max_dist = result[0].dis;
      }
    }
  }

  //cv::Mat dimg = cv::Mat(800, 800, CV_32FC1, dmap);
  //cv::Mat mappedDimg;
  //dimg = 100 * dimg;
  //dimg.convertTo(mappedDimg, CV_8UC1);
  ////cv::applyColorMap(dimg*100, mappedDimg, cv::COLORMAP_JET);
  //cv::imshow("dimg", mappedDimg);
  QVector<QColor> color_map = makeColorMap();

  for (int i = 0; i < 800; ++i)
  {
    for (int j = 0; j < 800; ++j)
    {
      QColor color = 
        qtJetColor(dmap[j + i * 800] / max_dist, 0, 0.01);
      bgmap[(j + i * 800) * 3 + 0] = color.red();
      bgmap[(j + i * 800) * 3 + 1] = color.green();
      bgmap[(j + i * 800) * 3 + 2] = color.blue();
    }
  }



  int newWidth = 1 << (int)(1 + log(800 - 1 + 1E-3) / log(2.0));
  int newHeight = 1 << (int)(1 + log(800 - 1 + 1E-3) / log(2.0));

  u_max = 1.0;//800 / (float)newWidth;
  v_max = 1.0;//800 / (float)newHeight;
  ratio = newWidth / (float)newHeight;

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D( GL_TEXTURE_2D, 0, 3, 800, 800, 0, GL_RGB, GL_UNSIGNED_BYTE, bgmap);

#define GL_DEBUG
#ifdef GL_DEBUG
  if (glGetError() != 0)
  {
    std::cout<<"GL Error when set texture image.\n";
  }
#endif

  delete[] dmap;
  delete[] bgmap;
}

void VectorFieldCanvas::setGLProperty()
{
  //this->setScalarField();
}

void VectorFieldCanvas::setVisualizationParas(std::vector<bool>& paras)
{
  this->vis_paras = paras;
}

void VectorFieldCanvas::updateSourceVectorField()
{
  feature_model->updateSourceVectorField();
}

void VectorFieldCanvas::addConstrainedLines(std::vector<double2>& line)
{
  if (!feature_model)
  {
    return;
  }

  if(render_mode == VectorField::SOURCE_MODE)
  {
    feature_model->source_vector_field_lines->lines.push_back(line);
    std::cout << "Current number of constrained lines in source: "
      << feature_model->source_vector_field_lines->lines.size() << std::endl;
  }
  else
  {
    feature_model->target_vector_field_lines->lines.push_back(line);
    std::cout << "Current number of constrained lines in target: "
      << feature_model->target_vector_field_lines->lines.size() << std::endl;
  }
}

void VectorFieldCanvas::deleteLastLine()
{
  if (!feature_model)
  {
    return;
  }

  if(render_mode == VectorField::SOURCE_MODE)
  {
    feature_model->source_vector_field_lines->lines.pop_back();
    std::cout << "Delete the last constrained line of source. Current number: "
              << feature_model->source_vector_field_lines->lines.size() << std::endl;
  }
  else
  {
    feature_model->target_vector_field_lines->lines.pop_back();
    std::cout << "Delete the last constrained line of target. Current number: "
              << feature_model->target_vector_field_lines->lines.size() << std::endl;
  }
}

std::shared_ptr<FeatureLine> VectorFieldCanvas::getFeatureLine()
{
  if (!feature_model)
  {
    return nullptr;
  }

  if(render_mode == VectorField::SOURCE_MODE)
  {
    return feature_model->source_vector_field_lines;
  }
  else
  {
    return feature_model->target_vector_field_lines;
  }
}