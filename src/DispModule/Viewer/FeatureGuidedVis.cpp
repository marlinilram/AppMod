#include "FeatureGuidedVis.h"
#include "FeatureGuided.h"

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

FeatureGuidedVis::FeatureGuidedVis()
{
  alpha = 1.0;
  alpha_bridging = 0.3;
  display_step = 5;
}

FeatureGuidedVis::~FeatureGuidedVis()
{

}

void FeatureGuidedVis::init(FeatureGuided* init_data_ptr)
{
  this->data_ptr = init_data_ptr;
}

bool FeatureGuidedVis::display()
{
  bool display_correct = true;
  display_correct = display_correct && this->displayVectorField();
  display_correct = display_correct && this->displayTargetCurves();
  //display_correct = display_correct && this->displaySourceCurves();
  return display_correct;
}

bool FeatureGuidedVis::displayVectorField()
{
  tele2d* teleRegister = data_ptr->GetTeleRegister();
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

        glColor3f( 1.0, 1, 1 ) ;

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


  // draw initial curves 
  //glLineWidth(5) ;


  //if( 1 ){

  //  CURVES curves = teleRegister->curves ;
  //  glColor4f( 1.0f, 0.0f, 1.0f, alpha ) ;
  //  for( int i=0; i<curves.size(); ++i ){
  //    glBegin(GL_LINE_STRIP);
  //    for( int j=0; j<curves[i].size(); ++j )
  //      glVertex3f(curves[i][j].x, curves[i][j].y, 0 );
  //    glEnd();
  //  }

  //  if( 1 ){
  //    // draw orientation 
  //    glColor4f( 1.0f, 1.0f, 0.0f, alpha ) ;
  //    for( int i=0; i<curves.size(); ++i ){
  //      if( curves[i].size()<3 )
  //        continue ;
  //      double2 p1 = curves[i][ curves[i].size() /2 - 1] ;
  //      double2 p2 = curves[i][ curves[i].size() /2 ] ;

  //      double2 dir = (p2-p1).normalize() * 0.04 ;

  //      double2 base = (p2+p1)/2 - dir/2 ;
  //      double2 ver  = (p2+p1)/2 + dir/2 ;

  //      double2 bv1, bv2 ;

  //      bv1.x = base.x + dir.y * 0.3 ;
  //      bv1.y = base.y - dir.x * 0.3 ;

  //      bv2.x = base.x - dir.y * 0.3 ;
  //      bv2.y = base.y + dir.x * 0.3 ;
  //      glBegin(GL_TRIANGLES);
  //      glVertex3f(ver.x, ver.y, 0 ) ;
  //      glVertex3f(bv2.x, bv2.y, 0 ) ;
  //      glVertex3f(bv1.x, bv1.y, 0 ) ;
  //      glEnd();
  //    }
  //  }

  //}


  // draw result curves 
  //glLineWidth(5) ;
  //if( 1 ){

  //  CURVES curves = teleRegister->resCurves ;
  //  glColor4f( 0.0f, 1.0f, 0.0f, alpha ) ;
  //  for( int i=0; i<curves.size(); ++i ){
  //    glBegin(GL_LINE_STRIP);
  //    for( int j=0; j<curves[i].size(); ++j )
  //      glVertex3f(curves[i][j].x, curves[i][j].y, 0 );
  //    glEnd();
  //  }

  //  if( 1 ){
  //    // draw orientation 
  //    glColor4f( 1.0f, 1.0f, 0.0f, alpha ) ;
  //    for( int i=0; i<curves.size(); ++i ){
  //      if( curves[i].size()<3 )
  //        continue ;
  //      double2 p1 = curves[i][ curves[i].size() /2 - 1] ;
  //      double2 p2 = curves[i][ curves[i].size() /2 ] ;

  //      double2 dir = (p2-p1).normalize() * 0.04 ;

  //      double2 base = (p2+p1)/2 - dir/2 ;
  //      double2 ver  = (p2+p1)/2 + dir/2 ;

  //      double2 bv1, bv2 ;

  //      bv1.x = base.x + dir.y * 0.3 ;
  //      bv1.y = base.y - dir.x * 0.3 ;

  //      bv2.x = base.x - dir.y * 0.3 ;
  //      bv2.y = base.y + dir.x * 0.3 ;
  //      glBegin(GL_TRIANGLES);
  //      glVertex3f(ver.x, ver.y, 0 ) ;
  //      glVertex3f(bv2.x, bv2.y, 0 ) ;
  //      glVertex3f(bv1.x, bv1.y, 0 ) ;
  //      glEnd();
  //    }
  //  }

  //}

  // draw interpolated curves
  //glLineWidth(4) ;
  //std::vector<std::vector<double2>>  bridging_curves = teleRegister->bridging_curves ;
  //if( 1 ){
  //  glColor4f( 0.3f,0.3f, 0.3f,alpha_bridging ) ;
  //  for( int i=0; i<bridging_curves.size(); ++i){
  //    glBegin(GL_LINE_STRIP);
  //    for( int j=0; j<bridging_curves[i].size(); ++j )
  //      glVertex3f(bridging_curves[i][j].x, bridging_curves[i][j].y, 0 );
  //    glEnd();

  //  }
  //}


  // draw endpoints of initial curves
  //glPointSize(12) ;
  //glColor4f( 0.19,0.39,1.0, alpha) ;
  //if(1){

  //  CURVES curves = teleRegister->curves ;
  //  std::vector<int2> endpoints = teleRegister->endpoints ; 
  //  glBegin( GL_POINTS) ;
  //  for( int i=0; i<curves.size(); ++i ){
  //    if( endpoints.size() == 0)
  //      break;
  //    double2 center ;
  //    if( endpoints[i][0] ){
  //      center = curves[i][0] ;
  //      glVertex3f(center.x, center.y, 0.0) ;
  //    }
  //    if( endpoints[i][1] ){
  //      center = curves[i].back() ;
  //      glVertex3f(center.x, center.y, 0.0) ;
  //    }
  //  }
  //  glEnd() ;

  //}

  return true;
}

bool FeatureGuidedVis::displayTargetCurves()
{
  tele2d* teleRegister = this->data_ptr->GetTeleRegister();
  display_step = teleRegister->resolution / 40;

  CURVES target_curves;
  data_ptr->NormalizedTargetCurves(target_curves);
  glPointSize(2) ;
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

bool FeatureGuidedVis::displaySourceCurves()
{
  tele2d* teleRegister = this->data_ptr->GetTeleRegister();
  display_step = teleRegister->resolution / 40;

  CURVES source_curves;
  data_ptr->NormalizedSourceCurves(source_curves);
  glPointSize(2) ;
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

Bound* FeatureGuidedVis::getBoundBox()
{
  int resolution = this->data_ptr->GetTeleRegister()->resolution;

  this->bound.minX = 0;
  this->bound.maxX = resolution;
  this->bound.minY = 0;
  this->bound.maxY = resolution;
  this->bound.minZ = 0.0;
  this->bound.maxZ = 0.1;
  return &this->bound;
}