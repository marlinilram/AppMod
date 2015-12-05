#include "VectorFieldCanvas.h"
#include "FeatureGuided.h"
#include "FeatureLine.h"
#include "ScalarField.h"
#include "tele2d.h"
#include "Colormap.h"
#include "Bound.h"
#include "KDTreeWrapper.h"

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

  crsp_oder = 1;
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
      if (vis_paras[9])
      {
        this->displayTargetCurves();
      }
    }
    if (vis_paras[4])
    {
      this->displayScalarField();
    }
    if (vis_paras[8])
    {
      this->displaySourceCrspList();
    }
    if (vis_paras[10])
    {
      this->displayUserCrsp();
    }
    if (vis_paras[11])
    {
      this->displayAllCurvesPoints();
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
      //this->displayTargetCurves();
      if (vis_paras[9])
      {
        this->displaySourceCurves();
      }
    }
    if (vis_paras[5])
    {
      this->displayScalarField();
    }
    if (vis_paras[8])
    {
      this->displayTargetCrspList();
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
  if (render_mode == VectorField::SOURCE_MODE)
  {
    glColor4f( 148.0/225.0, 178.0/225.0, 53.0/225.0, alpha ) ;
  }
  else if(render_mode == VectorField::TARGET_MODE)
  {
    glColor4f( 1.0f, 0.0f, 1.0f, alpha ) ;    
  }
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
  glLineWidth(2);
  if (render_mode == VectorField::SOURCE_MODE)      
  {
    glColor4f( 1.0f, 1.0f, 1.0f, alpha ) ;
  }
  else if(render_mode == VectorField::TARGET_MODE)
  {
    glColor4f( 148.0/225.0, 178.0/225.0, 53.0/225.0, alpha ) ;
  }

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
  std::shared_ptr<ScalarField> scalar_field;
  std::vector<float>* dmap = nullptr;
  double vmin = 0;
  double vmax = 1;
  if (render_mode == VectorField::SOURCE_MODE)
  {
    scalar_field = feature_model->getSourceScalarField();
    dmap = &scalar_field->variation_map;
  }
  else if (render_mode == VectorField::TARGET_MODE)
  {
    scalar_field = feature_model->getTargetScalarField();
    dmap = &scalar_field->distance_map;
    double wcenter = scalar_field->win_center;
    double wwidth = scalar_field->win_width;
    wwidth = wwidth * (1 - 0) * std::min(wcenter, 1 - wcenter);
    wcenter = wcenter * (1 - 0) + 0;
    vmin = wcenter - wwidth;
    vmax = wcenter + wwidth;
    std::cout << "vmin: " << vmin << "\tvmax: " << vmax << "\n";
  }

  int resolution = scalar_field->resolution;
  std::vector<GLubyte> bgmap(resolution * resolution * 3, 0);


  //std::vector<float> query(2, 0.0);
  //kdtree::KDTreeResultVector result;
  //std::shared_ptr<KDTreeWrapper> edge_KDTree = feature_model->getSourceKDTree();
  //float max_dist = std::numeric_limits<float>::min();
  //double2 normalized_translate(0.0, 0.0);
  //double normalized_scale = 0.0;
  //this->feature_model->GetSourceNormalizePara(normalized_translate, normalized_scale);

  //for (int i = 0; i < 800; ++i)
  //{
  //  for (int j = 0; j < 800; ++j)
  //  {
  //    query[0] = ((j / 800.0) - 0.5) / normalized_scale + 0.5 - normalized_translate.x;
  //    query[1] = ((i / 800.0) - 0.5) / normalized_scale + 0.5 - normalized_translate.y;
  //    dmap[j + i * 800] = edge_KDTree->nearestDis(query);
  //    if (result[0].dis > max_dist)
  //    {
  //      max_dist = result[0].dis;
  //    }
  //  }
  //}

  //cv::Mat dimg = cv::Mat(800, 800, CV_32FC1, dmap);
  //cv::Mat mappedDimg;
  //dimg = 100 * dimg;
  //dimg.convertTo(mappedDimg, CV_8UC1);
  ////cv::applyColorMap(dimg*100, mappedDimg, cv::COLORMAP_JET);
  //cv::imshow("dimg", mappedDimg);
  QVector<QColor> color_map = makeColorMap();

  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {
      //QColor color = 
      //  qtJetColor(dmap[j + i * 800] / max_dist, 0, 0.01);
      QColor color = 
          qtJetColor((*dmap)[j + i * resolution], vmin, vmax);
      bgmap[(j + i * resolution) * 3 + 0] = color.red();
      bgmap[(j + i * resolution) * 3 + 1] = color.green();
      bgmap[(j + i * resolution) * 3 + 2] = color.blue();
    }
  }



  int newWidth = 1 << (int)(1 + log(resolution - 1 + 1E-3) / log(2.0));
  int newHeight = 1 << (int)(1 + log(resolution - 1 + 1E-3) / log(2.0));

  u_max = 1.0;//800 / (float)newWidth;
  v_max = 1.0;//800 / (float)newHeight;
  ratio = newWidth / (float)newHeight;

  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

  glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
  glTexImage2D( GL_TEXTURE_2D, 0, 3, resolution, resolution, 0, GL_RGB, GL_UNSIGNED_BYTE, &bgmap[0]);

#define GL_DEBUG
#ifdef GL_DEBUG
  if (glGetError() != 0)
  {
    std::cout<<"GL Error when set texture image.\n";
  }
#endif
}

void VectorFieldCanvas::setGLProperty()
{
  this->setScalarField();
}

void VectorFieldCanvas::setVisualizationParas(std::vector<bool>& paras)
{
  this->vis_paras = paras;
}

void VectorFieldCanvas::updateSourceField(int update_type)
{
  feature_model->updateSourceField(update_type);
}

void VectorFieldCanvas::addConstrainedLines(std::vector<double2>& line, std::vector<double2>& selectedLine)
{
  if (!feature_model)
  {
    return;
  }

  if(render_mode == VectorField::SOURCE_MODE)
  {
    feature_model->source_vector_field_lines->lines.push_back(line);
    /*std::cout << "Current number of constrained lines in source: "
      << feature_model->source_vector_field_lines->lines.size() << std::endl;*/
    CURVES normalized_source_curves, normalized_target_curves;
    feature_model->NormalizedSourceCurves(normalized_source_curves);
    feature_model->NormalizedTargetCurves(normalized_target_curves);
    double2 middle_point = line[int(line.size() / 2)];
    middle_point = (middle_point - double2(0.5, 0.5)) / (feature_model->getCurveScale()) + double2(0.5, 0.5) - (feature_model->getCurveTranslate());
    //std::cout << "denormalized point: " << middle_point.x << "\t" << middle_point.y << std::endl;
    std::shared_ptr<KDTreeWrapper> source_KDTree = feature_model->getSourceKDTree();
    std::shared_ptr<KDTreeWrapper> target_KDTree = feature_model->getTargetKDTree();
    int source_pt_id,target_pt_id; 
    float source_dis,target_dis;
    std::vector<float> pt, pt2;
    pt.push_back(middle_point.x);
    pt.push_back(middle_point.y);
    pt2 = pt;
    source_KDTree->nearestPt(pt, source_pt_id, source_dis);
    target_KDTree->nearestPt(pt2, target_pt_id, target_dis);
    int curves_id;
    if(source_dis < target_dis)
    {
      curves_id = feature_model->getSourceKDTreeMapper()[source_pt_id].first;
      selectedLine = normalized_source_curves[curves_id];
      if(crsp_oder == 1)
      {
        int original_source_curve_id;
        original_source_curve_id = feature_model->getVisibleGlobalMapper()[curves_id];
        feature_model->user_marked_crsp.push_back(original_source_curve_id);
        crsp_oder = 2;
      }
      else
      {
        std::cout << "You should mark target curves this time!" << std::endl;
      }
    }
    else
    {
      curves_id = feature_model->getTargetKDTreeMapper()[target_pt_id].first;
      selectedLine = normalized_target_curves[curves_id];
      if(crsp_oder == 2)
      {
        feature_model->user_marked_crsp.push_back(curves_id);
        crsp_oder = 1;
        // update crsp now
        feature_model->updateSourceField(4);
      }
      else
      {
        std::cout << "You should mark source curves this time!" << std::endl;
      }
    }
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

  /*if(render_mode == VectorField::SOURCE_MODE)
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
  }*/

  /*if(render_mode == VectorField::SOURCE_MODE)
  {
    feature_model->user_marked_crsp.pop_back();
    crsp_oder = 1;
    std::cout << "Delete the last user marked source curves!" << std::endl;
  }
  else
  {
    feature_model->user_marked_crsp.pop_back();
    crsp_oder = 2;
    std::cout << "Delete the last user marked target curves!" << std::endl;
  }*/
  if(feature_model->user_marked_crsp.size() % 2 == 0)
  {
    crsp_oder = 2;
  }
  else
  {
    crsp_oder = 1;
  }
  feature_model->user_marked_crsp.pop_back();
  std::cout << "Delete the last user masked curves!" << std::endl;
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

void VectorFieldCanvas::setConstrainedPair(double start[2], double end[2])
{
  if (!feature_model)
  {
    return;
  }

  if (render_mode == VectorField::SOURCE_MODE)
  {
    feature_model->setUserCrspPair(start, end);
  }
}

bool VectorFieldCanvas::displaySourceCrspList()
{
  CURVES source_curves;
  CURVES target_curves;
  feature_model->NormalizedSourceCurves(source_curves);
  feature_model->NormalizedTargetCurves(target_curves);
  std::vector<std::pair<int, int> >& src_crsp_list = feature_model->src_crsp_list;
  std::vector<std::pair<int, int> >& tar_crsp_list = feature_model->tar_crsp_list;
  std::map<std::pair<int, int>, int>& src_vid_mapper= feature_model->getSrcVidMapper();
  std::map<int, std::pair<int, int> >& src_rev_vid_mapper = feature_model->getSrcRevVidMapper();
  std::map<int, double2>& user_correct_crsp_map = feature_model->user_correct_crsp_map;
  std::map<int, double2>::iterator map_iter;

  if (src_crsp_list.size() != tar_crsp_list.size())
  {
    std::cerr << "Error: size of source correspondence list doesn't match that of target.\n";
    return false;
  }

  glPointSize(5) ;

  for (size_t i = 0; i < src_crsp_list.size(); ++i)
  {
    QColor color = 
      qtJetColor(double(i)/src_crsp_list.size());
    glColor4f( color.redF(), color.greenF(), color.blueF(), 0.1f );

    glBegin(GL_LINES);
    double2 pos_src = source_curves[src_crsp_list[i].first][src_crsp_list[i].second];
    double2 pos_tar;
    map_iter = user_correct_crsp_map.find(src_vid_mapper[src_crsp_list[i]]); // TODO
    if (map_iter == user_correct_crsp_map.end())
    {
      pos_tar = target_curves[tar_crsp_list[i].first][tar_crsp_list[i].second];
    }
    else
    {
      pos_tar = map_iter->second;
      feature_model->NormalizedPts(pos_tar);
    }
    glVertex3f( pos_src.x,pos_src.y, 0 ) ;
    glVertex3f( pos_tar.x,pos_tar.y, 0 ) ;
    glEnd();

    glBegin(GL_POINTS);
    glVertex3f( pos_src.x,pos_src.y, 0 ) ;
    glVertex3f( pos_tar.x,pos_tar.y, 0 ) ;
    glEnd();
  }

  glColor4f( 0.75f, 0.75f, 0.75f, 0.1f );
  std::pair<int, int> cur_src_curve_id;
  for (auto i : user_correct_crsp_map)
  {
    glBegin(GL_LINES);
    cur_src_curve_id = src_rev_vid_mapper[i.first];
    double2 pos_src = source_curves[cur_src_curve_id.first][cur_src_curve_id.second];
    //feature_model->getNormalizedProjPt(i.first, pos_src);
    double2 pos_tar = i.second;
    feature_model->NormalizedPts(pos_tar);

    glVertex3f( pos_src.x,pos_src.y, 0 ) ;
    glVertex3f( pos_tar.x,pos_tar.y, 0 ) ;
    glEnd();

    glBegin(GL_POINTS);
    glVertex3f( pos_src.x,pos_src.y, 0 ) ;
    glVertex3f( pos_tar.x,pos_tar.y, 0 ) ;
    glEnd();
  }

  return true;
}

bool VectorFieldCanvas::displayTargetCrspList()
{
  CURVES target_curves;
  feature_model->NormalizedTargetCurves(target_curves);
  std::vector<std::pair<int, int> >& tar_crsp_list = feature_model->tar_crsp_list;

  glPointSize(2) ;
  glBegin(GL_POINTS);
  for (size_t i = 0; i < tar_crsp_list.size(); ++i)
  {
    QColor color = 
      qtJetColor(double(i)/tar_crsp_list.size());
    glColor4f( color.redF(), color.greenF(), color.blueF(), 0.1f );
    double2 pos = target_curves[tar_crsp_list[i].first][tar_crsp_list[i].second];
    glVertex3f( pos.x,pos.y, 0 ) ;
  }
  glEnd();

  return true;
}

bool VectorFieldCanvas::displayUserCrsp()
{
  CURVES source_curves, target_curves;
  feature_model->NormalizedSourceCurves(source_curves);
  feature_model->NormalizedTargetCurves(target_curves);
  for(size_t i = 0; i < feature_model->marked_source_curves.size(); i ++)
  {
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);
    for(size_t j = 0; j < source_curves[feature_model->marked_source_curves[i]].size(); j ++)
    {
      glColor3f(1,0,0);
      glVertex3f(source_curves[feature_model->marked_source_curves[i]][j].x, source_curves[feature_model->marked_source_curves[i]][j].y, 0);
    }
    glEnd();
  }

  for(size_t i = 0; i < feature_model->marked_target_curves.size(); i ++)
  {
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);
    for(size_t j = 0; j < target_curves[feature_model->marked_target_curves[i]].size(); j ++)
    {
      glColor3f(1,1,0);
      glVertex3f(target_curves[feature_model->marked_target_curves[i]][j].x, target_curves[feature_model->marked_target_curves[i]][j].y, 0);
    }
    glEnd();
  }

  if(feature_model->user_marked_crsp.size() % 2 != 0)
  {
    int id = feature_model->user_marked_crsp[feature_model->user_marked_crsp.size() - 1];
    std::vector<int> last_line = feature_model->getGlobalVisibleMapper()[id];
    for(size_t i = 0; i < last_line.size(); i ++)
    {
      glLineWidth(5);
      glBegin(GL_LINE_STRIP);
      for(size_t j = 0; j < source_curves[last_line[i]].size(); j ++)
      {
        glColor3f(1,0,0);
        glVertex3f(source_curves[last_line[i]][j].x, source_curves[last_line[i]][j].y, 0);
      }
      glEnd();
    }
  }
  return true;
}

bool VectorFieldCanvas::displayAllCurvesPoints()
{
  CURVES source_curves, target_curves;
  this->feature_model->NormalizedSourceCurves(source_curves);
  this->feature_model->NormalizedTargetCurves(target_curves);
  glColor3f(1,0,1);
  glPointSize(5);
  glBegin(GL_POINTS);
  for(int i = 0; i < source_curves.size(); ++ i)
  {
    for (int j = 0; j < source_curves[i].size(); ++j)
    {
      double2 pos = source_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
  }
  glEnd();

  /*glColor3f(0,1,1);
  glPointSize(5);
  glBegin(GL_POINTS);
  for(int i = 0; i < target_curves.size(); ++ i)
  {
    for (int j = 0; j < target_curves[i].size(); ++j)
    {
      double2 pos = target_curves[i][j];
      glVertex3f( pos.x,pos.y, 0 ) ;
    }
  }
  glEnd();*/
  return true;
}
