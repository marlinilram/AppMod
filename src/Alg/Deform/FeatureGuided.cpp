#include "FeatureGuided.h"
#include "FeatureGuidedVis.h"
#include "tele2d.h"

FeatureGuided::FeatureGuided()
{
  this->initAllPtr();
}

FeatureGuided::~FeatureGuided()
{
  if (this->disp_obj)
  {
    delete this->disp_obj;
  }
  if (this->tele_register)
  {
    delete this->tele_register;
  }
  if (this->source_KDTree)
  {
    delete this->source_KDTree;
  }
  if (this->target_KDTree)
  {
    delete this->target_KDTree;
  }
}

FeatureGuided::FeatureGuided(std::string sourceFile, std::string targetFile)
{
  this->initAllPtr();
  this->initImages(sourceFile, targetFile);
  this->initRegister();
}

void FeatureGuided::initAllPtr()
{
  this->disp_obj = nullptr;
  this->tele_register = nullptr;
  this->source_KDTree = nullptr;
  this->target_KDTree = nullptr;
}

void FeatureGuided::initDispObj()
{
  if (this->disp_obj)
  {
    delete this->disp_obj;
  }
  this->disp_obj = new FeatureGuidedVis;
  this->disp_obj->init(this);
}

void FeatureGuided::initImages(const cv::Mat& source, const cv::Mat& target)
{
  this->source_img = source.clone();
  this->target_img = target.clone();
}

void FeatureGuided::initImages(std::string sourceFile, std::string targetFile)
{
  this->source_img = cv::imread(sourceFile, CV_LOAD_IMAGE_GRAYSCALE);
  this->target_img = cv::imread(targetFile, CV_LOAD_IMAGE_GRAYSCALE);
}

void FeatureGuided::initRegister()
{
  // extract edges from souce image and target image
  FeatureGuided::ExtractCurves(this->source_img, source_curves);
  FeatureGuided::ExtractCurves(this->target_img, target_curves);

  // init tele2d
  std::vector<std::vector<int>> group(1);
  std::vector<int2> endps;
  for (int i = 0; i < source_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  if (this->tele_register)
  {
    delete this->tele_register;
  }
  this->tele_register = new tele2d(100, 0.02, 1);
  this->tele_register->init(source_curves, group, endps);
  this->tele_register->setInputField();

  // init source distance map
  this->BuildDispMap(source_img, source_KDTree_data);
  if (this->source_KDTree)
  {
    delete this->source_KDTree;
  }
  this->source_KDTree = new kdtree::KDTree(source_KDTree_data);

  // Optimize Connection
  //this->OptimizeConnection();
}

void FeatureGuided::ExtractCurves(const cv::Mat& source, CURVES& curves)
{
  // find curves
  std::vector<std::vector<bool>> visited_table(source.rows, std::vector<bool>(source.cols, false));
  std::vector<double2> curve;
  for (int i = 0; i < source.rows; ++i)
  {
    for (int j = 0; j < source.cols; ++j)
    {
      if (source.at<uchar>(i, j) > 200 && visited_table[i][j] == false)
      {
        curve.clear();
        FeatureGuided::SearchCurve(source, i, j, visited_table, curve);
        CURVES cur_curves = FeatureGuided::ReorganizeCurves(FeatureGuided::SplitCurve(curve));
        curves.insert(curves.end(), cur_curves.begin(), cur_curves.end());
      }
    }
  }

#ifdef DEBUG
  std::vector<std::vector<cv::Point>> contours;
  //cv::findContours(source, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // draw curves
  cv::Mat contour = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
  for (int i = 0; i < curves.size(); ++i)
  {
    for (int j = 0; j < curves[i].size(); ++j)
    {
      contour.at<uchar>(source.rows - 1 - curves[i][j].y, curves[i][j].x) = 255;
    }
    cv::imwrite(std::to_string(i) + ".png", contour);
  }
#endif
}

std::vector<double2> FeatureGuided::SearchCurve(
  const cv::Mat& source, int r, int c, 
  std::vector<std::vector<bool>>& visited_table)
{
  // search a curve recursively
  // seems to go out of memory...
  // use cv::findContours instead
  std::vector<double2> result;
  if (source.at<uchar>(r, c) < 200 || visited_table[r][c] == true)
  {
    return result;
  }

  visited_table[r][c] = true;
  result.push_back(double2(r, c));

  std::vector<int2> pos;
  std::vector<double2> temp_result;

  pos.push_back(int2((r - 1) < 0 ? 0 : r - 1, c));
  pos.push_back(int2(r, (c + 1) >= source.cols ? c : c + 1));
  pos.push_back(int2((r + 1) >= source.rows ? r : r + 1, c));
  pos.push_back(int2(r, (c - 1) < 0 ? 0 : c - 1));
  pos.push_back(int2((r - 1) < 0 ? 0 : r - 1, (c - 1) < 0 ? 0 : c - 1));
  pos.push_back(int2((r - 1) < 0 ? 0 : r - 1, (c + 1) >= source.cols ? c : c + 1));
  pos.push_back(int2((r + 1) >= source.rows ? r : r + 1, (c - 1) < 0 ? 0 : c - 1));
  pos.push_back(int2((r + 1) >= source.rows ? r : r + 1, (c + 1) >= source.cols ? c : c + 1));
  for (int i = 0; i < 8; ++i)
  {
    temp_result = FeatureGuided::SearchCurve(source, pos[i].first, pos[i].second, visited_table);
    result.insert(result.end(), temp_result.begin(), temp_result.end());
  }
  return result;
}

void FeatureGuided::initVisualization(BasicViewer* renderer)
{
  if (tele_register != NULL)
  {
    this->renderer = renderer;
    this->initDispObj();
    this->renderer->addDispObj(this->disp_obj);
  }
}

void FeatureGuided::SearchCurve(const cv::Mat& source,
  int cur_row, int cur_col,
  std::vector<std::vector<bool>>& visited_table,
  std::vector<double2>& curve)
{
  // if the current pos isn't a edge or is visited before or out of boundary, return
  if (source.at<uchar>(cur_row, cur_col) < 200 || visited_table[cur_row][cur_col] == true
    || cur_row < 0 || cur_row >= source.rows || cur_col < 0 || cur_col >= source.cols)
  {
    return;
  }

  curve.push_back(double2(cur_col, source.rows - 1 - cur_row));
  visited_table[cur_row][cur_col] = true;


  int new_row = cur_row;
  int new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, visited_table, curve);
  //for (int i = 0; i < 3; ++i)
  //{
  //  for (int j = 0; j < 3; ++j)
  //  {
  //    int new_row = cur_row + i - 1;
  //    int new_col = cur_col + j - 1;
  //    if (!((new_row == cur_row && new_col == cur_row)
  //      || (new_row == last_row && new_col == last_col)))
  //    {
  //      FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  //    }
  //  }
  //}
}

CURVES FeatureGuided::ReorganizeCurves(CURVES& curves)
{
  // check if all curves are single connected
  for (int i = 0; i < curves.size(); ++i)
  {
    int j = 1;
    for (; j < curves[i].size(); ++j)
    {
      if (sqrt(pow(curves[i][j].x - curves[i][j - 1].x, 2) + pow(curves[i][j].y - curves[i][j - 1].y, 2)) > 1.5)
      {
        // if it's connected no larger than 1.414
        break;
      }
    }
    if (j != curves[i].size())
    {
      // break early, i is the one end 
      std::cerr<<"Not a single connected curve.\n";
    }
  }

  // sample curve
  // don't save whole pixel position for the contour
  // lead to error in tele-reg
  CURVES reorganized;
  for (int i = 0; i < curves.size(); ++i)
  {
    double cur_length = FeatureGuided::CurveLength(curves[i]);

    if (cur_length > 50)
    {
      if (curves[i].size() > 50)
      {
        int step = 1 + curves[i].size() / 50;
        int tail = 0;
        for (int j = 0; j < curves[i].size(); ++j)
        {
          if (j % step == 0)
          {
            curves[i][tail] = curves[i][j];
            ++tail;
          }
        }
        curves[i].erase(curves[i].begin() + tail, curves[i].end());
        reorganized.push_back(curves[i]);
      }
    }
  }
  return reorganized;
}

CURVES FeatureGuided::SplitCurve(std::vector<double2> curve)
{
  CURVES curves;
  int i = 1;
  for (; i < curve.size(); ++i)
  {
    if (sqrt(pow(curve[i].x - curve[i - 1].x, 2) + pow(curve[i].y - curve[i - 1].y, 2)) > 1.5)
    {
      // if it's connected no larger than 1.414
      break;
    }
  }
  if (i == curve.size())
  {
    curves.push_back(curve);
    return curves;
  }
  else
  {
    // break early, i is the one end
    std::vector<double2> curve1(curve.begin() + i, curve.end());
    curves.push_back(std::vector<double2>(curve.begin(), curve.begin() + i));
    CURVES r_curves = FeatureGuided::SplitCurve(curve1);
    curves.insert(curves.end(), r_curves.begin(), r_curves.end());
    return curves;
  }
}

std::vector<double2> FeatureGuided::ConnectCurves(std::vector<double2> curve0, std::vector<double2> curve1,
  int endtag0, int endtag1)
{
  const int endpid1_id = (endtag0%2 == 0) ? 0 : curve0.size() -1;
  const int endpid2_id = (endtag1%2 == 0) ? 0 : curve1.size() -1;

  double2 p0 = curve0[endpid1_id] ;
  double2 p1 = curve1[endpid2_id] ;
  double2 tan0, tan1 ;

  // slope of tangent
  if( endpid1_id == 0 )
    tan0 = curve0[0] - curve0[1] ;
  else
    tan0 = curve0.back() - curve0[curve0.size()-2] ;

  if( endpid2_id == 0 )
    tan1 = curve1[1] - curve1[0] ; 
  else
    tan1 = curve1[curve1.size()-2] - curve1.back() ;

  tan0.normalize();
  tan1.normalize();


  std::vector<double2>  nullcurve;
  for( double t=0.0; t<=1.0; t+=0.01 )
  {
    nullcurve.push_back( tele2d::get_hermite_value( p0, p1, tan0, tan1, t ) );
  }

  return nullcurve;
}

void FeatureGuided::EliminateRedundancy(CURVES& curves)
{

}

void FeatureGuided::OptimizeConnection()
{
  // delete curves highly incompatible with the vector field

  CURVES new_target_curves;
  CURVES normalized_target_curves;
  this->NormalizedTargetCurves(normalized_target_curves);

  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    if (this->MatchScoreToVectorField(normalized_target_curves[i]) >= 0.6)
    {
      new_target_curves.push_back(this->target_curves[i]);
    }
  }

  CURVES best_connections;
  normalized_target_curves = new_target_curves;
  FeatureGuided::NormalizedCurves(normalized_target_curves);

  for (int i = 0; i < normalized_target_curves.size(); ++i)
  {
    double cur_best_score = std::numeric_limits<double>::min();
    std::vector<double2> cur_best_connection;
    int2 cur_best_end;
    int cur_best_curve_id;
    for (int j = 0; j < normalized_target_curves.size(); ++j)
    {
      if (i != j) {
      for (int endi = 0; endi < 2; ++endi)
      {
        for (int endj = 0; endj < 2; ++endj)
        {
          double cur_score = this->MatchScoreToVectorField(
            FeatureGuided::ConnectCurves(
              normalized_target_curves[i], normalized_target_curves[j], endi, endj));
          double2 dis = 
            normalized_target_curves[i][endi * (normalized_target_curves[i].size() - 1)]
            - normalized_target_curves[j][endi * (normalized_target_curves[j].size() - 1)];
          //cur_score /= dis.norm();
          if (cur_score > cur_best_score)
          {
            cur_best_score = cur_score;
            cur_best_end.first = endi;
            cur_best_end.second = endj;
            cur_best_curve_id = j;
          }
        }
      }}
    }
    best_connections.push_back(FeatureGuided::ConnectCurves(
      new_target_curves[i], new_target_curves[cur_best_curve_id],
      cur_best_end.first, cur_best_end.second));
  }

  for (int i = 0; i < best_connections.size(); ++i)
  {
    if (FeatureGuided::CurveLength(best_connections[i]) < 150)
    {
      new_target_curves.push_back(best_connections[i]);
    }
  }
  this->target_curves = new_target_curves;

#ifdef DEBUG
  // draw curves
  cv::Mat contour = cv::Mat::zeros(this->target_img.rows, this->target_img.cols, CV_8UC1);
  for (int i = 0; i < new_target_curves.size(); ++i)
  {
    for (int j = 0; j < new_target_curves[i].size(); ++j)
    {
      contour.at<uchar>(this->target_img.rows - 1 - new_target_curves[i][j].y, new_target_curves[i][j].x) = 255;
    }
    cv::imwrite("optimized_" + std::to_string(i) + ".png", contour);
  }
#endif
}

double FeatureGuided::MatchScoreToVectorField(std::vector<double2>& curve)
{
  // integrate
  int resolution = this->tele_register->resolution;
  std::vector<double2>& vector_field = this->tele_register->vector_field;
  double angle = 0;
  double curve_length = 0;
  for (int i = 1; i < curve.size(); ++i)
  {
    double2 curve_angle = curve[i] - curve[i - 1];
    curve_angle.normalize();
    double2 vector_field_pos = (curve[i] + curve[i - 1]) / 2;
    int x = (vector_field_pos.x * resolution + 0.5);
    int y = (vector_field_pos.y * resolution + 0.5);
    x = (x >= resolution ? resolution : x);
    y = (y >= resolution ? resolution : y);
    double2 vector_field_dir = vector_field[x + y * resolution];
    vector_field_dir.normalize();
    angle += abs(curve_angle.x * vector_field_dir.x + curve_angle.y * vector_field_dir.y);
  }
  angle /= (curve.size() - 1);
  return angle;
}

void FeatureGuided::NormalizedTargetCurves(CURVES& curves)
{
  curves = this->target_curves;
  FeatureGuided::NormalizedCurves(curves);
}

void FeatureGuided::NormalizedSourceCurves(CURVES& curves)
{
  curves = this->source_curves;
  FeatureGuided::NormalizedCurves(curves);
}

void FeatureGuided::NormalizedCurves(CURVES& curves)
{
  double2 normalize_translate;
  double normalize_scale;

  FeatureGuided::NormalizePara(curves, normalize_translate, normalize_scale);

  FeatureGuided::NormalizedCurves(curves, normalize_translate, normalize_scale);
}

void FeatureGuided::NormalizedCurves(CURVES& curves, double2 translate, double scale)
{
  for( int i=0; i<curves.size(); ++i )
    for( int j=0; j<curves[i].size(); ++j ){
      curves[i][j] = ( curves[i][j] + translate - double2(0.5, 0.5 ) ) * scale + double2(0.5, 0.5 );
    }
}

void FeatureGuided::NormalizedCurve(std::vector<double2>& curve, double2 translate, double scale)
{
  for (int i = 0; i < curve.size(); ++i)
  {
    curve[i] = ( curve[i] + translate - double2(0.5, 0.5 ) ) * scale + double2(0.5, 0.5 );
  }
}

void FeatureGuided::NormalizePara(CURVES& curves, double2& translate, double& scale)
{
  double2 minCorner ( 1e10, 1e10);
  double2 maxCorner (-1e10, -1e10);
  for( int i=0; i<curves.size(); ++i )
    for( int j=0; j<curves[i].size(); ++j ){
      if( curves[i][j].x < minCorner.x  ) minCorner.x =  curves[i][j].x ;
      if( curves[i][j].y < minCorner.y  ) minCorner.y =  curves[i][j].y ;
      if( curves[i][j].x > maxCorner.x  ) maxCorner.x =  curves[i][j].x ;
      if( curves[i][j].y > maxCorner.y  ) maxCorner.y =  curves[i][j].y ;
    }

    double2 cter = (minCorner+maxCorner) / 2 ;
    translate =  double2(0.5, 0.5 ) - cter ;
    scale = 0.6 / std::max( (maxCorner-minCorner).x, (maxCorner-minCorner).y  ) ;
}

void FeatureGuided::GetSourceNormalizePara(double2& translate, double& scale)
{
  FeatureGuided::NormalizePara(this->source_curves, translate, scale);
}

double FeatureGuided::CurveLength(std::vector<double2>& curve)
{
  double cur_length = 0;
  for (int j = 1; j < curve.size(); ++j)
  {
    cur_length += 
      sqrt(pow(curve[j].x - curve[j - 1].x, 2)
      + pow(curve[j].y - curve[j - 1].y, 2));
  }
  return cur_length;
}

void FeatureGuided::BuildDispMap(const cv::Mat& source, kdtree::KDTreeArray& KDTree_data)
{
  // find curves
  std::vector<std::vector<bool> > visited_table(source.rows, std::vector<bool>(source.cols, false));
  CURVES edge_points;
  std::vector<double2> edge;
  size_t n_all_edge_points = 0;
  for (int i = 0; i < source.rows; ++i)
  {
    for (int j = 0; j < source.cols; ++j)
    {
      if (source.at<uchar>(i, j) > 200 && visited_table[i][j] == false)
      {
        edge.clear();
        FeatureGuided::SearchCurve(source, i, j, visited_table, edge);
        edge_points.push_back(edge);
        n_all_edge_points += edge.size();
      }
    }
  }

  //FeatureGuided::NormalizedCurves(edge_points);
  KDTree_data.resize(boost::extents[n_all_edge_points][2]);
  int cnt_edge_points = 0;
  for (int i = 0; i < edge_points.size(); ++i)
  {
    for (int j = 0; j < edge_points[i].size(); ++j)
    {
      KDTree_data[cnt_edge_points][0] = float(edge_points[i][j].x);
      KDTree_data[cnt_edge_points][1] = float(edge_points[i][j].y);
      ++cnt_edge_points;
    }
  }
}

kdtree::KDTree* FeatureGuided::getSourceKDTree()
{
  return this->source_KDTree;
}


void FeatureGuided::GetFittedCurves(CURVES& curves)
{ 
  CURVES new_target_curves;
  CURVES normalized_target_curves;
  this->NormalizedTargetCurves(normalized_target_curves);

  std::vector<float> query(2, 0.0);
  kdtree::KDTreeResultVector result;
  std::vector<double2> crsp_target;
  std::vector<double2> crsp_source;

  double2 target_translate;
  double target_scale;
  double2 source_translate;
  double source_scale;
  FeatureGuided::NormalizePara(this->target_curves, target_translate, target_scale);
  FeatureGuided::NormalizePara(this->source_curves, source_translate, source_scale);

  for (int i = 0; i < this->target_curves.size(); ++i)
  {
    if (this->MatchScoreToVectorField(normalized_target_curves[i]) >= 0.85)
    {
      curves.push_back(this->target_curves[i]);

      for (int j = 0; j < this->target_curves[i].size(); ++j)
      {
        double2 target_pos_in_source =
          (this->target_curves[i][j] + target_translate - double2(0.5, 0.5))
          * target_scale / source_scale + double2(0.5, 0.5) - source_translate;
        query[0] = target_pos_in_source.x;
        query[1] = target_pos_in_source.y;
        this->source_KDTree->n_nearest(query, 1, result);
        crsp_target.push_back(this->target_curves[i][j]);
        crsp_source.push_back(double2(
          this->source_KDTree->the_data[result[0].idx][0],
          this->source_KDTree->the_data[result[0].idx][1]));
      }
    }
  }

  FeatureGuided::NormalizedCurves(curves, target_translate, target_scale);
  FeatureGuided::NormalizedCurve(crsp_target, target_translate, target_scale);
  FeatureGuided::NormalizedCurve(crsp_source, source_translate, source_scale);
  std::vector<double2> crsp_pair(2);
  for (int i = 0; i < crsp_target.size(); ++i)
  {
    crsp_pair[0] = crsp_target[i];
    crsp_pair[1] = crsp_source[i];
    curves.push_back(crsp_pair);
  }
}