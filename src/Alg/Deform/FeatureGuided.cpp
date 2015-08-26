#include "FeatureGuided.h"
#include "FeatureGuidedVis.h"

FeatureGuided::FeatureGuided()
{
  this->disp_obj = nullptr;
  this->tele_register = nullptr;
}

FeatureGuided::~FeatureGuided()
{
  if (!this->disp_obj)
  {
    delete this->disp_obj;
  }
  if (!this->tele_register)
  {
    delete this->tele_register;
  }
}

FeatureGuided::FeatureGuided(std::string sourceFile, std::string targetFile)
{
  this->disp_obj = nullptr;
  this->initImages(sourceFile, targetFile);
  this->initRegister();
}

void FeatureGuided::initDispObj()
{
  if (!this->disp_obj)
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
  FeatureGuided::ExtractCurves(this->source_img, source_curves);
  //FeatureGuided::ExtractCurves(this->target_img, target_curves);
  std::vector<std::vector<int>> group(1);
  std::vector<int2> endps;
  for (int i = 0; i < source_curves.size(); ++i)
  {
    group[0].push_back(i);
    endps.push_back(int2(1, 0));
  }
  if (!this->tele_register)
  {
    delete this->tele_register;
  }
  tele_register = new tele2d(100, 0.02, 1);
  tele_register->init(source_curves, group, endps);
  tele_register->setInputField();
}

void FeatureGuided::ExtractCurves(const cv::Mat& source, CURVES& curves)
{
  std::vector<std::vector<cv::Point>> contours;
  //cv::findContours(source, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

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
        FeatureGuided::SearchCurve(source, i, j, i, j, visited_table, curve);
        CURVES cur_curves = FeatureGuided::ReorganizeCurves(FeatureGuided::SplitCurve(curve));
        curves.insert(curves.end(), cur_curves.begin(), cur_curves.end());
      }
    }
  }
#define DEBUG
#ifdef DEBUG
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
  int cur_row, int cur_col, int last_row, int last_col,
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
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row + 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col - 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
  new_row = cur_row - 1;
  new_col = cur_col + 1;
  FeatureGuided::SearchCurve(source, new_row, new_col, cur_row, cur_col, visited_table, curve);
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
    double cur_length = 0;
    for (int j = 1; j < curves[i].size(); ++j)
    {
      cur_length += 
        sqrt(pow(curves[i][j].x - curves[i][j - 1].x, 2)
          + pow(curves[i][j].y - curves[i][j - 1].y, 2));
    }

    if (cur_length > 10)
    {
      if (curves[i].size() > 30)
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