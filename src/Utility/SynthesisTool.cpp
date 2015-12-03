#include "SynthesisTool.h"
#include "MeshParameterization.h"

#include <vector>
#include <cv.h>

void SynthesisTool::init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, cv::Mat& src_detail)
{
  // pyramids stored from up to down
  // [0] ------
  // [1]  ----
  // [2]   --
  levels = 3;
  NeighborRange.resize(3);
  NeighborRange[0].height = 7;
  NeighborRange[0].width = 7;
  NeighborRange[1].height = 5;
  NeighborRange[1].width = 5;
  NeighborRange[2].height = 3;
  NeighborRange[2].width = 3;
  

  // get image for pyramids
  gpsrc_feature.clear();
  gptar_feature.clear();
  gpsrc_detail.clear();
  gptar_detail.clear();

  gpsrc_feature.resize(src_feature.size());
  gptar_feature.resize(tar_feature.size());
  for(size_t i = 0; i < src_feature.size(); i ++)
  {
    gpsrc_feature[i].push_back(src_feature[i].clone());
    gptar_feature[i].push_back(tar_feature[i].clone());
  }
  gpsrc_detail.push_back(src_detail.clone());
  gptar_detail.push_back(cv::Mat::zeros(src_detail.rows, src_detail.cols, CV_32FC1));
  for(size_t i = 0; i < gptar_feature.size(); i ++)
  {
    this->generatePyramid(gpsrc_feature[i], levels);
    this->generatePyramid(gptar_feature[i], levels);
  }
  this->generatePyramid(gpsrc_detail, levels);
  this->generatePyramid(gptar_detail, levels);

  std::cout << "SynthesisTool Init success !" << std::endl;
  /*std::cout << "The size of gpsrc_detail is" << gpsrc_detail.size() << std::endl;
  std::cout << "The size of gptar_detail is" << gptar_detail.size() << std::endl;
  std::cout << "The size of gpsrc_feature is" << gpsrc_feature.size() << std::endl;
  std::cout << "The size of gptar_feature is" << gptar_feature.size() << std::endl;
  std::cout << "The size of gpsrc_feature[0] is" << gpsrc_feature[0].size() << std::endl;
  std::cout << "The size of gpsrc_feature[1] is" << gpsrc_feature[1].size() << std::endl;
  std::cout << "The size of gpsrc_feature[2] is" << gpsrc_feature[2].size() << std::endl;
  std::cout << "The size of gptar_feature[0] is" << gptar_feature[0].size() << std::endl;
  std::cout << "The size of gptar_feature[1] is" << gptar_feature[1].size() << std::endl;
  std::cout << "The size of gptar_feature[2] is" << gptar_feature[2].size() << std::endl;*/

  //std::cout << "The cols of gptar_detail[0] is :" << gptar_detail.at(0).cols << std::endl;
}

void SynthesisTool::generatePyramid(std::vector<cv::Mat>& pyr, int level)
{
  for (int i = 1; i < level; ++i)
  {
    cv::Mat dst;
    cv::pyrDown(pyr[i - 1], dst, cv::Size(pyr[i - 1].cols / 2, pyr[i - 1].rows / 2));
    pyr.push_back(dst);
  }
}

void SynthesisTool::doSynthesis()
{
  // find best match for each level
  double totalTime = 0.0;
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail.at(l).cols;
    int height = gptar_detail.at(l).rows;
    int findX;
    int findY;
    if(l == levels - 1)
    {
      double max, min;
      cv::minMaxLoc(gpsrc_detail.at(l),&min,&max);
      srand((unsigned)time(NULL));
      for(int i = 0; i < height; i ++)
      {
        for(int j = 0; j < width; j ++)
        {
          gptar_detail.at(l).at<float>(i, j) = (rand() / double(RAND_MAX)) * (max - min) + min;
        }
      }
    }
    else
    {
      cv::pyrUp(gptar_detail.at(l + 1), gptar_detail.at(l), cv::Size(gptar_detail.at(l).cols, gptar_detail.at(l).rows));
    }
    
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        std::set<distance_position> candidates;
        this->findCandidates(gpsrc_feature, gptar_feature, l, j, i, candidates);
        //std::cout << "The size of the candidates is :" << candidates.size() << std::endl;
        this->findBestMatch(gpsrc_detail, gptar_detail, l, j, i, candidates, findX, findY);
        gptar_detail.at(l).at<float>(i, j) = gpsrc_detail.at(l).at<float>(findY, findX);
        //std::cout << "Here is OK !" << std::endl;
      }
    }
    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}

void SynthesisTool::findCandidates(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates)
{
  /*findX = 0;
  findY = 0;*/
  double d = 0;
  //double dMin = std::numeric_limits<double>::max();
  //double threshhold = 37;
  
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
  for (int i = NeighborRange[level].height; i < sheight - NeighborRange[level].height; i = i + 5)
  {
    for (int j = NeighborRange[level].width; j < swidth - NeighborRange[level].width; j = j + 5)
    {
      d = this->distNeighborOnFeature(gpsrc, gptar, level, j, i, pointX, pointY);
      
      distance_position dis_pos;
      dis_pos.d = d;
      dis_pos.pos.first = j;
      dis_pos.pos.second = i;
      candidates.insert(dis_pos);
      if(candidates.size() > 20)
      {
        candidates.erase(--candidates.end());
      }
    }
  }


}

double SynthesisTool::distNeighborOnFeature(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY)
{
  // compute feature distance
  // the third dimension in the cv::Mat here could be much larger than 3
  /*int dim = gpsrc.at(0).dims;
  if (dim != 3)
  {
    std::cerr << "The dimension of the feature map isn't 3!.\n";
  }*/

  //int fdim = gpsrc.at(0).size[2];
  int fdim = gpsrc.size();
  double d = 0;
  double d1 = 0;
  int spy, spx, tpy, tpx;
  for (int i = 0; i < NeighborRange[level].height; ++i)   
  {
    for (int j = 0; j < NeighborRange[level].width; ++j) 
    {
      spy=srcpointY-NeighborRange[level].height/2+i;
      spx=srcpointX-NeighborRange[level].width/2+j;
      tpy=tarpointY-NeighborRange[level].height/2+i;
      tpx=tarpointX-NeighborRange[level].width/2+j;
      if(tpy<0)
        tpy+=gptar[0].at(level).rows;
      if(tpx<0)
        tpx+=gptar[0].at(level).cols;
      if(tpy>=gptar[0].at(level).rows)
        tpy-=gptar[0].at(level).rows;
      if(tpx>=gptar[0].at(level).cols)
        tpx-=gptar[0].at(level).cols;

      d1 = 0.0;
      for (int k = 0; k < fdim; ++k)
      {
        //d1 += pow(gpsrc.at(level).at<float>(spy, spx, k) - gptar.at(level).at<float>(tpy, tpx, k), 2);
        d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
      }
      d += sqrt(d1);
    }
  }
  
  /*spy = srcpointY;
  tpy = tarpointY;
  for (int j = 0; j < NeighborRange[level].width / 2; ++j)
  {
    spx=srcpointX-NeighborRange[level].width/2+j;
    tpx=tarpointX-NeighborRange[level].width/2+j;
    if(tpx<0)
      tpx+=gptar[0].at(level).cols;

    d1 = 0;
    for (int k = 0; k < fdim; ++k)
    {
      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
    }
    d += sqrt(d1);
  }

  if (level < gpsrc[0].size() - 1)
  {
    int srcpointXn = srcpointX/2;
    int srcpointYn = srcpointY/2;
    int tarpointXn = tarpointX/2;
    int tarpointYn = tarpointY/2;
    for (int i = 0; i < NeighborRange[level + 1].height; ++i)
    {
      for (int j = 0; j < NeighborRange[level + 1].width; ++j)
      {
        spy=srcpointYn-NeighborRange[level+1].height/2+i;
        spx=srcpointXn-NeighborRange[level+1].width/2+j;
        tpy=tarpointYn-NeighborRange[level+1].height/2+i;
        tpx=tarpointXn-NeighborRange[level+1].width/2+j;
        if(tpy<0)
          tpy+=gptar[0].at(level+1).rows;
        if(tpx<0)
          tpx+=gptar[0].at(level+1).cols;
        if(tpy>=gptar[0].at(level+1).rows)
          tpy-=gptar[0].at(level+1).rows;
        if(tpx>=gptar[0].at(level+1).cols)
          tpx-=gptar[0].at(level+1).cols;

        d1 = 0.0;
        for (int k = 0; k < fdim; ++k)
        {
          d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
        }
        d += sqrt(d1);
      }
    }
  }*/

  return d;
}

void SynthesisTool::findBestMatch(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int pointX, int pointY, std::set<distance_position> candidates, int& findX, int& findY)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  int sheight = gpsrc.at(level).rows;
  int swidth = gpsrc.at(level).cols;
  for(std::set<distance_position>::iterator iter = candidates.begin(); iter != candidates.end(); iter ++)
  {
    int srcX,srcY;
    srcX = (*iter).pos.first;
    srcY = (*iter).pos.second;
    d = this->distNeighborOnDetail(gpsrc, gptar, level, srcX, srcY, pointX, pointY);

    if (d < dMin)
    {
      dMin = d;
      findY = srcY;
      findX = srcX;
    }
  }
}

double SynthesisTool::distNeighborOnDetail(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY)
{
  //int fdim = gpsrc.size();
  double d = 0;
  double d1 = 0;
  int spy, spx, tpy, tpx;
  for (int i = 0; i < NeighborRange[level].height / 2; ++i)   
  {
    for (int j = 0; j < NeighborRange[level].width; ++j) 
    {
      spy=srcpointY-NeighborRange[level].height/2+i;
      spx=srcpointX-NeighborRange[level].width/2+j;
      tpy=tarpointY-NeighborRange[level].height/2+i;
      tpx=tarpointX-NeighborRange[level].width/2+j;
      if(tpy<0)
        tpy+=gptar.at(level).rows;
      if(tpx<0)
        tpx+=gptar.at(level).cols;
      if(tpy>=gptar.at(level).rows)
        tpy-=gptar.at(level).rows;
      if(tpx>=gptar.at(level).cols)
        tpx-=gptar.at(level).cols;

      //d1 = 0.0;
      //for (int k = 0; k < fdim; ++k)
      //{
      //  //d1 += pow(gpsrc.at(level).at<float>(spy, spx, k) - gptar.at(level).at<float>(tpy, tpx, k), 2);
      //  d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
      //}
      d1 += pow(gpsrc.at(level).at<float>(spy, spx) - gptar.at(level).at<float>(tpy,tpx), 2);
    }
  }
  d += sqrt(d1);

  spy = srcpointY;
  tpy = tarpointY;
  d1 = 0.0;
  for (int j = 0; j < NeighborRange[level].width / 2; ++j)
  {
    spx=srcpointX-NeighborRange[level].width/2+j;
    tpx=tarpointX-NeighborRange[level].width/2+j;
    if(tpx<0)
      tpx+=gptar.at(level).cols;

    /*d1 = 0;
    for (int k = 0; k < fdim; ++k)
    {
      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
    }*/
    d1 += pow(gpsrc.at(level).at<float>(spy, spx) - gptar.at(level).at<float>(tpy,tpx), 2);
  }
  d += sqrt(d1);

  if (level < gpsrc.size() - 1)
  {
    int srcpointXn = srcpointX/2;
    int srcpointYn = srcpointY/2;
    int tarpointXn = tarpointX/2;
    int tarpointYn = tarpointY/2;
    for (int i = 0; i < NeighborRange[level + 1].height; ++i)
    {
      for (int j = 0; j < NeighborRange[level + 1].width; ++j)
      {
        spy=srcpointYn-NeighborRange[level+1].height/2+i;
        spx=srcpointXn-NeighborRange[level+1].width/2+j;
        tpy=tarpointYn-NeighborRange[level+1].height/2+i;
        tpx=tarpointXn-NeighborRange[level+1].width/2+j;
        if(tpy<0)
          tpy+=gptar.at(level+1).rows;
        if(tpx<0)
          tpx+=gptar.at(level+1).cols;
        if(tpy>=gptar.at(level+1).rows)
          tpy-=gptar.at(level+1).rows;
        if(tpx>=gptar.at(level+1).cols)
          tpx-=gptar.at(level+1).cols;

        d1 = 0.0;
        /*for (int k = 0; k < fdim; ++k)
        {
          d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
        }*/
        d1 += pow(gpsrc.at(level).at<float>(spy, spx) - gptar.at(level).at<float>(tpy,tpx), 2);
      }
    }
    d += sqrt(d1);
  }

  return d;
}

void SynthesisTool::doImageSynthesis(cv::Mat& src_detail)
{
  levels = 3;
  NeighborRange.resize(3);
  NeighborRange[0].height = 7;
  NeighborRange[0].width = 7;
  NeighborRange[1].height = 5;
  NeighborRange[1].width = 5;
  NeighborRange[2].height = 3;
  NeighborRange[2].width = 3;
  gpsrc_detail.clear();
  gptar_detail.clear();
  gpsrc_detail.push_back(src_detail.clone());
  gptar_detail.push_back(cv::Mat::zeros(src_detail.rows, src_detail.cols, CV_32FC1));
  this->generatePyramid(gpsrc_detail, levels);
  this->generatePyramid(gptar_detail, levels);
  double totalTime = 0.0;
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail.at(l).cols;
    int height = gptar_detail.at(l).rows;
    int findX;
    int findY;
    if(l == levels - 1)
    {
      double max, min;
      cv::minMaxLoc(gpsrc_detail.at(l),&min,&max);
      srand((unsigned)time(NULL));
      for(int i = 0; i < height; i ++)
      {
        for(int j = 0; j < width; j ++)
        {
          gptar_detail.at(l).at<float>(i, j) = (rand() / double(RAND_MAX)) * (max - min) + min;
        }
      }
    }
    else
    {
      cv::pyrUp(gptar_detail.at(l + 1), gptar_detail.at(l), cv::Size(gptar_detail.at(l).cols, gptar_detail.at(l).rows));
    }
    for (int m = 0; m < height; ++m)
    {
      for (int n = 0; n < width; ++n)
      {
        findBest(gpsrc_detail,gptar_detail,l,n,m,findX,findY);
        gptar_detail.at(l).at<float>(m, n) = gpsrc_detail.at(l).at<float>(findY, findX);
      }
    }
    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}

void SynthesisTool::findBest(ImagePyramid& gpsrc, ImagePyramid& gptar, int level, int pointX, int pointY, int& findX, int& findY)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  int sheight = gpsrc.at(level).rows;
  int swidth = gpsrc.at(level).cols;
  for (int i = NeighborRange[level].height; i < sheight - NeighborRange[level].height; i ++)
  {
    for (int j = NeighborRange[level].width; j < swidth - NeighborRange[level].width; j ++)
    {
      d = this->distNeighborOnDetail(gpsrc, gptar, level, j, i, pointX, pointY);
      if(d < dMin)
      {
        dMin = d;
        findX = j;
        findY = i;
      }
    }
  }
}