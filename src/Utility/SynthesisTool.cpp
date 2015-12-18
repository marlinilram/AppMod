#include "SynthesisTool.h"
#include "MeshParameterization.h"

#include <vector>
#include <cv.h>

void SynthesisTool::init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail)
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
  gpsrc_detail.resize(src_detail.size());
  gptar_detail.resize(src_detail.size());
  for(size_t i = 0; i < src_feature.size(); i ++)
  {
    gpsrc_feature[i].push_back(src_feature[i].clone());
    gptar_feature[i].push_back(tar_feature[i].clone());
  }
  for (size_t i = 0; i < src_detail.size(); ++i)
  {
    gpsrc_detail[i].push_back(src_detail[i].clone());
    gptar_detail[i].push_back(cv::Mat::zeros(src_detail[i].rows, src_detail[i].cols, CV_32FC1));
  }
  for(size_t i = 0; i < gptar_feature.size(); i ++)
  {
    this->generatePyramid(gpsrc_feature[i], levels);
    this->generatePyramid(gptar_feature[i], levels);
  }
  for (size_t i = 0; i < gpsrc_detail.size(); ++i)
  {
    this->generatePyramid(gpsrc_detail[i], levels);
    this->generatePyramid(gptar_detail[i], levels);
  }

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
  
  std::vector<std::set<distance_position> > all_pixel_candidates;
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail[0].at(l).cols;
    int height = gptar_detail[0].at(l).rows;
    int findX;
    int findY;
    int detail_dim = gpsrc_detail.size();
    if(l == levels - 1)
    {
      generateFeatureCandidateForLowestLevel(all_pixel_candidates, gpsrc_feature, gptar_feature);
      for (int k = 0; k < detail_dim; ++k)
      {
        double max, min;
        cv::minMaxLoc(gpsrc_detail[k].at(l),&min,&max);
        srand((unsigned)time(NULL));
        for(int i = 0; i < height; i ++)
        {
          for(int j = 0; j < width; j ++)
          {
            gptar_detail[k].at(l).at<float>(i, j) = (rand() / double(RAND_MAX)) * (max - min) + min;
          }
        }
      }

      for (int i = 0; i < height; ++i)
      {
        for (int j = 0; j < width; ++j)
        {
          int offset = i * width + j;
          // for lowest level we have generated the candidates
          //candidates.clear();
          //FCandidates candidates;
          //this->findCandidates(gpsrc_feature, gptar_feature, l, j, i, candidates);//std::cout <<"found candidates finished. ";
          //std::cout << "The size of the candidates is :" << candidates.size() << std::endl;
          this->findBestMatch(gpsrc_detail, gptar_detail, l, j, i, all_pixel_candidates[offset], findX, findY);//std::cout<<"found best match finished.\n";
          for (int k = 0; k < detail_dim; ++k)
          {
            gptar_detail[k].at(l).at<float>(i, j) = gpsrc_detail[k].at(l).at<float>(findY, findX);
          }
          //std::cout << "Here is OK !" << std::endl;
        }
      }
    }
    else
    {
      for (int k = 0; k < detail_dim; ++k)
      {
        cv::pyrUp(gptar_detail[k].at(l + 1), gptar_detail[k].at(l), cv::Size(gptar_detail[k].at(l).cols, gptar_detail[k].at(l).rows));
      }

      // use candidates computed last time
      ImageFCandidates new_all_pixel_candidates;
      this->generateFeatureCandidateFromLastLevel(new_all_pixel_candidates, all_pixel_candidates, l, gpsrc_feature, gptar_feature);
      all_pixel_candidates.swap(new_all_pixel_candidates);
      for (int i = 0; i < height; ++i)
      {
        for (int j = 0; j < width; ++j)
        {
          int offset = i * width + j;
          //this->getFeatureCandidateFromLowestLevel(candidates, all_pixel_candidates, l, j, i);
          // for lowest level we have generated the candidates
          //candidates.clear();
          //FCandidates candidates;
          //this->findCandidates(gpsrc_feature, gptar_feature, l, j, i, candidates);//std::cout <<"found candidates finished. ";
          //std::cout << "The size of the candidates is :" << candidates.size() << std::endl;
          this->findBestMatch(gpsrc_detail, gptar_detail, l, j, i, all_pixel_candidates[offset], findX, findY);//std::cout<<"found best match finished.\n";
          for (int k = 0; k < detail_dim; ++k)
          {
            gptar_detail[k].at(l).at<float>(i, j) = gpsrc_detail[k].at(l).at<float>(findY, findX);
          }
          //std::cout << "Here is OK !" << std::endl;
        }
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

  // probably not research candidates

  if (candidates.empty())
  {
    int sheight = gpsrc[0].at(level).rows;
    int swidth = gpsrc[0].at(level).cols;
    for (int i = 0; i < sheight - 0; i = i + 1)// should be i = i + 1
    {
      for (int j = 0; j < swidth - 0; j = j + 1)
      {
        d = this->distNeighborOnFeature(gpsrc, gptar, level, j, i, pointX, pointY);

        distance_position dis_pos;
        dis_pos.d = d;
        dis_pos.pos.first = j;
        dis_pos.pos.second = i;
        candidates.insert(dis_pos);
        if(candidates.size() > 50)
        {
          candidates.erase(--candidates.end());
        }
      }
    }
  }

  else
  {
    std::set<distance_position> candidates_last_level = candidates;
    for (auto& i : candidates)
    {
      // search a window centered in this candidates
      //for (int i = 0; i < NeighborRange[level].height; ++i)
      //{
      //  for (int j = 0; j < NeighborRange[level].width; ++j)
      //  {
      //    spy=srcpointYn-NeighborRange[level].height/2+i;
      //    spx=srcpointXn-NeighborRange[level].width/2+j;
      //    tpy=tarpointYn-NeighborRange[level].height/2+i;
      //    tpx=tarpointXn-NeighborRange[level].width/2+j;
      //    if(tpy<0)
      //      tpy+=gptar[0].at(level+1).rows;
      //    if(tpx<0)
      //      tpx+=gptar[0].at(level+1).cols;
      //    if(tpy>=gptar[0].at(level+1).rows)
      //      tpy-=gptar[0].at(level+1).rows;
      //    if(tpx>=gptar[0].at(level+1).cols)
      //      tpx-=gptar[0].at(level+1).cols;

      //    d1 = 0.0;
      //    for (int k = 0; k < ddim; ++k)
      //    {
      //      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
      //    }
      //  }
      //}
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

  // to compute candidate we don't need to compute the neighbor window
  // just point to point distance is OK

  for (int k = 0; k < fdim; ++k)
  {
    //d1 += pow(gpsrc.at(level).at<float>(spy, spx, k) - gptar.at(level).at<float>(tpy, tpx, k), 2);
    d1 += pow(gpsrc[k].at(level).at<float>(srcpointY, srcpointX) - gptar[k].at(level).at<float>(tarpointY,tarpointX), 2);
  }
  d += sqrt(d1);
  return d;

  //int spy, spx, tpy, tpx;
  //for (int i = 0; i < NeighborRange[level].height; ++i)   
  //{
  //  for (int j = 0; j < NeighborRange[level].width; ++j) 
  //  {
  //    spy=srcpointY-NeighborRange[level].height/2+i;
  //    spx=srcpointX-NeighborRange[level].width/2+j;
  //    tpy=tarpointY-NeighborRange[level].height/2+i;
  //    tpx=tarpointX-NeighborRange[level].width/2+j;
  //    if(tpy<0)
  //      tpy+=gptar[0].at(level).rows;
  //    if(tpx<0)
  //      tpx+=gptar[0].at(level).cols;
  //    if(tpy>=gptar[0].at(level).rows)
  //      tpy-=gptar[0].at(level).rows;
  //    if(tpx>=gptar[0].at(level).cols)
  //      tpx-=gptar[0].at(level).cols;

  //    d1 = 0.0;
  //    for (int k = 0; k < fdim; ++k)
  //    {
  //      //d1 += pow(gpsrc.at(level).at<float>(spy, spx, k) - gptar.at(level).at<float>(tpy, tpx, k), 2);
  //      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
  //    }
  //    d += sqrt(d1);
  //  }
  //}


  //return d;
}

void SynthesisTool::findBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position> candidates, int& findX, int& findY)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
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

double SynthesisTool::distNeighborOnDetail(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY)
{
  int ddim = gpsrc.size();
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
      
      if(spy<0)
        spy+=gpsrc[0].at(level).rows;
      if(spx<0)
        spx+=gpsrc[0].at(level).cols;
      if(spy>=gpsrc[0].at(level).rows)
        spy-=gpsrc[0].at(level).rows;
      if(spx>=gpsrc[0].at(level).cols)
        spx-=gpsrc[0].at(level).cols;

      if(tpy<0)
        tpy+=gptar[0].at(level).rows;
      if(tpx<0)
        tpx+=gptar[0].at(level).cols;
      if(tpy>=gptar[0].at(level).rows)
        tpy-=gptar[0].at(level).rows;
      if(tpx>=gptar[0].at(level).cols)
        tpx-=gptar[0].at(level).cols;

      //d1 = 0.0;
      //for (int k = 0; k < fdim; ++k)
      //{
      //  d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
      //}
      for (int k = 0; k < ddim; ++k)
      {
        d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
      }
    }
  }
  d += (d1);

  spy = srcpointY;
  tpy = tarpointY;
  d1 = 0.0;
  for (int j = 0; j < NeighborRange[level].width / 2; ++j)
  {
    spx=srcpointX-NeighborRange[level].width/2+j;
    tpx=tarpointX-NeighborRange[level].width/2+j;
    if(spx<0)
      spx+=gpsrc[0].at(level).cols;
    if(tpx<0)
      tpx+=gptar[0].at(level).cols;

    /*d1 = 0;
    for (int k = 0; k < fdim; ++k)
    {
      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
    }*/
    for (int k = 0; k < ddim; ++k)
    {
      d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tpy,tpx), 2);
    }
  }
  d += (d1);

  if (level < gpsrc[0].size() - 1)
  {
    int srcpointXn = srcpointX/2;
    int srcpointYn = srcpointY/2;
    int tarpointXn = tarpointX/2;
    int tarpointYn = tarpointY/2;
    d1 = 0.0;
    for (int i = 0; i < NeighborRange[level + 1].height; ++i)
    {
      for (int j = 0; j < NeighborRange[level + 1].width; ++j)
      {
        spy=srcpointYn-NeighborRange[level+1].height/2+i;
        spx=srcpointXn-NeighborRange[level+1].width/2+j;
        tpy=tarpointYn-NeighborRange[level+1].height/2+i;
        tpx=tarpointXn-NeighborRange[level+1].width/2+j;

        if(spy<0)
          spy+=gpsrc[0].at(level+1).rows;
        if(spx<0)
          spx+=gpsrc[0].at(level+1).cols;
        if(spy>=gpsrc[0].at(level+1).rows)
          spy-=gpsrc[0].at(level+1).rows;
        if(spx>=gpsrc[0].at(level+1).cols)
          spx-=gpsrc[0].at(level+1).cols;

        if(tpy<0)
          tpy+=gptar[0].at(level+1).rows;
        if(tpx<0)
          tpx+=gptar[0].at(level+1).cols;
        if(tpy>=gptar[0].at(level+1).rows)
          tpy-=gptar[0].at(level+1).rows;
        if(tpx>=gptar[0].at(level+1).cols)
          tpx-=gptar[0].at(level+1).cols;

        for (int k = 0; k < ddim; ++k)
        {
          d1 += pow(gpsrc[k].at(level+1).at<float>(spy, spx) - gptar[k].at(level+1).at<float>(tpy,tpx), 2);
        }
      }
    }
    d += (d1);
  }

  return sqrt(d);
}

void SynthesisTool::doImageSynthesis(std::vector<cv::Mat>& src_detail)
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
  gpsrc_detail.resize(src_detail.size());
  gptar_detail.resize(src_detail.size());
  for (size_t i = 0; i < src_detail.size(); ++i)
  {
    gpsrc_detail[i].push_back(src_detail[i].clone());
    gptar_detail[i].push_back(cv::Mat::zeros(src_detail[i].rows, src_detail[i].cols, CV_32FC1));
  }
  for (size_t i = 0; i < gpsrc_detail.size(); ++i)
  {
    this->generatePyramid(gpsrc_detail[i], levels);
    this->generatePyramid(gptar_detail[i], levels);
  }
  double totalTime = 0.0;
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int detail_dim = gpsrc_detail.size();
    int width = gptar_detail[0].at(l).cols;
    int height = gptar_detail[0].at(l).rows;
    int findX;
    int findY;
    if(l == levels - 1)
    {
      for (int k = 0; k < detail_dim; ++k)
      {
        double max, min;
        cv::minMaxLoc(gpsrc_detail[k].at(l),&min,&max);
        srand((unsigned)time(NULL));
        for(int i = 0; i < height; i ++)
        {
          for(int j = 0; j < width; j ++)
          {
            gptar_detail[k].at(l).at<float>(i, j) = (rand() / double(RAND_MAX)) * (max - min) + min;
          }
        }
      }
    }
    else
    {
      for (int k = 0; k < detail_dim; ++k)
      {
        cv::pyrUp(gptar_detail[k].at(l + 1), gptar_detail[k].at(l), cv::Size(gptar_detail[k].at(l).cols, gptar_detail[k].at(l).rows));
      }
    }
    for (int m = 0; m < height; ++m)
    {
      for (int n = 0; n < width; ++n)
      {
        findBest(gpsrc_detail,gptar_detail,l,n,m,findX,findY);
        for (int k = 0; k < detail_dim; ++k)
        {
          gptar_detail[k].at(l).at<float>(m, n) = gpsrc_detail[k].at(l).at<float>(findY, findX);
        }
      }
    }
    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}

void SynthesisTool::findBest(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, int& findX, int& findY)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
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

void SynthesisTool::generateFeatureCandidateForLowestLevel(std::vector<std::set<distance_position> >& all_pixel_candidates, std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar)
{
  // generate candidate for lowest level
  int height = gptar[0][levels - 1].rows;
  int width  = gptar[0][levels - 1].cols;

  all_pixel_candidates.clear();
  all_pixel_candidates.resize(height * width, std::set<distance_position>());

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      int offset = i * width + j;
      all_pixel_candidates[offset].clear();
      this->findCandidates(gpsrc_feature, gptar_feature, levels - 1, j, i, all_pixel_candidates[offset]);//std::cout <<"found candidates finished. ";
    }
  }
}

void SynthesisTool::getFeatureCandidateFromLowestLevel(std::set<distance_position>& candidates, std::vector<std::set<distance_position> >& all_pixel_candidates, int l, int pointX, int pointY)
{
  int height = gptar_feature[0][levels - 1].rows;
  int width  = gptar_feature[0][levels - 1].cols;

  int scale = pow(2, levels - 1 - l);
  int offset = int(float(pointY) / scale) * width + int(float(pointX) / scale);

  candidates = all_pixel_candidates[offset];
}

void SynthesisTool::generateFeatureCandidateFromLastLevel(ImageFCandidates& new_image_candidates, ImageFCandidates& last_image_candidates, int l, ImagePyramidVec& gpsrc, ImagePyramidVec gptar)
{
  // generate new image candidate from last image candidates
  int height = gptar[0][l].rows;
  int width  = gptar[0][l].cols;
  int last_height = gptar[0][l + 1].rows;
  int last_width  = gptar[0][l + 1].cols;

  new_image_candidates.clear();
  new_image_candidates.resize(height * width, FCandidates());

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      int offset = i * width + j;
      int last_offset = (i / 2) * last_width + (j / 2);
      FCandidates last_candidates = last_image_candidates[last_offset];
      this->findCandidatesFromLastLevel(gpsrc, gptar, l, j, i, last_candidates);
      new_image_candidates[offset] = last_candidates;
    }
  }
}

void SynthesisTool::findCandidatesFromLastLevel(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int tarpointX, int tarpointY, std::set<distance_position>& candidates)
{
  int spy, spx;
  int ddim = gpsrc.size();
  double d1 = 0.0;
  FCandidates new_candidates;
  for (auto& i_candidates : candidates)
  {
    distance_position expanded;
    expanded.d = i_candidates.d;
    expanded.pos = i_candidates.pos;
    expanded.pos.first = 2 * expanded.pos.first - 1;
    expanded.pos.second = 2 * expanded.pos.second - 1;

    // search neighbor
    for (int i = 0; i < NeighborRange[level].height; ++i)   
    {
      for (int j = 0; j < NeighborRange[level].width; ++j) 
      {
        spy=expanded.pos.second-NeighborRange[level].height/2+i;
        spx=expanded.pos.first-NeighborRange[level].width/2+j;
        if(spy<0)
          spy+=gpsrc[0].at(level).rows;
        if(spx<0)
          spx+=gpsrc[0].at(level).cols;
        if(spy>=gpsrc[0].at(level).rows)
          spy-=gpsrc[0].at(level).rows;
        if(spx>=gpsrc[0].at(level).cols)
          spx-=gpsrc[0].at(level).cols;

        d1 = 0.0;
        for (int k = 0; k < ddim; ++k)
        {
          d1 += pow(gpsrc[k].at(level).at<float>(spy, spx) - gptar[k].at(level).at<float>(tarpointY,tarpointX), 2);
        }

        distance_position dis_pos;
        dis_pos.d = sqrt(d1);
        dis_pos.pos.first = spx;
        dis_pos.pos.second = spy;
        new_candidates.insert(dis_pos);
        if(new_candidates.size() > 50)
        {
          new_candidates.erase(--new_candidates.end());
        }
      }
    }
  }
  candidates = new_candidates;
}