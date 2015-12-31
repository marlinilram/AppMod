#include "SynthesisTool.h"
#include "MeshParameterization.h"

#include "KDTreeWrapper.h"

#include <vector>
#include <cv.h>

SynthesisTool::SynthesisTool()
{
  levels = 5;
  NeighborRange.resize(5);
  NeighborRange[0].height = 9;
  NeighborRange[0].width = 9;
  NeighborRange[1].height = 9;
  NeighborRange[1].width = 9;
  NeighborRange[2].height = 9;
  NeighborRange[2].width = 9;
  NeighborRange[3].height = 9;
  NeighborRange[3].width = 9;
  NeighborRange[4].height = 9;
  NeighborRange[4].width = 9;

  candidate_size = 20;
  best_random_size = 5;
  patch_size = 10;
  bias_rate = 0.1;
  lamd_occ = 0.0;
  max_iter = 5;
}

void SynthesisTool::init(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail)
{
  // pyramids stored from up to down
  // [0] ------
  // [1]  ----
  // [2]   --

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

  //this->buildAllFeatureButkects(gpsrc_feature, gpsrc_feature_buckets);

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
  py_scale = std::pow(float(std::min(pyr[0].cols, pyr[0].rows)) / 70, 1.0 / float(level - 1)); // minimal size 35 * 35
  for (int i = 1; i < level; ++i)
  {
    cv::Mat dst;
    cv::Size d_size(float(pyr[i - 1].cols) / py_scale, float(pyr[i - 1].rows) / py_scale);
    //cv::pyrDown(pyr[i - 1], dst, d_size);
    cv::resize(pyr[i - 1], dst, d_size);
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
      this->initializeTarDetail(gptar_detail, l);

      FCandidates candidates;
      FCandidates best_match;
      all_pixel_candidates.clear();
      all_pixel_candidates.resize(width * height);
      //generateFeatureCandidateForLowestLevel(all_pixel_candidates, gpsrc_feature, gptar_feature);
      std::vector<float> reference_cnt(width * height, 0.0);
      for (int i = 0; i < height; ++i)
      {
        for (int j = 0; j < width; ++j)
        {
          int offset = i * width + j;
          // for lowest level we have generated the candidates
          candidates.clear();
          best_match.clear();
          //this->findCandidates(gpsrc_feature, gptar_feature, l, j, i, candidates);//std::cout <<"found candidates finished. ";
          //std::cout << "The size of the candidates is :" << candidates.size() << std::endl;
          //this->findCandidatesInBuckets(gpsrc_feature_buckets, gptar_feature, l, j, i, candidates);
          //this->findCandidatesWithRefCount(gpsrc_feature, gptar_feature, reference_cnt, l, j, i, candidates);
          //this->findBestMatchWithRefCount(gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates, best_match);//std::cout<<"found best match finished.\n";
          this->findCombineCandidates(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates);
          this->getValFromBestMatch(gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates);
          //for (int k = 0; k < detail_dim; ++k)
          //{
          //  gptar_detail[k].at(l).at<float>(i, j) = gpsrc_detail[k].at(l).at<float>(findY, findX);
          //}
          // add ref_cnt
          //reference_cnt[findY * width + findX] += 1.0;
          all_pixel_candidates[offset].swap(candidates);
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
      FCandidates candidates;
      FCandidates best_match;
      ImageFCandidates new_all_pixel_candidates(width * height);
      std::vector<float> reference_cnt(width * height, 0.0);
      //this->generateFeatureCandidateFromLastLevel(new_all_pixel_candidates, all_pixel_candidates, l, gpsrc_feature, gptar_feature);
      //all_pixel_candidates.swap(new_all_pixel_candidates);
      for (int i = 0; i < height; ++i)
      {
        for (int j = 0; j < width; ++j)
        {
          int offset = i * width + j;
          //this->getFeatureCandidateFromLowestLevel(candidates, all_pixel_candidates, l, j, i);
          // for lowest level we have generated the candidates
          candidates.clear();
          best_match.clear();
          //FCandidates candidates;
          //this->findCandidates(gpsrc_feature, gptar_feature, l, j, i, candidates);//std::cout <<"found candidates finished. ";
          //std::cout << "The size of the candidates is :" << candidates.size() << std::endl;
          //this->findCandidatesInBuckets(gpsrc_feature_buckets, gptar_feature, l, j, i, candidates);
          this->getLastLevelCandidate(gptar_feature, all_pixel_candidates, l, j, i, candidates);
          //this->findCandidatesFromLastLevelWithRefCount(gpsrc_feature, gptar_feature, reference_cnt, l, j, i, candidates);
          //this->findBestMatchWithRefCount(gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates, best_match);//std::cout<<"found best match finished.\n";
          this->findCombineCandidatesFromLastLevel(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates);
          this->getValFromBestMatch(gpsrc_detail, gptar_detail, reference_cnt, l, j, i, candidates);
          //for (int k = 0; k < detail_dim; ++k)
          //{
          //  gptar_detail[k].at(l).at<float>(i, j) = gpsrc_detail[k].at(l).at<float>(findY, findX);
          //}
          //reference_cnt[findY * width + findX] += 1.0;
          new_all_pixel_candidates[offset].swap(candidates);
          //std::cout << "Here is OK !" << std::endl;
        }
      }
      all_pixel_candidates.swap(new_all_pixel_candidates);
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
        if(candidates.size() > candidate_size)
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
  d += d1 / fdim;
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

void SynthesisTool::findBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates, int& findX, int& findY)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  std::set<distance_position> best_set;
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
  for(std::set<distance_position>::iterator iter = candidates.begin(); iter != candidates.end(); iter ++)
  {
    int srcX,srcY;
    srcX = (*iter).pos.first;
    srcY = (*iter).pos.second;
    d = this->distNeighborOnDetail(gpsrc, gptar, level, srcX, srcY, pointX, pointY);

    best_set.insert(distance_position(d, Point2D(srcX, srcY)));
    if(best_set.size() > best_random_size)
    {
      best_set.erase(--best_set.end());
    }

    //if (d < dMin)
    //{
    //  dMin = d;
    //  findY = srcY;
    //  findX = srcX;
    //}
  }

  int choose = int((rand() / double(RAND_MAX)) * best_random_size);
  choose = choose >= best_random_size ? (best_random_size - 1) : choose;
  std::set<distance_position>::const_iterator iter = best_set.begin();
  std::advance(iter, choose);
  findX = iter->pos.first;
  findY = iter->pos.second;
}

double SynthesisTool::distNeighborOnDetail(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, int level, int srcpointX, int srcpointY, int tarpointX, int tarpointY)
{
  int ddim = gpsrc.size();
  double d = 0;
  double d1 = 0;
  int spy, spx, tpy, tpx;
  int pixel_cnt = 0;
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
      ++pixel_cnt;
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
    ++pixel_cnt;
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
        ++pixel_cnt;
      }
    }
    d += (d1);
  }

  return (d) / (pixel_cnt * ddim);
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
        if(new_candidates.size() > candidate_size)
        {
          new_candidates.erase(--new_candidates.end());
        }
      }
    }
  }
  candidates = new_candidates;
}

void SynthesisTool::buildAllFeatureButkects(std::vector<ImagePyramid>& gpsrc, std::vector<FBucketPryamid>& gpsrc_buckets)
{
  // build feature buckets for each feature dimension
  gpsrc_buckets.clear();
  gpsrc_buckets.resize(gpsrc.size());
  for (size_t i = 0; i < gpsrc.size(); ++i)
  {
    buildPryFeatureBuckets(gpsrc[i], gpsrc_buckets[i]); // for each dimension
  }
}

void SynthesisTool::buildPryFeatureBuckets(ImagePyramid& gpsrc, FBucketPryamid& gpsrc_buckets)
{
  // build feature buckets for one pyramid
  gpsrc_buckets.clear();
  gpsrc_buckets.resize(gpsrc.size());
  for (size_t i = 0; i < gpsrc.size(); ++i)
  {
    buildImgFeatureBuckets(gpsrc[i], gpsrc_buckets[i]);
  }
}

void SynthesisTool::buildImgFeatureBuckets(cv::Mat& img, FBucket& buket)
{
  // build bucket for one image
  int n_bin = 10;
  buket.clear();
  buket.resize(n_bin); // assume the value has been normalized to 0~1 and we use a 10 bins bucket
  float* mat_ptr = (float*)img.data;
  for (int i = 0; i < img.rows; ++i)
  {
    for (int j = 0; j < img.cols; ++j)
    {
      int offset = i * img.cols + j;
      int bin_id = int(mat_ptr[offset] * n_bin); // assume normalized to 0~1 already !!!
      if (bin_id == n_bin) bin_id = bin_id - 1;
      buket[bin_id].insert(Point2D(j, i)); // store as (x,y) not (i,j)
    }
  }
}

void SynthesisTool::findCandidatesInBuckets(std::vector<FBucketPryamid>& gpsrc_buckets, std::vector<ImagePyramid>& gptar, int level, int pointX, int pointY, std::set<distance_position>& candidates)
{
  // pointX and pointY indicate a position in certain level of gptar (feature map)
  // we try to find all candidates in the same level of gpsrc (feature map)
  int n_bin = (int)gpsrc_buckets[0][0].size();
  std::set<Point2D> bucket_candidates;
  this->getElementsFromBuckets(gpsrc_buckets[0][level], bucket_candidates, gptar[0][level].at<float>(pointY, pointX));
  for (size_t i = 1; i < gpsrc_buckets.size(); ++i) // for each feature dimension
  {
    std::set<Point2D> cur_f_candidates;
    this->getElementsFromBuckets(gpsrc_buckets[i][level], cur_f_candidates, gptar[i][level].at<float>(pointY, pointX));

    std::set<Point2D> bucket_intersection;
    std::set_intersection(cur_f_candidates.begin(), cur_f_candidates.end(), bucket_candidates.begin(), bucket_candidates.end(), std::inserter(bucket_intersection, bucket_intersection.begin()));
    if (!bucket_intersection.empty())
    {
      bucket_candidates.swap(bucket_intersection);
    }
  }

  // now we have the candidates
  for (auto i : bucket_candidates)
  {
    candidates.insert(distance_position(0, i));
  }
}

void SynthesisTool::getElementsFromBuckets(FBucket& bucket, std::set<Point2D>& elements, float val)
{
  int n_bin = (int)bucket.size();
  float bin_val = n_bin * val;
  int bin_id = int(bin_val); // target feature in dimension i
  bin_id = (bin_id >= n_bin) ? (n_bin - 1) : bin_id;
  int bin_id_n = int(bin_val + 0.5 - int(bin_val)) == 0 ? (bin_id - 1) : (bin_id + 1);
  bin_id_n  = (bin_id_n < 0) ? 0 : ((bin_id_n >= n_bin) ? (n_bin - 1) : bin_id_n); // get a next bin

  elements = bucket[bin_id];
  elements.insert(bucket[bin_id_n].begin(), bucket[bin_id_n].end());
}

void SynthesisTool::findCandidatesWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int pointX, int pointY, std::set<distance_position>& candidates)
{
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

        // here we need to consider the reference count
        d = sqrt(d * d + pow(ref_cnt[i * swidth + j], 2));
        candidates.insert(distance_position(d, Point2D(j, i)));
        if(candidates.size() > candidate_size)
        {
          candidates.erase(--candidates.end());
        }
      }
    }
  }
}

void SynthesisTool::findCandidatesFromLastLevelWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& candidates)
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
    expanded.pos.first = 2 * expanded.pos.first - 1; // the position is from last level
    expanded.pos.second = 2 * expanded.pos.second - 1; // need to expand

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

        d1 = sqrt(d1 + pow(ref_cnt[tarpointY * gptar[0].at(level).cols + tarpointX], 2));

        new_candidates.insert(distance_position(d1, Point2D(spx, spy)));
        if(new_candidates.size() > candidate_size)
        {
          new_candidates.erase(--new_candidates.end());
        }
      }
    }
  }
  candidates.swap(new_candidates);
}

void SynthesisTool::getLastLevelCandidate(std::vector<ImagePyramid>& gptar, ImageFCandidates& last_image_candidates, int level, int pointX, int pointY, std::set<distance_position>& candidates)
{
  int height = gptar[0][level].rows;
  int width  = gptar[0][level].cols;
  int last_height = gptar[0][level + 1].rows;
  int last_width  = gptar[0][level + 1].cols;

  int offset = pointY * width + pointX;
  int last_offset = (pointY / 2) * last_width + (pointX / 2);
  candidates = last_image_candidates[last_offset];
}

void SynthesisTool::findBestMatchWithRefCount(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int pointX, int pointY, std::set<distance_position>& candidates, std::set<distance_position>& best_match)
{
  double d = 0;
  double dMin = std::numeric_limits<double>::max();
  std::set<distance_position> best_set;
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
  for(std::set<distance_position>::iterator iter = candidates.begin(); iter != candidates.end(); iter ++)
  {
    int srcX,srcY;
    srcX = (*iter).pos.first;
    srcY = (*iter).pos.second;
    d = this->distNeighborOnDetail(gpsrc, gptar, level, srcX, srcY, pointX, pointY);
    d = sqrt(d * d + pow(ref_cnt[srcY * swidth + srcX], 2));

    best_set.insert(distance_position(d, Point2D(srcX, srcY)));
    if(best_set.size() > best_random_size)
    {
      best_set.erase(--best_set.end());
    }

    //if (d < dMin)
    //{
    //  dMin = d;
    //  findY = srcY;
    //  findX = srcX;
    //}
  }

  //int choose = int((rand() / double(RAND_MAX)) * best_random_size);
  //choose = choose >= best_random_size ? (best_random_size - 1) : choose;
  //std::set<distance_position>::const_iterator iter = best_set.begin();
  //std::advance(iter, choose);
  //findX = iter->pos.first;
  //findY = iter->pos.second;
  best_match.swap(best_set);
}

void SynthesisTool::getValFromBestMatch(std::vector<ImagePyramid>& gpsrc, std::vector<ImagePyramid>& gptar, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match)
{
  int detail_dim = (int)gpsrc.size();
  int sheight = gpsrc[0].at(level).rows;
  int swidth = gpsrc[0].at(level).cols;
  for (auto i : best_match)
  {
    for (int k = 0; k < detail_dim; ++k)
    {
      gptar[k].at(level).at<float>(tarpointY, tarpointX) = 0.0;
    }

    for (int k = 0; k < detail_dim; ++k)
    {
      gptar[k].at(level).at<float>(tarpointY, tarpointX) += (gpsrc[k].at(level).at<float>(i.pos.second, i.pos.first));// / best_match.size();
    }

    //ref_cnt[i.pos.second * swidth + i.pos.first] += 1.0 / best_match.size();
  }
}

void SynthesisTool::findCombineCandidates(std::vector<ImagePyramid>& gpsrc_f, std::vector<ImagePyramid>& gptar_f, std::vector<ImagePyramid>& gpsrc_d, std::vector<ImagePyramid>& gptar_d, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match)
{
  double d1 = 0.0;
  double d2 = 0.0;
  double d = 0.0;
  double lambda_d1 = 0;
  double lambda_d2 = 0;
  int sheight = gpsrc_f[0].at(level).rows;
  int swidth = gpsrc_f[0].at(level).cols;
  for (int i = 0; i < sheight - 0; i = i + 1)// should be i = i + 1
  {
    for (int j = 0; j < swidth - 0; j = j + 1)
    {
      d1 = this->distNeighborOnFeature(gpsrc_f, gptar_f, level, j, i, tarpointX, tarpointY);
      d2 = this->distNeighborOnDetail(gpsrc_d, gptar_d, level, j, i, tarpointX, tarpointY);

      // here we need to consider the reference count
      if (d1 < 0.001)  lambda_d1 = 1, lambda_d2 = 0;
      else  lambda_d1 = 0, lambda_d2 = 1;
      d = lambda_d1 * d1 + lambda_d2 * d2 + pow(ref_cnt[i * swidth + j], 2);
      best_match.insert(distance_position(d, Point2D(j, i)));
      if(best_match.size() > candidate_size)
      {
        best_match.erase(--best_match.end());
      }
    }
  }
}

void SynthesisTool::findCombineCandidatesFromLastLevel(std::vector<ImagePyramid>& gpsrc_f, std::vector<ImagePyramid>& gptar_f, std::vector<ImagePyramid>& gpsrc_d, std::vector<ImagePyramid>& gptar_d, std::vector<float>& ref_cnt, int level, int tarpointX, int tarpointY, std::set<distance_position>& best_match)
{
  int spy, spx;
  int ddim = gpsrc_d.size();
  int fdim = gpsrc_f.size();
  double d1 = 0.0;
  double d2 = 0.0;
  double d = 0.0;
  double lambda_d1 = 0;
  double lambda_d2 = 0;
  FCandidates new_candidates;
  for (auto& i_candidates : best_match)
  {
    distance_position expanded;
    expanded.d = i_candidates.d;
    expanded.pos = i_candidates.pos;
    expanded.pos.first = 2 * expanded.pos.first - 1; // the position is from last level
    expanded.pos.second = 2 * expanded.pos.second - 1; // need to expand

    // search neighbor
    for (int i = 0; i < NeighborRange[level].height; ++i)   
    {
      for (int j = 0; j < NeighborRange[level].width; ++j) 
      {
        spy=expanded.pos.second-NeighborRange[level].height/2+i;
        spx=expanded.pos.first-NeighborRange[level].width/2+j;
        if(spy<0)
          spy+=gpsrc_d[0].at(level).rows;
        if(spx<0)
          spx+=gpsrc_d[0].at(level).cols;
        if(spy>=gpsrc_d[0].at(level).rows)
          spy-=gpsrc_d[0].at(level).rows;
        if(spx>=gpsrc_d[0].at(level).cols)
          spx-=gpsrc_d[0].at(level).cols;

        d1 = this->distNeighborOnFeature(gpsrc_f, gptar_f, level, spx, spy, tarpointX, tarpointY);
        d2 = this->distNeighborOnDetail(gpsrc_d, gptar_d, level, spx, spy, tarpointX, tarpointY);

        if (d1 < 0.001)  lambda_d1 = 1, lambda_d2 = 0;
        else  lambda_d1 = 0, lambda_d2 = 1;
        d = lambda_d1 * d1 + lambda_d2 * d2 + pow(ref_cnt[i * gpsrc_d[0].at(level).cols + j], 2);
        new_candidates.insert(distance_position(d1, Point2D(spx, spy)));
        if(new_candidates.size() > candidate_size)
        {
          new_candidates.erase(--new_candidates.end());
        }
      }
    }
  }
  best_match.swap(new_candidates);
}

void SynthesisTool::doSynthesisNew()
{
  // synthesis based on constrained patch match

  // find best match for each level
  double totalTime = 0.0;
  srand((unsigned)time(NULL));
  for(int l = 0; l < levels; l ++)
  {
    std::vector<int> source_patch_mask_l;
    this->buildSourcePatchMask(gpsrc_detail[0].at(l), source_patch_mask_l);
    src_patch_mask.push_back(source_patch_mask_l);
  }
  std::cout << "OK1!!!!\n";
  std::vector<Point2D> nnf; // Point2D stores the nearest patch offset according to current pos
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail[0].at(l).cols;
    int height = gptar_detail[0].at(l).rows;


    this->exportSrcFeature(gpsrc_feature, l);
    this->exportTarFeature(gptar_feature, l);
    this->exportSrcDetail(gpsrc_detail, l, 0);



    if(l == levels - 1)
    {
      this->initializeNNF(gptar_detail[0], nnf, l);
      this->initializeTarDetail(gptar_detail, l);
      for (int i_iter = 0; i_iter < max_iter; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        //if (i_iter % 2 == 0)
        {
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l, 0);
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l, 1);
        }
        //else
        //{
          //this->updateNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        //}
        this->voteImage(gpsrc_detail, gptar_detail, nnf, l);
        this->exportNNF(nnf, gptar_feature, l, i_iter);
        this->exportTarDetail(gptar_detail, l, i_iter);
      }
    }
    else
    {

      for (int k = 0; k < gptar_detail.size(); ++k)
      {
        //cv::pyrUp(gptar_detail[k].at(l + 1), gptar_detail[k].at(l), cv::Size(gptar_detail[k].at(l).cols, gptar_detail[k].at(l).rows));
        cv::resize(gptar_detail[k].at(l + 1), gptar_detail[k].at(l), cv::Size(gptar_detail[k].at(l).cols, gptar_detail[k].at(l).rows));
      }

      std::vector<Point2D> nnf_new;
      this->initializeNNFFromLastLevel(gptar_detail[0], nnf, l, nnf_new);
      nnf.swap(nnf_new);
      for (int i_iter = 0; i_iter < max_iter; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        //if (i_iter % 2 == 0)
        {
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l, 0);
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l, 1);
        }
        //else
        //{
        //  this->updateNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        //}
        this->voteImage(gpsrc_detail, gptar_detail, nnf, l);
        this->exportNNF(nnf, gptar_feature, l, i_iter);
        this->exportTarDetail(gptar_detail, l, i_iter);
      }
    }

    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}

void SynthesisTool::initializeNNF(ImagePyramid& gptar_d, NNF& nnf, int level)
{
  // first check if the level is the coarsest level
  if (level != (gptar_d.size() - 1))
  {
    std::cout << "Level of initialization error." << std::endl;
    return;
  }

  // allocate for nnf, based on the patch size and the image size
  int height = gptar_d[level].rows;
  int width  = gptar_d[level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);
  nnf.clear();

  // initialize the nnf with random position
  this->getRandomPosition(level, nnf, nnf_height * nnf_width, nnf_height, nnf_width);
}

void SynthesisTool::initializeNNFFromLastLevel(ImagePyramid& gptar_d, NNF& nnf_last, int level, NNF& nnf_new)
{
  int height = gptar_d[level].rows;
  int width  = gptar_d[level].cols;
  int last_height = gptar_d[level + 1].rows;
  int last_width  = gptar_d[level + 1].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);
  int last_nnf_height = (last_height - this->patch_size + 1);
  int last_nnf_width = (last_width - this->patch_size + 1);
  nnf_new.clear();
  nnf_new.resize(nnf_height * nnf_width);
  float scale = float(height) / last_height;

  std::vector<float> mask_kd_data;
  for (int i = 0; i < nnf_height; ++i)
  {
    for (int j = 0; j < nnf_width; ++j)
    {
      if (src_patch_mask[level][i  *nnf_width + j] == 0)
      {
        mask_kd_data.push_back(j);
        mask_kd_data.push_back(i);
      }
    }
  }
  KDTreeWrapper mask_kd;
  mask_kd.initKDTree(mask_kd_data, mask_kd_data.size() / 2, 2);

  for (int i = 0; i < nnf_height; ++i)
  {
    for (int j = 0; j < nnf_width; ++j)
    {
      int i_last = (i / scale);
      int j_last = (j / scale);
      i_last = (i_last >= last_nnf_height) ? (last_nnf_height - 1) : i_last;
      j_last = (j_last >= last_nnf_width) ? (last_nnf_width - 1) : j_last;
      Point2D patch = nnf_last[i_last * last_nnf_width + j_last];
      patch.first = scale * (patch.first);
      patch.second = scale * (patch.second);
      patch.first = (patch.first >= nnf_width) ? (nnf_width - 1) : patch.first;
      patch.second = (patch.second >= nnf_height) ? (nnf_height - 1) : patch.second;
      if(src_patch_mask[level][patch.second * nnf_width + patch.first] == 0)
        nnf_new[i * nnf_width + j] = patch;
      else
      {
        std::vector<float> query(2, 0);
        query[0] = patch.first;
        query[1] = patch.second;
        mask_kd.nearestPt(query);
        nnf_new[i * nnf_width + j] = Point2D(query[0], query[1]);
      }
    }
  }
}

void SynthesisTool::initializeTarDetail(ImagePyramidVec& gptar_d, int level)
{
  // first check if the level is the coarsest level
  if (level != (gptar_d[0].size() - 1))
  {
    std::cout << "Level of initialization error." << std::endl;
    return;
  }

  // iterate the cv::Mat
  int ddim = gptar_d.size();
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      for (int k = 0; k < ddim; ++k)
      {
        gptar_d[k][level].at<float>(i, j) = (rand() / double(RAND_MAX));
      }
    }
  }
}

void SynthesisTool::getRandomPosition(int l, std::vector<Point2D>& random_set, int n_set, int max_height, int max_width, int min_height, int min_width)
{
  random_set.clear();
  random_set.resize(n_set);
  int x, y;
  for (int i = 0; i < n_set; ++i)
  {
    do{
    x = (rand() / double(RAND_MAX)) * (max_width - min_width) + min_width;
    y = (rand() / double(RAND_MAX)) * (max_height - min_height) + min_height;

    // clamp if the random value is 1
    x = (x >= max_width) ? (max_width - 1) : x;
    y = (y >= max_height) ? (max_height - 1) : y;
    }while(src_patch_mask[l][y * max_width + x] == 1);
    random_set[i] = Point2D(x, y);
  }
}

void SynthesisTool::updateNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
  ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
  NNF& nnf, std::vector<float>& ref_cnt, int level, int iter)
{
  // update nnf based on current nnf, random position and position nearby
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);

  //// deal with the left upper corner
  //{
  //  std::vector<Point2D> rand_pos;
  //  this->getRandomPosition(rand_pos, best_random_size, nnf_height, nnf_width);
  //  int offset = 0 * nnf_width + 0;
  //  Point2D best_patch;
  //  rand_pos.push_back(nnf[offset]);
  //  this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(0, 0), rand_pos, best_patch);
  //  nnf[offset] = best_patch;
  //  this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  //}

  //// deal with first row
  //for (int j = 1; j < nnf_width; ++j)
  //{
  //  std::vector<Point2D> rand_pos;
  //  int offset = 0 * nnf_width + j;
  //  this->getRandomPosition(rand_pos, best_random_size, nnf_height, nnf_width);
  //  Point2D best_rand;
  //  double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, 0), rand_pos, best_rand);
  //  Point2D left_nnf = nnf[j - 1];
  //  left_nnf.first = (left_nnf.first + 1) >= nnf_width ? (nnf_width - 1) : (left_nnf.first + 1);
  //  rand_pos.clear();
  //  rand_pos.push_back(left_nnf);
  //  rand_pos.push_back(nnf[offset]);
  //  Point2D best_bias;
  //  double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, 0), rand_pos, best_bias);
  //  if (d_best_rand < (bias_rate * d_best_bias))
  //  {
  //    nnf[offset] = best_rand;
  //  }
  //  else
  //  {
  //    nnf[offset] = best_bias;
  //  }
  //  this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  //}

  //// deal with first col
  //for (int i = 1; i < nnf_height; ++i)
  //{
  //  std::vector<Point2D> rand_pos;
  //  int offset = i * nnf_width + 0;
  //  this->getRandomPosition(rand_pos, best_random_size, nnf_height, nnf_width);
  //  Point2D best_rand;
  //  double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(0, i), rand_pos, best_rand);
  //  Point2D up_nnf = nnf[(i - 1) * nnf_width];
  //  up_nnf.second = (up_nnf.second + 1) >= nnf_height ? (nnf_height - 1) : (up_nnf.second + 1);
  //  rand_pos.clear();
  //  rand_pos.push_back(up_nnf);
  //  rand_pos.push_back(nnf[offset]);
  //  Point2D best_bias;
  //  double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(0, i), rand_pos, best_bias);
  //  if (d_best_rand < (bias_rate * d_best_bias))
  //  {
  //    nnf[offset] = best_rand;
  //  }
  //  else
  //  {
  //    nnf[offset] = best_bias;
  //  }
  //  this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  //}

  int istart = 0, iend = nnf_height, ichange = 1;
  int jstart = 0, jend = nnf_width, jchange = 1;
  if (iter % 2 == 1)
  {
    istart = iend - 1; iend = -1; ichange = -1;
    jstart = jend - 1; jend = -1; jchange = -1;
  }


  // deal with all pixels left
  for (int i = istart; i != iend; i += ichange)
  {
    for (int j = jstart; j != jend; j += jchange)
    {
      std::vector<Point2D> rand_pos;
      int offset = i * nnf_width + j;
      
      // random search
      this->getRandomPosition(level, rand_pos, best_random_size, nnf_height, nnf_width);
      Point2D best_rand;
      double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_rand);
      
      // propagation
      rand_pos.clear();
      // left
      if ((unsigned)(j - jchange) < (unsigned)nnf_width)
      {
        Point2D left_nnf = nnf[i * nnf_width + j - jchange];
        if ((unsigned)(left_nnf.first + jchange) < unsigned(nnf_width))
        {
          left_nnf.first = left_nnf.first + jchange;
          if(this->validPatchWithMask(left_nnf, src_patch_mask[level], nnf_height, nnf_width))
            rand_pos.push_back(left_nnf);
        }
      }
      // up
      if ((unsigned)(i - ichange) < (unsigned)nnf_height)
      {
        Point2D up_nnf = nnf[(i - ichange) * nnf_width + j];
        if ((unsigned)(up_nnf.second + ichange) < unsigned(nnf_height))
        {
          up_nnf.second = (up_nnf.second + ichange);
          if(this->validPatchWithMask(up_nnf, src_patch_mask[level], nnf_height, nnf_width))
            rand_pos.push_back(up_nnf);
        }
      }
      Point2D best_bias;
      rand_pos.push_back(nnf[offset]);
      double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_bias);

      if (d_best_rand < (bias_rate * d_best_bias))
      {
        nnf[offset] = best_rand;
      }
      else
      {
        nnf[offset] = best_bias;
      }
      this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
    }
  }
}

void SynthesisTool::updateNNFWithMask(std::vector<int>& source_patch_mask, ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
                 ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
                 NNF& nnf, std::vector<float>& ref_cnt, int level, int iter)
{
  //// update nnf based on current nnf, random position and position nearby
  //int height = gptar_d[0][level].rows;
  //int width  = gptar_d[0][level].cols;
  //int nnf_height = (height - this->patch_size + 1);
  //int nnf_width  = (width - this->patch_size + 1);

  //
  //int istart = 0, iend = nnf_height, ichange = 1;
  //int jstart = 0, jend = nnf_width, jchange = 1;
  //if (iter % 2 == 1)
  //{
  //  istart = iend - 1; iend = -1; ichange = -1;
  //  jstart = jend - 1; jend = -1; jchange = -1;
  //}


  //// deal with all pixels left
  //for (int i = istart; i != iend; i += ichange)
  //{
  //  for (int j = jstart; j != jend; j += jchange)
  //  {
  //    std::vector<Point2D> rand_pos;
  //    int offset = i * nnf_width + j;
  //    
  //    // random search
  //    this->getRandomPosition(rand_pos, best_random_size, nnf_height, nnf_width);
  //    Point2D best_rand;
  //    double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_rand);
  //    
  //    // propagation
  //    rand_pos.clear();
  //    // left
  //    if ((unsigned)(j - jchange) < (unsigned)nnf_width)
  //    {
  //      Point2D left_nnf = nnf[i * nnf_width + j - jchange];
  //      if ((unsigned)(left_nnf.first + jchange) < unsigned(nnf_width))
  //      {
  //        left_nnf.first = left_nnf.first + jchange;
  //        rand_pos.push_back(left_nnf);
  //      }
  //    }
  //    // up
  //    if ((unsigned)(i - ichange) < (unsigned)nnf_height)
  //    {
  //      Point2D up_nnf = nnf[(i - ichange) * nnf_width + j];
  //      if ((unsigned)(up_nnf.second + ichange) < unsigned(nnf_height))
  //      {
  //        up_nnf.second = (up_nnf.second + ichange);
  //        rand_pos.push_back(up_nnf);
  //      }
  //    }
  //    Point2D best_bias;
  //    rand_pos.push_back(nnf[offset]);
  //    double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_bias);

  //    if (d_best_rand < (bias_rate * d_best_bias))
  //    {
  //      nnf[offset] = best_rand;
  //    }
  //    else
  //    {
  //      nnf[offset] = best_bias;
  //    }
  //    this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  //  }
  //}
}

void SynthesisTool::updateNNFReverse(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f, ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, std::vector<float>& ref_cnt, int level)
{
  // update nnf based on current nnf, random position and position nearby
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);

  // deal with the right bottom corner
  {
    std::vector<Point2D> rand_pos;
    this->getRandomPosition(level, rand_pos, best_random_size, nnf_height, nnf_width); // TODO: random sample in a more local region
    int offset = (nnf_height - 1) * nnf_width + (nnf_width - 1);
    Point2D best_patch;
    rand_pos.push_back(nnf[offset]);
    this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(nnf_width - 1, nnf_height - 1), rand_pos, best_patch);
    nnf[offset] = best_patch;
    this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  }

  // deal with first row
  for (int j = nnf_width - 2; j >= 0; --j)
  {
    std::vector<Point2D> rand_pos;
    int offset = (nnf_height - 1) * nnf_width + j;
    this->getRandomPosition(level, rand_pos, best_random_size, nnf_height, nnf_width);
    Point2D best_rand;
    double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, nnf_height - 1), rand_pos, best_rand);
    Point2D right_nnf = nnf[(nnf_height - 1) * nnf_width + j + 1];
    right_nnf.first = (right_nnf.first - 1) < 0 ? 0 : (right_nnf.first - 1);
    rand_pos.clear();
    rand_pos.push_back(right_nnf);
    rand_pos.push_back(nnf[offset]);
    Point2D best_bias;
    double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, nnf_height - 1), rand_pos, best_bias);
    if (d_best_rand < (bias_rate * d_best_bias))
    {
      nnf[offset] = best_rand;
    }
    else
    {
      nnf[offset] = best_bias;
    }
    this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  }

  // deal with first col
  for (int i = nnf_height - 2; i >= 0; --i)
  {
    std::vector<Point2D> rand_pos;
    int offset = i * nnf_width + (nnf_width - 1);
    this->getRandomPosition(level, rand_pos, best_random_size, nnf_height, nnf_width);
    Point2D best_rand;
    double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(nnf_width - 1, i), rand_pos, best_rand);
    Point2D down_nnf = nnf[(i + 1) * nnf_width + nnf_width - 1];
    down_nnf.second = (down_nnf.second - 1) < 0 ? 0 : (down_nnf.second - 1);
    rand_pos.clear();
    rand_pos.push_back(down_nnf);
    rand_pos.push_back(nnf[offset]);
    Point2D best_bias;
    double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(nnf_width - 1, i), rand_pos, best_bias);
    if (d_best_rand < (bias_rate * d_best_bias))
    {
      nnf[offset] = best_rand;
    }
    else
    {
      nnf[offset] = best_bias;
    }
    this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
  }

  // deal with all pixels left
  for (int i = nnf_height - 2; i >= 0; --i)
  {
    for (int j = nnf_width - 2; j >= 0; --j)
    {
      std::vector<Point2D> rand_pos;
      int offset = i * nnf_width + j;
      this->getRandomPosition(level, rand_pos, best_random_size, nnf_height, nnf_width);
      Point2D best_rand;
      double d_best_rand = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_rand);
      Point2D right_nnf = nnf[i * nnf_width + j + 1];
      right_nnf.first = (right_nnf.first - 1) < 0 ? 0 : (right_nnf.first - 1);
      Point2D down_nnf = nnf[(i + 1) * nnf_width + j];
      down_nnf.second = (down_nnf.second - 1) < 0 ? 0 : (down_nnf.second - 1);
      rand_pos.clear();
      rand_pos.push_back(right_nnf);
      rand_pos.push_back(down_nnf);
      rand_pos.push_back(nnf[offset]);
      Point2D best_bias;
      double d_best_bias = this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_bias);
      if (d_best_rand < (bias_rate * d_best_bias))
      {
        nnf[offset] = best_rand;
      }
      else
      {
        nnf[offset] = best_bias;
      }
      this->updateRefCount(ref_cnt, nnf[offset], gpsrc_d, level);
    }
  }
}

double SynthesisTool::distPatch(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
  ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
  std::vector<float>& ref_cnt, int level, Point2D& srcPatch, Point2D& tarPatch)
{
  double d = 0.0;
  double d_f = 0.0;
  double d_d = 0.0;
  double d_occ = 0.0;
  double lambda_d_f = 0;
  double lambda_d_d = 0;
  double beta = 0.0;

  int fdim = (int)gpsrc_f.size();
  int ddim = (int)gpsrc_d.size();
  int width = gptar_d[0][level].cols;
  int nnf_width  = (width - this->patch_size + 1);

  // for each pixel in the patch
  for (int i = 0; i < this->patch_size; ++i)
  {
    for (int j = 0; j < this->patch_size; ++j)
    {
      // for each feature dimension
      for (int k = 0; k < fdim; ++k)
      {
        d_f += pow(gpsrc_f[k][level].at<float>(srcPatch.second + i, srcPatch.first + j) - gptar_f[k][level].at<float>(tarPatch.second + i, tarPatch.first + j), 2);
      }
      // for each detail dimension
      for (int k = 0; k < ddim; ++k)
      {
        d_d += pow(gpsrc_d[k][level].at<float>(srcPatch.second + i, srcPatch.first + j) - gptar_d[k][level].at<float>(tarPatch.second + i, tarPatch.first + j), 2);
      }

      d_occ += ref_cnt[(srcPatch.second + i) * gpsrc_d[0][level].cols + srcPatch.first + j];
    }
  }

  d_f /= this->patch_size * this->patch_size * fdim;
  d_d /= this->patch_size * this->patch_size * ddim;
  d_occ /= pow(this->patch_size, 4);

  //if (d_f < 0.001)  lambda_d_f = 1, lambda_d_d = 0;
  //else  lambda_d_f = 0, lambda_d_d = 1;
  beta = 1.0 / (1 + exp(5 * (d_f - 0.5)));
  d =  beta * d_f + (1 - beta) * d_d + lamd_occ * d_occ;// + lambda_d_d * d_d + d_occ;
  return d;
}

double SynthesisTool::bestPatchInSet(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
  ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
  std::vector<float>& ref_cnt, int level, Point2D& tarPatch, std::vector<Point2D>& srcPatches, Point2D& best_patch)
{
  size_t best_id = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < srcPatches.size(); ++i)
  {
    double cur_dist = this->distPatch(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, srcPatches[i], tarPatch);
    if (cur_dist < min_dist)
    {
      best_id = i;
      min_dist = cur_dist;
    }
  }
  best_patch = srcPatches[best_id];


  return min_dist;
}

void SynthesisTool::voteImage(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level)
{
  // compute the value of each pixel
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;


  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      this->votePixel(gpsrc_d, gptar_d, nnf, level, Point2D(j, i));
    }
  }
}

void SynthesisTool::votePixel(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, int level, Point2D& tarPos)
{
  int width  = gptar_d[0][level].cols;
  int height = gptar_d[0][level].rows;
  int nnf_width  = (width - this->patch_size + 1);
  int nnf_height = (height - this->patch_size + 1);
  int ddim = (int)gptar_d.size();
  std::vector<float> final_val(ddim, 0.0);
  int n_pixel = 0;
  std::vector<std::vector<float> > bins_acc(ddim, std::vector<float>(10, 0));
  std::vector<std::vector<int> > bins_cnt(ddim, std::vector<int>(10, 0));

  for (int i = 0; i < this->patch_size; ++i)
  {
    for (int j = 0; j < this->patch_size; ++j)
    {
      // get patch position
      int i_p = tarPos.second - i;
      int j_p = tarPos.first - j;
      // patch is out side of the image
      if (i_p < 0 || i_p >= nnf_height || j_p < 0 || j_p >= nnf_width) continue;
      Point2D patch = nnf[i_p * nnf_width + j_p];
      // get the value of corresponding position
      for (int k = 0; k < ddim; ++k)
      {
        float cur_val = gpsrc_d[k][level].at<float>(patch.second + i, patch.first + j);
        final_val[k] += cur_val;
        int bin_id = std::max(0, std::min(9, int(cur_val * 10)));
        bins_cnt[k][bin_id] += 1;
        bins_acc[k][bin_id] += cur_val;
      }
      ++n_pixel;
    }
  }

  // assume all values are between 0~1
  //for (size_t i = 0; i < bins_cnt.size(); ++i)
  //{
  //  int most = bins_cnt[i][0];
  //  int most_bin_id = 0;
  //  for (size_t j = 1; j < bins_cnt[i].size(); ++j)
  //  {
  //    if (bins_cnt[i][j] > most)
  //    {
  //      most = bins_cnt[i][j];
  //      most_bin_id = j;
  //    }
  //  }
  //  gptar_d[i][level].at<float>(tarPos.second, tarPos.first) = bins_acc[i][most_bin_id] / bins_cnt[i][most_bin_id];
  //}


  for (int k = 0; k < ddim; ++k)
  {
    gptar_d[k][level].at<float>(tarPos.second, tarPos.first) = final_val[k] / n_pixel;
  }
}

void SynthesisTool::updateRefCount(STLVectorf& ref_cnt, Point2D& best_patch, ImagePyramidVec& gpsrc_d, int level)
{
  // add the reference count to each pixel
  for (int i = 0; i < this->patch_size; ++i)
  {
    for (int j = 0; j < this->patch_size; ++j)
    {
      ref_cnt[(best_patch.second + i) * gpsrc_d[0][level].cols + best_patch.first + j] += 1.0;
    }
  }
};