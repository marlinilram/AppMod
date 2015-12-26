#include "SynthesisTool.h"
#include "MeshParameterization.h"

#include <vector>
#include <cv.h>

void SynthesisTool::doFilling(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& src_detail)
{
  // first init the pyramid

  gpsrc_feature.clear();
  gptar_feature.clear();
  gpsrc_detail.clear();
  gptar_detail.clear();

  gpsrc_feature.resize(src_feature.size());
  gptar_feature.resize(src_feature.size());
  gpsrc_detail.resize(src_detail.size());
  gptar_detail.resize(src_detail.size());

  for(size_t i = 0; i < src_feature.size(); i ++)
  {
    gpsrc_feature[i].push_back(src_feature[i].clone());
    gptar_feature[i].push_back(src_feature[i].clone());
  }
  for (size_t i = 0; i < src_detail.size(); ++i)
  {
    gpsrc_detail[i].push_back(src_detail[i].clone());
    gptar_detail[i].push_back(src_detail[i].clone());
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

  std::cout << "FillingTool Init success !" << std::endl;

  // 

  // find best match for each level
  double totalTime = 0.0;
  srand((unsigned)time(NULL));

  std::vector<Point2D> nnf; // Point2D stores the nearest patch offset according to current pos
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail[0].at(l).cols;
    int height = gptar_detail[0].at(l).rows;

    if(l == levels - 1)
    {
      std::vector<int> pixel_mask;
      std::vector<int> patch_mask;
      this->buildMask(gpsrc_detail[0][l], pixel_mask, patch_mask);
      this->initializeFillingNNF(gptar_detail[0], nnf, patch_mask, l);
      this->initializeFillingTarDetail(gptar_detail, pixel_mask, l);
      for (int i_iter = 0; i_iter < 5; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        if (i_iter % 2 == 0)
        {
          this->updateFillingNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, patch_mask, l);
        }
        else
        {
          this->updateFillingNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, patch_mask, l);
        }
        this->voteFillingImage(gpsrc_detail, gptar_detail, nnf, pixel_mask, l);
      }
    }
    else
    {
      std::vector<Point2D> nnf_new;
      std::vector<int> pixel_mask;
      std::vector<int> patch_mask;
      this->buildMask(gpsrc_detail[0][l], pixel_mask, patch_mask);
      this->initializeNNFFromLastLevel(gptar_detail[0], nnf, l, nnf_new);
      this->initializeFillingUpTarDetail(gpsrc_detail, gptar_detail, pixel_mask, l);
      nnf.swap(nnf_new);
      for (int i_iter = 0; i_iter < 5; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        if (i_iter % 2 == 0)
        {
          this->updateFillingNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, patch_mask, l);
        }
        else
        {
          this->updateFillingNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, patch_mask, l);
        }
        this->voteFillingImage(gpsrc_detail, gptar_detail, nnf, pixel_mask, l);
      }
    }

    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}

void SynthesisTool::buildMask(cv::Mat& src_detail, std::vector<int>& pixel_mask, std::vector<int>& patch_mask)
{
  int img_height = src_detail.rows;
  int img_width  = src_detail.cols;
  int nnf_height = (img_height - this->patch_size + 1);
  int nnf_width  = (img_width - this->patch_size + 1);

  patch_mask.clear();
  patch_mask.resize(nnf_width * nnf_height, 0);
  pixel_mask.clear();
  pixel_mask.resize(img_width * img_height, 0);

  for (int i = 0; i < img_height; ++i)
  {
    for (int j = 0; j < img_width; ++j)
    {
      if (src_detail.at<float>(i, j) < 0)
      {
        pixel_mask[i * img_width + j] = 1;

        // set mask for all patch cover this pixel
        for (int i_off = 0; i_off < this->patch_size; ++i_off)
        {
          for (int j_off = 0; j_off < this->patch_size; ++j_off)
          {
            // patch position
            int i_p = i - i_off;
            int j_p = j - j_off;
            if (i_p < 0 || i_p >= nnf_height || j_p < 0 || j_p >= nnf_width) continue;
            patch_mask[i_p * nnf_width + j_p] = 1;
          }
        }
      }
    }
  }
}

void SynthesisTool::getRandomPositionWithMask(std::vector<Point2D>& random_set, std::vector<int>& patch_mask, int nnf_width, int nnf_height,
  int n_set, int max_height, int max_width, int min_height /* = 0 */, int min_width /* = 0 */)
{
  random_set.clear();
  random_set.resize(n_set);

  for (int i = 0; i < n_set; ++i)
  {
    int x;
    int y;
    do 
    {
      x = (rand() / double(RAND_MAX)) * (max_width - min_width) + min_width;
      y = (rand() / double(RAND_MAX)) * (max_height - min_height) + min_height;

      // clamp if the random value is 1
      x = (x >= max_width) ? (max_width - 1) : x;
      y = (y >= max_height) ? (max_height - 1) : y;

    } while (patch_mask[y * nnf_width + x] != 0);

    random_set[i] = Point2D(x, y);
  }
}

void SynthesisTool::initializeFillingNNF(ImagePyramid& gptar_d, NNF& nnf, std::vector<int>& patch_mask, int level)
{
  // for filling task, initial NNF in non mask region can be set to the same position
  int height = gptar_d[level].rows;
  int width  = gptar_d[level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);
  nnf.clear();

  // initialize the nnf with random position
  this->getRandomPositionWithMask(nnf, patch_mask, nnf_width, nnf_height, nnf_height * nnf_width, nnf_height, nnf_width);

  // patch mask has the same size with nnf
  for (int i = 0; i < nnf_height; ++i)
  {
    for (int j = 0; j < nnf_width; ++j)
    {
      if (patch_mask[i * nnf_width + j] == 0)
      {
        // non mask region, set them to same position
        nnf[i * nnf_width + j] = Point2D(j, i);
      }
    }
  }
}

void SynthesisTool::initializeFillingTarDetail(ImagePyramidVec& gptar_d, std::vector<int>& pixel_mask, int level)
{
  // only initialize masked region
  // the gptar_d has been set to the same value to gpsrc_d already

  int ddim = gptar_d.size();
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      if (pixel_mask[i * width + j] == 1)
      {
        for (int k = 0; k < ddim; ++k)
        {
          gptar_d[k][level].at<float>(i, j) = (rand() / double(RAND_MAX));
        }
      }
    }
  }
}

void SynthesisTool::updateFillingNNF(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
  ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
  NNF& nnf, std::vector<float>& ref_cnt, std::vector<int>& patch_mask, int level)
{
  // update nnf based on current nnf, random position and position nearby
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);

  // deal with the left upper corner
  int offset = 0 * nnf_width + 0;
  if (patch_mask[offset] == 1)
  {
    std::vector<Point2D> rand_pos;
    this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
    rand_pos.push_back(nnf[offset]);
    Point2D best_patch;
    this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(0, 0), rand_pos, best_patch);
    nnf[offset] = best_patch;
  }

  // deal with first row
  for (int j = 1; j < nnf_width; ++j)
  {
    offset = 0 * nnf_width + j;
    if (patch_mask[offset] == 1)
    {
      // random search
      std::vector<Point2D> rand_pos;
      this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
      // propagation
      Point2D left_nnf = nnf[j - 1];
      left_nnf.first = (left_nnf.first + 1) >= nnf_width ? (nnf_width - 1) : (left_nnf.first + 1);
      if (this->validPatchWithMask(left_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(left_nnf);
      // original best
      rand_pos.push_back(nnf[offset]);
      Point2D best_patch;
      this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, 0), rand_pos, best_patch);
      nnf[offset] = best_patch;
    }
  }

  // deal with first col
  for (int i = 1; i < nnf_height; ++i)
  {
    offset = i * nnf_width + 0;
    if (patch_mask[offset] == 1)
    {
      std::vector<Point2D> rand_pos;
      this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
      Point2D up_nnf = nnf[(i - 1) * nnf_width];
      up_nnf.second = (up_nnf.second + 1) >= nnf_height ? (nnf_height - 1) : (up_nnf.second + 1);
      if (this->validPatchWithMask(up_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(up_nnf);
      rand_pos.push_back(nnf[offset]);
      Point2D best_patch;
      this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(0, i), rand_pos, best_patch);
      nnf[offset] = best_patch;
    }
  }

  // deal with all pixels left
  for (int i = 1; i < nnf_height; ++i)
  {
    for (int j = 1; j < nnf_width; ++j)
    {
      offset = i * nnf_width + j;
      if (patch_mask[offset] == 1)
      {
        std::vector<Point2D> rand_pos;
        this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
        Point2D left_nnf = nnf[i * nnf_width + j - 1];
        left_nnf.first = (left_nnf.first + 1) >= nnf_width ? (nnf_width - 1) : (left_nnf.first + 1);
        Point2D up_nnf = nnf[(i - 1) * nnf_width + j];
        up_nnf.second = (up_nnf.second + 1) >= nnf_height ? (nnf_height - 1) : (up_nnf.second + 1);
        if (this->validPatchWithMask(left_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(left_nnf);
        if (this->validPatchWithMask(up_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(up_nnf);
        rand_pos.push_back(nnf[offset]);
        Point2D best_patch;
        this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_patch);
        nnf[offset] = best_patch;
      }
    }
  }
}

void SynthesisTool::updateFillingNNFReverse(ImagePyramidVec& gpsrc_f, ImagePyramidVec& gptar_f,
  ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d,
  NNF& nnf, std::vector<float>& ref_cnt, std::vector<int>& patch_mask, int level)
{
  // update nnf based on current nnf, random position and position nearby
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;
  int nnf_height = (height - this->patch_size + 1);
  int nnf_width  = (width - this->patch_size + 1);

  // deal with the right bottom corner
  int offset = (nnf_height - 1) * nnf_width + (nnf_width - 1);
  if (patch_mask[offset] == 1)
  {
    std::vector<Point2D> rand_pos;
    this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width); // TODO: random sample in a more local region
    rand_pos.push_back(nnf[offset]);
    Point2D best_patch;
    this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(nnf_width - 1, nnf_height - 1), rand_pos, best_patch);
    nnf[offset] = best_patch;
  }

  // deal with first row
  for (int j = nnf_width - 2; j >= 0; --j)
  {
    offset = (nnf_height - 1) * nnf_width + j;
    if (patch_mask[offset] == 1)
    {
      std::vector<Point2D> rand_pos;
      this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
      Point2D right_nnf = nnf[(nnf_height - 1) * nnf_width + j + 1];
      right_nnf.first = (right_nnf.first - 1) < 0 ? 0 : (right_nnf.first - 1);
      if (this->validPatchWithMask(right_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(right_nnf);
      rand_pos.push_back(nnf[offset]);
      Point2D best_patch;
      this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, nnf_height - 1), rand_pos, best_patch);
      nnf[offset] = best_patch;
    }
  }

  // deal with first col
  for (int i = nnf_height - 2; i >= 0; --i)
  {
    offset = i * nnf_width + (nnf_width - 1);
    if (patch_mask[offset] == 1)
    {
      std::vector<Point2D> rand_pos;
      this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
      Point2D down_nnf = nnf[(i + 1) * nnf_width + nnf_width - 1];
      down_nnf.second = (down_nnf.second - 1) < 0 ? 0 : (down_nnf.second - 1);
      if (this->validPatchWithMask(down_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(down_nnf);
      rand_pos.push_back(nnf[offset]);
      Point2D best_patch;
      this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(nnf_width - 1, i), rand_pos, best_patch);
      nnf[offset] = best_patch;
    }
  }

  // deal with all pixels left
  for (int i = nnf_height - 2; i >= 0; --i)
  {
    for (int j = nnf_width - 2; j >= 0; --j)
    {
      offset = i * nnf_width + j;
      if (patch_mask[offset] == 1)
      {
        std::vector<Point2D> rand_pos;
        this->getRandomPositionWithMask(rand_pos, patch_mask, nnf_width, nnf_height, best_random_size, nnf_height, nnf_width);
        Point2D right_nnf = nnf[i * nnf_width + j + 1];
        right_nnf.first = (right_nnf.first - 1) < 0 ? 0 : (right_nnf.first - 1);
        Point2D down_nnf = nnf[(i + 1) * nnf_width + j];
        down_nnf.second = (down_nnf.second - 1) < 0 ? 0 : (down_nnf.second - 1);
        if (this->validPatchWithMask(right_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(right_nnf);
        if (this->validPatchWithMask(down_nnf, patch_mask, nnf_height, nnf_width)) rand_pos.push_back(down_nnf);
        rand_pos.push_back(nnf[offset]);
        Point2D best_patch;
        this->bestPatchInSet(gpsrc_f, gptar_f, gpsrc_d, gptar_d, ref_cnt, level, Point2D(j, i), rand_pos, best_patch);
        nnf[offset] = best_patch;
      }
    }
  }
}

void SynthesisTool::voteFillingImage(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, NNF& nnf, std::vector<int>& pixel_mask, int level)
{
  // compute the value of each pixel
  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      if (pixel_mask[i * width + j] == 1)
      {
        this->votePixel(gpsrc_d, gptar_d, nnf, level, Point2D(j, i));
      }
    }
  }
}

void SynthesisTool::initializeFillingUpTarDetail(ImagePyramidVec& gpsrc_d, ImagePyramidVec& gptar_d, std::vector<int>& pixel_mask, int level)
{
  int ddim = gptar_detail.size();
  for (int k = 0; k < ddim; ++k)
  {
    cv::pyrUp(gptar_detail[k][level + 1], gptar_detail[k][level], cv::Size(gptar_detail[k][level].cols, gptar_detail[k][level].rows));
  }

  int height = gptar_d[0][level].rows;
  int width  = gptar_d[0][level].cols;

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      if (pixel_mask[i * width + j] == 0)
      {
        for (int k = 0; k < ddim; ++k)
        {
          gptar_d[k][level].at<float>(i, j) = gpsrc_d[k][level].at<float>(i, j);
        }
      }
    }
  }
}

bool SynthesisTool::validPatchWithMask(Point2D& patch_pos, std::vector<int>& patch_mask, int nnf_height, int nnf_width)
{
  if (patch_mask[patch_pos.second * nnf_width + patch_pos.first] == 0) return true;
  else return false;
}

void SynthesisTool::doSynthesisWithMask(std::vector<cv::Mat>& src_feature, std::vector<cv::Mat>& tar_feature, std::vector<cv::Mat>& src_detail, std::vector<cv::Mat>& tar_detail)
{
  gpsrc_feature.clear();
  gptar_feature.clear();
  gpsrc_detail.clear();
  gptar_detail.clear();

  gpsrc_feature.resize(src_feature.size());
  gptar_feature.resize(tar_feature.size());
  gpsrc_detail.resize(src_detail.size());
  gptar_detail.resize(tar_detail.size());
  ImagePyramidVec gptar_detail_bk;
  gptar_detail_bk.resize(tar_detail.size());

  for(size_t i = 0; i < src_feature.size(); i ++)
  {
    gpsrc_feature[i].push_back(src_feature[i].clone());
    gptar_feature[i].push_back(tar_feature[i].clone());
  }
  for (size_t i = 0; i < src_detail.size(); ++i)
  {
    gpsrc_detail[i].push_back(src_detail[i].clone());
    gptar_detail[i].push_back(tar_detail[i].clone());
    gptar_detail_bk[i].push_back(tar_detail[i].clone());
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
    this->generatePyramid(gptar_detail_bk[i], levels);
  }

  // build target mask for each level of pyramid
  std::vector<std::vector<int> > patch_masks(levels, std::vector<int>());
  std::vector<std::vector<int> > pixel_masks(levels, std::vector<int>());
  for (int i = 0; i < levels; ++i)
  {
    this->buildMask(gptar_detail[0][i], pixel_masks[i], patch_masks[i]);
  }

  std::cout << "MaskSynthesisTool Init success !" << std::endl;

  // find best match for each level
  double totalTime = 0.0;
  srand((unsigned)time(NULL));

  std::vector<Point2D> nnf; // Point2D stores the nearest patch offset according to current pos
  for (int l = levels - 1; l >= 0; --l)                      
  {
    double duration;
    clock_t start, end;
    start = clock();

    int width = gptar_detail[0].at(l).cols;
    int height = gptar_detail[0].at(l).rows;

    if(l == levels - 1)
    {
      this->initializeNNF(gptar_detail[0], nnf, l);
      this->initializeFillingTarDetail(gptar_detail, pixel_masks[l], l);
      for (int i_iter = 0; i_iter < 5; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        if (i_iter % 2 == 0)
        {
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        }
        else
        {
          this->updateNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        }
        this->voteFillingImage(gpsrc_detail, gptar_detail, nnf, pixel_masks[l], l);
      }
    }
    else
    {
      std::vector<Point2D> nnf_new;
      this->initializeNNFFromLastLevel(gptar_detail[0], nnf, l, nnf_new);
      this->initializeFillingUpTarDetail(gptar_detail_bk, gptar_detail, pixel_masks[l], l);
      nnf.swap(nnf_new);
      for (int i_iter = 0; i_iter < 5; ++i_iter)
      {
        std::vector<float> ref_cnt(width * height, 0.0);
        if (i_iter % 2 == 0)
        {
          this->updateNNF(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        }
         else
        {
          this->updateNNFReverse(gpsrc_feature, gptar_feature, gpsrc_detail, gptar_detail, nnf, ref_cnt, l);
        }
        this->voteFillingImage(gpsrc_detail, gptar_detail, nnf, pixel_masks[l], l);
      }
    }

    end = clock();
    duration = (double)(end - start) / CLOCKS_PER_SEC;
    totalTime += duration;
    std::cout << "Level " << l << " is finished ! " << "Running time is : " << duration << " seconds." << std::endl;
  }
  std::cout << "All levels is finished !" << " The total running time is :" << totalTime << " seconds." << std::endl;
}