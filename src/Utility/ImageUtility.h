#ifndef ImageUtility_H
#define ImageUtility_H

#include <cv.h>
#include "BasicHeader.h"
#include "LgMeshTypes.h"

namespace ImageUtility
{
  struct MouseArgs{
    IplImage* img;
    CvPoint p_start;
    CvPoint p_end;
    CvSeq* seq;
    CvMemStorage* storage;
    int points;
    // init
    MouseArgs():img(0),points(0){
      p_start = cvPoint(-1,-1);
      p_end = cvPoint(-1,-1);
      storage = cvCreateMemStorage(0);
      seq = cvCreateSeq( CV_32SC2,sizeof(CvSeq),sizeof(CvPoint), storage );
    }
    // destroy
    void Destroy(){
      if(!img)
        cvReleaseImage(&img);
      cvReleaseMemStorage(&storage );
      seq = NULL;
      img = NULL;
    }
  };

  void MouseDraw(int event,int x,int y,int flags,void* param);
  bool generateMask(cv::Mat& img_in, cv::Mat& mask_out);
  void generateMultiMask(cv::Mat& img_in, cv::Mat& mask_out);
  // TODO: use stroke to generate mask instead of generateMask
  bool debugGenerateMask(cv::Mat& img_in, cv::Mat& mask_out);
  bool generateMaskStroke(cv::Mat& img_in, std::vector<CvPoint>& stroke);

  void generateMaskedMatVec(std::vector<cv::Mat>& mat_vec_in, std::vector<cv::Mat>& mat_vec_out, cv::Mat& mask);
  void mergeMatVecFromMask(std::vector<cv::Mat>& mat_vec_src, std::vector<cv::Mat>& mat_vec_tar, cv::Mat& mask);

  void exportMatVecImage(std::vector<cv::Mat>& mat_vec, std::string fname);

  bool meetZero(std::vector<int>& vec);

  void centralizeMat(cv::Mat& mat, int dim, std::vector<float>& min_vec, std::vector<float>& max_vec, bool use_ext);
  void normalizeMat(cv::Mat& mat, int dim, std::vector<float> min_max);
}

#endif // !ImageUtility_H
