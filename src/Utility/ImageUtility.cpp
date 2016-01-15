#include "ImageUtility.h"

#include "highgui.h"
#include "cxcore.h"

namespace ImageUtility
{
  void MouseDraw(int event,int x,int y,int flags,void* param)
  {
    MouseArgs* m_arg = (MouseArgs*) param;
    if( !m_arg->img )
      return;

    if( event == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON) )
    {
      m_arg->p_start = cvPoint(x,y);
    }
    else if( event == CV_EVENT_LBUTTONDOWN )
    {
      m_arg->p_start = cvPoint(x,y);
      cvSeqPush( m_arg->seq, &m_arg->p_start);
      m_arg->points += 1;
      if(m_arg->p_start.x>0 && m_arg->p_end.x>0){
        cvLine( m_arg->img, m_arg->p_start, m_arg->p_start, cvScalar(128,0,255) );
      }
    }
    else if( event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) )
    {
      CvPoint pt = cvPoint(x,y);
      if( m_arg->p_start.x > 0 ){
        cvLine( m_arg->img, m_arg->p_start, pt, cvScalar(128,0,255) );
        m_arg->p_start = pt;
        cvSeqPush( m_arg->seq, &m_arg->p_start);
        m_arg->points += 1;
      }

    }
  }

  bool generateMask(cv::Mat& img_in, cv::Mat& mask_out)
  {
    // Attention!!! it will modify the img_in photo
    bool is_finished = false;
    IplImage reflectance_map_iplimage = IplImage(img_in);

    MouseArgs* m_arg = new MouseArgs();
    m_arg->img = &reflectance_map_iplimage;
    cvNamedWindow("Draw ROI", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Draw ROI", MouseDraw, (void*)m_arg);
    while(1)
    {
      cvShowImage("Draw ROI", m_arg->img);
      if(cvWaitKey(100) == 27)
        break;
      
      if(cvWaitKey(100) == 32)
      {
        is_finished = true;
        break;
      }
    }

    if(m_arg->points < 1)
    {
      std::cout << "Get no points!!!\n";
    }
    else
    {
      std::cout<< m_arg->points << std::endl;
      IplImage* mask = cvCreateImage(cvGetSize(&reflectance_map_iplimage), 8, 1);
      cvZero(mask);
      CvPoint* PointArr = new CvPoint[m_arg->points];
      cvCvtSeqToArray(m_arg->seq, PointArr);
      cvFillConvexPoly(mask, PointArr, m_arg->points, cvScalarAll(255), CV_AA, 0);
      delete[] PointArr;
      cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
      cvShowImage("Mask", mask);

      cv::Mat mask_mat(mask, 0);

      for (int i = 0; i < mask_out.rows; i++)
      {
        for (int j = 0; j < mask_out.cols; j++)
        {
          if (mask_mat.at<uchar>(i, j) == 0)
          {
            mask_out.at<float>(i, j) = 0;
          }
        }
      }
    }

    m_arg->Destroy();
    delete m_arg;

    cvDestroyWindow("Draw ROI");
    return is_finished;
  }

  void generateMultiMask(cv::Mat& img_in, cv::Mat& mask_out)
  {
    bool is_finished = false;
    bool is_cur_finished = false;
    std::vector<cv::Mat>  mask_group;
    while (!is_finished)
    {
      cv::Mat cur_mask(img_in.rows, img_in.cols, CV_32FC1, 1);
      if (generateMask(img_in, cur_mask))
      {
        is_finished = true;
      }
      mask_group.push_back(cur_mask);
    }
    
    
    for (int i = 0; i < mask_out.rows; i++)
    {
      for (int j = 0; j < mask_out.cols; j++)
      {
        bool in_mask = false;
        for(size_t k = 0; k < mask_group.size(); k ++)
        {
          if (mask_group[k].at<float>(i, j) > 0.5)
          {
            in_mask = true;
            break;
          }
        }
        if (!in_mask)
        {
          mask_out.at<float>(i, j) = 0;
        }
      }
    }
    cv::imshow("mask_out", mask_out);
    cvDestroyWindow("Draw ROI");
  }
}