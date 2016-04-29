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

  bool generateMaskStroke(cv::Mat& img_in, std::vector<CvPoint>& stroke)
  {
    // Attention!!! it will modify the img_in photo
    bool is_finished = false;
    IplImage reflectance_map_iplimage = IplImage(img_in);

    MouseArgs* m_arg = new MouseArgs();
    m_arg->img = &reflectance_map_iplimage;
    cvNamedWindow("Draw ROI", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Draw ROI", MouseDraw, (void*)m_arg);
    while (1)
    {
      cvShowImage("Draw ROI", m_arg->img);
      if (cvWaitKey(100) == 27)
        break;

      if (cvWaitKey(100) == 32)
      {
        is_finished = true;
        break;
      }
    }

    if (m_arg->points < 1)
    {
      std::cout << "Get no points!!!\n";
    }
    else
    {
      std::cout << m_arg->points << std::endl;
      stroke.clear();
      stroke.resize(m_arg->points);
      cvCvtSeqToArray(m_arg->seq, &stroke[0]);
    }

    m_arg->Destroy();
    delete m_arg;

    return is_finished;
  }

  void generateMaskedMatVec(std::vector<cv::Mat>& mat_vec_in, std::vector<cv::Mat>& mat_vec_out, cv::Mat& mask)
  {
    mat_vec_out.clear();
    for (size_t i = 0; i < mat_vec_in.size(); ++i)
    {
      mat_vec_out.push_back(mat_vec_in[i].clone());
    }

    int vec_dim = (int)mat_vec_out.size();
    for (int i = 0; i < mask.rows; ++i)
    {
      for (int j = 0; j < mask.cols; ++j)
      {
        if (mask.at<float>(i, j) < 0.5)
        {
          for (int k = 0; k < vec_dim; ++k)
          {
            mat_vec_out[k].at<float>(i, j) = -1;
          }
        }
      }
    }
  }

  void mergeMatVecFromMask(std::vector<cv::Mat>& mat_vec_src, std::vector<cv::Mat>& mat_vec_tar, cv::Mat& mask)
  {
    if (mat_vec_src.size() != mat_vec_tar.size())
    {
      std::cout << "size doesn't match!!!" << std::endl;
    }
    std::cout << "DEBUG" << std::endl;
    int dim_vec = int(mat_vec_src.size());
    for (int i = 0; i < mask.rows; ++i)
    {
      for (int j = 0; j < mask.cols; ++j)
      {
        if (mask.at<float>(i, j) > 0.5)
        {
          for (int k = 0; k < dim_vec; ++k)
          {
            mat_vec_tar[k].at<float>(i, j) = mat_vec_src[k].at<float>(i, j);
          }
        }
      }
    }
    std::cout << "DEBUG" << std::endl;
  }

  void exportMatVecImage(std::vector<cv::Mat>& mat_vec, std::string fname)
  {
    cv::Mat for_merge;
    std::vector<cv::Mat> temp_mat_vec;
    for (size_t i = 0; i < mat_vec.size(); ++i)
    {
      temp_mat_vec.push_back(mat_vec[i].clone());
    }
    std::swap(temp_mat_vec[0], temp_mat_vec[2]);
    cv::merge(temp_mat_vec, for_merge);
    cv::imwrite(fname, 255 * for_merge);
  }

  bool meetZero(std::vector<int>& vec)
  {
    for (auto i : vec)
    {
      if (i == 0)
      {
        return true;
      }
    }

    return false;
  }

  void centralizeMat(cv::Mat& mat, int dim, std::vector<float>& min_vec, std::vector<float>& max_vec, bool use_ext)
  {
    // 0 == col, 1 == row
    if (mat.dims > 2)
    {
      std::cout << "Dimension of matrix should be less than 2." << std::endl;
      return;
    }

    std::vector<float> mean_vec;
    //std::vector<float> min_vec;
    //std::vector<float> max_vec;
    if (dim == 0)
    {
      if (!use_ext)
      {
        min_vec.clear();
        max_vec.clear();
        min_vec.resize(mat.cols, std::numeric_limits<float>::max());
        max_vec.resize(mat.cols, -std::numeric_limits<float>::max());

        for (int i = 0; i < mat.rows; ++i)
        {
          for (int j = 0; j < mat.cols; ++j)
          {
            float cur_val = mat.at<float>(i, j);
            if (cur_val > max_vec[j]) max_vec[j] = cur_val;
            if (cur_val < min_vec[j]) min_vec[j] = cur_val;
            //mean_vec[j] += mat.at<float>(i, j);
          }
        }
      }

      //for (size_t i = 0; i < mean_vec.size(); ++i)
      //{
      //  mean_vec[i] = mean_vec[i] / (float)(mat.rows);
      //}
      for (int i = 0; i < mat.rows; ++i)
      {
        for (int j = 0; j < mat.cols; ++j)
        {
          // mat.at<float>(i, j) -= mean_vec[j];
          mat.at<float>(i, j) = (mat.at<float>(i, j) - min_vec[j]) / (max_vec[j] - min_vec[j]);
        }
      }
    }
    else
    {
      mean_vec.resize(mat.rows, 0);
      for (int i = 0; i < mat.cols; ++i)
      {
        for (int j = 0; j < mat.rows; ++j)
        {
          mean_vec[j] += mat.at<float>(j, i);
        }
      }
      for (size_t i = 0; i < mean_vec.size(); ++i)
      {
        mean_vec[i] = mean_vec[i] / (float)(mat.cols);
      }
      for (int i = 0; i < mat.cols; ++i)
      {
        for (int j = 0; j < mat.rows; ++j)
        {
          mat.at<float>(j, i) -= mean_vec[j];
        }
      }
    }
  }

  void normalizeMat(cv::Mat& mat, int dim, std::vector<float> min_max)
  {

  }
}