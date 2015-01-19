#ifndef SIFTFlowWrapper_H
#define SIFTFlowWrapper_H

#include "Image.h"
#include "BPFlow.h"
#include "stdio.h"
#include "time.h"
#include "ImageFeature.h"
#include "Vector.h"
#include "ImageIO.h"
#include <vector>
#include <fstream>
#include "ImagePyrd.h"



class SIFTFlowWrapper
{
public:
    SIFTFlowWrapper(){};
    ~SIFTFlowWrapper(){};

    bool testSIFTFLow();

    FImage computeDenseSift(FImage im1);

    DImage matchSiftFlow( FImage &Im1, FImage &Im2,siftflowParam param,
        double* pEnergyList);

    void detectMaskRect(BiImage mask1,int &minxInMask1, int &maxxInMask1,
        int &minyInMask1, int &maxyInMask1);

    bool cvtCVMatToFImg(cv::Mat &cv_img, FImage &f_img);

    bool cvtCVMatToBImg(cv::Mat &cv_img, BiImage &b_img);

    bool doSIFTFlow(cv::Mat &displace_img, cv::Mat &src_img, cv::Mat &src_mask, cv::Mat &dst_img, cv::Mat &dst_mask);

    void findMskBoundRect(int &mask_x_min, int &mask_x_max, int &mask_y_min, int &mask_y_max, cv::Mat &mask);

};

#endif