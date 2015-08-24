#include "SIFTFlowWrapper.h"

FImage SIFTFlowWrapper::computeDenseSift(FImage im1)
{
    FImage imsift;

    bool IsMultiScale = false;
    vector<int> cellSizeVect;
    int cellSize = 3;
    int stepSize = 1;
    bool IsBoundaryIncluded = true;

    double normalization = 0.1;
    if(IsMultiScale)
        ImageFeature::imSIFT(im1,imsift,cellSizeVect,stepSize,IsBoundaryIncluded,8);
    else
        ImageFeature::imSIFT(im1,imsift,cellSize,stepSize,IsBoundaryIncluded,8);

    return imsift;

}


DImage SIFTFlowWrapper::matchSiftFlow(FImage &Im1, FImage &Im2,siftflowParam param, double* pEnergyList)
{
    int height1 = Im1.height();
    int width1 = Im1.width();
    int height2 = Im2.height();
    int width2 = Im2.width();
    float ratiox = (float)(width2-1)/(float)(width1-1);
    float ratioy = (float)(height2-1)/(float)(height1-1);

    IntImage *OffsetX  = new IntImage(Im1.height(),Im1.width(),1);
    IntImage *OffsetY  = new IntImage(Im1.height(),Im1.width(),1);
    for(int iy=0;iy<height1;iy++)
        for(int ix=0;ix<width1;ix++)
        {
            OffsetX->data()[iy*width1+ix] = floor((ix)*(ratiox-1)+0.5);//(((float)ix*ratiox)+1-ix);
            OffsetY->data()[iy*width1+ix] = floor((iy)*(ratioy-1)+0.5);//(((float)iy*ratioy)+1-iy);
        }


        IntImage *WinSizeX = new IntImage(param.topwsize,Im1.height(),Im1.width(),1);
        IntImage *WinSizeY = new IntImage(param.topwsize,Im1.height(),Im1.width(),1);

        double alpha = param.alpha;
        double d = param.d;
        double gamma = param.dgamma;
        int nIterations = param.nIterations;
        int nHierarchy = param.nlevels;
        int wsize = param.dsize;

        BPFlow bpflow;
        //bpflow.setDataTermTruncation(true);
        bpflow.LoadImages(Im1.width(),Im1.height(),Im1.nchannels(),Im1.data(),Im2.width(),Im2.height(),Im2.data());
        bpflow.setPara(alpha,d);
        bpflow.setHomogeneousMRF(wsize);

        if(OffsetX!=NULL && OffsetY!=NULL)
            bpflow.LoadOffset(OffsetX->data(),OffsetY->data());

        if(WinSizeX!=NULL && WinSizeY!=NULL)
            bpflow.LoadWinSize(WinSizeX->data(),WinSizeY->data());
        if(pEnergyList==NULL)
            pEnergyList = new double[nIterations];
        bpflow.ComputeDataTerm();
        bpflow.ComputeRangeTerm(gamma);
        bpflow.MessagePassing(nIterations,nHierarchy,pEnergyList);
        bpflow.ComputeVelocity();

        delete OffsetX;
        delete OffsetY;
        delete WinSizeX;
        delete WinSizeY;

        return bpflow.flow();
}

void SIFTFlowWrapper::detectMaskRect(BiImage mask1,int &minxInMask1, int &maxxInMask1, int &minyInMask1, int &maxyInMask1)
{
    for(int ih=0;ih<mask1.height();ih++)
        for(int iw=0;iw<mask1.width();iw++)
        {
            int offset = (ih*mask1.width()+iw)*mask1.nchannels();
            if(mask1[offset]>250)
            {
                if(ih>maxyInMask1)
                    maxyInMask1 = ih;
                if(ih<minyInMask1)
                    minyInMask1 = ih;

                if(iw>maxxInMask1)
                    maxxInMask1 = iw;
                if(iw<minxInMask1)
                    minxInMask1 = iw;
            }
        }
}

bool SIFTFlowWrapper::testSIFTFLow()
{
    time_t start,end;
    double dif;


    time (&start);

    BiImage sim1,sim2;
    FImage Im1,Im2;
    if(!sim1.imread("./object.png"))
    {
        printf("Error in loading frame 1!");
        return false;
    }

    if(!sim2.imread("./ren.png"))
    {
        printf("Error in loading frame 2!");
        return false;
    }



    BiImage mask1,mask2;
    mask1.imread("./omask.png");
    mask2.imread("./rmask.png");


    FImage dsim2(sim2.width(),sim2.height(),3);
    FImage dsim1(sim1.width(),sim1.height(),3);
    for(int ih=0;ih<sim2.height();ih++)
        for(int iw=0;iw<sim2.width();iw++)
            for(int ic=0;ic<sim2.nchannels();ic++)
            {
                int offset = (ih*sim2.width()+iw)*sim2.nchannels()+ic;
                dsim2[offset]=(double)sim2[offset]/255.0;
            }

            for(int ih=0;ih<sim1.height();ih++)
                for(int iw=0;iw<sim1.width();iw++)
                    for(int ic=0;ic<sim1.nchannels();ic++)
                    {
                        int offset = (ih*sim1.width()+iw)*sim1.nchannels()+ic;
                        dsim1[offset]=(double)sim1[offset]/255.0;
                    }

                    FImage im2 = computeDenseSift(dsim2);
                    FImage im1 = computeDenseSift(dsim1);



                    siftflowParam SIFTflowpara;
                    SIFTflowpara.alpha=1.5*255;
                    SIFTflowpara.d=40*255;
                    SIFTflowpara.gamma=0.005*255;
                    SIFTflowpara.nlevels=5;
                    SIFTflowpara.wsize=2;
                    SIFTflowpara.topwsize=10;
                    SIFTflowpara.nTopIterations = 60;
                    SIFTflowpara.nIterations= 30;
                    ImagePyrd<FImage> *pyrd = new ImagePyrd<FImage>(SIFTflowpara.nlevels,&im1,&im2,&mask1,&mask2);


                    DImage flow = pyrd->matchPyrdSiftFlow(SIFTflowpara,true);
                    ofstream file("./displace.txt");
                    for(int ih=0;ih<flow.height();ih++)
                        for(int iw=0;iw<flow.width();iw++)
                            file<<ih<<"\t"<<iw<<"\t"<<flow[(ih*flow.width()+iw)*2+0]<<"\t"<<flow[(ih*flow.width()+iw)*2+1]<<endl;
                    file.close();

                    cv::Mat crsp_img(flow.height(), flow.width(), CV_64FC2, flow.pData);

                    delete pyrd;
                    return true;	
}

bool SIFTFlowWrapper::cvtCVMatToFImg(cv::Mat &cv_img, FImage &f_img)
{

    f_img = FImage(cv_img.cols,cv_img.rows,cv_img.channels());
    float *cv_img_ptr = (float *)cv_img.data;

    for(int ih=0;ih<f_img.height();ih++)
    {
        for(int iw=0;iw<f_img.width();iw++)
        {
            for(int ic=0;ic<f_img.nchannels();ic++)
            {
                int offset = (ih*f_img.width()+iw)*f_img.nchannels()+ic;
                f_img[offset]=cv_img_ptr[offset];
            }
        }
    }

    return true;


}

bool SIFTFlowWrapper::cvtCVMatToBImg(cv::Mat &cv_img, BiImage &b_img)
{
    b_img = BiImage(cv_img.cols,cv_img.rows,cv_img.channels());
    uchar *cv_img_ptr = (uchar *)cv_img.data;

    for(int ih=0;ih<b_img.height();ih++)
    {
        for(int iw=0;iw<b_img.width();iw++)
        {
            for(int ic=0;ic<b_img.nchannels();ic++)
            {
                int offset = (ih*b_img.width()+iw)*b_img.nchannels()+ic;
                b_img[offset]=cv_img_ptr[offset];
            }
        }
    }

    return true;
}

bool SIFTFlowWrapper::doSIFTFlow(cv::Mat &displace_img, cv::Mat &src_img, cv::Mat &src_mask, cv::Mat &dst_img, cv::Mat &dst_mask)
{

    FImage sim2; //dsim2 is target
    FImage sim1; //dsim1 is source


    cvtCVMatToFImg(src_img, sim1);
    cvtCVMatToFImg(dst_img, sim2);

    BiImage mask1;
    BiImage mask2;


    cvtCVMatToBImg(src_mask, mask1);
    cvtCVMatToBImg(dst_mask, mask2);

    std::ofstream f_debug("../src_mask.mat");
    if (f_debug)
    {
        f_debug << src_mask;
        f_debug.close();
    }
    mask1.imwrite("../mask1.png");
    mask2.imwrite("../mask2.png");
    //mask1.imread("../mask1.png");
    //mask2.imread("../mask2.png");
    sim1.imwrite("../sim1.png");
    sim2.imwrite("../sim2.png");


    int minxInMask1 = mask1.width();
    int minyInMask1 = mask1.height();
    int maxxInMask1 = 0;
    int maxyInMask1 = 0;
    detectMaskRect(mask1,minxInMask1,maxxInMask1,minyInMask1,maxyInMask1);

    int minxInMask2 = mask2.width();
    int minyInMask2 = mask2.height();
    int maxxInMask2 = 0;
    int maxyInMask2 = 0;
    detectMaskRect(mask2,minxInMask2,maxxInMask2,minyInMask2,maxyInMask2);

    FImage dsim1(maxxInMask1 - minxInMask1 + 1,maxyInMask1-minyInMask1+1,3);
    FImage dsim2(maxxInMask2 - minxInMask2 + 1,maxyInMask2-minyInMask2+1,3);
    //FImage dsim2(sim2.width(),sim2.height(),3);
    //FImage dsim1(sim1.width(),sim1.height(),3);
    for(int ih=0;ih<dsim1.height();ih++)
    {
        for(int iw=0;iw<dsim1.width();iw++)
        {
            for(int ic=0;ic<dsim1.nchannels();ic++)
            {
                int noffset = (ih*dsim1.width()+iw)*dsim1.nchannels()+ic;
                int ooffset = ((ih+minyInMask1)*sim1.width()+(iw+minxInMask1))*sim1.nchannels()+ic;
                dsim1[noffset]=(double)sim1[ooffset];
            }
        }
    }

    for(int ih=0;ih<dsim2.height();ih++)
    {
        for(int iw=0;iw<dsim2.width();iw++)
        {
            for(int ic=0;ic<dsim2.nchannels();ic++)
            {
                int noffset = (ih*dsim2.width()+iw)*dsim2.nchannels()+ic;
                int ooffset = ((ih+minyInMask2)*sim2.width()+(iw+minxInMask2))*sim2.nchannels()+ic;
                dsim2[noffset]=(double)sim2[ooffset];
            }
        }
    }

    /*for(int ih=0;ih<sim1.height();ih++)
    for(int iw=0;iw<sim1.width();iw++)
    for(int ic=0;ic<sim1.nchannels();ic++)
    {
    int offset = (ih*sim1.width()+iw)*sim1.nchannels()+ic;
    dsim1[offset]=(double)sim1[offset]/255.0;
    }*/

    FImage sift2 = computeDenseSift(dsim2);
    FImage sift1 = computeDenseSift(dsim1);
    double tsum = 0;
    for(int ih=0;ih<sift2.height();ih++)
    {
        for(int iw=0;iw<sift2.width();iw++)
        {
            for(int ic=0;ic<sift2.nchannels();ic++)
            {
                tsum += sift2[(ih*sift2.width()+iw)*sift2.nchannels()+ic];
            }
        }
    }


    //FImage im1 = computeDenseSift(sim1);
    siftflowParam SIFTflowpara;
    SIFTflowpara.alpha=1.5*255;
    SIFTflowpara.d=40*255;
    SIFTflowpara.gamma=0.005*255;
    SIFTflowpara.nlevels=5;
    SIFTflowpara.wsize=2;
    SIFTflowpara.topwsize=10;
    SIFTflowpara.nTopIterations = 60;
    SIFTflowpara.nIterations= 30;


    //ImagePyrd<FImage> *pyrd = new ImagePyrd<FImage>(SIFTflowpara.nlevels,&im1,&im2,&mask1,&mask2);

    double *pEnergyList = new double[SIFTflowpara.nIterations];
    DImage flow = matchSiftFlow(sift1,sift2,SIFTflowpara,pEnergyList);
    DImage flow_full(sim1.width(), sim1.height(), 2);
    for(int ih=0;ih<flow_full.height();ih++)
    {
        for(int iw=0;iw<flow_full.width();iw++)
        {
            for(int ic=0;ic<flow_full.nchannels();ic++)
            {
                int noffset = (ih*flow_full.width()+iw)*flow_full.nchannels()+ic;
                flow_full[noffset] = 0;
            }
        }
    }

    ofstream file("./displace.txt");
    for(int ih=0;ih<flow.height();ih++)
    {
        for(int iw=0;iw<flow.width();iw++)
        {
            int offset = (ih*flow.width()+iw);
            //if(mask1.pData[offset*3]>250)
            int tvy = ih + flow[offset*2+1];
            int tvx = iw + flow[offset*2+0];
            if(tvx<=0)
                tvx=iw;

            if(tvy<=0)
                tvy=ih;

            int sy = ih + minyInMask1;
            int sx = iw + minxInMask1;
            int ty = tvy + minyInMask2;
            int tx = tvx + minxInMask2;
            flow[offset*2+1] = ty;
            flow[offset*2+0] = tx;

            flow_full[(sy*flow_full.width()+sx)*flow_full.nchannels() + 0] = tx;
            flow_full[(sy*flow_full.width()+sx)*flow_full.nchannels() + 1] = ty;

            file<<sy<<"\t"<<sx<<"\t"<<ty<<"\t"<<tx<<endl;
            //file<<ih<<"\t"<<iw<<"\t"<<flow[offset*2+1]<<"\t"<<flow[offset*2+0]<<endl;
        }
    }
    file.close();

    std::cout<<"finished matchpyrd..\n";
    displace_img = cv::Mat(flow_full.height(), flow_full.width(), CV_64FC2, &flow_full[0]).clone();

    delete pEnergyList;
    return true;
}

void SIFTFlowWrapper::findMskBoundRect(int &mask_x_min, int &mask_x_max, int &mask_y_min, int &mask_y_max, cv::Mat &mask)
{

    mask_x_min = mask_y_min = std::numeric_limits<int>::max();
    mask_x_max = mask_y_max = std::numeric_limits<int>::min();

    if (mask.channels() > 1)
    {
        std::cout <<"mask channels larger than 1\n";
        return;
    }
    uchar *mask_ptr = (uchar *)mask.data;

    for (size_t i = 0; i < mask.rows; ++i)
    {
        for (size_t j = 0; j < mask.cols; ++j)
        {
            if (mask_ptr[i*mask.cols + j] > 0)
            {
                if (i < mask_y_min)
                    mask_y_min = i;
                if (i > mask_y_max)
                    mask_y_max = i;
                if (j < mask_x_min)
                    mask_x_min = j;
                if (j > mask_x_max)
                    mask_x_max = j;
            }
        }
    }
}