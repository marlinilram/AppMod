#pragma once
#include "Image.h"
#include <windows.h> 
#include <set>
typedef struct siftflowParam
{
	//public:
	//	siftflowParam();
	//	siftflowParam(float talpha,float td,float tgamma,int tnit,int ntopiter,int tnHi,int tws);
	//	~siftflowParam();
public:
	float alpha; //(0.01) the weight of the truncated L1-norm regularization on the flow
	float d; // (1) the threshold of the truncation
	float gamma; //(0.001) the weight of the magnitude of the flow
	float dgamma;
	int dsize;
	int nIterations; //(40) the number of iterations
	int topwsize;// (10) the size of the matching window at the top level
	int nTopIterations;// (100) the number of BP iterations at the top level
	int nlevels; //(2)  the number of hierarchies in the efficient BP implementation
	int wsize;// (5)  the half size of the search window 

}siftflowParam;

template <class T>
class ImagePyrd
{
public:
	ImagePyrd(int nlevel, T *im1, T *im2,  BiImage *mask1, BiImage *mask2);
	~ImagePyrd(void);
public:
	
	DImage matchPyrdSiftFlow(siftflowParam param,bool);
	DImage matchSiftFlow( T &Im1, T &Im2,siftflowParam param,
		IntImage *offsetX, IntImage *offsetY,
		IntImage *winSizeX,IntImage *winSizeY,
		double *pEnergyList=NULL);
private:
	T** im1;
	T** im2;
	BiImage** mask1;
	BiImage** mask2;
	IntImage** xx;
	IntImage** yy;
	int nlevels;
};



template <class T>
ImagePyrd<T>::~ImagePyrd(void)
{
	for(int i=nlevels-1;i>=0;i--)
	{
		//cout<<i<<"\t"<<nlevels<<endl;
		delete xx[i];
		delete yy[i];	
		//cout<<i<<"done \t"<<nlevels<<endl;
		//delete im1[i];
		//delete mask1[i];
		//cout<<i<<"delete mask \t"<<nlevels<<endl;
		//delete im2[i];
		//delete mask2[i];
		//cout<<i<<"delete displace \t"<<nlevels<<endl;
		//
	}
	delete []im1;
	//cout<<"done im1\t"<<endl;
	delete []mask1;
	//	cout<<"done ms1\t"<<endl;
	delete []im2;
	delete []mask2;
	delete []xx;
		//	cout<<"done xx\t"<<endl;
	delete []yy;
		//cout<<"done "<<endl;
}

template<class T>
ImagePyrd<T>::ImagePyrd(int nlevel, T *tim1, T *tim2,  BiImage *tmask1, BiImage *tmask2)
{
	nlevels = nlevel;
	im1 = new T*[nlevel];
	im2 = new T*[nlevel];
	mask1 = new BiImage*[nlevel];
	mask2 = new BiImage*[nlevel];
	xx = new IntImage*[nlevel];
	yy = new IntImage*[nlevel];
	im1[0] = tim1;
	im2[0] = tim2;
	double sigma = 0.67;
	int fsize = 5;
	if(mask1!=NULL&&mask2!=NULL)
	{
		mask1[0] = tmask1;
		mask2[0] = tmask2;
	}
	double ratio = 0.5;
	for(int i=1;i<nlevels;i++)
	{
		T *tmpImg1 = new T(*im1[i-1]);
		im1[i-1]->GaussianSmoothing(*tmpImg1,sigma,fsize);

		int DstWidth1 = (double)tmpImg1->width()*ratio;
		int DstHeight1 = (double)tmpImg1->height()*ratio;

		im1[i] = new T(DstWidth1,DstHeight1,tmpImg1->nchannels());
		mask1[i] = new BiImage(DstWidth1,DstHeight1,3);
		tmpImg1->imresize(*im1[i],ratio);
		delete tmpImg1;
		//im1[i]->imwrite("./im1-1.png");
		
		

		T *tmpImg2 = new T(*im2[i-1]);
		im2[i-1]->GaussianSmoothing(*tmpImg2,sigma,fsize);
		int DstWidth2 = (double)tmpImg2->width()*ratio;
		int DstHeight2 = (double)tmpImg2->height()*ratio;
			
		im2[i] = new T(DstWidth2,DstHeight2,tmpImg2->nchannels());
		mask2[i] = new BiImage(DstWidth2,DstHeight2,3);

		tmpImg2->imresize(*im2[i],ratio);
		delete tmpImg2;
		

		//im2[i]->imwrite("./im2-1.png");

		if(mask1!=NULL&&mask2!=NULL)
		{
			mask1[i-1]->imresize(*mask1[i],ratio,0);
			mask2[i-1]->imresize(*mask2[i],ratio,0);
			
			//string namefile1 = "./1-" + to_string(i) + ".png";
			//im1[i]->imwrite(namefile1.c_str());

			///string namefile2 = "./2-" + to_string(i) + ".png";
			//im2[i]->imwrite(namefile2.c_str());

			//string mnamefile1 = "./1-" + to_string(i) + "-m.png";
			//mask1[i]->imwrite(mnamefile1.c_str());

			//string mnamefile2 = "./2-" + to_string(i) + "-m.png";
			//mask2[i]->imwrite(mnamefile2.c_str());
			
		}
	}
	for(int i=0;i<nlevels;i++)
	{
		int height1 = im1[i]->height();
		int width1 = im1[i]->width();
		int height2 = im2[i]->height();
		int width2 = im2[i]->width();
		float ratiox = (float)(width2-1)/(float)(width1-1);
		float ratioy = (float)(height2-1)/(float)(height1-1);
		xx[i] = new IntImage(width1,height1);
		yy[i] = new IntImage(width1,height1);
		for(int iy=0;iy<height1;iy++)
			for(int ix=0;ix<width1;ix++)
			{
				xx[i]->data()[iy*width1+ix] = floor((ix)*(ratiox-1)+0.5);//(((float)ix*ratiox)+1-ix);
				yy[i]->data()[iy*width1+ix] = floor((iy)*(ratioy-1)+0.5);//(((float)iy*ratioy)+1-iy);
			}
	}

}

template<class T>
DImage ImagePyrd<T>::matchPyrdSiftFlow(siftflowParam param,bool masked)
{
	DImage flow;
	vector<int> nIterationArray;
	nIterationArray.push_back(param.nIterations);
	int step = (param.nIterations - param.nIterations*0.6)/(nlevels-1);
	for(int i=1;i<nlevels;i++)
		nIterationArray.push_back(nIterationArray[i-1]-step);
	double *pEnergyList = NULL;
	IntImage *vx = NULL;
	IntImage *vy = NULL;
	for(int i=nlevels-1;i>=0;i--)
	{
		printf("Level: %d...",i);
		int height1 = im1[i]->height();
		int width1 = im1[i]->width();
		int nchannel = im1[i]->nchannels();
		int height2 = im2[i]->height();
		int width2 = im2[i]->width();
		
		if(pEnergyList)
			delete pEnergyList;
		
		IntImage *winSizeX = NULL;
		IntImage *winSizeY = NULL;
		//double sumvx = 0;
				//double sumvy = 0;
		if(i==nlevels-1)
		{
			//vx = xx[i];
			//vy = yy[i];
			vx = new IntImage(xx[i]->width(),xx[i]->height());
			vy = new IntImage(xx[i]->width(),xx[i]->height());
			for(int ih=0;ih<xx[i]->height();ih++)
				for(int iw=0;iw<xx[i]->width();iw++)
				{
					vx->data()[ih*xx[i]->width()+iw] = xx[i]->data()[ih*xx[i]->width()+iw];
					vy->data()[ih*xx[i]->width()+iw] = yy[i]->data()[ih*xx[i]->width()+iw];
					//sumvx += vx->data()[ih*xx[i]->width()+iw];
				//sumvy += vy->data()[ih*xx[i]->width()+iw] ;
				}

			winSizeX= new IntImage(param.topwsize,height1,width1,1);
			winSizeY= new IntImage(param.topwsize,height1,width1,1);
		}
		else
		{
			IntImage tmpx(vx->width(),vx->height());
			IntImage tmpy(vy->width(),vy->height());
			//double sumtmpx = 0;
			//double sumtmpy = 0;
			//ofstream fileb("./before-tmpx.txt");
			for(int ih=0;ih<xx[i+1]->height();ih++)
				for(int iw=0;iw<xx[i+1]->width();iw++)
				{
					int sx1 = vx->data()[ih*vx->width()+iw] ;
					int tx1 = xx[i+1]->data()[ih*vx->width()+iw] ;
					tmpx[ih*vx->width()+iw] = sx1 - tx1;
					//sumtmpx += tmpx[ih*vx->width()+iw];
					int sy1 = vy->data()[ih*vx->width()+iw] ;
					int ty1 = yy[i+1]->data()[ih*vx->width()+iw] ;
					tmpy[ih*vx->width()+iw] = sy1 - ty1;
					//sumtmpy += tmpy[ih*vx->width()+iw] ;
					//fileb<<ih<<"\t"<<iw<<"\t"<<tmpx[ih*xx[i+1]->width()+iw] <<"\t"<<tmpy[ih*xx[i+1]->width()+iw]<<endl;

				}
				//fileb.close();
				IntImage rtmpx,rtmpy;
				tmpx.imresize(rtmpx,width1,height1);
				tmpy.imresize(rtmpy,width1,height1);
				delete vx;
				delete vy;
				vx = new IntImage(xx[i]->width(),xx[i]->height());
				vy = new IntImage(xx[i]->width(),xx[i]->height());
				//double rtx = 0;
				//double rty = 0;
				//	ofstream file("./resize-tmpx.txt");
				for(int ih=0;ih<height1;ih++)
					for(int iw=0;iw<width1;iw++)
					{
						//<<ih<<"\t"<<iw<<"\t"<<rtmpx[ih*width1+iw] <<"\t"<<rtmpy[ih*width1+iw]<<endl;
						//rtx += rtmpx[ih*width1+iw]  ;
						//rty += rtmpy[ih*width1+iw] ;
				vx->data()[ih*width1+iw] = rtmpx[ih*width1+iw] *2 + xx[i]->data()[ih*width1+iw];
				vy->data()[ih*width1+iw] = rtmpy[ih*width1+iw] *2 + yy[i]->data()[ih*width1+iw];
				//sumvx += vx->data()[ih*width1+iw];
				//sumvy += vy->data()[ih*width1+iw];
					//	vx->setValue(rtmpx[ih][iw]*2 + xx[i][ih][iw],iw,ih);
					//	vy->setValue(rtmpy[ih][iw]*2 + yy[i][ih][iw],iw,ih);
					}
					//file.close();
				delete winSizeX;
				delete winSizeY;
				winSizeX= new IntImage(param.wsize,height1,width1,1);
				winSizeY= new IntImage(param.wsize,height1,width1,1);
		}

		param.dgamma = param.gamma * pow(2,i);
		if(i==nlevels-1)
		{
			param.nlevels = 2;
			param.nIterations = param.nTopIterations;
			param.dsize = param.topwsize;
		}
		else
		{
			param.nlevels = nlevels-1-i;
			param.nIterations = nIterationArray[i];
			param.dsize = param.wsize;
		}


		pEnergyList = new double[param.nIterations];
		T tmpIm1 = *im1[i];
		T tmpIm2 = *im2[i];
		if(masked)
		{

			double sum1 = 0;
			int summask1=0;
			int summask2=0;
			for(int ih=0;ih<height1;ih++)
				for(int iw=0;iw<width1;iw++)
				{
					unsigned char ismask = mask1[i]->data()[(ih*width1+iw)*3];
					if(ismask>250)
						summask1++;
					for(int ic=0;ic<nchannel;ic++)
					{
						
						if(ismask<200)
							tmpIm1[(ih*width1+iw)*nchannel+ic] = 0;
						else
							sum1 += tmpIm1[(ih*width1+iw)*nchannel+ic];
					}
				}
		
			double sum2 = 0;
			for(int ih=0;ih<height2;ih++)
				for(int iw=0;iw<width2;iw++)
				{
					unsigned char ismask = mask2[i]->data()[(ih*width2+iw)*3];
					if(ismask>250)
						summask2++;
					for(int ic=0;ic<nchannel;ic++)
					{
						
						if(ismask<200)
							tmpIm2[(ih*width2+iw)*nchannel+ic] = 0;
						sum2 += tmpIm2[(ih*width2+iw)*nchannel+ic];
					}
				}

			flow = matchSiftFlow(tmpIm1,tmpIm2,param,vx,vy,winSizeX,winSizeY,pEnergyList);
		}
		else
			flow =matchSiftFlow(tmpIm1,tmpIm2,param,vx,vy,winSizeX,winSizeY,pEnergyList);
		
		//set<int> uniquex,uniquey;
		for(int ih=0;ih<xx[i]->height();ih++)
			for(int iw=0;iw<xx[i]->width();iw++)
			{
				//uniquex.insert(flow[(ih*xx[i]->width()+iw)*2+0]);
				//uniquey.insert(flow[(ih*xx[i]->width()+iw)*2+1]);
				vx->data()[ih*xx[i]->width()+iw] = flow[(ih*xx[i]->width()+iw)*2+0];
				vy->data()[ih*xx[i]->width()+iw] = flow[(ih*xx[i]->width()+iw)*2+1];
			}
	//		printf("uniquex: %d and uniquey. %d.\n",uniquex.size(),uniquey.size());
	///*		Sleep(25);
	//		for(set<int>::iterator it=uniquey.begin();it!=uniquey.end();it++)
	//			cout<<*it<<endl;*/
			
	/*ofstream file("./displace.txt");
	for(int ih=0;ih<flow.height();ih++)
		for(int iw=0;iw<flow.width();iw++)
				file<<ih<<"\t"<<iw<<"\t"<<flow[(ih*flow.width()+iw)*2+0]<<"\t"<<flow[(ih*flow.width()+iw)*2+1]<<endl;
	file.close();*/
			
		//delete vx;
		//delete vy;
		delete winSizeX;
		delete winSizeY;

	}

	return flow;
}

template <class T>
DImage ImagePyrd<T>::matchSiftFlow( T &Im1, T &Im2,siftflowParam param,
								   IntImage *OffsetX, IntImage *OffsetY,
								   IntImage *WinSizeX,IntImage *WinSizeY,
								   double* pEnergyList)
{
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

	
	return bpflow.flow();
}
