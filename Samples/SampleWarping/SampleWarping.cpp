#include "ZQ_Warping.h"
#include "cv.h"
#include "highgui.h"

using namespace ZQ;

template<class T>
class GlobalData
{
public:
	static const int npts = 8;
	static T* before_pts;
	static T* after_pts;
	static int cur_pt_num;

};

template<class T>
int GlobalData<T>::cur_pt_num = 0;

template<class T>
T* GlobalData<T>::before_pts = 0;

template<class T>
T* GlobalData<T>::after_pts = 0;

template<class T>
void mousehanler(int event, int x, int y, int flags, void* param);

template<class T>
void test2d()
{
	GlobalData<T>::before_pts = new T[GlobalData<T>::npts*2];
	GlobalData<T>::after_pts = new T[GlobalData<T>::npts*2];

	IplImage* img = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
	for(int i = 0;i < 600;i++)
	{
		for(int j = 0;j < 600;j++)
		{
			if((i/100 + j/100)%2==0)
				cvSet2D(img,i,j,cvScalar(0,0,0));
			else
				cvSet2D(img,i,j,cvScalar(255,255,255));
		}
	}

	IplImage* dstimg = cvCloneImage(img);
	cvNamedWindow("source");
	cvShowImage("source",img);


	cvSetMouseCallback("source",mousehanler<T>);

	IplImage* copy_img;

	while(true)
	{

		copy_img = cvCloneImage(img);
		if(GlobalData<T>::cur_pt_num <= GlobalData<T>::npts)
		{
			for(int i = 0;i < GlobalData<T>::cur_pt_num;i++)
			{
				cvCircle(copy_img,cvPoint(GlobalData<T>::before_pts[i*2],GlobalData<T>::before_pts[i*2+1]),2,cvScalar(0,0,255),2);
			}
		}
		else
		{
			for(int i = 0;i < GlobalData<T>::npts;i++)
			{
				cvCircle(copy_img,cvPoint(GlobalData<T>::before_pts[i*2],GlobalData<T>::before_pts[i*2+1]),2,cvScalar(0,0,255),2);
			}
			for(int i = 0;i < GlobalData<T>::cur_pt_num-GlobalData<T>::npts;i++)
			{
				cvCircle(copy_img,cvPoint(GlobalData<T>::after_pts[i*2],GlobalData<T>::after_pts[i*2+1]),2,cvScalar(0,255,0),2);
			}
		}
		cvShowImage("source",copy_img);

		int k = cvWaitKey(10);
		if(k == 'y' || k == 'Y')
		{
			if(GlobalData<T>::cur_pt_num == GlobalData<T>::npts*2)
				break;
		}
		cvReleaseImage(&copy_img);
	}

	ZQ_Warping<T,2>* m_warp = new ZQ_Warping<T,2>();
	m_warp->Solve(GlobalData<T>::npts,GlobalData<T>::before_pts,GlobalData<T>::after_pts,GlobalData<T>::npts*2000,10);

	for(int i = 0;i < 600;i++)
	{
		for(int j = 0;j < 600;j++)
		{
			T input[2] = {j,i};
			T output[2];
			m_warp->WarpCoord(1,input,output);
			int x = output[0];
			int y = output[1];

			if(x < 0 || x >= 600 || y < 0 || y >= 600)
			{
				cvSet2D(dstimg,i,j,cvScalar(0,0,0));
			}
			else
			{
				cvSet2D(dstimg,i,j,cvGet2D(img,y,x));
			}

		}
	}

	cvNamedWindow("dst");
	cvShowImage("dst",dstimg);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&copy_img);
	cvReleaseImage(&img);
	cvReleaseImage(&dstimg);

	delete []GlobalData<T>::after_pts;
	delete []GlobalData<T>::before_pts;

	delete m_warp;


}

template<class T>
void mousehanler(int event, int x, int y, int flags, void* param)
{

	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		if(GlobalData<T>::cur_pt_num < GlobalData<T>::npts*2)
		{
			if(GlobalData<T>::cur_pt_num < GlobalData<T>::npts)
			{
				GlobalData<T>::before_pts[GlobalData<T>::cur_pt_num*2+0] = x;
				GlobalData<T>::before_pts[GlobalData<T>::cur_pt_num*2+1] = y;
			}
			else
			{
				GlobalData<T>::after_pts[(GlobalData<T>::cur_pt_num-GlobalData<T>::npts)*2+0] = x;
				GlobalData<T>::after_pts[(GlobalData<T>::cur_pt_num-GlobalData<T>::npts)*2+1] = y;
			}
			GlobalData<T>::cur_pt_num++;
		}
		printf("cur_pt_num = %d\n",GlobalData<T>::cur_pt_num);
		break;

	case CV_EVENT_RBUTTONDOWN:
		if(GlobalData<T>::cur_pt_num > 0)
		{
			GlobalData<T>::cur_pt_num -- ;
		}
		break;
	}
}

const double before_pts[78] = {
	//8 corners
	-1,		-1,		-1,
	-1,		-1,		 1,
	-1,		 1,		-1,
	-1,		 1,		 1,
	1,		-1,		-1,
	1,		-1,		 1,
	1,		 1,		-1,
	1,		 1,		 1,
	//12 edges
	-1,		-1,		 0,
	-1,		 1,		 0,
	1,		-1,		 0,
	1,		 1,		 0,
	-1,		 0,		-1,
	-1,		 0,		 1,
	1,		 0,		-1,
	1,		 0,		 1,
	0,		-1,		-1,
	0,		-1,		 1,
	0,		 1,		-1,
	0,		 1,		 1,
	//6 faces
	0,		 0,		-1,
	0,		 0,		 1,
	0,		-1,		 0,
	0,		 1,		 0,
	-1,		 0,		 0,
	1,		 0,		 0
};

const double frac1rt3 = 1.0/sqrt(3.0);
const double frac1rt2 = 1.0/sqrt(2.0);

const double after_pts[78] = {
	//8 corners
	-1*frac1rt3,		-1*frac1rt3,		-1*frac1rt3,
	-1*frac1rt3,		-1*frac1rt3,		 1*frac1rt3,
	-1*frac1rt3,		 1*frac1rt3,		-1*frac1rt3,
	-1*frac1rt3,		 1*frac1rt3,		 1*frac1rt3,
	1*frac1rt3,		-1*frac1rt3,		-1*frac1rt3,
	1*frac1rt3,		-1*frac1rt3,		 1*frac1rt3,
	1*frac1rt3,		 1*frac1rt3,		-1*frac1rt3,
	1*frac1rt3,		 1*frac1rt3,		 1*frac1rt3,
	//12 edges
	-1*frac1rt2,		-1*frac1rt2,		 0*frac1rt2,
	-1*frac1rt2,		 1*frac1rt2,		 0*frac1rt2,
	1*frac1rt2,		-1*frac1rt2,		 0*frac1rt2,
	1*frac1rt2,		 1*frac1rt2,		 0*frac1rt2,
	-1*frac1rt2,		 0*frac1rt2,		-1*frac1rt2,
	-1*frac1rt2,		 0*frac1rt2,		 1*frac1rt2,
	1*frac1rt2,		 0*frac1rt2,		-1*frac1rt2,
	1*frac1rt2,		 0*frac1rt2,		 1*frac1rt2,
	0*frac1rt2,		-1*frac1rt2,		-1*frac1rt2,
	0*frac1rt2,		-1*frac1rt2,		 1*frac1rt2,
	0*frac1rt2,		 1*frac1rt2,		-1*frac1rt2,
	0*frac1rt2,		 1*frac1rt2,		 1*frac1rt2,
	//6 faces
	0,		 0,		-1,
	0,		 0,		 1,
	0,		-1,		 0,
	0,		 1,		 0,
	-1,		 0,		 0,
	1,		 0,		 0
};


template<class T>
void test3d()
{

	ZQ_Warping<T,3>* m_warp = new ZQ_Warping<T,3>();
	T tmp_before_pts[78],tmp_after_pts[78];
	for(int i = 0;i < 78;i++)
	{
		tmp_after_pts[i] = after_pts[i];
		tmp_before_pts[i] = before_pts[i];
	}
	m_warp->Solve(26,tmp_before_pts,tmp_after_pts,1000,6,3);

	int N = 100;
	for(int k = 0;k < N;k++)
	{
		for(int j = 0;j < N;j++)
		{
			for(int i = 0;i < N;i++)
			{
				T input[3] = {(i+0.5)/N*2-1,(j+0.5)/N*2-1,(k+0.5)/N*2-1};
				T output[3];
				m_warp->WarpCoord(1,input,output);
			}
		}
	}
	delete m_warp;
}


int main()
{
	test3d<float>();
	test3d<double>();
	test2d<float>();
	test2d<double>();
	
	return 0;
}
