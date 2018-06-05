
#include "ZQ_ShapeDeformation.h"
#include "cv.h"
#include "highgui.h"

using namespace ZQ;

template<class T>
void _show(int nTri, int nPts, const int* indices, const T* vert, int waittime = 0);

template<class T>
void test()
{
	int triangle_num = 6;
	int pts_num = 8;
	int indices[36] = {
		0,1,4, 1,5,4, //0,1,5, 0,4,5, 
		1,2,5, 2,6,5, //1,2,6, 1,5,6, 
		2,3,6, 3,7,6//, 2,3,7, 2,6,7
	};
	T p[16] = {100,100,200,100,300,100,400,100,100,200,200,200,300,200,400,200};
	T q[16] = {100,100,180,150,250,220,300,250,100,200,200,250,300,300,400,300};
	bool fixed[8] = {1,0,0,0,1,0,0,1};
	/*int triangle_num = 2;
	int pts_num = 4;
	int indices[6] = {0,1,3, 0,2,3};
	T p[16] = {100,100, 300,100, 100,200, 250,250};
	T q[16] = {100,100, 300,100, 100,200, 400,400};
	bool fixed[8] = {1,0,0,1};*/

	T out[16];

	ZQ_ShapeDeformationOptions opt;
	opt.methodType = ZQ_ShapeDeformationOptions::METHOD_ARAP_TRIANGLE_AS_CENTER;
	opt.FPIteration = 3;
	opt.Iteration = 100;
	ZQ_ShapeDeformation<T> deform;
	if(!deform.BuildMatrix(triangle_num,indices,pts_num,p,fixed,opt))
	{
		printf("failed to set source\n");
		return;
	}


	if(!deform.Deformation(q,out))
	{
		printf("failed to deform ARAP\n");
		return;
	}
	_show(triangle_num,pts_num,indices,out,0);

}

template<class T>
void _show(int nTri, int nPts, const int* indices, const T* vert, int waittime)
{
	IplImage* img = cvCreateImage(cvSize(500,500),IPL_DEPTH_8U,3);
	cvZero(img);

	CvScalar color = cvScalar(250,0,0);
	for(int tr = 0;tr < nTri;tr++)
	{
		int id0 = indices[tr*3+0];
		int id1 = indices[tr*3+1];
		int id2 = indices[tr*3+2];

		cvLine(img,cvPoint(vert[id0*2+0],vert[id0*2+1]),cvPoint(vert[id1*2+0],vert[id1*2+1]),color,1);
		cvLine(img,cvPoint(vert[id1*2+0],vert[id1*2+1]),cvPoint(vert[id2*2+0],vert[id2*2+1]),color,1);
		cvLine(img,cvPoint(vert[id2*2+0],vert[id2*2+1]),cvPoint(vert[id0*2+0],vert[id0*2+1]),color,1);
	}

	cvNamedWindow("show");
	cvShowImage("show",img);
	cvWaitKey(waittime);
	cvReleaseImage(&img);
}

void main()
{
	test<float>();
	test<double>();
}