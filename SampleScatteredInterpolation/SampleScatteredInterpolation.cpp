#include "ZQ_ScatteredInterpolationRBF.h"
#include "opencv\cv.h"
#include "opencv\highgui.h"

#include <time.h>

using namespace ZQ;

template<class T>
void test2d()
{
	const char* file = "input.png";
	IplImage* img = cvLoadImage(file,1);

	if(img == 0)
	{
		printf("load img %s fail\n",file);
		return ;
	}

	int width = img->width;
	int height = img->height;

	int nchannels = img->nChannels;

	IplImage* scattered_img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nchannels);
	cvZero(scattered_img);

	int npoints = width*height/16;
	T* points = new T[npoints*2];
	T* val = new T[npoints*3];

	int offset = 0;
	for(int i = 0;i < height;i+=4)
	{
		for(int j = 0;j < width;j+=4)
		{
			int x = j;
			int y = i;
			CvScalar scalar = cvGet2D(img,y,x);
			cvSet2D(scattered_img,y,x,scalar);
			points[2*offset+0] = x;
			points[2*offset+1] = y;
			val[offset*3+0] = scalar.val[0];
			val[offset*3+1] = scalar.val[1];
			val[offset*3+2] = scalar.val[2];
			offset ++;
		}
	}

	T* inter_pts = new T[width*height*2];
	T* inter_val = new T[width*height*3];

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			inter_pts[(i*width+j)*2+0] = j;
			inter_pts[(i*width+j)*2+1] = i;
		}
	}

	ZQ_ScatteredInterpolationRBF<T>* rbf = new ZQ_ScatteredInterpolationRBF<T>();

	rbf->SetLandmarks(npoints,2,points,val,3);

	clock_t t1 = clock();
	rbf->SolveCoefficient(4,3,1000,ZQ_RBFKernel::COMPACT_CPC6);	
	clock_t t2 = clock();

	int o_npts,o_dim,o_nChannels;
	T* o_ptsCoords;
	T* o_radius;
	T* o_coeff;
	T* o_linear_coeff;
	rbf->CopyDataOut(o_npts,o_dim,o_ptsCoords,o_radius,o_nChannels,o_coeff,o_linear_coeff);
	delete []o_ptsCoords;
	delete []o_radius;
	delete []o_coeff;
	delete []o_linear_coeff;

	printf("solve cost: %f seconds\n",0.001*(t2-t1));
	rbf->GridData2D(width,height,0,width-1,0,height-1,inter_val);
	
	clock_t t3 = clock();
	printf("grid cost: %f seconds\n",0.001*(t3-t2));

	IplImage* inter_img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nchannels);
	cvZero(inter_img);

	for(int i = 0;i < width*height;i++)
	{
		CvScalar scalar = cvScalar(inter_val[i*3+0],inter_val[i*3+1],inter_val[i*3+2]);
		cvSet2D(inter_img,inter_pts[i*2+1],inter_pts[i*2+0],scalar);
	}

	cvNamedWindow("original");
	cvNamedWindow("scattered");
	cvNamedWindow("interpolated");
	cvShowImage("original",img);
	cvShowImage("scattered",scattered_img);
	cvShowImage("interpolated",inter_img);
	cvWaitKey(0);

	cvDestroyAllWindows();
	cvReleaseImage(&img);
	cvReleaseImage(&scattered_img);
	cvReleaseImage(&inter_img);
	delete []inter_val;
	delete []inter_pts;
	delete []val;
	delete []points;
	delete rbf;
	 
}


template<class T>
void test3d()
{
	int width = 32;
	int height = 32;
	int depth = 32;
	int nChannels = 3;
	int npoints = width*height*depth/64;
	T* points = new T[npoints*nChannels];
	T* val = new T[npoints*nChannels];

	int offset = 0;
	for(int k = 0;k < depth;k+=4)
	{
		for(int j = 0;j < height;j+=4)
		{
			for(int i = 0;i < width;i+=4)
			{
				int x = i;
				int y = j;
				int z = k;
				
				points[3*offset+0] = x;
				points[3*offset+1] = y;
				points[3*offset+2] = z;
				val[offset*3+0] = x*x+2*y*z;
				val[offset*3+1] = y*y+2*z*x;
				val[offset*3+2] = z*z+2*x*y;
				offset ++;
			}
		}

	}
	

	T* inter_pts = new T[width*height*depth*3];
	T* inter_val = new T[width*height*depth*3];

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int idx = k*height*width+j*width+i;
				inter_pts[idx*3+0] = i;
				inter_pts[idx*3+1] = j;
				inter_pts[idx*3+2] = k;
			}
		}
	}

	ZQ_ScatteredInterpolationRBF<T>* rbf = new ZQ_ScatteredInterpolationRBF<T>();

	rbf->SetLandmarks(npoints,3,points,val,3);

	clock_t t1 = clock();
	rbf->SolveCoefficient(6,3,1000,ZQ_RBFKernel::COMPACT_CPC6);	
	clock_t t2 = clock();

	printf("solve cost: %f seconds\n",0.001*(t2-t1));
	rbf->GridData3D(width,height,depth,0,width-1,0,height-1,0,depth-1,inter_val);

	clock_t t3 = clock();
	printf("grid cost: %f seconds\n",0.001*(t3-t2));

	IplImage* inter_img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	cvZero(inter_img);

	cvNamedWindow("interpolated");
	for(int z = 0;z < depth;z++)
	{
		int off = height*width*z*nChannels;
		for(int i = 0;i < width*height;i++)
		{
			CvScalar scalar = cvScalar(inter_val[off+i*3+0],inter_val[off+i*3+1],inter_val[off+i*3+2]);
			cvSet2D(inter_img,inter_pts[i*3+1],inter_pts[i*3+0],scalar);
		}
		cvShowImage("interpolated",inter_img);
		cvWaitKey(50);
	}
	


	cvDestroyAllWindows();
	cvReleaseImage(&inter_img);
	delete []inter_val;
	delete []inter_pts;
	delete []val;
	delete []points;
	delete rbf;

}

int main()
{
	test2d<float>();
	test2d<double>();
	test3d<float>();
	test3d<double>();
	return 0;
}