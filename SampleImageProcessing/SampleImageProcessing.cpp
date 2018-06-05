
#define ZQLIB_USE_OPENMP
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include <time.h>
#include <vector>
#include <iostream>


using namespace ZQ;
using namespace std;

void main()
{
	
	typedef ZQ_DImage<float> DImage;

	DImage input;
	bool flag = ZQ_ImageIO::loadImage(input,"input.jpg",1);

	if(!flag)
	{
		printf("load image fail\n");
		return;
	}

	int width = input.width();
	int height = input.height();
	int nChannels = input.nchannels();

	int big_width = width*4;
	int big_height = height*4;
	int small_width = width/4;
	int small_height = height/4;

	DImage Big(big_width,big_height,nChannels);
	DImage Small(small_width,small_height,nChannels);

	double t1 = omp_get_wtime();
	ZQ_ImageProcessing::ResizeImageBicubic(input.data(), Big.data(), width, height, nChannels, big_width, big_height, true);

	double t2 = omp_get_wtime();

	ZQ_ImageProcessing::ResizeImageBicubic(input.data(), Big.data(), width, height, nChannels, big_width, big_height, false);

	double t3 = omp_get_wtime();

	printf("%f, %f\n", (t2 - t1), (t3 - t2));

	ZQ_ImageProcessing::ResizeImageBicubic(input.data(), Small.data(), width, height, nChannels, small_width, small_height, false);

	IplImage* input_img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	IplImage* big_img = cvCreateImage(cvSize(big_width,big_height),IPL_DEPTH_8U,nChannels);
	IplImage* small_img = cvCreateImage(cvSize(small_width,small_height),IPL_DEPTH_8U,nChannels);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			CvScalar sca;
			for(int c = 0;c < nChannels;c++)
				sca.val[c] = input.data()[(i*width+j)*nChannels+c]*255;
			cvSet2D(input_img,i,j,sca);
		}
	}

	for(int i = 0;i < big_height;i++)
	{
		for(int j = 0;j < big_width;j++)
		{
			CvScalar sca;
			for(int c = 0;c < nChannels;c++)
				sca.val[c] = Big.data()[(i*big_width+j)*nChannels+c]*255;
			cvSet2D(big_img,i,j,sca);
		}
	}

	for(int i = 0;i < small_height;i++)
	{
		for(int j = 0;j < small_width;j++)
		{
			CvScalar sca;
			for(int c = 0;c < nChannels;c++)
				sca.val[c] = Small.data()[(i*small_width+j)*nChannels+c]*255;
			cvSet2D(small_img,i,j,sca);
		}
	}

	DImage gx,gy;
	input.collapse();
	input.dx(gx,true);
	input.dy(gy,true);
	//input.dx_3pt(gx);
	//input.dy_3pt(gy);
	IplImage* gradient_img = ZQ_ImageIO::SaveFlowToColorImage(gx,gy,false,0,64,0,false);

	cvNamedWindow("input");
	cvNamedWindow("big");
	cvNamedWindow("small");
	cvNamedWindow("gradient");

	cvShowImage("input",input_img);
	cvShowImage("big",big_img);
	cvShowImage("small",small_img);
	cvShowImage("gradient",gradient_img);
	cvWaitKey(0);
	cvReleaseImage(&input_img);
	cvReleaseImage(&big_img);
	cvReleaseImage(&small_img);
	cvReleaseImage(&gradient_img);


}

template<class T>
void test_image_min_max_clamp(int nthreads)
{
#ifdef ZQLIB_USE_OPENMP
	int ncores = omp_get_num_procs();
	omp_set_num_threads(__min(nthreads,ncores));
#endif
	typedef ZQ_DImage<T> DImage;
	cout << "Test: image_min_max_clamp (type " << typeid(T).name() << ")\n";

	DImage input;
	bool flag = ZQ_ImageIO::loadImage(input, "input.jpg", 1);

	if (!flag)
	{
		printf("load image fail\n");
		return;
	}
	input.imresize(4096,4096, false);
	cout << "resize image done!\n";
	int width = input.width();
	int height = input.height();
	int nChannels = input.nchannels();
	T*& input_data = input.data();

	bool use_omp = true;
	double t1 = omp_get_wtime();
	for (int i = 0; i < 100; i++)
	T min_v = ZQ_ImageProcessing::ImageMinValue(input_data, width, height, nChannels, use_omp);
	for (int i = 0; i < 100; i++)
	T max_v = ZQ_ImageProcessing::ImageMaxValue(input_data, width, height, nChannels, use_omp);
	double t2 = omp_get_wtime();
	cout << "min_max cost (omp=" << use_omp << "): " << (t2 - t1) << "secs\n";
	use_omp = false;
	for (int i = 0; i < 100; i++)
	T min_v = ZQ_ImageProcessing::ImageMinValue(input_data, width, height, nChannels, use_omp);
	for (int i = 0; i < 100; i++)
	T max_v = ZQ_ImageProcessing::ImageMaxValue(input_data, width, height, nChannels, use_omp);
	double t3 = omp_get_wtime();
	cout << "min_max cost (omp=" << use_omp << "): " << (t3 - t2) << "secs\n";
	
	use_omp = true;
	T min_v = 0.2;
	T max_v = 0.8;
	DImage im1(input), im2(input);
	T*& im1_data = im1.data();
	T*& im2_data = im2.data();
	double t4 = omp_get_wtime();
	use_omp = true;
	for (int i = 0; i < 100; i++)
	ZQ_ImageProcessing::ImageClamp(im1_data, min_v, max_v, width, height, nChannels, use_omp);
	double t5 = omp_get_wtime();
	cout << "clamp cost (omp=" << use_omp << "): " << (t5 - t4) << "secs\n";
	use_omp = false;
	for (int i = 0; i < 100; i++)
	ZQ_ImageProcessing::ImageClamp(im2_data, min_v, max_v, width, height, nChannels, use_omp);
	double t6 = omp_get_wtime();
	cout << "clamp cost (omp=" << use_omp << "): " << (t6 - t5) << "secs\n";
}