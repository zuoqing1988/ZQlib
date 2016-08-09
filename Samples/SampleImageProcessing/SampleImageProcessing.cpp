#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;

void main()
{
	typedef ZQ_DImage<float> DImage;

	DImage input;
	bool flag = ZQ_ImageIO::loadImage(input,"input.png",1);

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

	ZQ_ImageProcessing::ResizeImageBicubic(input.data(),Big.data(),width,height,nChannels,big_width,big_height);
	ZQ_ImageProcessing::ResizeImageBicubic(input.data(),Small.data(),width,height,nChannels,small_width,small_height);

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
