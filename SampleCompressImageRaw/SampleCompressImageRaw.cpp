#include "ZQ_CompressedImageRaw.h"
#include "ZQ_CompressedImage.h"
#include "ZQ_ImageIO.h"
#include "cv.h"
#include "highgui.h"

using namespace ZQ;

template<class T>
void test2d_raw()
{
	float quality = 0.999;
	
	const char* infile = "input.png";
	const char* outfile = "compress_2d.craw";
	ZQ_WaveletRawHead raw;
	ZQ_DImage<T> im;
	if(!ZQ_ImageIO::loadImage(im,infile,0))
	{
		printf("failed to load file %s\n",infile);
		return;
	}

	if(!ZQ_CompressedImageRaw::CompressImage(im,raw,1000000.0,quality))
	{
		printf("compress fail\n");
		return;
	}

	if(!raw.WriteToFile(outfile))
	{
		printf("failed to write file %s\n",outfile);
		return;
	}

	//load

	ZQ_DImage<T> im2;
	ZQ_WaveletRawHead raw2;
	if(!raw2.LoadFromFile(outfile))
	{
		printf("failed to load file %s\n",outfile);
		return;
	}
	if(!ZQ_CompressedImageRaw::DecompressImage(raw2,im2))
	{
		printf("decompress fail\n");
		return;
	}

	int width = im.width();
	int height = im.height();
	int nChannels = im.nchannels();

	IplImage* ori = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	IplImage* compress = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			CvScalar sca1,sca2;
			for(int c = 0;c < nChannels;c++)
			{
				sca1.val[c] = im.data()[(i*width+j)*nChannels+c] * 255;
				sca2.val[c] = im2.data()[(i*width+j)*nChannels+c] * 255;
			}
			cvSet2D(ori,i,j,sca1);
			cvSet2D(compress,i,j,sca2);
		}
	}

	cvNamedWindow("ori");
	cvNamedWindow("compress");
	cvShowImage("ori",ori);
	cvShowImage("compress",compress);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&ori);
	cvReleaseImage(&compress);

}


template<class T>
void test2d()
{
	float quality = 0.999;

	const char* infile = "input.png";
	const char* outfile = "compress_2d.zqci";
	ZQ_DImage<T> im;
	if(!ZQ_ImageIO::loadImage(im,infile,0))
	{
		printf("failed to load file %s\n",infile);
		return;
	}

	if(!ZQ_CompressedImage::SaveCompressedImage(outfile,im,1000,quality))
	{
		printf("save compress image fail\n");
		return;
	}
	

	//load

	ZQ_DImage<T> im2;
	if(!ZQ_CompressedImage::LoadCompressedImage(outfile,im2))
	{

		printf("load compress image fail\n");
		return;

	}

	int width = im.width();
	int height = im.height();
	int nChannels = im.nchannels();

	IplImage* ori = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	IplImage* compress = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			CvScalar sca1,sca2;
			for(int c = 0;c < nChannels;c++)
			{
				sca1.val[c] = im.data()[(i*width+j)*nChannels+c] * 255;
				sca2.val[c] = im2.data()[(i*width+j)*nChannels+c] * 255;
			}
			cvSet2D(ori,i,j,sca1);
			cvSet2D(compress,i,j,sca2);
		}
	}

	cvNamedWindow("ori");
	cvNamedWindow("compress");
	cvShowImage("ori",ori);
	cvShowImage("compress",compress);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&ori);
	cvReleaseImage(&compress);

}



template<class T>
void test3d_raw()
{
	float quality = 0.995;

	const char* infile = "input.png";
	const char* outfile = "compress_3d.craw";
	ZQ_WaveletRawHead3D raw;
	ZQ_DImage<T> im;
	if(!ZQ_ImageIO::loadImage(im,infile,1))
	{
		printf("failed to load file %s\n",infile);
		return;
	}

	int width = im.width();
	int height = im.height();
	int nChannels = im.nchannels();

	ZQ_DImage3D<T> im1(width,height,nChannels,1);
	memcpy(im1.data(),im.data(),sizeof(T)*width*height*nChannels);

	if(!ZQ_CompressedImageRaw::CompressImage(im1,raw,1000000.0,quality))
	{
		printf("compress fail\n");
		return;
	}

	if(!raw.WriteToFile(outfile))
	{
		printf("failed to write file %s\n",outfile);
		return;
	}

	//load

	ZQ_DImage3D<T> im2;
	ZQ_WaveletRawHead3D raw2;
	if(!raw2.LoadFromFile(outfile))
	{
		printf("failed to load file %s\n",outfile);
		return;
	}
	if(!ZQ_CompressedImageRaw::DecompressImage(raw2,im2))
	{
		printf("decompress fail\n");
		return;
	}

	IplImage* ori = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	IplImage* compress = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			CvScalar sca1,sca2;
			for(int c = 0;c < nChannels;c++)
			{
				sca1.val[c] = im1.data()[(i*width+j)*nChannels+c] * 255;
				sca2.val[c] = im2.data()[(i*width+j)*nChannels+c] * 255;
			}
			cvSet2D(ori,i,j,sca1);
			cvSet2D(compress,i,j,sca2);
		}
	}

	cvNamedWindow("ori");
	cvNamedWindow("compress");
	cvShowImage("ori",ori);
	cvShowImage("compress",compress);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&ori);
	cvReleaseImage(&compress);

}


template<class T>
void test3d()
{
	float quality = 0.995;

	const char* infile = "input.png";
	const char* outfile = "compress_3d.zqci";
	ZQ_DImage<T> im;
	if(!ZQ_ImageIO::loadImage(im,infile,1))
	{
		printf("failed to load file %s\n",infile);
		return;
	}

	int width = im.width();
	int height = im.height();
	int nChannels = im.nchannels();

	ZQ_DImage3D<T> im1(width,height,nChannels,1);
	memcpy(im1.data(),im.data(),sizeof(T)*width*height*nChannels);

	if(!ZQ_CompressedImage::SaveCompressedImage(outfile,im1,1000,quality))
	{
		printf("save compress image fail\n");
		return;
	}

	//load

	ZQ_DImage3D<T> im2;
	if(!ZQ_CompressedImage::LoadCompressedImage(outfile,im2))
	{
		printf("load compress image fail\n");
		return;
	}

	IplImage* ori = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	IplImage* compress = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,nChannels);
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			CvScalar sca1,sca2;
			for(int c = 0;c < nChannels;c++)
			{
				sca1.val[c] = im1.data()[(i*width+j)*nChannels+c] * 255;
				sca2.val[c] = im2.data()[(i*width+j)*nChannels+c] * 255;
			}
			cvSet2D(ori,i,j,sca1);
			cvSet2D(compress,i,j,sca2);
		}
	}

	cvNamedWindow("ori");
	cvNamedWindow("compress");
	cvShowImage("ori",ori);
	cvShowImage("compress",compress);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&ori);
	cvReleaseImage(&compress);

}

void main(const int argc, const char** argv)
{
	test3d_raw<float>();
	test3d_raw<double>();
	test2d_raw<float>();
	test2d_raw<double>();

	test3d<float>();
	test3d<double>();
	test2d<float>();
	test2d<double>();
}