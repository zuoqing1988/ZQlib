#include "ZQ_BinaryImageContour.h"
#include "ZQ_ImageIO.h"
#include "cv.h"
#include "highgui.h"

using namespace ZQ;
int main()
{
	const char* inputfile = "mask.png";
	ZQ_DImage<float> im;
	if(!ZQ_ImageIO::loadImage(im,inputfile,0))
	{
		printf("failed to load %s\n",inputfile);
		return 0;
	}

	int width = im.width();
	int height = im.height();

	ZQ_DImage<bool> mask(width,height);
	bool* mask_data = mask.data();
	float* im_data = im.data();
	for(int i = 0;i < width*height;i++)
		mask_data[i] = im_data[i] > 0.5;

	std::vector<std::vector<ZQ_Vec2D>> contours;

	if(!ZQ_BinaryImageContour::GetBinaryImageContour(mask_data,width,height,contours))
	{
		printf("failed to find contours\n");
		return 0;
	}

	IplImage* show_img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	cvZero(show_img);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(mask_data[i*width+j])
				cvSet2D(show_img,i,j,cvScalar(255,255,255));
			else
				cvSet2D(show_img,i,j,cvScalar(0,0,0));
		}
	}

	for(int cc = 0;cc < contours.size();cc++)
	{
		for(int i = 0;i < contours[cc].size()-1;i++)
		{
			ZQ_Vec2D pt1 = contours[cc][i];
			ZQ_Vec2D pt2 = contours[cc][i+1];
			cvLine(show_img,cvPoint(pt1.x,pt1.y),cvPoint(pt2.x,pt2.y),cvScalar(0,200,0));
		}
	}
	cvNamedWindow("show");
	cvShowImage("show",show_img);
	cvWaitKey(0);
	cvReleaseImage(&show_img);
	cvDestroyAllWindows();
	return 0;
}
