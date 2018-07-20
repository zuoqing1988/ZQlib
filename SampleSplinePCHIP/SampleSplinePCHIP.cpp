#include "ZQ_SplinePCHIP.h"
#include "ZQ_Spline.h"
#include <opencv2\opencv.hpp>
using namespace ZQ;
int main()
{
	ZQ_Spline spline;
	ZQ_SplinePCHIP pchip;
	std::vector<double> x;
	std::vector<double> y;
	x.push_back(-3);	y.push_back(-1);
	x.push_back(-2);	y.push_back(-1);
	x.push_back(-1);	y.push_back(-1);
	x.push_back(0);		y.push_back(0);
	x.push_back(1);		y.push_back(1);
	x.push_back(2);		y.push_back(1);
	x.push_back(3);		y.push_back(1);
	spline.SetPoints(x, y);
	pchip.SetPoints(x, y);

	float step = 0.01;
	float begin = -4;
	float end = 4;
	int width = (end - begin) / step + 1;
	int height = 800;
	IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvZero(img);
	CvScalar color_spline = cvScalar(0, 0, 255);
	CvScalar color_pchip = cvScalar(0, 255, 0);
	for (int i = 0; i < x.size(); i++)
	{
		int cur_w = (x[i] - begin) / step;
		int val = (y[i] + 2) * 200;
		if (cur_w >= 0 && cur_w < width && val >= 0 && val < height)
			cvCircle(img, cvPoint(cur_w, val), 5, color_spline);
	}
	for (float i = 0; i < width; i++)
	{
		double cur_x = i*step + begin;
		int val1 = (spline(cur_x) + 2) * 200;
		int val2 = (pchip(cur_x) + 2) * 200;
		if (val1 >= 0 && val1 < height)
			cvSet2D(img, val1, i, color_spline);
		if (val2 >= 0 && val2 < height)
			cvSet2D(img, val2, i, color_pchip);
	}
	cvFlip(img);
	cvNamedWindow("show");
	cvShowImage("show", img);
	cvWaitKey(0);
	cvDestroyWindow("show");
	cvReleaseImage(&img);
	return EXIT_SUCCESS;
}