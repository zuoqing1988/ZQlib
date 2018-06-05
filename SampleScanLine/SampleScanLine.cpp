#include "cv.h"
#include "highgui.h"
#include "ZQ_Vec2D.h"
#include <vector>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "ZQ_ScanLinePolygonFill.h"

using namespace ZQ;


void main1();

void main()
{
	int width = 80, height = 60;
	std::vector<ZQ_Vec2D> ply_pts,pixels;
	ply_pts.push_back(ZQ_Vec2D(0, 0));
	ply_pts.push_back(ZQ_Vec2D(79, 0));
	ply_pts.push_back(ZQ_Vec2D(79, 59));
	ply_pts.push_back(ZQ_Vec2D(0, 59));
	if (!ZQ_ScanLinePolygonFill::ScanLinePolygonFillWithClip(ply_pts, width, height, pixels))
	{
		printf("fail\n");
	}
	/*for(int i = 0;i < 10000;i++)
	{
		main1();
	}*/
}

void main1()
{
	srand(time(0)+rand());
	int seed =  rand();//time(0);
	srand(seed);
	printf("seed = %d\n",seed);

	int width = 800, height = 600;
	int off_x = 100, off_y = 100;
	int size_x = 600, size_y = 400;
	std::vector<ZQ_Vec2D> ply_pts;
	int pts_num = 20;
	for(int i = 0;i < pts_num;i++)
	{
		float x = off_x + (rand()%(size_x*10))/10.0;
		float y = off_y + (rand()%(size_y*10))/10.0;
		ply_pts.push_back(ZQ_Vec2D(x,y));
	}
	//ply_pts.push_back(ZQ_Vec2D(   300,   100));
	//ply_pts.push_back(ZQ_Vec2D(   500,   200));
	//ply_pts.push_back(ZQ_Vec2D(   700,   200));
	//ply_pts.push_back(ZQ_Vec2D(   700,   400));
	//ply_pts.push_back(ZQ_Vec2D(   610,   350));
	//ply_pts.push_back(ZQ_Vec2D(   400,   400));
	//ply_pts.push_back(ZQ_Vec2D(   200,   400));
	//ply_pts.push_back(ZQ_Vec2D(   100,   200));

	std::vector<ZQ_Vec2D> pixels;
	ZQ_ScanLinePolygonFill::ScanLinePolygonFill(ply_pts,pixels);

	IplImage* img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	for(int i = 0;i < pixels.size();i++)
	{
		int y = pixels[i].y;
		int x = pixels[i].x;
		CvScalar sca = cvScalar(0,0,200);
		cvSet2D(img,y,x,sca);
	}
	pts_num = ply_pts.size();
	for(int i = 0;i < pts_num;i++)
	{
		int next_i = (i+1)%pts_num;
		int x1 = ply_pts[i].x;
		int y1 = ply_pts[i].y;
		int x2 = ply_pts[next_i].x;
		int y2 = ply_pts[next_i].y;
		cvLine(img,cvPoint(x1,y1),cvPoint(x2,y2),cvScalar(0,200,0));
	}
	cvNamedWindow("show");
	cvShowImage("show",img);
	cvWaitKey(30);
	cvReleaseImage(&img);

}
