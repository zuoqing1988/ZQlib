#include "ZQ_ShapeInterpolation.h"
#include "opencv2\opencv.hpp"

using namespace ZQ;

template<class T>
void _show(const char* winName, int nTri, int nPts, const int* indices, const T* vert);

template<class T>
void test()
{
	int triangle_num = 20;
	int pts_num = 12;
	int indices[60] = {
		0, 1, 6,  1, 7, 6,  0,1, 7,  0, 6, 7, 
		1, 2, 7,  2, 8, 7,  1,2, 8,  1, 7, 8,
		2, 3, 8,  3, 9, 8,  2,3, 9,  2, 8, 9,
		3, 4, 9,  4,10, 9,  3,4,10,  3, 9,10,
		4, 5,10,  5,11,10,  4,5,11,  4,10,11
	};
	T p[24] = { 
		100,100,200,100,300,100,400,100,500,100,600,100, 
		100,200,200,200,300,200,400,200,500,200,600,200 
	};
	T q[24] = { 
		100,100,200,100,320,120,450,250,500,450,500,550, 
		100,200,200,200,300,220,370,350,400,450,400,550};
	T out1[24],out2[24];
	
	ZQ_ShapeInterpolation<T> deform1,deform2;
	if (!deform1.BuildMatrix(triangle_num, indices, pts_num, p, q, false, 0)
		|| !deform2.BuildMatrix(triangle_num, indices, pts_num, p, q, true, 0))
	{
		printf("failed to set source\n");
		return;
	}

	const char* winName1 = "show1";
	const char* winName2 = "show2";
	cv::namedWindow(winName1);
	cv::namedWindow(winName2);
	float ori_step = 0.01f;
	float step = ori_step;
	for (float t = 0; ; t += step)
	{
		printf("t = %.2f\n", t);
		if (!deform1.Interpolation(t, out1)
			|| !deform2.Interpolation(t,out2))
		{
			printf("failed to deform ARAP\n");
			return;
		}
		_show(winName1, triangle_num, pts_num, indices, out1);
		_show(winName2, triangle_num, pts_num, indices, out2);
		
		int key = cvWaitKey(20);
		if (key == 'q')
			break;
		if (t > 1)
			step = -ori_step;

		if (t < 0)
			step = ori_step;
	}
	return;
}

template<class T>
void _show(const char* winName, int nTri, int nPts, const int* indices, const T* vert)
{
	cv::Mat img = cv::Mat(600, 800, CV_MAKETYPE(8, 3), cv::Scalar(0));

	cv::Scalar color(0, 0, 255);
	for (int tr = 0; tr < nTri; tr++)
	{
		int id0 = indices[tr * 3 + 0];
		int id1 = indices[tr * 3 + 1];
		int id2 = indices[tr * 3 + 2];

		cv::line(img, cv::Point(vert[id0 * 2 + 0], vert[id0 * 2 + 1]), cv::Point(vert[id1 * 2 + 0], vert[id1 * 2 + 1]), color, 2);
		cv::line(img, cv::Point(vert[id1 * 2 + 0], vert[id1 * 2 + 1]), cv::Point(vert[id2 * 2 + 0], vert[id2 * 2 + 1]), color, 2);
		cv::line(img, cv::Point(vert[id2 * 2 + 0], vert[id2 * 2 + 1]), cv::Point(vert[id0 * 2 + 0], vert[id0 * 2 + 1]), color, 2);
	}
	cv::imshow(winName, img);
}

int main()
{
	test<float>();
	//test<double>();

	return EXIT_SUCCESS;
}