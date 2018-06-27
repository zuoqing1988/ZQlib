
#include "ZQ_ShapeDeformation.h"
#include "opencv2\opencv.hpp"
#include <vector>

using namespace ZQ;

template<class T>
void _show(const char* winName, int nTri, int nPts, const int* indices, const T* vert, bool* fixed);

template<class T>
void test()
{
	int NW = 8;
	int NH = 5;
	int pt_num = NH*NW;
	std::vector<int> indices;
	std::vector<T> p(pt_num * 2);
	std::vector<T> q(pt_num * 2);
	for (int h = 0; h < NH; h++)
	{
		for (int w = 0; w < NW; w++)
		{
			int idx = h*NW + w;
			p[idx * 2 + 0] = w * 100 + 50;
			p[idx * 2 + 1] = h * 100 + 100;
		}
	}
	for (int h = 0; h < NH - 1; h++)
	{
		for (int w = 0; w < NW - 1; w++)
		{
			indices.push_back(h*NW + w + 0);
			indices.push_back(h*NW + w + 1);
			indices.push_back((h + 1)*NW + w);
			indices.push_back(h*NW + w + 1);
			indices.push_back((h + 1)*NW + w);
			indices.push_back((h + 1)*NW + w + 1);
			indices.push_back(h*NW + w + 0);
			indices.push_back(h*NW + w + 1);
			indices.push_back((h + 1)*NW + w + 1);
			indices.push_back(h*NW + w + 0);
			indices.push_back((h + 1)*NW + w);
			indices.push_back((h + 1)*NW + w + 1);
		}
	}
	int triangle_num = indices.size() / 3;

	bool* fixed = new bool[pt_num];
	memset(fixed, 0, sizeof(bool)*pt_num);
	fixed[NH / 2 * NW + 0] = 1;
	fixed[NH / 2 * NW + NW/2] = 1;
	fixed[NH / 2 * NW + NW-1] = 1;
	int pivot_id = NH / 2 * NW + NW - 1;
	std::vector<T> out(pt_num*2);

	ZQ_ShapeDeformationOptions opt;
	//opt.methodType = ZQ_ShapeDeformationOptions::METHOD_ARAP_TRIANGLE_AS_CENTER;
	opt.methodType = ZQ_ShapeDeformationOptions::METHOD_ARAP_VERT_AS_CENTER;
	//opt.methodType = ZQ_ShapeDeformationOptions::METHOD_LAPLACIAN;
	opt.FPIteration = 5;
	opt.Iteration = 200;
	ZQ_ShapeDeformation<T> deform;
	if(!deform.BuildMatrix(triangle_num,&indices[0],pt_num,&p[0],fixed,opt))
	{
		printf("failed to set source\n");
		return;
	}
	const char* winName = "show";
	cv::namedWindow(winName);
	const double M_PI = 3.1415926535;
	const double radius = 100;
	for (int t = 0; ; t++)
	{
		double angle = t / 100.0*M_PI;
		q = p;
		q[pivot_id * 2 + 0] = p[pivot_id * 2 + 0] + radius*cos(angle);
		q[pivot_id * 2 + 1] = p[pivot_id * 2 + 1] + radius*sin(angle);
		if (!deform.Deformation(&q[0], &out[0]))
		{
			printf("failed to deform ARAP\n");
			break;
		}
		_show(winName, triangle_num, pt_num, &indices[0], &out[0], fixed);
		int key = cvWaitKey(20);
		if (key == 'q')
			break;
	}
	delete[]fixed;
}

template<class T>
void _show(const char* winName, int nTri, int nPts, const int* indices, const T* vert, bool* fixed)
{
	cv::Mat img = cv::Mat(600, 1000, CV_MAKETYPE(8, 3), cv::Scalar(0));

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
	for (int i = 0; i < nPts; i++)
	{
		if (fixed[i])
			cv::circle(img, cv::Point(vert[i * 2 + 0], vert[i * 2 + 1]), 2, cv::Scalar(0, 255, 0), 2);
	}
	cv::imshow(winName, img);
}

void main()
{
	test<float>();
	//test<double>();
}