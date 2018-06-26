#include <stdio.h>
#include <vector>
#include <math.h>
#include "opencv2\opencv.hpp"
#include "ZQ_BinaryImageContour.h"
#include "ZQ_ConstrainedDelaunayTriangulation.h"

using namespace ZQ;

void DrawCDT(cv::Mat& img, const std::vector<ZQ_Vec2D>& pts, const std::vector<int>& poly_indices, 
	const std::vector<std::vector<int>>& hole_indices,
	std::vector<int>& triangles, cv::Scalar out_color, cv::Scalar hole_color, cv::Scalar interior_color);

int main(int argc, char** argv)
{
	int N = 128;
	int width, height;
	bool* data = 0;

	std::vector<ZQ_BinaryImageContour::Contour> contours;
	//prepare data
	if (argc == 1)
	{
		width = N;
		height = N;
		data = new bool[width*height];
		float radius = 100.0 / 512 * N;
		float cx1 = 2 * radius;
		float cx2 = 3 * radius;
		float cy1 = 2 * radius;
		float cy2 = 3 * radius;

		memset(data, 0, sizeof(bool)*width*height);
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				float dis_x = j - cx1;
				float dis_y = i - cy1;
				if (dis_x*dis_x + dis_y*dis_y < radius*radius)
					data[i*width + j] = true;
				dis_x = j - cx2;
				dis_y = i - cy2;
				if (dis_x*dis_x + dis_y*dis_y < radius*radius)
					data[i*width + j] = true;
			}
		}

		ZQ_BinaryImageContour::GetBinaryImageContour(data, width, height, contours);
		for (int i = 0; i < contours.size(); i++)
		{
			for (int j = 0; j < contours[i].outer_polygon.size(); j++)
				contours[i].outer_polygon[j] *= 4;
			for (int j = 0; j < contours[i].hole_polygon.size(); j++)
			{
				for(int k = 0;k < contours[i].hole_polygon[j].size();k++)
					contours[i].hole_polygon[j][k] *= 4;
			}
				
		}
		width *= 4;
		height *= 4;
	}
	else
	{
		cv::Mat inputImg = cv::imread(argv[1], 0);
		if (inputImg.empty())
		{
			printf("load image fail\n");
			return EXIT_FAILURE;
		}
		width = inputImg.cols;
		height = inputImg.rows;
		data = new bool[width*height];
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				data[i*width + j] = inputImg.data[i*inputImg.step[0] + j] > 127.5f;
			}
		}

		ZQ_BinaryImageContour::GetBinaryImageContour(data, width, height, contours);
	}

	if (contours.size() == 0)
	{
		printf("no contours\n");
		return EXIT_FAILURE;
	}

	cv::Mat cdt_img = cv::Mat(height, width, CV_MAKETYPE(8, 3), cv::Scalar(0, 0, 0));
	ZQ_ConstrianedDelaunayTriangulation cdt;

	for (int i = 0; i < contours.size(); i++)
	{
		int out_len = contours[i].outer_polygon.size();
		int out_len1 = out_len / 8;
		if (out_len1 <= 4)
		{
			printf("vertex num of contours [%d] < 4\n", i);
			continue;
		}

		double vertex_per_line = (double)out_len / out_len1;

		std::vector<ZQ_Vec2D> pts;
		std::vector<int> poly_indices;
		std::vector<std::vector<int>> hole_indices;
		std::vector<int> triangles;
		for (int j = 0; j < out_len1; j++)
		{
			int real_j = (int)(j*vertex_per_line + 0.5);
			if (real_j >= out_len)
				real_j = out_len - 1;
			pts.push_back(contours[i].outer_polygon[real_j]);
		}
		for (int j = 0; j < out_len1; j++)
			poly_indices.push_back(j);
		int offset = out_len1;

		for (int nn = 0; nn < contours[i].hole_polygon.size(); nn++)
		{
			int hole_len = contours[i].hole_polygon[nn].size();
			int hole_len1 = hole_len / 8;
			if (hole_len1 <= 4)
			{
				printf("vertex num of contours [%d] < 4\n", i);
				continue;
			}

			double vertex_per_line = (double)hole_len / hole_len1;
			for (int j = 0; j < hole_len1; j++)
			{
				int real_j = (int)(j*vertex_per_line + 0.5);
				if (real_j >= hole_len)
					real_j = hole_len - 1;
				pts.push_back(contours[i].hole_polygon[nn][real_j]);
			}
			std::vector<int> cur_hole_indices;
			for (int j = 0; j < hole_len1; j++)
				cur_hole_indices.push_back(j+offset);
			hole_indices.push_back(cur_hole_indices);
			offset += hole_len1;
		}

		//PolyLine to CDT
		
		cdt.SetPolygon(pts, poly_indices, hole_indices);
		cdt.Triangulate();
		triangles = cdt.GetTriangleIndices();
		DrawCDT(cdt_img, pts, poly_indices, hole_indices, triangles, cv::Scalar(0, 255, 0), cv::Scalar(255,100,0), cv::Scalar(0, 0, 255));
	}

	cv::namedWindow("CDT");
	cv::imshow("CDT", cdt_img);
	cv::waitKey(0);

	delete[]data;
	return EXIT_SUCCESS;
}

void DrawCDT(cv::Mat& img, const std::vector<ZQ_Vec2D>& pts, const std::vector<int>& poly_indices,
	const std::vector<std::vector<int>>& hole_indices,
	std::vector<int>& triangles, cv::Scalar out_color, cv::Scalar hole_color, cv::Scalar interior_color)
{

	for (int i = 0; i < triangles.size(); i+=3)
	{
		ZQ_Vec2D p0 = pts[triangles[i]];
		ZQ_Vec2D p1 = pts[triangles[i+1]];
		ZQ_Vec2D p2 = pts[triangles[i+2]];
		cv::line(img, cv::Point(p0.x, p0.y), cv::Point(p1.x, p1.y), interior_color, 1);
		cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), interior_color, 1);
		cv::line(img, cv::Point(p2.x, p2.y), cv::Point(p0.x, p0.y), interior_color, 1);
	}
	
	for (int i = 0; i < poly_indices.size(); i++)
	{
		int j = (i == poly_indices.size() - 1) ? 0 : (i + 1);
		int id0 = poly_indices[i];
		int id1 = poly_indices[j];
		cv::line(img, cv::Point(pts[id0].x, pts[id0].y), cv::Point(pts[id1].x, pts[id1].y), out_color, 2);
	}

	for (int nn = 0; nn < hole_indices.size(); nn++)
	{
		for (int i = 0; i < hole_indices[nn].size(); i++)
		{
			int j = (i == hole_indices[nn].size() - 1) ? 0 : (i + 1);
			int id0 = hole_indices[nn][i];
			int id1 = hole_indices[nn][j];
			cv::line(img, cv::Point(pts[id0].x, pts[id0].y), cv::Point(pts[id1].x, pts[id1].y), hole_color, 2);
		}
	}
}

