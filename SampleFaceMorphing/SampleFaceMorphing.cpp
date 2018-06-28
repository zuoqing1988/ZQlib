#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "facedetect-dll.h"
#include "ZQ_Warping.h"

//#pragma comment(lib,"libfacedetect.lib")
#pragma comment(lib,"libfacedetect-x64.lib")

//define the buffer size. Do not change the size!
#define DETECT_BUFFER_SIZE 0x20000
using namespace cv;
using namespace ZQ;

int main()
{
	std::string files[2] = { "obama.jpg","trump.jpg" };
	cv::Mat rgb_faces[2], gray_faces[2];
	std::vector<float> landmarks[2];
	int target_width = 200, target_height = 200;
	std::vector<float> stand_coords(target_width*target_height*2);
	std::vector<float> sample_coords0(target_width*target_height * 2);
	std::vector<float> sample_coords1(target_width*target_height * 2);
	for (int h = 0; h < target_height; h++)
	{
		for (int w = 0; w < target_width; w++)
		{
			int offset = h*target_width + w;
			stand_coords[offset * 2 + 0] = w;
			stand_coords[offset * 2 + 1] = h;
		}
	}
	for (int i = 0; i < 2; i++)
	{
		rgb_faces[i] = cv::imread(files[i], 1);
		if (rgb_faces[i].empty())
		{
			printf("failed to load image %s\n", files[i].c_str());
			return EXIT_FAILURE;
		}
		cv::resize(rgb_faces[i], rgb_faces[i], cv::Size(target_width, target_height));
		cv::cvtColor(rgb_faces[i], gray_faces[i], CV_BGR2GRAY);
	}


	int * pResults = NULL;
	//pBuffer is used in the detection functions.
	//If you call functions in multiple threads, please create one buffer for each thread!
	unsigned char * pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
	if (!pBuffer)
	{
		fprintf(stderr, "Can not alloc buffer.\n");
		return -1;
	}

	///////////////////////////////////////////
	// frontal face detection / 68 landmark detection
	// it's fast, but cannot detect side view faces
	//////////////////////////////////////////
	//!!! The input image must be a gray one (single-channel)
	//!!! DO NOT RELEASE pResults !!!

	cv::Mat show_landmarks[2];
	show_landmarks[0] = rgb_faces[0].clone();
	show_landmarks[1] = rgb_faces[1].clone();

	

	for (int i = 0; i < 2; i++)
	{
		pResults = facedetect_frontal(pBuffer, (unsigned char*)(gray_faces[i].ptr(0)), gray_faces[i].cols, gray_faces[i].rows, (int)gray_faces[i].step,
			1.2f, 2, 48, 0, 1);

		int detected_num = pResults ? *pResults : 0;
		if (detected_num != 1)
		{
			printf("failed to detect face in %s\n", files[i].c_str());
			free(pBuffer);
			return EXIT_FAILURE;
		}
		short * p = ((short*)(pResults + 1));
		int x = p[0];
		int y = p[1];
		int w = p[2];
		int h = p[3];
		int neighbors = p[4];
		int angle = p[5];

		for (int j = 0; j < 68; j++)
		{
			landmarks[i].push_back(p[6 + 2 * j]);
			landmarks[i].push_back(p[6 + 2 * j + 1]);
		}

		for (int j = 0; j < 68; j++)
			circle(show_landmarks[i], cv::Point((int)p[6 + 2 * j], (int)p[6 + 2 * j + 1]), 1, cv::Scalar(0, 255, 0));
	}
	
	cv::namedWindow("landmark0");
	cv::namedWindow("landmark1");
	cv::imshow("landmark0", show_landmarks[0]);
	cv::imshow("landmark1", show_landmarks[1]);

	cv::Mat map0x, map0y, map1x, map1y;
	map0x.create(cv::Size(target_width, target_height), CV_32FC1);
	map0y.create(cv::Size(target_width, target_height), CV_32FC1);
	map1x.create(cv::Size(target_width, target_height), CV_32FC1);
	map1y.create(cv::Size(target_width, target_height), CV_32FC1);

	int landmark_num = landmarks[0].size() / 2;
	ZQ_Warping<float, 2> warp;
	float ori_step = 0.1;
	float step = ori_step;
	for (float t = 0;; t += step)
	{
		if (t < 0)
			step = ori_step;
		if (t > 1)
			step = -ori_step;
		t = __min(1, __max(0, t));
		std::vector<float> tmp_pts;
		for (int i = 0; i < landmarks[0].size(); i++)
		{
			tmp_pts.push_back(landmarks[0][i] * (1 - t) + landmarks[1][i] * t);
		}
		warp.Solve(landmark_num, &tmp_pts[0], &landmarks[0][0], 100, 5, 3);
		warp.WarpCoord(target_width*target_height, &stand_coords[0], &sample_coords0[0]);
		warp.Solve(landmark_num, &tmp_pts[0], &landmarks[1][0], 100, 5, 3);
		warp.WarpCoord(target_width*target_height, &stand_coords[0], &sample_coords1[0]);

		
		for (int h = 0; h < target_height; h++)
		{
			for (int w = 0; w < target_width; w++)
			{
				int offset = h*target_width + w;
				map0x.ptr<float>(h)[w] = __max(0, __min(target_width - 1, sample_coords0[offset * 2]));
				map0y.ptr<float>(h)[w] = __max(0, __min(target_height - 1, sample_coords0[offset * 2 + 1]));
				map1x.ptr<float>(h)[w] = __max(0, __min(target_width - 1, sample_coords1[offset * 2]));
				map1y.ptr<float>(h)[w] = __max(0, __min(target_height - 1, sample_coords1[offset * 2 + 1]));
			}
		}
		cv::Mat warp_face0(target_height, target_width, CV_MAKETYPE(8, 3));
		cv::Mat warp_face1(target_height, target_width, CV_MAKETYPE(8, 3));
		cv::remap(rgb_faces[0], warp_face0, map0x, map0y, cv::INTER_LINEAR);
		cv::remap(rgb_faces[1], warp_face1, map1x, map1y, cv::INTER_LINEAR);
		cv::Mat final_face;
		cv::addWeighted(warp_face0, 1 - t, warp_face1, t, 0, final_face);
		cv::namedWindow("morphing");
		cv::imshow("morphing", final_face);
		int key = cv::waitKey(20);
		if (key == 'Q' || key == 'q')
			break;
	}

	//release the buffer
	free(pBuffer);

	return EXIT_SUCCESS;
}

