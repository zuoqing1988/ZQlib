#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "ZQ_CPURayCasting.h"
#include "opencv2\opencv.hpp"

using namespace ZQ;

unsigned int win_width = 320;
unsigned int win_height = 320;

float* volumeData = 0;
bool zAxis_in = true;
ZQ_CPURayCasting m_CPURaycast(zAxis_in);

bool initVolumeData(float*& vdata, int size, char model[]);
bool Init();
void ShutDown();

int main()
{
	srand(time(NULL));
	
	if(!Init())
	{
		printf("init fail\n");
		return EXIT_FAILURE;
	}

	int count = 0;
	while(count ++ < 1000)
	{
		float* renderBuffer = new float[win_height*win_width*4];
		ZQ_CPURayCasting::ColorFormat color_fmt = ZQ_CPURayCasting::COLOR_BGRA;
		float angle = count * 0.05f;
		float worldMatrix[16] = {
			cos(angle), 0.0f, -sin(angle) * (zAxis_in ? 1 : -1), 0.0f,
			0.0f,		1.0f,  0.0f, 0.0f,
			sin(angle) * (zAxis_in ? 1 : -1), 0.0f, cos(angle), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f };
		m_CPURaycast.SetWorldMatrix(worldMatrix);

		m_CPURaycast.RenderToBuffer(renderBuffer,64,color_fmt);

		cv::Mat rc_im = cv::Mat(win_height, win_width, CV_MAKETYPE(8, 4));
		for(int h = 0;h < win_height;h++)
		{
			for(int w = 0;w < win_width;w++)
			{	
				int offset = h*win_width+w;
				rc_im.ptr<uchar>(h)[w * 4 + 0] = __min(255, __max(0, renderBuffer[4 * offset + 0] * 255));
				rc_im.ptr<uchar>(h)[w * 4 + 1] = __min(255, __max(0, renderBuffer[4 * offset + 1] * 255));
				rc_im.ptr<uchar>(h)[w * 4 + 2] = __min(255, __max(0, renderBuffer[4 * offset + 2] * 255));
				rc_im.ptr<uchar>(h)[w * 4 + 3] = __min(255, __max(0, renderBuffer[4 * offset + 3] * 255));
			}
		}
		delete []renderBuffer;
		cv::flip(rc_im, rc_im, 0);
		cv::namedWindow("ZQ_RayCasting");
		cv::imshow("ZQ_RayCasting", rc_im);
		cv::waitKey(30);
	}
	cv::destroyWindow("ZQ_RayCasting");
	
	ShutDown();
	return EXIT_SUCCESS;
}

bool Init()
{
	int volumeSize = 32;
	const float densityScale = 0.80f;
	const float opacityScale = 0.0001f;
	if(!initVolumeData(volumeData,volumeSize,"ball"))
		return false;

	const float pi = 3.1415926f;

	ZQ_Vec3D boxmin(-1,-1,-1),boxmax(1,1,1);

	float worldMatrix[16] = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f };
	
	float viewMatrix[16] = {   
		1.0f,	0.0f,		0.0f,		0.0f,
		0.0f,	cos(0.2f),	-sin(0.2f) * (zAxis_in ? 1 : -1),	0.0f,
		0.0f,	sin(0.2f) * (zAxis_in ? 1 : -1), cos(0.2f),	10.0f * (zAxis_in ? 1 : -1),
		0.0f,		0.0f,	0.0f,		1.0f};

	float fovy = pi/6;
	float focal_len = win_height/tan(fovy*0.5);
	m_CPURaycast.SetWindowSize(win_width,win_height);
	m_CPURaycast.SetInnerPara(win_width/2,win_height/2,focal_len,focal_len);
	m_CPURaycast.SetVolumeData(volumeData,volumeSize,volumeSize,volumeSize);
	m_CPURaycast.SetVolumeDensityScale(densityScale);
	m_CPURaycast.SetOpacityScale(opacityScale);
	m_CPURaycast.SetVolumeBoundingBox(boxmin,boxmax);
	m_CPURaycast.SetViewMatrix(viewMatrix);
	m_CPURaycast.SetWorldMatrix(worldMatrix);
	return true;
}

void ShutDown()
{
	if(volumeData)
	{
		delete []volumeData;
		volumeData = 0;
	}
}


bool initVolumeData(float*& vdata, int size, char model[])
{
	int xsize = size;
	int ysize = size;
	int zsize = size;

	vdata = new float[xsize*ysize*zsize];
	int k_start = zAxis_in ? (zsize - 1) : 0;
	int k_end = zAxis_in ? -1 : zsize;
	int k_step = zAxis_in ? -1 : 1;
	for(int k = k_start;k != k_end;k+= k_step)
	{
		for(int j = 0;j < ysize;j++)
		{
			for(int i = 0;i < xsize;i++)
			{
				float value = 0.0f;
				if(strcmp(model,"ball") == 0)
				{
					value = sqrt((i - (xsize / 2.0f))*(i - (xsize / 2.0f)) 
						+ (j - (ysize / 2.0f)) * (j - (ysize / 2.0f)) 
						+ (k - (zsize / 2.0f)) * (k - (zsize / 2.0f)));
				}
				else if(strcmp(model,"hypoid") == 0)
				{
					float d1 = sqrt((i - (xsize / 4.0f)) * (i - (xsize / 4.0f))
						+ (j - (ysize / 2.0f)) * (j - (ysize / 2.0f))
						+ (k - (zsize / 2.0f)) * (k - (zsize / 2.0f)));
					float d2 = sqrt((i - (xsize * 3.0f / 4.0f)) * (i - (xsize * 3.0f / 4.0f))
						+ (j - (ysize / 2.0f)) * (j - (ysize / 2.0f))
						+ (k - (zsize / 2.0f)) * (k - (zsize / 2.0f)));
					value = fabs(d1 - d2);
				}
				else if(strcmp(model,"four-core") == 0)
				{
					float d[4] = {0};
					int w[4][3] = {
						{0,0,0},
						{1,1,0},
						{0,1,1},
						{1,0,1}};
					for(int s = 0;s < 4;s++)
					{
						d[s] = sqrt((i - (xsize - 1.0f) * w[s][0]) * (i - (xsize - 1.0f) * w[s][0])
						+ (j - (ysize - 1.0f) * w[s][1]) * (j - (ysize - 1.0f) * w[s][1])
						+ (k - (zsize - 1.0f) * w[s][2]) * (k - (zsize - 1.0f) * w[s][2]));
					}
					
					float max = d[0], min = d[0];
					for(int s = 1;s < 4;s++)
					{
						if(d[s] < min)
							min = d[s];
						if(d[s] > max)
							max = d[s];
					}
					value = (max - min) / 1.414f;
				}
				else if(strcmp(model,"eight-core") == 0)
				{
					float d[8] = {0};
					int w[8][3] = {
						{0,0,0},
						{1,0,0},
						{0,1,0},
						{1,1,0},
						{0,0,1},
						{1,0,1},
						{0,1,1},
						{1,1,1}};
					for(int s = 0;s < 8;s++)
					{
						d[s] = sqrt((i - (xsize - 1.0f) * w[s][0]) * (i - (xsize - 1.0f) * w[s][0])
						+ (j - (ysize - 1.0f) * w[s][1]) * (j - (ysize - 1.0f) * w[s][1])
						+ (k - (zsize - 1.0f) * w[s][2]) * (k - (zsize - 1.0f) * w[s][2]));
					}
					float min = d[0];
					for(int s = 1;s < 8; s++)
					{
						if(min > d[s])
							min = d[s];
					}
					value = fabs(min - (float)size * 1.732 / 2.0f) / ((float)size * 1.732 / 2.0f) * (float)(size);
				}
				else
				{
					delete []vdata;
					vdata = 0;
					return false;
				}
				vdata[k*ysize*xsize+j*xsize+i] = value / (float)size;
			}	
		}
	}
	return true;
}