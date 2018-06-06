#include "ZQ_CPURenderer3DWorkspace.h"
#include "ZQ_DoubleImage.h"
#include "opencv\cv.h"
#include "opencv\highgui.h"

using namespace ZQ;

void main()
{
	int tex_width = 10;
	int tex_height = 10;
	ZQ_DImage<float> tex_img(tex_width, tex_height, 4);
	float color_black[4] = { 1, 0, 0, 0.1 };
	float color_white[4] = { 0, 1, 0, 0.1 };
	for (int i = 0; i < tex_height; i++)
	{
		for (int j = 0; j < tex_width; j++)
		{
			if ((i + j) % 2 == 0)
			{
				memcpy(tex_img.data() + (i*tex_width + j) * 4, color_white, sizeof(float)* 4);
			}
			else
			{
				memcpy(tex_img.data() + (i*tex_width + j) * 4, color_black, sizeof(float)* 4);
			}
		}
	}
	ZQ_TextureSampler<float> sampler;
	sampler.BindImage(tex_img, false);

	ZQ_CPURenderer3DWorkspace render3D(640, 480, false);
	render3D.ClearColorBuffer(0, 0, 0, 0);
	render3D.ClearDepthBuffer(1000);
	render3D.SetClip(10, 10000);

	float vertices[] =
	{
		-100, -100, 500, 0, 0,
		100, -100, 500, 0, 1,
		-100, 100, 300, 1, 0,
		100, 100, 300, 1, 1,
		-200, 200, 600, 0, 0,
		-300, 50, 600, 1, 0,
		400, 450, 500, 0, 1,
		500, 350, 500, 1, 1
	};
	int indices[12] =
	{
		0, 1, 2,
		1, 2, 3,
		4, 5, 6,
		5, 6, 7
	};
	
	render3D.BindSampler(&sampler);
	//render3D.EnableTextureSampleCubic();
	//render3D.DisableDepthTest();
	render3D.EnableDepthTest();
	//render3D.EnableAlphaBlend();
	//render3D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_ONE_DST_ONE_MINUS_SRC);
	//render3D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_PLUS_DST);

	render3D.RenderIndexedTriangles(vertices, indices, 8, 4, ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_TEXCOORD2);

	int buffer_width3D = render3D.GetBufferWidth();
	int buffer_height3D = render3D.GetBufferHeight();
	const float*& buffer_data3D = render3D.GetColorBufferPtr();
	IplImage* show_img3D = cvCreateImage(cvSize(buffer_width3D, buffer_height3D), IPL_DEPTH_8U, 3);
	for (int i = 0; i < buffer_height3D; i++)
	{
		for (int j = 0; j < buffer_width3D; j++)
		{
			cvSet2D(show_img3D, buffer_height3D - 1 - i, j, cvScalar(buffer_data3D[(i*buffer_width3D + j) * 4 + 2] * 255, buffer_data3D[(i*buffer_width3D + j) * 4 + 1] * 255, buffer_data3D[(i*buffer_width3D + j) * 4 + 0] * 255));
		}
	}
	cvNamedWindow("show3D");
	cvShowImage("show3D", show_img3D);
	cvWaitKey(0);
	cvReleaseImage(&show_img3D);
}