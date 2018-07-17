#include "ZQ_CPURenderer2DWorkspace.h"
#include "ZQ_DoubleImage.h"
#include <opencv2\opencv.hpp>
using namespace ZQ;

int main()
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
				memcpy(tex_img.data() + (i*tex_width + j) * 4, color_white, sizeof(float)* 4);
			else
				memcpy(tex_img.data() + (i*tex_width + j) * 4, color_black, sizeof(float)* 4);
		}
	}
	ZQ_TextureSampler<float> sampler;
	sampler.BindImage(tex_img, false);

	ZQ_CPURenderer2DWorkspace render2D(640, 480);
	render2D.ClearColorBuffer(0, 0, 0, 0);
	render2D.ClearDepthBuffer(1000);
	
	float vertices[] =
	{
		100, 100, 1000, 0, 0,
		100, 500, 1000, 0, 1,
		400, 100, 800, 1, 0,
		400, 400, 800, 1, 1,
		200, 200, 600, 0, 0,
		300, 50, 600, 1, 0,
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
	render2D.BindSampler(&sampler);
	//render2D.EnableTextureSampleCubic();
	render2D.EnableDepthTest();
	//render2D.DisableDepthTest();
	//render2D.EnableAlphaBlend();
	render2D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_ONE_DST_ONE_MINUS_SRC);
	//render2D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_PLUS_DST);
	render2D.RenderIndexedTriangles(vertices, indices, 8, 4, ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_TEXCOORD2);

	int buffer_width2D = render2D.GetBufferWidth();
	int buffer_height2D = render2D.GetBufferHeight();
	const float*& buffer_data2D = render2D.GetColorBufferPtr();
	IplImage* show_img2D = cvCreateImage(cvSize(buffer_width2D, buffer_height2D), IPL_DEPTH_8U, 3);
	for (int i = 0; i < buffer_height2D; i++)
	{
		for (int j = 0; j < buffer_width2D; j++)
		{
			cvSet2D(show_img2D, buffer_height2D - 1 - i, j, cvScalar(buffer_data2D[(i*buffer_width2D + j) * 4 + 2] * 255, buffer_data2D[(i*buffer_width2D + j) * 4 + 1] * 255, buffer_data2D[(i*buffer_width2D + j) * 4 + 0] * 255));
		}
	}


	cvNamedWindow("show2D");
	cvShowImage("show2D", show_img2D);
	cvWaitKey(0);
	cvReleaseImage(&show_img2D);

	return EXIT_SUCCESS;
}