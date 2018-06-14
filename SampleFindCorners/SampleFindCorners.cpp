#include "ZQ_FindCorners.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;

int main()
{
	const char* file = "cam1_image0.jpg";
	ZQ_DImage<float> img;
	ZQ_ImageIO::loadImage(img, file, 0);
	//img.GaussianSmoothing(2, 3);
	ZQ_DImage<float> grid;
	bool sus = 0;
	if (!ZQ_FindCorners::FindCorners(img, grid, sus))
	{
		printf("failed to extract grid!");
		return -1;
	}
	printf("done!\n");
	IplImage* show_img = cvLoadImage(file, 1);
	int width = grid.width();
	int height = grid.height();
	float*& grid_data = grid.data();
	for (int h = 0; h < height; h++)
	{
		CvScalar scal = cvScalar((height - 1.0 - h) / height*255.0, (h + 1.0) / height*255.0, 0);
		for (int w = 0; w < width; w++)
		{
			int off = h*width + w;
			int x = grid_data[off * 2 + 0] + 0.5;
			int y = grid_data[off * 2 + 1] + 0.5;
			cvCircle(show_img, cvPoint(x, y), 5, scal, 2);
		}
	}
	cvNamedWindow("show", 1);
	cvShowImage("show", show_img);
	cvWaitKey(0);
	cvReleaseImage(&show_img);
	return 0;
}

int main2()
{
	bool flag[] =
	{
		1, 1, 1, 1, 1,
		0, 0, 0, 0, 1,
		1, 1, 1, 1, 1,
		1, 0, 1, 0, 1,
		0, 1, 1, 0, 1
	};
	int width = 5;
	int height = 5;
	int connect_N = 4;
	std::vector<int> area_size1, area_size2;
	int* label1 = new int[width*height];
	int* label2 = new int[width*height];
	ZQ_BinaryImageProcessing::BWlabel_naive(flag, width, height, label1, area_size1, connect_N);
	ZQ_BinaryImageProcessing::BWlabel(flag, width, height, label2, area_size2, connect_N);
	return 0;
}