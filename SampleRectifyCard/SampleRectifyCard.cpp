#include "ZQ_RectifyCard.h"
using namespace ZQ;
using namespace cv;

void test_rect()
{
	cv::Mat img = cv::imread("8.jpg");
	cv::Mat output;

	
#if defined(_DEBUG)
	bool display = true;
#else
	bool display = false;
#endif
	ZQ_Vec2D pt_lt, pt_lb, pt_rt, pt_rb;
	if (ZQ_RectifyCard::DetectRectCorners(img,pt_lt,pt_lb,pt_rt,pt_rb,2,0,display))
	{
		
	}
}

void test_drivercard()
{
	cv::Mat img = cv::imread("7.jpg");
	const char* out_name = "dst.jpg";
	cv::Rect red_roi(cv::Point(52, 146), cv::Point(151, 243));
	cv::Rect c1_roi(cv::Point(184, 200), cv::Point(310, 240));
	/*cv::Mat img = cv::imread("1.jpg");
	cv::Rect red_roi(cv::Point(42, 241), cv::Point(194, 394));
	cv::Rect c1_roi(cv::Point(260, 340), cv::Point(461, 397));*/
	cv::Mat output;

	int dstWidth = 440;
	int dstHeight = 300;

	int rects[11][4] =
	{
		{175,49,371,71},
		{52,76,194,101},
		{238,76,277,101},
		{318,76,417,101},
		{52,105,414,132},
		{52,126,414,159},
		{178,158,311,188},
		{201,192,311,217},
		{183,225,311,252},
		{87,254,192,283},
		{213,256,319,283}
	};
	dstWidth *= 1.0;
	dstHeight *= 1.0;
	for (int i = 0; i < 11; i++)
	{
		for (int j = 0; j < 4; j++)
			rects[i][j] *= 1.0;
	}

#if defined(_DEBUG)
	bool display = true;
#else
	bool display = false;
#endif

	
	if (ZQ_RectifyCard::RectifyDriverCard(img, dstWidth, dstHeight, output, red_roi, c1_roi, display))
	{
		for (int i = 0; i < 11; i++)
		{
			cv::rectangle(output, cv::Rect(cv::Point(rects[i][0], rects[i][1]), cv::Point(rects[i][2], rects[i][3])), cv::Scalar(0, 0, 255), 1);
		}
		cv::imwrite(out_name, output);
		cv::namedWindow("src");
		cv::namedWindow("dst");
		cv::imshow("src", img);
		cv::imshow("dst", output);
		cv::waitKey(0);

	}
}

void test_IDcard()
{
	cv::Mat img = cv::imread("3.jpg");
	cv::Mat output;

	int dstWidth = 430;
	int dstHeight = 270;

#if defined(_DEBUG)
	bool display = true;
#else
	bool display = false;
#endif

	if (ZQ_RectifyCard::RectifyCard(img, dstWidth, dstHeight, output, 50, display))
	{
		cv::imwrite("3_dst.jpg", output);
		cv::namedWindow("src");
		cv::namedWindow("dst");
		cv::imshow("src", img);
		cv::imshow("dst", output);
		cv::waitKey(0);

	}
}

int main()
{
	//test_rect();
	//test_drivercard();
	test_IDcard();
	
	return 0;
}