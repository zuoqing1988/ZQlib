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
	cv::Mat output;

	int dstWidth = 440;
	int dstHeight = 300;

	int rects[11][4] =
	{
		{ 175,50,371,75 },		//证号
		{ 54,78,194,105 },		//姓名
		{ 238,78,277,105 },		//性别
		{ 318,78,417,105 },		//国籍
		{ 52,105,414,132 },		//地址第一行
		{ 52,132,414,159 },		//地址第二行
		{ 178,160,311,190 },	//生日
		{ 203,194,311,220 },	//发证日期
		{ 183,225,311,254 },	//类型
		{ 87,256,192,285 },		//有效期第一格
		{ 213,256,325,285 }		//有效期第二格
	};
	
	float scale = 2.0;
	dstWidth *= scale;
	dstHeight *= scale;
	for (int i = 0; i < 11; i++)
	{
		for (int j = 0; j < 4; j++)
			rects[i][j] *= scale;
	}

#if defined(_DEBUG)
	bool display = true;
#else
	bool display = false;
#endif

	cv::Mat transmat;
	cv::Mat smooth_img;
	img.copyTo(smooth_img);
	cv::GaussianBlur(smooth_img, smooth_img, cv::Size(3, 3), 0);
	if (ZQ_RectifyCard::RectifyDriverCard(smooth_img, dstWidth, dstHeight, output, transmat, 50, display))
	{
		cv::warpPerspective(img, output, transmat, cv::Size(dstWidth, dstHeight));
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
	cv::Mat transmat;
	if (ZQ_RectifyCard::RectifyCard(img, dstWidth, dstHeight, output, transmat, 50, display))
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
	test_drivercard();
	//test_IDcard();
	
	return 0;
}