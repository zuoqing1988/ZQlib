#include "ZQ_RectifyCard.h"
using namespace ZQ;
using namespace cv;
int main()
{
	cv::Mat img = cv::imread("2.jpg");
	cv::Mat output;

	int dstWidth = 444;
	int dstHeight = 280;

#if defined(_DEBUG)
	bool display = true;
#else
	bool display = false;
#endif

	if (ZQ_RectifyCard::RectifyCard(img, dstWidth,dstHeight, output, display))
	{
		cv::namedWindow("src");
		cv::namedWindow("dst");
		cv::imshow("src", img);
		cv::imshow("dst", output);
		cv::waitKey(0);
	}
	return 0;
}