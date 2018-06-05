#include "ZQ_CvCapture_FFMPEG.h"
#include "cv.h"
#include "highgui.h"

int main()
{
	const char* file = "C:\\360极速浏览器下载\\videos\\乡村爱情9EP01.mp4";
	ZQ::ZQ_CvCapture_FFMPEG decode(file);
	if (!decode.IsOpened())
	{
		printf("failed to open %s\n", file);
		return 0;
	}
	ZQ::ZQ_Image_FFMPEG fr;
	int count = 0;
	decode.Seek((int64_t)8688);

	while (true)
	{
		if (!decode.Read(fr))
			break;

		IplImage* img = cvCreateImageHeader(cvSize(fr.width, fr.height), IPL_DEPTH_8U, fr.cn);
		img->imageData = (char*)fr.data;
		cvNamedWindow("show");
		cvShowImage("show", img);
		cvWaitKey(0);
		cvReleaseImageHeader(&img);
	}

	return 0;
}