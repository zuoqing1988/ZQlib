#include "ZQ_CvCapture_FFMPEG.h"
#include "opencv\cv.h"
#include "opencv\highgui.h"

int main()
{
	printf("WARNING: current version can only handle I-P frame, B frame is not supported\n");
	const char* file = "C:\\Users\\ZQ\\Desktop\\ffmpeg-3.4.1-win64-shared\\bin\\output4.mp4";
	ZQ::ZQ_CvCapture_FFMPEG decode(file);
	if (!decode.IsOpened())
	{
		printf("failed to open %s\n", file);
		return 0;
	}
	ZQ::ZQ_Image_FFMPEG fr;
	int count = 0;
	//decode.Seek((int64_t)8688);
	decode.Seek((int64_t)0);

	while (true)
	{
		if (!decode.Read(fr))
			break;

		IplImage* img = cvCreateImageHeader(cvSize(fr.width, fr.height), IPL_DEPTH_8U, fr.cn);
		img->imageData = (char*)fr.data;
		cvNamedWindow("show");
		cvShowImage("show", img);
		cvWaitKey(24);
		cvReleaseImageHeader(&img);
	}

	return 0;
}