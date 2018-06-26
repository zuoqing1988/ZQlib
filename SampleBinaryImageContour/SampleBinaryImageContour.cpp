#include "ZQ_BinaryImageContour.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;
int main()
{
	const char* inputfile = "mask.png";
	ZQ_DImage<float> im;
	if(!ZQ_ImageIO::loadImage(im,inputfile,0))
	{
		printf("failed to load %s\n",inputfile);
		return EXIT_FAILURE;
	}

	int width = im.width();
	int height = im.height();

	ZQ_DImage<bool> mask(width,height);
	bool* mask_data = mask.data();
	float* im_data = im.data();
	for(int i = 0;i < width*height;i++)
		mask_data[i] = im_data[i] > 0.5;

	std::vector<ZQ_BinaryImageContour::Contour> contours;

	if(!ZQ_BinaryImageContour::GetBinaryImageContour(mask_data,width,height,contours))
	{
		printf("failed to find contours\n");
		return EXIT_FAILURE;
	}

	cv::Mat show_img = cv::Mat(height, width, CV_MAKETYPE(8, 3),cv::Scalar(0));

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if (mask_data[i*width + j])
			{
				show_img.data[i*show_img.step[0] + j * 3] = 255;
				show_img.data[i*show_img.step[0] + j * 3 + 1] = 255;
				show_img.data[i*show_img.step[0] + j * 3 + 2] = 255;
			}
			else
			{
				show_img.data[i*show_img.step[0] + j * 3] = 0;
				show_img.data[i*show_img.step[0] + j * 3 + 1] = 0;
				show_img.data[i*show_img.step[0] + j * 3 + 2] = 0;
			}	
		}
	}

	for(int cc = 0;cc < contours.size();cc++)
	{
		for(int i = 0;i < contours[cc].outer_polygon.size();i++)
		{
			ZQ_Vec2D pt1 = contours[cc].outer_polygon[i];
			ZQ_Vec2D pt2 = contours[cc].outer_polygon[(i+1)%contours[cc].outer_polygon.size()];
			cv::line(show_img,cv::Point(pt1.x,pt1.y),cv::Point(pt2.x,pt2.y),cv::Scalar(0,0,255),2);
		}
		for (int nn = 0; nn < contours[cc].hole_polygon.size(); nn++)
		{
			for (int i = 0; i < contours[cc].hole_polygon[nn].size(); i++)
			{
				ZQ_Vec2D pt1 = contours[cc].hole_polygon[nn][i];
				ZQ_Vec2D pt2 = contours[cc].hole_polygon[nn][(i + 1) % contours[cc].hole_polygon[nn].size()];
				cv::line(show_img, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), cv::Scalar(0, 255, 0), 2);
			}
		}
	}
	cv::namedWindow("show");
	cv::imshow("show",show_img);
	cv::waitKey(0);
	return EXIT_SUCCESS;
}
