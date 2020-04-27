#ifndef _ZQ_RECTIFY_CARD_H_
#define _ZQ_RECTIFY_CARD_H_
#pragma once

#include <opencv2/opencv.hpp>

#include <time.h>
#include <iostream>
#include <vector>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ZQ_Vec2D.h"
#include "ZQ_SVD.h"
#include "ZQ_LSD.h"

namespace ZQ
{
	class ZQ_RectifyCard
	{
		class ZQ_LineSeg
		{
		public:
			ZQ_Vec2D pt0, pt1;

			float Length() const
			{
				float x = pt1.x - pt0.x;
				float y = pt1.y - pt0.y;
				return sqrt(x*x + y*y);
			}

			float Angle() const
			{
				const double m_pi = 3.1415926535;
				float x = pt1.x - pt0.x;
				float y = pt1.y - pt0.y;
				if (x >= 0)
				{
					return atan2(y, x) / m_pi * 180;
				}
				else
				{
					return atan2(-y, -x) / m_pi * 180;
				}
			}

			float DistanceTo(const ZQ_Vec2D pt) const
			{
				ZQ_Vec2D dir01(pt1.x - pt0.x, pt1.y - pt0.y);
				if (!dir01.Normalized())
					return 0;
				else
				{
					ZQ_Vec2D dir02(pt.x - pt0.x, pt.y - pt0.y);
					float proj_len = dir02.DotProduct(dir01);
					ZQ_Vec2D dir02_p = dir02 - dir01*proj_len;
					return dir02_p.Length();
				}
			}
		};

		class ZQ_LineSegWithInfo
		{
		public:
			ZQ_LineSeg line;
			float angle;
			float length;
			float distance;

			void UpdateInfo(const ZQ_Vec2D center)
			{
				angle = line.Angle();
				length = line.Length();
				distance = line.DistanceTo(center);
			}
		};

		class ZQ_Ray
		{
		public:
			ZQ_Vec2D ori, dir;

			static bool RayCross(const ZQ_Ray& ray1, const ZQ_Ray& ray2, float& depth1, float& depth2, ZQ_Vec2D& crossPos)
			{
				if (ray1.dir.Length() == 0)
					return false;
				if (ray2.dir.Length() == 0)
					return false;

				ZQ_Matrix<float> matA(2, 2);
				ZQ_Matrix<float> Ainv(2, 2);
				ZQ_Matrix<float> matB(2, 1);

				matA.SetData(0, 0, ray1.dir.x);
				matA.SetData(0, 1, -ray2.dir.x);
				matA.SetData(1, 0, ray1.dir.y);
				matA.SetData(1, 1, -ray2.dir.y);

				matB.SetData(0, 0, ray2.ori.x - ray1.ori.x);
				matB.SetData(1, 0, ray2.ori.y - ray1.ori.y);

				ZQ_SVD::Invert(matA, Ainv);

				ZQ_Matrix<float> matX = Ainv*matB;

				bool flag;
				depth1 = matX.GetData(0, 0, flag);
				depth2 = matX.GetData(1, 0, flag);

				crossPos = (ray1.ori + ray1.dir*depth1 + ray2.ori + ray2.dir*depth2)*0.5;

				return true;
			}
		};
	private:
		static bool _is_in_rect(cv::Rect& rect, ZQ_Vec2D& pt)
		{
			return pt.x >= rect.x && pt.x < rect.x + rect.width && pt.y >= rect.y && pt.y < rect.y + rect.height;
		}

		static void _choose_line_by_angle(const std::vector<ZQ_LineSegWithInfo>& input, std::vector<ZQ_LineSegWithInfo>& output)
		{
			float bucket[360] = { 0 };
			const int range = 2;

			for (int i = 0; i < input.size(); i++)
			{
				float cur_angle = input[i].angle;
				float cur_len = input[i].length;
				for (int j = -range; j <= range; j++)
				{
					int tmp_angle = cur_angle + j + 0.5;
					tmp_angle = (tmp_angle + 360) % 360;
					bucket[tmp_angle] += cur_len;
				}
			}

			int max_angle = -1;
			float max_bucket = -1;
			for (int i = 0; i < 360; i++)
			{
				if (max_bucket < bucket[i])
				{
					max_bucket = bucket[i];
					max_angle = i;
				}
			}

			output.clear();
			for (int i = 0; i < input.size(); i++)
			{
				float cur_angle = input[i].angle;
				int cur_i = cur_angle + 0.5;
				if (abs((cur_i - max_angle + 360) % 360) <= range)
					output.push_back(input[i]);
			}
		}

		static void _choose_line_by_distance(const std::vector<ZQ_LineSegWithInfo>& input, std::vector<ZQ_LineSegWithInfo>& output)
		{
			float max_distance = -1;
			float min_distance = 1e9;
			for (int i = 0; i < input.size(); i++)
			{
				float cur_dis = input[i].distance;
				max_distance = __max(max_distance, cur_dis);
				min_distance = __min(min_distance, cur_dis);
			}

			int min_dis_i = floor(min_distance);
			int max_dis_i = ceil(max_distance);
			int num_bucket = max_dis_i - min_dis_i + 1;
			std::vector<double> bucket(num_bucket, 0);
			const int range = 1;

			for (int i = 0; i < input.size(); i++)
			{
				float cur_dis = input[i].distance;
				float cur_len = input[i].length;
				for (int j = -range; j <= range; j++)
				{
					int tmp_dis = cur_dis + j + 0.5;
					if (tmp_dis >= min_dis_i && tmp_dis <= max_dis_i)
					{
						bucket[tmp_dis - min_dis_i] += cur_len;
					}
				}
			}

			int max_dis = -1;
			float max_bucket = -1;
			for (int i = 0; i < num_bucket; i++)
			{
				if (max_bucket < bucket[i])
				{
					max_bucket = bucket[i];
					max_dis = i + min_dis_i;
				}
			}

			output.clear();
			for (int i = 0; i < input.size(); i++)
			{
				float cur_dis = input[i].distance;
				int cur_dis_i = cur_dis + 0.5;
				if (abs(cur_dis_i - max_dis) <= range)
				{
					output.push_back(input[i]);
				}
			}
		}

		static bool _regress_line(const std::vector<ZQ_LineSegWithInfo>& input, ZQ_Ray& ray)
		{
			if (input.size() == 0)
				return false;
			ray.ori = ZQ_Vec2D(0, 0);
			ray.dir = ZQ_Vec2D(0, 0);
			for (int i = 0; i < input.size(); i++)
			{
				ray.ori += input[i].line.pt0;
				ray.ori += input[i].line.pt1;
			}
			ray.ori *= 1.0 / (2.0*input.size());
			for (int i = 0; i < input.size(); i++)
			{
				ZQ_Vec2D dir0 = input[i].line.pt0 - ray.ori;
				ZQ_Vec2D dir1 = input[i].line.pt1 - ray.ori;
				if (dir0.x >= 0)
					ray.dir += dir0;
				else
					ray.dir -= dir0;
				if (dir1.x >= 0)
					ray.dir += dir1;
				else
					ray.dir -= dir1;
			}
			return ray.dir.Normalized();
		}

	public:
		static bool RectifyCard(const cv::Mat& src, int dstWidth, int dstHeight, cv::Mat& dst, bool display = false)
		{
			//int dstWidth = 444;
			//int dstHeight = 280;
			cv::Mat m_imResize;
			cv::resize(src, m_imResize, cv::Size(dstWidth, dstHeight));
			//cv::GaussianBlur(m_imResize, m_imResize, cv::Size(3, 3), 0);
			//cv::GaussianBlur(m_imResize, m_imResize, cv::Size(3, 3), 0);
			//cv::GaussianBlur(m_imResize, m_imResize, cv::Size(3, 3), 0);
			//cv::GaussianBlur(m_imResize, m_imResize, cv::Size(3, 3), 0);

			ZQ_LSD::ntuple_list detected_lines;
			clock_t start, finish;
			double duration, rate;
			start = clock();
			cv::Mat im_gray;
			cv::cvtColor(m_imResize, im_gray, CV_BGR2GRAY);
			ZQ_LSD::image_double  image = ZQ_LSD::new_image_double(im_gray.cols, im_gray.rows);
			uchar* im_src = (uchar*)im_gray.data;
			int xsize = image->xsize;
			int ysize = image->ysize;

			for (int y = 0; y < ysize; y++)
			{
				for (int x = 0; x < xsize; x++)  //x是横坐标，y是纵坐标
				{
					image->data[y*xsize + x] = im_src[y*im_gray.step[0] + x];//im_gray是灰度图像，没有颜色通道
				}
			}
			detected_lines = ZQ_LSD::lsd(image);//detected_lines中存储提取直线的首位坐标及宽度，具体意义见说明文档
			ZQ_LSD::free_image_double(image);
			finish = clock();
			duration = (double)(finish - start) / CLOCKS_PER_SEC;
			rate = duration / detected_lines->size;
			if (display)
			{
				std::cout << "total time of extract lines is:" << duration << std::endl;
				std::cout << "time of extract per line is :" << rate << std::endl;
			}

			int dim = detected_lines->dim;
			m_imResize.copyTo(dst);


			int tempWidth = 50;
			ZQ_Vec2D center(dstWidth*0.5, dstHeight*0.5);
			cv::Rect rectL(0, 0, tempWidth, dst.rows);
			cv::Rect rectR(dst.cols - 1 - tempWidth, 0, tempWidth, dst.rows);
			cv::Rect rectT(0, 0, dst.cols, tempWidth);
			cv::Rect rectB(0, dst.rows - 1 - tempWidth, dst.cols, tempWidth);
			if (display)
			{
				cv::rectangle(dst, rectL, cv::Scalar(0, 0, 255));
				cv::rectangle(dst, rectR, cv::Scalar(0, 0, 255));
				cv::rectangle(dst, rectT, cv::Scalar(0, 0, 255));
				cv::rectangle(dst, rectB, cv::Scalar(0, 0, 255));
			}

			std::vector<ZQ_LineSegWithInfo> lineTop, lineBottom, lineLeft, lineRight;

			for (unsigned int j = 0; j < detected_lines->size; j++)
			{
				ZQ_LineSegWithInfo lineseg;
				ZQ_Vec2D& pt0 = lineseg.line.pt0;
				ZQ_Vec2D& pt1 = lineseg.line.pt1;
				pt0 = ZQ_Vec2D(detected_lines->values[j*dim + 0], detected_lines->values[j*dim + 1]);
				pt1 = ZQ_Vec2D(detected_lines->values[j*dim + 2], detected_lines->values[j*dim + 3]);
				lineseg.UpdateInfo(center);

				if (lineseg.length > 10)
				{
					if (_is_in_rect(rectL, pt0) && _is_in_rect(rectL, pt1)
						&& (lineseg.angle < -70 || lineseg.angle > 70))
					{
						if (display)
						{
							cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
						}
						lineLeft.push_back(lineseg);
					}

					if (_is_in_rect(rectR, pt0) && _is_in_rect(rectR, pt1)
						&& (lineseg.angle < -70 || lineseg.angle > 70))
					{
						if (display)
						{
							cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
						}
						lineRight.push_back(lineseg);

					}

					if (_is_in_rect(rectT, pt0) && _is_in_rect(rectT, pt1)
						&& (lineseg.angle > -20 && lineseg.angle < 20))
					{
						if (display)
						{
							cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
						}
						lineTop.push_back(lineseg);
					}

					if (_is_in_rect(rectB, pt0) && _is_in_rect(rectB, pt1)
						&& (lineseg.angle > -20 && lineseg.angle < 20))
					{
						if (display)
						{
							cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
						}
						lineBottom.push_back(lineseg);
					}
				}
			}
			ZQ_LSD::free_ntuple_list(detected_lines);



			if (display)
			{
				cv::namedWindow("show1");
				cv::imshow("show1", dst);
				cv::waitKey(0);
			}

			std::vector<ZQ_LineSegWithInfo> lineT, lineB, lineL, lineR;
			_choose_line_by_angle(lineTop, lineT);
			_choose_line_by_angle(lineBottom, lineB);
			_choose_line_by_angle(lineLeft, lineL);
			_choose_line_by_angle(lineRight, lineR);

			if (display)
			{
				m_imResize.copyTo(dst);
				for (int i = 0; i < lineT.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineT[i].line.pt0;
					ZQ_Vec2D& pt1 = lineT[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineB.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineB[i].line.pt0;
					ZQ_Vec2D& pt1 = lineB[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineL.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineL[i].line.pt0;
					ZQ_Vec2D& pt1 = lineL[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineR.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineR[i].line.pt0;
					ZQ_Vec2D& pt1 = lineR[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				cv::namedWindow("show2");
				cv::imshow("show2", dst);
				cv::waitKey(0);
			}

			lineTop.swap(lineT);
			lineBottom.swap(lineB);
			lineLeft.swap(lineL);
			lineRight.swap(lineR);
			_choose_line_by_distance(lineTop, lineT);
			_choose_line_by_distance(lineBottom, lineB);
			_choose_line_by_distance(lineLeft, lineL);
			_choose_line_by_distance(lineRight, lineR);

			if (display)
			{
				m_imResize.copyTo(dst);
				for (int i = 0; i < lineT.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineT[i].line.pt0;
					ZQ_Vec2D& pt1 = lineT[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineB.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineB[i].line.pt0;
					ZQ_Vec2D& pt1 = lineB[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineL.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineL[i].line.pt0;
					ZQ_Vec2D& pt1 = lineL[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				for (int i = 0; i < lineR.size(); i++)
				{
					ZQ_Vec2D& pt0 = lineR[i].line.pt0;
					ZQ_Vec2D& pt1 = lineR[i].line.pt1;
					cv::line(dst, cv::Point(pt0.x, pt0.y), cv::Point(pt1.x, pt1.y), cv::Scalar(0, 255, 0), 2, CV_AA);
				}
				cv::namedWindow("show3");
				cv::imshow("show3", dst);
				cv::waitKey(0);
			}

			if (lineT.size() == 0 || lineB.size() == 0 || lineL.size() == 0 || lineR.size() == 0)
				return false;

			/* line regression */
			ZQ_Ray rayT, rayB, rayL, rayR;
			_regress_line(lineT, rayT);
			_regress_line(lineB, rayB);
			_regress_line(lineL, rayL);
			_regress_line(lineR, rayR);

			if (display)
			{
				m_imResize.copyTo(dst);
				float max_ray_len = dstWidth + dstHeight;
				cv::line(dst, cv::Point(rayT.ori.x - max_ray_len*rayT.dir.x, rayT.ori.y - max_ray_len*rayT.dir.y),
					cv::Point(rayT.ori.x + max_ray_len*rayT.dir.x, rayT.ori.y + max_ray_len*rayT.dir.y),
					cv::Scalar(0, 255, 255), 1, CV_AA);
				cv::line(dst, cv::Point(rayB.ori.x - max_ray_len*rayB.dir.x, rayB.ori.y - max_ray_len*rayB.dir.y),
					cv::Point(rayB.ori.x + max_ray_len*rayB.dir.x, rayB.ori.y + max_ray_len*rayB.dir.y),
					cv::Scalar(0, 255, 255), 1, CV_AA);
				cv::line(dst, cv::Point(rayL.ori.x - max_ray_len*rayL.dir.x, rayL.ori.y - max_ray_len*rayL.dir.y),
					cv::Point(rayL.ori.x + max_ray_len*rayL.dir.x, rayL.ori.y + max_ray_len*rayL.dir.y),
					cv::Scalar(0, 255, 255), 1, CV_AA);
				cv::line(dst, cv::Point(rayR.ori.x - max_ray_len*rayR.dir.x, rayR.ori.y - max_ray_len*rayR.dir.y),
					cv::Point(rayR.ori.x + max_ray_len*rayR.dir.x, rayR.ori.y + max_ray_len*rayR.dir.y),
					cv::Scalar(0, 255, 255), 1, CV_AA);
				cv::namedWindow("show4");
				cv::imshow("show4", dst);
				cv::waitKey(0);
			}

			/* get corners */

			ZQ_Vec2D corner_LT, corner_LB, corner_RT, corner_RB;
			float depth1, depth2;
			ZQ_Ray::RayCross(rayL, rayT, depth1, depth2, corner_LT);
			ZQ_Ray::RayCross(rayL, rayB, depth1, depth2, corner_LB);
			ZQ_Ray::RayCross(rayR, rayT, depth1, depth2, corner_RT);
			ZQ_Ray::RayCross(rayR, rayB, depth1, depth2, corner_RB);


			float scale_x = (float)src.cols / dstWidth;
			float scale_y = (float)src.rows / dstHeight;

			std::vector<cv::Point2f> src_pts, dst_pts;
			src_pts.push_back(cv::Point2f((corner_LT.x + 0.5)*scale_x - 0.5, (corner_LT.y + 0.5)*scale_y - 0.5));
			src_pts.push_back(cv::Point2f((corner_LB.x + 0.5)*scale_x - 0.5, (corner_LB.y + 0.5)*scale_y - 0.5));
			src_pts.push_back(cv::Point2f((corner_RT.x + 0.5)*scale_x - 0.5, (corner_RT.y + 0.5)*scale_y - 0.5));
			src_pts.push_back(cv::Point2f((corner_RB.x + 0.5)*scale_x - 0.5, (corner_RB.y + 0.5)*scale_y - 0.5));
			dst_pts.push_back(cv::Point2f(0, 0));
			dst_pts.push_back(cv::Point2f(0, dstHeight));
			dst_pts.push_back(cv::Point2f(dstWidth, 0));
			dst_pts.push_back(cv::Point2f(dstWidth, dstHeight));

			cv::Mat transmtx = cv::getPerspectiveTransform(src_pts, dst_pts);
			cv::warpPerspective(src, dst, transmtx, cv::Size(dstWidth, dstHeight));

			return true;
		}
	};
}

#endif
