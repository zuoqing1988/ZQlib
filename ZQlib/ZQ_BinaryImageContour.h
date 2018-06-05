#ifndef _ZQ_BINARY_IMAGE_CONTOUR_H_
#define _ZQ_BINARY_IMAGE_CONTOUR_H_

#pragma once

#include <vector>
#include "ZQ_Vec2D.h"

namespace ZQ
{
	class ZQ_BinaryImageContour
	{
	public:
		static bool GetBinaryImageContour(const bool* image, int width, int height, std::vector<std::vector<ZQ_Vec2D>>& contours)
		{
			if (image == 0)
				return false;

			int imWidth = width + 2;
			int imHeight = height + 2;
			bool* padding_image = new bool[imWidth*imHeight];
			memset(padding_image, 0, sizeof(bool)*imWidth*imHeight);

			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					padding_image[(i + 1)*imWidth + (j + 1)] = image[i*width + j];
				}
			}

			bool* used_flag = new bool[imWidth*imHeight];
			memset(used_flag, 0, sizeof(bool)*imWidth*imHeight);

			while (true)
			{
				int x, y;
				bool flag = _findSeedAndSeedFilling(padding_image, used_flag, imWidth, imHeight, x, y);
				if (flag == false)
					break;

				std::vector<ZQ_Vec2D> contour;
				_findContour(padding_image, imWidth, imHeight, x, y, contour);
				contours.push_back(contour);
				contour.clear();
			}

			for (int i = 0; i < contours.size(); i++)
			{
				for (int j = 0; j < contours[i].size(); j++)
				{
					contours[i][j].x -= 1;
					contours[i][j].y -= 1;
				}
			}

			delete[]used_flag;
			delete[]padding_image;
			return true;
		}

		static bool GetBinaryImageMaxContour(const bool* image, int width, int height, std::vector<ZQ_Vec2D>& contour)
		{
			std::vector<std::vector<ZQ_Vec2D>> contours;
			if (!GetBinaryImageContour(image, width, height, contours))
				return false;

			if (contours.size() == 0)
				return false;
			int len = contours[0].size();
			int idx = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				if (len < contours[i].size())
				{
					idx = i;
					len = contours[i].size();
				}
			}
			contour = contours[idx];
			return true;
		}

	private:
		static bool _findSeedAndSeedFilling(const bool* image, bool* used_flag, int width, int height,int& x, int& y)
		{
			bool flag = false;
			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					if (image[i*width + j] && !used_flag[i*width + j])
					{
						flag = true;
						x = j;
						y = i;
					}
				}
			}

			if (flag)
			{
				int* idx_x = new int[width*height];
				int* idx_y = new int[width*height];
				bool* tag = new bool[width*height];
				memset(tag, 0, sizeof(bool)*width*height);

				int head = 0;
				int tail = 1;
				idx_x[head] = x;
				idx_y[head] = y;
				tag[y*width + x] = true;

				do
				{
					int cur_x = idx_x[head];
					int cur_y = idx_y[head];
					head++;

					if (image[(cur_y - 1)*width + cur_x] && !tag[(cur_y - 1)*width + cur_x])
					{
						idx_x[tail] = cur_x;
						idx_y[tail] = cur_y - 1;
						tail++;
						tag[(cur_y - 1)*width + cur_x] = true;
					}
					if (image[(cur_y + 1)*width + cur_x] && !tag[(cur_y + 1)*width + cur_x])
					{
						idx_x[tail] = cur_x;
						idx_y[tail] = cur_y + 1;
						tail++;
						tag[(cur_y + 1)*width + cur_x] = true;
					}
					if (image[cur_y*width + cur_x - 1] && !tag[cur_y*width + cur_x - 1])
					{
						idx_x[tail] = cur_x - 1;
						idx_y[tail] = cur_y;
						tail++;
						tag[cur_y*width + cur_x - 1] = true;
					}
					if (image[cur_y*width + cur_x + 1] && !tag[cur_y*width + cur_x + 1])
					{
						idx_x[tail] = cur_x + 1;
						idx_y[tail] = cur_y;
						tail++;
						tag[cur_y*width + cur_x + 1] = true;
					}
				} while (head < tail);


				for (int i = 0; i < tail; i++)
				{
					used_flag[idx_y[i] * width + idx_x[i]] = true;
				}

				delete[]tag;
				delete[]idx_x;
				delete[]idx_y;

				return true;

			}
			else
			{
				return false;
			}
		}

		static void _findContour(const bool* image, int width, int height, int x, int y, std::vector<ZQ_Vec2D>& contour)
		{
			int cur_x = x;
			int cur_y = y;
			while (image[cur_y*width + cur_x - 1])
				cur_x--;
			ZQ_Vec2D pt;
			pt.x = cur_x - 0.5;
			pt.y = cur_y + 0.5;

			contour.push_back(pt);
			pt.x = cur_x - 0.5;
			pt.y = cur_y - 0.5;
			contour.push_back(pt);

			//DOWN:[0,-1],UP:[0,1],LEFT:[-1,0],RIGHT:[1,0]
			const int DOWN_DIR = 0, UP_DIR = 1, LEFT_DIR = 2, RIGHT_DIR = 3;

			int cur_dir = DOWN_DIR;

			while (true)
			{
				switch (cur_dir)
				{
				case DOWN_DIR:
					if (!image[(cur_y - 1)*width + cur_x])
					{
						pt.x = cur_x + 0.5;
						pt.y = cur_y - 0.5;
						contour.push_back(pt);
						cur_dir = RIGHT_DIR;
					}
					else
					{
						if (image[(cur_y - 1)*width + cur_x - 1])
						{
							cur_x = cur_x - 1;
							cur_y = cur_y - 1;
							pt.x = cur_x - 0.5;
							pt.y = cur_y + 0.5;
							contour.push_back(pt);
							cur_dir = LEFT_DIR;
						}
						else
						{
							cur_x = cur_x;
							cur_y = cur_y - 1;
							pt.x = cur_x - 0.5;
							pt.y = cur_y - 0.5;
							contour.push_back(pt);
							cur_dir = DOWN_DIR;
						}
					}
					break;
				case UP_DIR:
					if (!image[(cur_y + 1)*width + cur_x])
					{
						pt.x = cur_x - 0.5;
						pt.y = cur_y + 0.5;
						contour.push_back(pt);
						cur_dir = LEFT_DIR;
					}
					else
					{
						if (image[(cur_y + 1)*width + cur_x + 1])
						{
							cur_x = cur_x + 1;
							cur_y = cur_y + 1;
							pt.x = cur_x + 0.5;
							pt.y = cur_y - 0.5;
							contour.push_back(pt);
							cur_dir = RIGHT_DIR;
						}
						else
						{
							cur_x = cur_x;
							cur_y = cur_y + 1;
							pt.x = cur_x + 0.5;
							pt.y = cur_y + 0.5;
							contour.push_back(pt);
							cur_dir = UP_DIR;
						}
					}
					break;
				case LEFT_DIR:
					if (!image[cur_y*width + cur_x - 1])
					{
						pt.x = cur_x - 0.5;
						pt.y = cur_y - 0.5;
						contour.push_back(pt);
						cur_dir = DOWN_DIR;
					}
					else
					{
						if (image[(cur_y + 1)*width + cur_x - 1])
						{
							cur_x = cur_x - 1;
							cur_y = cur_y + 1;
							pt.x = cur_x + 0.5;
							pt.y = cur_y + 0.5;
							contour.push_back(pt);
							cur_dir = UP_DIR;
						}
						else
						{
							cur_x = cur_x - 1;
							cur_y = cur_y;
							pt.x = cur_x - 0.5;
							pt.y = cur_y + 0.5;
							contour.push_back(pt);
							cur_dir = LEFT_DIR;
						}
					}
					break;
				case RIGHT_DIR:
					if (!image[cur_y*width + cur_x + 1])
					{
						pt.x = cur_x + 0.5;
						pt.y = cur_y + 0.5;
						contour.push_back(pt);
						cur_dir = UP_DIR;
					}
					else
					{
						if (image[(cur_y - 1)*width + cur_x + 1])
						{
							cur_x = cur_x + 1;
							cur_y = cur_y - 1;
							pt.x = cur_x - 0.5;
							pt.y = cur_y - 0.5;
							contour.push_back(pt);
							cur_dir = DOWN_DIR;
						}
						else
						{
							cur_x = cur_x + 1;
							cur_y = cur_y;
							pt.x = cur_x + 0.5;
							pt.y = cur_y - 0.5;
							contour.push_back(pt);
							cur_dir = RIGHT_DIR;
						}
					}
					break;
				}

				int len = contour.size();
				if (contour[0].x == contour[len - 1].x && contour[0].y == contour[len - 1].y)
					break;

			}

			contour.pop_back();
		}
	};

}

#endif