#ifndef _ZQ_LAZY_SNAPPING_GUI_H_
#define _ZQ_LAZY_SNAPPING_GUI_H_
#pragma once

#include <opencv\cv.h>
#include <vector>
#include "ZQ_DoubleImage.h"
#include "ZQ_LazySnapping.h"
#include "ZQ_StructureFromTexture.h"

namespace ZQ
{
	class ZQ_LazySnappingGUI
	{
	public:
		typedef double BaseType;
		static const int MAX_CLUSTER_NUM = 32;
		static const int standard_width = 960;
		static const int standard_height = 540;
	private:
		static const char* winName;
		static std::vector<CvPoint> forePts;
		static std::vector<CvPoint> backPts;
		static std::vector<CvPoint> add_forePts;
		static std::vector<CvPoint> add_backPts;
		static int currentMode;
		static IplImage* image;
		static IplImage* imageDraw;
		static ZQ_DImage<BaseType> scaled_image, im2;
		static ZQ_LazySnappingOptions ls_opt;
		static ZQ_StructureFromTextureOptions opt;
		static ZQ_DImage<bool> mask;
		static bool has_scaled;
		static ZQ_LazySnapping<BaseType, MAX_CLUSTER_NUM>* lazySnap;
		static bool use_old_method;
		static int cur_mouse_pos_x;
		static int cur_mouse_pos_y;
		static bool has_last_pos;
		static int last_mouse_pos_x;
		static int last_mouse_pos_y;
		static double color_draw[2][3];

	public:
		static bool Run(const ZQ_DImage<BaseType>& in_image, ZQ_DImage<BaseType>& tri_map);

	private:
		static void _clear();
		static void _mouseHandler(int event, int x, int y, int flags, void* param);
		static void _drawMask(IplImage* imageDraw, const ZQ_DImage<bool>& mask);
		static void _drawMask(IplImage* imageDraw, const bool* mask);
		static void _drawSelectPoints(IplImage* imageDraw, const std::vector<CvPoint>& forePts, const std::vector<CvPoint>& backPts);
	};
}

#endif