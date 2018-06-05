#include "ZQ_LazySnappingGUI.h"
#include "ZQ_ScanLinePolygonFill.h"
#include "highgui.h"

using namespace ZQ;
const char* ZQ_LazySnappingGUI::winName = "Lazy Snapping @ Zuo Qing";
std::vector<CvPoint> ZQ_LazySnappingGUI::forePts;
std::vector<CvPoint> ZQ_LazySnappingGUI::backPts;
std::vector<CvPoint> ZQ_LazySnappingGUI::add_forePts;
std::vector<CvPoint> ZQ_LazySnappingGUI::add_backPts;
int ZQ_LazySnappingGUI::currentMode = 0;
IplImage* ZQ_LazySnappingGUI::image = 0;
IplImage* ZQ_LazySnappingGUI::imageDraw = 0;
ZQ_DImage<ZQ_LazySnappingGUI::BaseType> ZQ_LazySnappingGUI::scaled_image;
ZQ_DImage<ZQ_LazySnappingGUI::BaseType> ZQ_LazySnappingGUI::im2;
ZQ_LazySnappingOptions ZQ_LazySnappingGUI::ls_opt;
ZQ_StructureFromTextureOptions ZQ_LazySnappingGUI::opt;
ZQ_DImage<bool> ZQ_LazySnappingGUI::mask;
bool ZQ_LazySnappingGUI::has_scaled = false;
ZQ_LazySnapping<ZQ_LazySnappingGUI::BaseType, ZQ_LazySnappingGUI::MAX_CLUSTER_NUM>* ZQ_LazySnappingGUI::lazySnap;
bool ZQ_LazySnappingGUI::use_old_method = false;
int ZQ_LazySnappingGUI::cur_mouse_pos_x = 0;
int ZQ_LazySnappingGUI::cur_mouse_pos_y = 0;
bool ZQ_LazySnappingGUI::has_last_pos = false;
int ZQ_LazySnappingGUI::last_mouse_pos_x = 0;
int ZQ_LazySnappingGUI::last_mouse_pos_y = 0;

double ZQ_LazySnappingGUI::color_draw[2][3] =
{
	{ 0, 0, 255 },
	{ 255, 0, 0 }
};

bool ZQ_LazySnappingGUI::Run(const ZQ_DImage<BaseType>& ori_image, ZQ_DImage<BaseType>& tri_map)
{
	_clear();
	opt.fsize_for_filter = 1;
	ls_opt.dilate_erode_size = 2;

	int ori_width = ori_image.width();
	int ori_height = ori_image.height();
	if (ori_width > standard_width || ori_height > standard_height)
	{
		double scale_x = (double)standard_width / ori_width;
		double scale_y = (double)standard_height / ori_height;
		double scale = __min(scale_x, scale_y);
		int dst_width = ori_width*scale + 0.5;
		int dst_height = ori_height*scale + 0.5;
		ori_image.imresize(scaled_image, dst_width, dst_height);
		im2 = scaled_image;
		/*if (!ZQ_StructureFromTexture::StructureFromTexture(scaled_image, im2, opt))
		{
			printf("failed to run StructureFromTexture\n");
			return false;
		}*/
		has_scaled = true;
	}
	else
	{
		im2 = scaled_image;
		/*if (!ZQ_StructureFromTexture::StructureFromTexture(ori_image, im2, opt))
		{
			printf("failed to run StructureFromTexture\n");
			return false;
		}*/
		has_scaled = false;
	}


	int width = im2.width();
	int height = im2.height();
	int nChannels = im2.nchannels();
	mask.allocate(width, height, 1);

	if (!use_old_method)
	{
		lazySnap = new ZQ_LazySnapping<BaseType, MAX_CLUSTER_NUM>(width, height);
		lazySnap->SetImage(im2, ls_opt.lambda_for_E2, ls_opt.color_scale_for_E2);
	}


	image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	BaseType*& im2_data = im2.data();
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			int offset = h*width + w;
			if (nChannels == 1)
				cvSet2D(image, h, w, cvScalar(im2_data[offset] * 255, im2_data[offset] * 255, im2_data[offset] * 255));
			else
				cvSet2D(image, h, w, cvScalar(im2_data[offset * 3 + 0] * 255, im2_data[offset * 3 + 1] * 255, im2_data[offset * 3 + 2] * 255));
		}
	}
	cvNamedWindow(winName, 1);
	cvSetMouseCallback(winName, _mouseHandler, 0);
	imageDraw = cvCloneImage(image);
	cvShowImage(winName, image);

	while (true)
	{
		int c = cvWaitKey(0);
		c = (char)c;
		if (c == 27 || c == 's')
		{
			break;
		}
		else if (c == 'r')
		{
			lazySnap->SetImage(im2, ls_opt.lambda_for_E2, ls_opt.color_scale_for_E2);
			imageDraw = cvCloneImage(image);
			forePts.clear();
			backPts.clear();
			add_forePts.clear();
			add_backPts.clear();
			currentMode = 0;
			cvShowImage(winName, image);
		}
		else if (c == 'b')
		{
			currentMode = 1;
		}
		else if (c == 'f')
		{
			currentMode = 0;
		}
		cvNamedWindow(winName, 1);
	}
	cvReleaseImage(&image);
	cvReleaseImage(&imageDraw);
	if (has_scaled)
	{
		mask.imresize(ori_width, ori_height);
	}

	tri_map.allocate(ori_width, ori_height);
	for (int i = 0; i < ori_width*ori_height; i++)
	{
		tri_map.data()[i] = mask.data()[i] ? 1.0 : 0;
	}
	cvDestroyWindow(winName);
	return true;
}

void ZQ_LazySnappingGUI::_clear()
{
	forePts.clear();
	backPts.clear();
	add_forePts.clear();
	add_backPts.clear();
	currentMode = 0;
	if (image)
	{
		cvReleaseImage(&image);
	}
	if (imageDraw)
	{
		cvReleaseImage(&imageDraw);
	}
	
	scaled_image.clear();
	im2.clear();
	mask.clear();
	has_scaled = false;
	if (lazySnap)
	{
		delete lazySnap;
		lazySnap = 0;
	}
	cur_mouse_pos_x = 0;
	cur_mouse_pos_y = 0;
	has_last_pos = false;
	last_mouse_pos_x = 0;
	last_mouse_pos_y = 0;
}

void ZQ_LazySnappingGUI::_mouseHandler(int event, int x, int y, int flags, void*)
{
	cur_mouse_pos_x = x;
	cur_mouse_pos_y = y;
	if (event == CV_EVENT_LBUTTONUP)
	{
		if (use_old_method)
		{
			int fore_num = forePts.size();
			int back_num = backPts.size();
			if (fore_num == 0 && back_num == 0)
			{
				return;
			}
			cvReleaseImage(&imageDraw);
			imageDraw = cvCloneImage(image);
			int* fore_pts = NULL;
			int* back_pts = NULL;
			if (fore_num > 0 && back_num > 0)
			{
				fore_pts = new int[fore_num * 2];
				back_pts = new int[back_num * 2];
				for (int i = 0; i < fore_num; i++)
				{
					fore_pts[i * 2 + 0] = forePts[i].x;
					fore_pts[i * 2 + 1] = forePts[i].y;
				}
				for (int i = 0; i < back_num; i++)
				{
					back_pts[i * 2 + 0] = backPts[i].x;
					back_pts[i * 2 + 1] = backPts[i].y;
				}
				if (!ZQ_LazySnapping<BaseType, MAX_CLUSTER_NUM>::LazySnapping(im2, back_pts, back_num, fore_pts, fore_num, mask, ls_opt))
				{
					delete[]fore_pts;
					delete[]back_pts;
					fore_pts = NULL;
					back_pts = NULL;
				}

				_drawMask(imageDraw, mask);
			}
			if (fore_pts)
			{
				delete[]fore_pts;
			}
			fore_pts = 0;
			if (back_pts)
			{
				delete[]back_pts;
			}
			back_pts = 0;
			_drawSelectPoints(imageDraw, forePts, backPts);
			cvShowImage(winName, imageDraw);

		}
		else
		{
			int add_fore_num = add_forePts.size();
			int add_back_num = add_backPts.size();
			if (add_fore_num == 0 && add_back_num == 0)
			{
				return;
			}
			cvReleaseImage(&imageDraw);
			imageDraw = cvCloneImage(image);
			int* add_fore_pts = NULL;
			int* add_back_pts = NULL;
			if (add_fore_num > 0)
			{
				add_fore_pts = new int[add_fore_num * 2];

				for (int i = 0; i < add_fore_num; i++)
				{
					add_fore_pts[i * 2 + 0] = add_forePts[i].x;
					add_fore_pts[i * 2 + 1] = add_forePts[i].y;
				}
				if (!lazySnap->EditSnappingAddForeground(add_fore_num, add_fore_pts))
				{
					delete[]add_fore_pts;
					add_fore_pts = NULL;
				}
				else
				{
					forePts.insert(forePts.end(), add_forePts.begin(), add_forePts.end());
					add_forePts.clear();
				}
			}

			if (add_back_num > 0)
			{
				add_back_pts = new int[add_back_num * 2];

				for (int i = 0; i < add_back_num; i++)
				{
					add_back_pts[i * 2 + 0] = add_backPts[i].x;
					add_back_pts[i * 2 + 1] = add_backPts[i].y;

				}

				if (!lazySnap->EditSnappingAddBackground(add_back_num, add_back_pts))
				{
					delete[]add_back_pts;
					add_back_pts = NULL;
				}
				else
				{
					backPts.insert(backPts.end(), add_backPts.begin(), add_backPts.end());
					add_backPts.clear();
				}
			}
			if (!ZQ_LazySnapping<BaseType, MAX_CLUSTER_NUM>::FilterMask(lazySnap->GetForegroundMaskPtr(), mask.data(), mask.width(), mask.height(), ls_opt.area_thresh, ls_opt.dilate_erode_size))
			{
				memcpy(mask.data(), lazySnap->GetForegroundMaskPtr(), sizeof(bool)*mask.width()*mask.height());
			}
			_drawMask(imageDraw, mask);
			if (add_fore_pts)
			{
				delete[]add_fore_pts;
			}
			add_fore_pts = 0;
			if (add_back_pts)
			{
				delete[]add_back_pts;
			}
			add_back_pts = 0;

			_drawSelectPoints(imageDraw, forePts, backPts);
			cvShowImage(winName, imageDraw);
		}
		has_last_pos = false;
	}
	else if (event == CV_EVENT_LBUTTONDOWN)
	{
		last_mouse_pos_x = cur_mouse_pos_x;
		last_mouse_pos_y = cur_mouse_pos_y;
		has_last_pos = true;
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
	{
		if (!has_last_pos)
		{
			CvPoint pt = cvPoint(x, y);
			if (currentMode == 0)
			{
				if (!use_old_method)
					add_forePts.push_back(pt);
				else
					forePts.push_back(pt);
			}
			else
			{
				if (!use_old_method)
					add_backPts.push_back(pt);
				else
					backPts.push_back(pt);
			}
			CvScalar color = cvScalar(color_draw[currentMode][0], color_draw[currentMode][1], color_draw[currentMode][2]);
			cvCircle(imageDraw, pt, 2, color);
			cvShowImage(winName, imageDraw);
		}
		else
		{
			int im_width = imageDraw->width;
			int im_height = imageDraw->height;
			if (last_mouse_pos_x >= 0 && last_mouse_pos_x < im_width && last_mouse_pos_y >= 0 && last_mouse_pos_y < im_height &&
				cur_mouse_pos_x >= 0 && cur_mouse_pos_x < im_width && cur_mouse_pos_y >= 0 && cur_mouse_pos_y < im_height)
			{

				std::vector<ZQ_Vec2D> pixels;
				ZQ_ScanLinePolygonFill::FillOneStrokeWithClip(ZQ_Vec2D(last_mouse_pos_x, last_mouse_pos_y), ZQ_Vec2D(cur_mouse_pos_x, cur_mouse_pos_y), 1, im_width, im_height, pixels);

				for (int p = 0; p < pixels.size(); p++)
				{
					CvPoint pt = cvPoint(pixels[p].x, pixels[p].y);
					if (currentMode == 0)
					{
						if (!use_old_method)
							add_forePts.push_back(pt);
						else
							forePts.push_back(pt);
					}
					else
					{
						if (!use_old_method)
							add_backPts.push_back(pt);
						else
							backPts.push_back(pt);
					}
					CvScalar color = cvScalar(color_draw[currentMode][0], color_draw[currentMode][1], color_draw[currentMode][2]);
					cvCircle(imageDraw, pt, 2, color);
				}
				cvShowImage(winName, imageDraw);
			}
		}
		last_mouse_pos_x = cur_mouse_pos_x;
		last_mouse_pos_y = cur_mouse_pos_y;
		has_last_pos = true;
	}
}


void ZQ_LazySnappingGUI::_drawMask(IplImage* imageDraw, const ZQ_DImage<bool>& mask)
{
	_drawMask(imageDraw, mask.data());
}

void ZQ_LazySnappingGUI::_drawMask(IplImage* imageDraw, const bool* mask)
{
	int width = imageDraw->width;
	int height = imageDraw->height;
	CvScalar border_color = cvScalar(0, 255, 0);
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			int offset = h*width + w;
			if (!mask[offset] && ((w > 0 && mask[offset - 1]) || (w < width - 1 && mask[offset + 1]) || (h > 0 && mask[offset - width]) || (h < height - 1 && mask[offset + width])))
			{
				cvSet2D(imageDraw, h, w, border_color);
			}

		}
	}
}

void ZQ_LazySnappingGUI::_drawSelectPoints(IplImage* imageDraw, const std::vector<CvPoint>& forePts, const std::vector<CvPoint>& backPts)
{
	for (int i = 0; i < forePts.size(); i++)
	{
		CvScalar color = cvScalar(color_draw[0][0], color_draw[0][1], color_draw[0][2]);
		cvCircle(imageDraw, forePts[i], 2, color);
	}
	for (int i = 0; i < backPts.size(); i++)
	{
		CvScalar color = cvScalar(color_draw[1][0], color_draw[1][1], color_draw[1][2]);
		cvCircle(imageDraw, backPts[i], 2, color);
	}
}