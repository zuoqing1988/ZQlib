#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "ZQ_CPURenderer3DWorkSpace.h"
#include "ZQ_StereoRectify.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;

template<class T>
void inv_RT(const T* in_R, const T* in_T, T* out_R, T* out_T)
{
	out_R[0] = in_R[0]; out_R[1] = in_R[3]; out_R[2] = in_R[6];
	out_R[3] = in_R[1]; out_R[4] = in_R[4]; out_R[5] = in_R[7];
	out_R[6] = in_R[2]; out_R[7] = in_R[5]; out_R[8] = in_R[8];
	out_T[0] = -(out_R[0] * in_T[0] + out_R[1] * in_T[1] + out_R[2] * in_T[2]);
	out_T[1] = -(out_R[3] * in_T[0] + out_R[4] * in_T[1] + out_R[5] * in_T[2]);
	out_T[2] = -(out_R[6] * in_T[0] + out_R[7] * in_T[1] + out_R[8] * in_T[2]);
}


template<class T>
bool first_to_second_rT(const T* R1, const T* T1, const T* R2, const T* T2, T* rT)
{
	T inv_R[9], inv_T[3];
	inv_RT(R2, T2, inv_R, inv_T);
	T R1_2[9], T1_2[3];
	ZQ_MathBase::MatrixMul(inv_R, R1, 3, 3, 3, R1_2);
	ZQ_MathBase::MatrixMul(inv_R, T1, 3, 3, 1, T1_2);
	T1_2[0] += inv_T[0];
	T1_2[1] += inv_T[1];
	T1_2[2] += inv_T[2];
	if (!ZQ_Rodrigues::ZQ_Rodrigues_R2r(R1_2, rT))
	{
		return false;
	}
	memcpy(rT + 3, T1_2, sizeof(T)* 3);
	return true;
}

template<class T>
void auto_adjust(ZQ_DImage<T>& img)
{
	T*& data = img.data();
	double min_dis = data[0];
	double max_dis = data[0];
	for (int i = 1; i < img.nelements(); i++)
	{
		if (min_dis > data[i])
			min_dis = data[i];
		if (max_dis < data[i])
			max_dis = data[i];
	}
	if (max_dis == min_dis)
		max_dis = min_dis + 1;
	double scale = max_dis - min_dis;
	img.Addwith(-min_dis);
	img.Multiplywith(1.0 / scale);
}

template<class T>
int test_stereo_rectify();

template<class T>
int test_stereo_disparity_opticalflow_L2();

template<class T>
int test_stereo_disparity_opticalflow_L1();

int main()
{
	test_stereo_rectify<float>();
	test_stereo_rectify<double>();
	test_stereo_disparity_opticalflow_L2<float>();
	test_stereo_disparity_opticalflow_L2<double>();
	test_stereo_disparity_opticalflow_L1<double>();

	return 0;
}


template<class T>
int test_stereo_rectify()
{

	int width = 960, height = 540;

	/*******************************  load image begin *******************************/
	const char* left_name = "bino_left.jpg";
	const char* right_name = "bino_right.jpg";

	ZQ_DImage<T> left_ori_img, right_ori_img;
	if (!ZQ_ImageIO::loadImage(left_ori_img, left_name, 0))
	{
		printf("failed to load %s\n", left_name);
		return 0;
	}
	if (!ZQ_ImageIO::loadImage(right_ori_img, right_name, 0))
	{
		printf("failed to load %s\n", right_name);
		return 0;
	}

	T*& left_o_i_data = left_ori_img.data();
	T*& right_o_i_data = right_ori_img.data();

	left_ori_img.FlipY();
	right_ori_img.FlipY();
	if (!left_ori_img.matchDimension(width, height, 1) || !right_ori_img.matchDimension(width, height, 1))
	{
		printf("dimensions donot match\n");
		return 0;
	}
	/*******************************  load image end *******************************/

	/*******************************  set binocular paras begin *******************************/
	float focal_len = height, cx = width*0.5, cy = height*0.5;
	T left_ori_intrinsic[10] = { focal_len, focal_len, cx, cy };
	T right_ori_intrinsic[10] = { focal_len, focal_len, cx, cy };

	float view_mat1[16], view_mat2[16];
	ZQ_CPURenderer3DWorkspace renderer(width, height, false);
	renderer.LookAt(ZQ_Vec3D(-20, 400, -200), ZQ_Vec3D(0, 0, 500), ZQ_Vec3D(0, 1, 0));
	memcpy(view_mat1, renderer.GetViewMatrix(), sizeof(float)* 16);
	renderer.LookAt(ZQ_Vec3D(20, 400, -200), ZQ_Vec3D(0, 0, 500), ZQ_Vec3D(0, 1, 0));
	memcpy(view_mat2, renderer.GetViewMatrix(), sizeof(float)* 16);

	T left_ori_R[9] =
	{
		view_mat1[0], view_mat1[4], view_mat1[8],
		view_mat1[1], view_mat1[5], view_mat1[9],
		view_mat1[2], view_mat1[6], view_mat1[10]
	};
	T right_ori_R[9] =
	{
		view_mat2[0], view_mat2[4], view_mat2[8],
		view_mat2[1], view_mat2[5], view_mat2[9],
		view_mat2[2], view_mat2[6], view_mat2[10]
	};

	T left_ori_rT[6];
	T right_ori_rT[6];
	ZQ_Rodrigues::ZQ_Rodrigues_R2r(left_ori_R, left_ori_rT);
	ZQ_Rodrigues::ZQ_Rodrigues_R2r(right_ori_R, right_ori_rT);

	for (int i = 0; i < 3; i++)
	{
		left_ori_rT[3 + i] = -(left_ori_R[i * 3 + 0] * view_mat1[3] + left_ori_R[i * 3 + 1] * view_mat1[7] + left_ori_R[i * 3 + 2] * view_mat1[11]);
		right_ori_rT[3 + i] = -(right_ori_R[i * 3 + 0] * view_mat2[3] + right_ori_R[i * 3 + 1] * view_mat2[7] + right_ori_R[i * 3 + 2] * view_mat2[11]);
	}

	T right_to_left_rT[6];
	first_to_second_rT(right_ori_R, right_ori_rT + 3, left_ori_R, left_ori_rT + 3, right_to_left_rT);

	/*******************************  set binocular paras end *******************************/



	ZQ_DImage<T> left_map_rectify_from_ori(width, height, 2);
	T*& left_map_rfo = left_map_rectify_from_ori.data();
	ZQ_DImage<bool> left_mask_rectify_from_ori(width, height);
	bool*& left_mask_rfo = left_mask_rectify_from_ori.data();
	ZQ_DImage<T> right_map_rectify_from_ori(width, height, 2);
	T*& right_map_rfo = right_map_rectify_from_ori.data();
	ZQ_DImage<bool> right_mask_rectify_from_ori(width, height);
	bool*& right_mask_rfo = right_mask_rectify_from_ori.data();
	ZQ_DImage<T> left_map_ori_from_rectify(width, height, 2);
	T*& left_map_ofr = left_map_ori_from_rectify.data();
	ZQ_DImage<bool> left_mask_ori_from_rectify(width, height);
	bool*& left_mask_ofr = left_mask_ori_from_rectify.data();
	ZQ_DImage<T> right_map_ori_from_rectify(width, height, 2);
	T*& right_map_ofr = right_map_ori_from_rectify.data();
	ZQ_DImage<bool> right_mask_ori_from_rectify(width, height);
	bool*& right_mask_ofr = right_mask_ori_from_rectify.data();
	T left_rectify_intrinsic[10], right_rectify_intrinsic[10];
	T left_rectify_rT[6], right_rectify_rT[6];
	double right_to_left_transX;

	if (!ZQ_StereoRectify::stereo_rectify(width, height, left_ori_rT, right_ori_rT, left_ori_intrinsic, right_ori_intrinsic, left_rectify_rT, right_rectify_rT,
		left_rectify_intrinsic, right_rectify_intrinsic, left_map_rfo, left_mask_rfo, right_map_rfo, right_mask_rfo))
	{
		printf("stereo rectify fail\n");
		return 0;
	}


	ZQ_DImage<T> left_rectify_img(width, height), right_rectify_img(width, height);
	T*& left_r_i_data = left_rectify_img.data();
	T*& right_r_i_data = right_rectify_img.data();

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			float x = left_map_rfo[(i*width + j) * 2 + 0];
			float y = left_map_rfo[(i*width + j) * 2 + 1];
			ZQ_ImageProcessing::BilinearInterpolate(left_o_i_data, width, height, 1, x, y, left_r_i_data + i*width + j, false);
		}
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			float x = right_map_rfo[(i*width + j) * 2 + 0];
			float y = right_map_rfo[(i*width + j) * 2 + 1];
			ZQ_ImageProcessing::BilinearInterpolate(right_o_i_data, width, height, 1, x, y, right_r_i_data + i*width + j, false);
		}
	}


	ZQ_DImage<T> left_mapback_img(width, height), right_mapback_img(width, height);
	T*& left_m_i_data = left_mapback_img.data();
	T*& right_m_i_data = right_mapback_img.data();

	/*if (!global_coord)
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				float x = left_map_ofr[(i*width + j) * 2 + 0];
				float y = left_map_ofr[(i*width + j) * 2 + 1];
				ZQ_ImageProcessing::BilinearInterpolate(left_r_i_data, width, height, 1, x, y, left_m_i_data + i*width + j, false);
			}
		}

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				float x = right_map_ofr[(i*width + j) * 2 + 0];
				float y = right_map_ofr[(i*width + j) * 2 + 1];
				ZQ_ImageProcessing::BilinearInterpolate(right_r_i_data, width, height, 1, x, y, right_m_i_data + i*width + j, false);
			}
		}
	}*/

	left_rectify_img.FlipY();
	right_rectify_img.FlipY();
	ZQ_ImageIO::saveImage(left_rectify_img, "left_rectify.png");
	ZQ_ImageIO::saveImage(right_rectify_img, "right_rectify.png");

	left_mask_rectify_from_ori.FlipY();
	right_mask_rectify_from_ori.FlipY();
	ZQ_ImageIO::saveImage(left_mask_rectify_from_ori, "left_rectify_mask.png");
	ZQ_ImageIO::saveImage(right_mask_rectify_from_ori, "right_rectify_mask.png");

	/*if (!global_coord)
	{
		left_mapback_img.FlipY();
		right_mapback_img.FlipY();
		ZQ_ImageIO::saveImage(left_mapback_img, "left_mapback.png");
		ZQ_ImageIO::saveImage(right_mapback_img, "right_mapback.png");
	}*/

	return 0;
}
template<class T>
int test_stereo_disparity_opticalflow_L2()
{
	const char* left_name = "left_rectify.png";
	const char* right_name = "right_rectify.png";
	const char* left_rectify_mask_name = "left_rectify_mask.png";
	const char* right_rectify_mask_name = "right_rectify_mask.png";
	const char* left_disparity_name = "left_disparity_of_l2.png";
	const char* right_disparity_name = "right_disparity_of_l2.png";
	const char* left_disparity_mask_name = "left_dis_mask_of_l2.png";
	const char* right_disparity_mask_name = "right_dis_mask_of_l2.png";
	ZQ_DImage<T> left_rectify, right_rectify;
	if (!ZQ_ImageIO::loadImage(left_rectify, left_name, 1))
	{
		printf("failed to load %s\n", left_name);
		return 0;
	}
	if (!ZQ_ImageIO::loadImage(right_rectify, right_name, 1))
	{
		printf("failed to load %s\n", right_name);
		return 0;
	}
	ZQ_DImage<bool> left_rectify_mask, right_rectify_mask;
	if (!ZQ_ImageIO::loadImage(left_rectify_mask, left_rectify_mask_name, 0))
	{
		printf("failed to load %s\n", left_rectify_mask_name);
		return 0;
	}
	if (!ZQ_ImageIO::loadImage(right_rectify_mask, right_rectify_mask_name, 0))
	{
		printf("failed to load %s\n", right_rectify_mask_name);
		return 0;
	}
	ZQ_DImage<T> left_disparity,right_disparity;
	int max_disparity = 128;
	double alpha = 0.06;
	int nFPIter = 3;
	int nSORIter = 50;
	left_rectify_mask.Addwith(1);
	right_rectify_mask.Addwith(1);
	if (!ZQ_StereoRectify::stereo_disparity_opticalflow_L2(left_rectify, right_rectify,  left_disparity, max_disparity, alpha, nFPIter, nSORIter))
	{
		printf("failed to run stereo_disparity_opticalflow_L2\n");
		return 0;
	}
	printf("hello\n");
	if (!ZQ_StereoRectify::stereo_disparity_opticalflow_L2(right_rectify, left_rectify,  right_disparity, max_disparity, alpha, nFPIter, nSORIter))
	{
		printf("failed to run stereo_disparity_opticalflow_L2\n");
		return 0;
	}

	printf("hello\n");
	ZQ_DImage<bool> left_mask, right_mask;
	double tol_disparity_E = 1;
	ZQ_StereoRectify::stereo_disparity_cross_check(left_disparity, left_rectify_mask, right_disparity, right_rectify_mask, left_mask, right_mask, tol_disparity_E);

	auto_adjust(left_disparity);
	auto_adjust(right_disparity);
	
	if (!ZQ_ImageIO::saveImage(left_disparity, left_disparity_name))
	{
		printf("failed to save %s\n", left_disparity_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(right_disparity, right_disparity_name))
	{
		printf("failed to save %s\n", right_disparity_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(left_mask, left_disparity_mask_name))
	{
		printf("failed to save %s\n", left_disparity_mask_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(right_mask, right_disparity_mask_name))
	{
		printf("failed to save %s\n", right_disparity_mask_name);
		return 0;
	}


	
	return 0;

}

template<class T>
int test_stereo_disparity_opticalflow_L1()
{
	const char* left_name = "left_rectify.png";
	const char* right_name = "right_rectify.png";
	const char* left_rectify_mask_name = "left_rectify_mask.png";
	const char* right_rectify_mask_name = "right_rectify_mask.png";
	const char* left_disparity_name = "left_disparity_of_l1.png";
	const char* right_disparity_name = "right_disparity_of_l1.png";
	const char* left_disparity_mask_name = "left_dis_mask_of_l1.png";
	const char* right_disparity_mask_name = "right_dis_mask_of_l1.png";
	ZQ_DImage<T> left_rectify, right_rectify;
	if (!ZQ_ImageIO::loadImage(left_rectify, left_name, 1))
	{
		printf("failed to load %s\n", left_name);
		return 0;
	}
	if (!ZQ_ImageIO::loadImage(right_rectify, right_name, 1))
	{
		printf("failed to load %s\n", right_name);
		return 0;
	}
	ZQ_DImage<bool> left_rectify_mask, right_rectify_mask;
	if (!ZQ_ImageIO::loadImage(left_rectify_mask, left_rectify_mask_name, 0))
	{
		printf("failed to load %s\n", left_rectify_mask_name);
		return 0;
	}
	if (!ZQ_ImageIO::loadImage(right_rectify_mask, right_rectify_mask_name, 0))
	{
		printf("failed to load %s\n", right_rectify_mask_name);
		return 0;
	}
	ZQ_DImage<T> left_disparity, right_disparity;
	int max_disparity = 128;
	double alpha = 0.06;
	int nFPIter = 20;
	int nInnerIter = 3;
	int nSORIter = 50;
	left_rectify_mask.Addwith(1);
	right_rectify_mask.Addwith(1);
	if (!ZQ_StereoRectify::stereo_disparity_opticalflow_L1(left_rectify, right_rectify, left_disparity, max_disparity, alpha, nFPIter, nInnerIter, nSORIter))
	{
		printf("failed to run stereo_disparity_opticalflow_L1\n");
		return 0;
	}
	if (!ZQ_StereoRectify::stereo_disparity_opticalflow_L1(right_rectify, left_rectify, right_disparity, max_disparity, alpha, nFPIter, nInnerIter, nSORIter))
	{
		printf("failed to run stereo_disparity_opticalflow_L1\n");
		return 0;
	}

	ZQ_DImage<bool> left_mask, right_mask;
	double tol_disparity_E = 1;
	ZQ_StereoRectify::stereo_disparity_cross_check(left_disparity, left_rectify_mask, right_disparity, right_rectify_mask, left_mask, right_mask, tol_disparity_E);

	auto_adjust(left_disparity);
	auto_adjust(right_disparity);

	if (!ZQ_ImageIO::saveImage(left_disparity, left_disparity_name))
	{
		printf("failed to save %s\n", left_disparity_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(right_disparity, right_disparity_name))
	{
		printf("failed to save %s\n", right_disparity_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(left_mask, left_disparity_mask_name))
	{
		printf("failed to save %s\n", left_disparity_mask_name);
		return 0;
	}

	if (!ZQ_ImageIO::saveImage(right_mask, right_disparity_mask_name))
	{
		printf("failed to save %s\n", right_disparity_mask_name);
		return 0;
	}



	return 0;

}
