#define _USE_UMFPACK 1
#include "ZQ_CameraCalibrationMulti.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
using namespace ZQ;


template<class T>
void m_printf_mxn_e(const T* A, int m, int n)
{
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
			printf("%13.4e", A[i*n + j]);
		printf("\n");
	}
}


double m_noise(double scale)
{
	return rand() % 10001 / 10000.0*scale;
}

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
bool check_visible(int width, int height, const T* cam_R, const T* cam_T, const T* board_R, const T* board_T, const T* fc_cc_alpha_kc,
	const T* X3, int n_pts, T* X2, bool zAxis_in)
{
	if (cam_R == 0 || cam_T == 0 || board_R == 0 || board_T == 0)
		return false;

	T view_R[9], view_T[3];
	inv_RT(cam_R, cam_T, view_R, view_T);
	T modelview_R[9], modelview_T[3];
	ZQ_MathBase::MatrixMul(view_R, board_R, 3, 3, 3, modelview_R);
	ZQ_MathBase::MatrixMul(view_R, board_T, 3, 3, 1, modelview_T);
	modelview_T[0] += view_T[0];
	modelview_T[1] += view_T[1];
	modelview_T[2] += view_T[2];

	T mv_rT[6];
	if (!ZQ_Rodrigues::ZQ_Rodrigues_R2r(modelview_R, mv_rT))
	{
		return false;
	}


	T w2 = 1.0 - mv_rT[0] * mv_rT[0] - mv_rT[1] * mv_rT[1] - mv_rT[2] * mv_rT[2];
	if (w2 < 0.64) // if w2 = 0.5, it means the angle is 90 degree
	{
		return false;
	}

	memcpy(mv_rT + 3, modelview_T, sizeof(T) * 3);
	if (fc_cc_alpha_kc == 0 || X3 == 0 || X2 == 0)
	{
		return false;
	}

	if (!ZQ_CameraProjection::project_points_fun(n_pts, X3, mv_rT, fc_cc_alpha_kc, fc_cc_alpha_kc + 2, fc_cc_alpha_kc + 5, fc_cc_alpha_kc[4], X2, zAxis_in))
	{
		return false;
	}

	for (int i = 0; i < n_pts; i++)
	{
		if (X2[i * 2] < 0 || X2[i * 2] > width - 1 || X2[i * 2 + 1] < 0 || X2[i * 2 + 1] > height - 1)
		{
			return false;
		}
	}
	return true;
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
	memcpy(rT + 3, T1_2, sizeof(T) * 3);
	return true;
}


template<class T, const bool zAxis_in>
int test_multicalib_with_known_intrinsic();

int main()
{
	/*float will return false, may be precision problem*/
	//test_multicalib_with_known_intrinsic<float, false>();
	//test_multicalib_with_known_intrinsic<float, true>();
	test_multicalib_with_known_intrinsic<double, false>();
	test_multicalib_with_known_intrinsic<double, true>();



	return 0;
}

template<class T, const bool zAxis_in>
int test_multicalib_with_known_intrinsic()
{
	double proj_noise_scale = 0.0;
	double eps = 1e-9;
	int n_cams = 8;
	int n_checkboard = 16;
	const double m_pi = atan(1.0) * 4;
	double circle_radius = 10;


	T* cam_T = new T[n_cams * 3];
	T* cam_R = new T[n_cams * 9];

	for (int i = 0; i < n_cams; i++)
	{
		double theta = (double)i / n_cams * 2 * m_pi;
		cam_T[i * 3 + 0] = circle_radius*cos(theta - 0.5*m_pi) * (zAxis_in ? 1 : -1);
		cam_T[i * 3 + 1] = 0;
		cam_T[i * 3 + 2] = circle_radius*sin(theta - 0.5*m_pi) * (zAxis_in ? 1 : -1);
		T tmp_R[9] =
		{
			cos(theta), 0, -sin(theta),
			0,1,0,
			sin(theta), 0, cos(theta)
		};
		memcpy(cam_R + i * 9, tmp_R, sizeof(T) * 9);
	}

	T* fc_cc_alpha_kc = new T[10 * n_cams];

	int width = 1000, height = 1000;
	for (int i = 0; i < n_cams; i++)
	{
		fc_cc_alpha_kc[i * 10 + 0] = 500;
		fc_cc_alpha_kc[i * 10 + 1] = 500;
		fc_cc_alpha_kc[i * 10 + 2] = 512;
		fc_cc_alpha_kc[i * 10 + 3] = 512;
		fc_cc_alpha_kc[i * 10 + 4] = 0.001;
		fc_cc_alpha_kc[i * 10 + 5] = 0.05;
		fc_cc_alpha_kc[i * 10 + 6] = 0.1;
		fc_cc_alpha_kc[i * 10 + 7] = 0.2;
		fc_cc_alpha_kc[i * 10 + 8] = 1;
		fc_cc_alpha_kc[i * 10 + 9] = 10;
	}


	T* checkboard_R = new T[n_checkboard * 9];
	T* checkboard_T = new T[n_checkboard * 3];
	for (int i = 0; i < n_checkboard; i++)
	{
		double theta = (double)i / n_checkboard * 2 * m_pi;
		T tmp_R[9] =
		{
			cos(theta), 0, -sin(theta),
			0, 1, 0,
			sin(theta), 0, cos(theta)
		};
		memcpy(checkboard_R + i * 9, tmp_R, sizeof(T) * 9);
		T tmp_T[3] =
		{
			0,0,0
		};
		memcpy(checkboard_T + i * 3, tmp_T, sizeof(T) * 3);
	}



	//
	int M = 8;
	int N = 7;
	int num_of_pts = M*N;
	T* X3 = new T[num_of_pts * 3];
	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			X3[(i*N + j) * 3 + 0] = j - N / 2;
			X3[(i*N + j) * 3 + 1] = i - M / 2;
			X3[(i*N + j) * 3 + 2] = 0;
		}
	}

	std::vector<int> cam_idx;
	std::vector<int> checkboard_idx;
	std::vector<T*> X2_pts;

	for (int i = 0; i < n_cams; i++)
	{
		for (int j = 0; j < n_checkboard; j++)
		{
			T* X2 = new T[num_of_pts * 2];
			if (check_visible(width, height, cam_R + i * 9, cam_T + i * 3, checkboard_R + j * 9, checkboard_T + j * 3, fc_cc_alpha_kc + i * 10, X3, num_of_pts, X2, zAxis_in))
			{
				cam_idx.push_back(i);
				checkboard_idx.push_back(j);
				X2_pts.push_back(X2);
				for (int pp = 0; pp < num_of_pts * 2; pp++)
					X2[pp] += m_noise(2.0) - 1.0;
			}
			else
				delete[]X2;
		}
	}

	int total_visible_num = cam_idx.size();
	T* all_X2 = new T[num_of_pts * 2 * total_visible_num];
	int* visible_idx = new int[total_visible_num];
	for (int i = 0; i < total_visible_num; i++)
	{
		T* tmp_X2 = X2_pts[i];
		memcpy(all_X2 + num_of_pts * 2 * i, tmp_X2, sizeof(T)*num_of_pts * 2);
		delete[]tmp_X2;
		visible_idx[i] = checkboard_idx[i];
	}
	X2_pts.clear();

	int* visible_num = new int[n_cams];
	int* visible_offset = new int[n_cams];

	memset(visible_num, 0, sizeof(int)*n_cams);
	for (int i = 0; i < total_visible_num; i++)
	{
		int cur_id = cam_idx[i];
		visible_num[cur_id]++;
	}
	memset(visible_offset, 0, sizeof(int)*n_cams);
	for (int i = 1; i < n_cams; i++)
		visible_offset[i] = visible_offset[i - 1] + visible_num[i - 1];

	/*********************************/
	T* ori_check_to_cam0_rT = new T[n_checkboard * 6];
	T* ori_cam0_to_other_rT = new T[(n_cams - 1) * 6];
	for (int i = 0; i < n_checkboard; i++)
	{
		first_to_second_rT(checkboard_R + i * 9, checkboard_T + i * 3, cam_R, cam_T, ori_check_to_cam0_rT + i * 6);
	}
	for (int i = 1; i < n_cams; i++)
	{
		first_to_second_rT(cam_R, cam_T, cam_R + i * 9, cam_T + i * 3, ori_cam0_to_other_rT + (i - 1) * 6);
	}

	/*********************************/
	int max_iter_levmar = 500;
	double tol_E = 0.1;

	T* out_check_to_cam0_rT = new T[n_checkboard * 6];
	T* out_cam0_to_other_rT = new T[(n_cams - 1) * 6];
	//memcpy(out_check_to_cam0_rT, ori_check_to_cam0_rT, sizeof(T)*n_checkboard);
	//memcpy(out_cam0_to_other_rT, ori_cam0_to_other_rT, sizeof(T)*n_cams);

	if (!ZQ_CameraCalibrationMulti::CalibrateMultiCamera<T>(n_checkboard, num_of_pts, n_cams, X3, all_X2, visible_num, visible_offset, visible_idx, fc_cc_alpha_kc, zAxis_in,
		out_check_to_cam0_rT, out_cam0_to_other_rT, max_iter_levmar, true, true))
	{
		printf("failed\n");
		return EXIT_FAILURE;
	}

	for (int cc = 0; cc < n_checkboard; cc++)
	{
		m_printf_mxn_e(ori_check_to_cam0_rT + cc * 6, 1, 6);
		m_printf_mxn_e(out_check_to_cam0_rT + cc * 6, 1, 6);
		printf("----------------------\n");
	}

	for (int cc = 0; cc < n_cams - 1; cc++)
	{
		m_printf_mxn_e(ori_cam0_to_other_rT + cc * 6, 1, 6);
		m_printf_mxn_e(out_cam0_to_other_rT + cc * 6, 1, 6);
		printf("----------------------\n");
	}

	delete[]ori_check_to_cam0_rT;
	delete[]ori_cam0_to_other_rT;
	delete[]out_cam0_to_other_rT;
	delete[]out_check_to_cam0_rT;
	delete[]cam_R;
	delete[]cam_T;
	delete[]checkboard_R;
	delete[]checkboard_T;
	delete[]all_X2;
	delete[]X3;
	delete[]fc_cc_alpha_kc;
	delete[]visible_idx;
	delete[]visible_num;
	delete[]visible_offset;
	return 0;
}
