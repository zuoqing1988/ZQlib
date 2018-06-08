#define _USE_UMFPACK 1
#include "ZQ_Calibration.h"
#include "ZQ_CameraCalibration.h"
#include "ZQ_StereoCalibration.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "ZQ_CPURenderer3DWorkSpace.h"


using namespace ZQ;

template<class T>
void m_printf_mxn_f(const T* A, int m, int n)
{
	for(int i = 0;i < m;i++)
	{
		for(int j = 0;j < n;j++)
			printf("%13.4f",A[i*n+j]);
		printf("\n");
	}
}

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
	return rand()%10001/10000.0*scale;
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
bool check_visible(const T* cam_R, const T* cam_T, const T* board_R, const T* board_T, const T* cam_intrinsic, int width, int height, const T* X3, int n_pts, T* X2, double eps)
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

	T mv_r[3];
	if (!ZQ_Rodrigues::ZQ_Rodrigues_R2r(modelview_R, mv_r))
		return false;

	T w2 = 1.0 - mv_r[0] * mv_r[0] - mv_r[1] * mv_r[1] - mv_r[2] * mv_r[2];
	if (w2 < 0.64) // if w2 = 0.5, it means the angle is 90 degree
		return false;

	if (cam_intrinsic == 0 || X3 == 0 || X2 == 0)
		return false;

	T A[9] =
	{
		cam_intrinsic[0], 0, cam_intrinsic[2],
		0, cam_intrinsic[1], cam_intrinsic[3],
		0, 0, 1
	};
	ZQ_Calibration::proj_no_distortion(n_pts, A, modelview_R, modelview_T, X3, X2, eps);

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
	memcpy(rT + 3, T1_2, sizeof(T)* 3);
	return true;
}


template<class T>
int test_posit_no_coplanar_and_pose_estimation();

template<class T,  const bool zAxis_in>
int test_posit_coplanar_robust();

template<class T, const bool zAxis_in>
int test_binocalib_with_known_intrinsic();

template<class T>
int test_multicalib_with_known_intrinsic();

template<class T>
int test_stickCalib_closed_form_solution();

template<class T>
int test_stickCalib();

template<class T, const bool zAxis_in>
int test_CamCalib();

template<class T, const bool zAxis_in>
int test_proj();

int main()
{
	/*test_posit_no_coplanar_and_pose_estimation<float>();
	test_posit_no_coplanar_and_pose_estimation<double>();*/
	/*test_posit_coplanar_robust<float, true>();
	test_posit_coplanar_robust<float, false>();
	test_posit_coplanar_robust<double, true>();
	test_posit_coplanar_robust<double, false>();*/
	/*test_binocalib_with_known_intrinsic<float, true>();
	test_binocalib_with_known_intrinsic<float, false>();
	test_binocalib_with_known_intrinsic<double,true>();
	test_binocalib_with_known_intrinsic<double,false>();*/
	/*test_multicalib_with_known_intrinsic<float>();
	test_multicalib_with_known_intrinsic<double>();*/
	/*test_stickCalib_closed_form_solution<float>();
	test_stickCalib_closed_form_solution<double>();*/
	/*test_stickCalib<float>();
	test_stickCalib<double>();*/
	test_CamCalib<float, true>();
	test_CamCalib<float, false>();
	test_CamCalib<double, true>();
	test_CamCalib<double, false>();
	test_proj<float, true>();
	test_proj<float, false>();
	test_proj<double, true>();
	test_proj<double, false>();

	return 0;
}


template<class T>
int test_posit_no_coplanar_and_pose_estimation()
{
	
	srand(time(0));

	for(int cc = 0;cc < 100;cc++)
	{
		clock_t t1 = clock();
		T intrinsic_para[5] = {1200,1200,512,512};

		T A[9] =
		{
			intrinsic_para[0],0,intrinsic_para[2],
			0,intrinsic_para[1],intrinsic_para[3],
			0,0,1
		};

		T rT[6] = 
		{
			m_noise(0.2),
			m_noise(0.2),
			m_noise(0.2),
			m_noise(1),
			m_noise(1),
			m_noise(20)+80

		};

		int num_of_pts = 100;
		T* X3 = new T[num_of_pts*3];
		for(int i = 0;i < num_of_pts;i++)
		{
			X3[i*3+0] = m_noise(10)-5;
			X3[i*3+1] = m_noise(10)-5;
			X3[i*3+2] = m_noise(10)-5;
		}
		
		T* X2 = new T[num_of_pts*2];

		double eps = 1e-16;


		T R[9];
		T* TT = rT+3;
		ZQ_Rodrigues::ZQ_Rodrigues_r2R(rT,R);

		ZQ_Calibration::proj_no_distortion(num_of_pts,A,R,TT,X3,X2,eps);

		for(int i = 0;i < num_of_pts*2;i++)
		{
			double scale = 0.02;
			X2[i] += m_noise(scale)-0.5*scale;
		}

		int max_iter = 5000;
		T out_rT[6] = {0,0,0,0,0,100};


		ZQ_Calibration::posit_no_coplanar(num_of_pts,X3,X2,max_iter,intrinsic_para,out_rT);
		double avg_err;
		ZQ_Calibration::pose_estimate_no_distortion_with_init(num_of_pts,X3,X2,max_iter,intrinsic_para,out_rT,avg_err,eps);
		clock_t t2 = clock();

		printf("cost time: %f seconds\n",0.001*(t2-t1));

		m_printf_mxn_e(rT,1,6);
		m_printf_mxn_e(out_rT,1,6);
		delete []X2;
		delete []X3;

	}
	return 0;

}

template<class T, const bool zAxis_in>
int test_posit_coplanar_robust()
{
	int seed = time(0);
	printf("%d\n",seed);
	srand(seed);

	int posit_suc = 0;
	int optmi_count = 0;
	int max_cams = 10000;
	for (int vv = 0; vv < max_cams; vv++)
	{


		clock_t t1 = clock();
		T fc[2] = { 1200, 1200 };
		T cc[2] = { 512, 512 };
		T alpha_c = 0.001;
		T kc[5] = { 0.1, -2, 0.01, 0.02, 20 };
		
		int M = 7;
		int N = 7;
		int num_of_pts = M*N;
		ZQ_DImage<T> X3_im(num_of_pts * 3, 1);
		T*& X3 = X3_im.data();
		for(int i = 0;i < M;i++)
		{
			for(int j = 0;j < N;j++)
			{
				X3[(i*N+j)*3+0] = j - N/2;
				X3[(i*N+j)*3+1] = i - M/2;
				X3[(i*N+j)*3+2] = 0;
			}
		}

		double z_shift = 20;
		double x_shift = (cc[0]/fc[0])*z_shift-N/2;
		double y_shift = (cc[1]/fc[1])*z_shift-M/2;

		T rT[6] = 
		{
			m_noise(0.5)-0.25,
			m_noise(0.5)-0.25,
			m_noise(0.4)-0.2,
			m_noise(x_shift*.8)-x_shift*.4,
			m_noise(y_shift*.8)-y_shift*.4,
			m_noise(z_shift*0.2)+z_shift

		};
		if (!zAxis_in)
			rT[5] = -rT[5];
	
		ZQ_DImage<T> X2_im(num_of_pts * 2, 1);
		T*& X2 = X2_im.data();

		ZQ_CameraCalibration::project_points_fun(num_of_pts, X3, rT, fc, cc, kc, alpha_c, X2, zAxis_in);

		for(int i = 0;i < num_of_pts*2;i++)
		{
			double scale = 0.2;
			X2[i] += m_noise(scale)-0.5*scale;
		}
		
		int max_iter = 10;
		int max_iter_levmar = 50;
		double tol_E = 0.1;
		T out_rT[6] = {0,0,0,0,0,100};
		T out_rT2[6] = { 0, 0, 0, 0, 0, 100 };
		bool active_images[1] = { true };
		double avg_E;
		
		ZQ_CameraCalibration::PositCoplanarRobust(num_of_pts, X3, X2, fc, cc, kc, alpha_c, max_iter, max_iter_levmar, tol_E, out_rT, avg_E, zAxis_in);
		ZQ_CameraCalibration::_compute_extrinsic_param(1, num_of_pts, X2, X3, fc, cc, kc, alpha_c, out_rT2, active_images, max_iter, 1e6, false, zAxis_in);
		
		T diff_rT[6];
		ZQ_MathBase::VecMinus(6, rT, out_rT, diff_rT);
		double norm_inf1 = ZQ_MathBase::NormVector_Linf(6, diff_rT);
		ZQ_MathBase::VecMinus(6, rT, out_rT2, diff_rT);
		double norm_inf2 = ZQ_MathBase::NormVector_Linf(6, diff_rT);
		ZQ_MathBase::VecMinus(6, out_rT, out_rT2, diff_rT);
		double norm_inf_1_2 = ZQ_MathBase::NormVector_Linf(6, diff_rT);
		if (norm_inf_1_2 > 1e-4 && (norm_inf1 > 1e-4 || norm_inf2 > 1e-4))
		{
			printf("vv=%d\n", vv);
			m_printf_mxn_e(rT, 1, 6);
			m_printf_mxn_e(out_rT, 1, 6);
			m_printf_mxn_e(out_rT2, 1, 6);
		}
		
		if((vv+1)%100==0)
			printf("vv=%d\n",vv+1);

	}
	
	return 0;

}

template<class T, const bool zAxis_in>
int test_binocalib_with_known_intrinsic()
{
	double proj_noise_scale = 0.005;
	T left_fc[2] = { 500, 500 };
	T left_cc[2] = { 512, 512 };
	T left_alpha_c = 0.001;
	T left_kc[5] = { 0.01, -0.01, 0.001, 0.001, 0.01 };
	
	T right_fc[2] = { 600, 600 };
	T right_cc[2] = { 512, 512 };
	T right_alpha_c = 0.001;
	T right_kc[5] = { 0.02, -0.02, 0.002, 0.002, 0.02 };

	int n_views = 30;
	int M = 5;
	int N = 4;
	int num_of_pts = M*N;
	ZQ_DImage<T> X3_im(num_of_pts * 3 * n_views, 1);
	T*& X3 = X3_im.data();
	for (int i = 0; i < M; i++)
	{
		for (int j = 0; j < N; j++)
		{
			X3[(i*N + j) * 3 + 0] = j - N / 2;
			X3[(i*N + j) * 3 + 1] = i - M / 2;
			X3[(i*N + j) * 3 + 2] = 0;
		}
	}
	for (int vv = 1; vv < n_views; vv++)
		memcpy(X3 + vv*num_of_pts * 3, X3, sizeof(T)*num_of_pts * 3);

	double z_shift = 20;
	double x_shift = (right_cc[0] / right_fc[0])*z_shift - N / 2;
	double y_shift = (right_cc[1] / right_fc[1])*z_shift - M / 2;

	ZQ_DImage<T> right_rT_im(n_views * 6, 1);
	T*& right_rT = right_rT_im.data();
	for (int cc = 0; cc < n_views; cc++)
	{
		right_rT[cc * 6 + 0] = m_noise(0.5) - 0.25;
		right_rT[cc * 6 + 1] = m_noise(0.5) - 0.25;
		right_rT[cc * 6 + 2] = m_noise(0.4) - 0.2;
		right_rT[cc * 6 + 3] = m_noise(x_shift*.8) - x_shift*.4;
		right_rT[cc * 6 + 4] = m_noise(y_shift*.8) - y_shift*.4;
		right_rT[cc * 6 + 5] = m_noise(z_shift*.2) + z_shift;
		if (!zAxis_in)
			right_rT[cc * 6 + 5] = -right_rT[cc * 6 + 5];
	}
	
	T right_to_left_rT[6] = 
	{
		m_noise(0.1) - 0.05,
		m_noise(0.1) - 0.05,
		m_noise(0.1) - 0.05,
		z_shift*0.05,
		0.1,
		-0.1
	};
	if (!zAxis_in)
		right_to_left_rT[5] = -right_to_left_rT[5];

	ZQ_DImage<T> left_X2_im(n_views*num_of_pts * 2, 1);
	ZQ_DImage<T> right_X2_im(n_views*num_of_pts * 2, 1);
	T*& left_X2 = left_X2_im.data();
	T*& right_X2 = right_X2_im.data();

	for (int vv = 0; vv < n_views; vv++)
	{
		T left_rT[6];
		ZQ_StereoCalibration::_get_left_rT_from_right_rT_fun(right_rT + vv * 6, right_to_left_rT, left_rT);
		ZQ_CameraCalibration::project_points_fun(num_of_pts, X3 + num_of_pts * 3 * vv, left_rT, left_fc, left_cc, left_kc, left_alpha_c, left_X2 + num_of_pts * 2 * vv, zAxis_in);
		ZQ_CameraCalibration::project_points_fun(num_of_pts, X3 + num_of_pts * 3 * vv, right_rT + vv * 6, right_fc, right_cc, right_kc, right_alpha_c, right_X2 + num_of_pts * 2 * vv, zAxis_in);
	}

	for (int pp = 0; pp < n_views*num_of_pts * 2; pp++)
	{
		left_X2[pp] += m_noise(proj_noise_scale) - 0.5*proj_noise_scale;
		right_X2[pp] += m_noise(proj_noise_scale) - 0.5*proj_noise_scale;
	}

	ZQ_DImage<T> out_right_rT_im(n_views * 6, 1);
	T*& out_right_rT = out_right_rT_im.data();
	T out_right_to_left_rT[6];
	int max_iter = 30;
	clock_t t1 = clock();
	if (!ZQ_StereoCalibration::CalibrateBinocularCamera<T>(n_views, num_of_pts, X3, left_X2, right_X2, out_right_to_left_rT, out_right_rT, left_fc, left_cc, left_kc, left_alpha_c,
		right_fc, right_cc, right_kc, right_alpha_c, zAxis_in, 0, 0, 0, 0, max_iter, true, true))
	{
		printf("failed \n");
		return -1;
	}
	clock_t t2 = clock();
	printf("cost: %.3f secs\n", 0.001*(t2 - t1));
	printf("\n\n\n");
	m_printf_mxn_e(right_to_left_rT, 1, 6);
	m_printf_mxn_e(out_right_to_left_rT, 1, 6);
	printf("----------------------\n");
	printf("----------------------\n");
	for (int cc = 0; cc < n_views; cc++)
	{
		m_printf_mxn_e(right_rT + cc * 6, 1, 6);
		m_printf_mxn_e(out_right_rT + cc * 6, 1, 6);
		printf("----------------------\n");
	}
	return 0;
}

template<class T>
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
	T* intrinsic_para = new T[4 * n_cams];
	for (int i = 0; i < n_cams; i++)
	{
		double theta = (double)i / n_cams * 2 * m_pi;
		cam_T[i * 3 + 0] = circle_radius*cos(theta - 0.5*m_pi);
		cam_T[i * 3 + 1] = 0;
		cam_T[i * 3 + 2] = circle_radius*sin(theta - 0.5*m_pi);
		T tmp_R[9] = 
		{
			cos(theta), 0, -sin(theta),
			0,1,0,
			sin(theta), 0, cos(theta)
		};
		memcpy(cam_R+i*9, tmp_R, sizeof(T)* 9);
	}

	int width = 1000, height = 1000;
	for (int i = 0; i < n_cams; i++)
	{
		intrinsic_para[i * 4 + 0] = 500;
		intrinsic_para[i * 4 + 1] = 500;
		intrinsic_para[i * 4 + 2] = 512;
		intrinsic_para[i * 4 + 3] = 512;
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
		memcpy(checkboard_R + i * 9, tmp_R, sizeof(T)* 9);
		T tmp_T[3] = 
		{
			0,0,0
		};
		memcpy(checkboard_T + i * 3, tmp_T, sizeof(T)* 3);
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
			if (check_visible(cam_R + i * 9, cam_T + i * 3, checkboard_R + j * 9, checkboard_T + j * 3, intrinsic_para + i * 4, width, height, X3, num_of_pts, X2, eps))
			{
				cam_idx.push_back(i);
				checkboard_idx.push_back(j);
				X2_pts.push_back(X2);
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
	for (int i = 0; i < n_checkboard;i ++)
	{
		first_to_second_rT(checkboard_R + i * 9, checkboard_T + i * 3, cam_R, cam_T, ori_check_to_cam0_rT + i * 6);
	}
	for (int i = 1; i < n_cams; i++)
	{
		first_to_second_rT(cam_R, cam_T, cam_R + i * 9, cam_T + i * 3, ori_cam0_to_other_rT + (i - 1) * 6);
	}

	/*********************************/
	int max_iter_posit = 100;
	int max_iter_levmar = 500;
	double tol_E = 0.1;

	T* out_check_to_cam0_rT = new T[n_checkboard * 6];
	T* out_cam0_to_other_rT = new T[(n_cams - 1) * 6];
	
	double avg_E_square;
	if (!ZQ_Calibration::multicalib_with_known_intrinsic(n_checkboard, n_cams,num_of_pts, X3, all_X2, visible_num,visible_offset,visible_idx, intrinsic_para, 
		max_iter_posit, max_iter_levmar, tol_E, out_check_to_cam0_rT, out_cam0_to_other_rT, avg_E_square, eps))
	{
		printf("failed\n");
	}

	
	printf("avg_err_square = %e\n", avg_E_square);
	for (int cc = 0; cc < n_checkboard; cc++)
	{
		m_printf_mxn_e(ori_check_to_cam0_rT + cc * 6, 1, 6);
		m_printf_mxn_e(out_check_to_cam0_rT + cc * 6, 1, 6);
		printf("----------------------\n");
	}

	for (int cc = 0; cc < n_cams-1; cc++)
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
	delete[]intrinsic_para;
	delete[]visible_idx;
	delete[]visible_num;
	delete[]visible_offset;
	return 0;
}


template<class T>
int test_stickCalib_closed_form_solution()
{
	srand(0);
	T len_ac = 20.0;
	T len_bc = 10.0;
	T len_ab = len_ac + len_bc;
	int n_views = 16;
	T* X2 = new T[n_views * 6];
	T A_pos[3] = { -10, -5, 50 };
	T* theta = new T[n_views];
	T* phi = new T[n_views];
	T standard_intrinsic_para[4] = { 500, 500, 500, 500 };
	double m_pi = atan(1.0) * 4;
	T A[9] = 
	{
		standard_intrinsic_para[0], 0, standard_intrinsic_para[2],
		0, standard_intrinsic_para[1], standard_intrinsic_para[3],
		0,0,1
	};
	T R[9] = 
	{
		1,0,0,
		0,1,0,
		0,0,1
	};
	T t[3] = { 0, 0, 0 };

	for (int i = 0; i < n_views; i++)
	{
		theta[i] = 0.1+m_noise(0.8)*m_pi;
		phi[i] = 0.1+m_noise(0.8)*m_pi;
		T dir[3] = 
		{ 
			sin(theta[i])*cos(phi[i]),
			sin(theta[i])*sin(phi[i]),
			cos(theta[i])
		};
		T X3[9] =
		{
			A_pos[0], A_pos[1], A_pos[2],
			A_pos[0] + dir[0] * len_ac, A_pos[1] + dir[1] * len_ac, A_pos[2] + dir[2] * len_ac,
			A_pos[0] + dir[0] * len_ab, A_pos[1] + dir[1] * len_ab, A_pos[2] + dir[2] * len_ab
		};
		ZQ_Calibration::proj_no_distortion(3, A, R, t, X3, X2 + 6 * i,0);
	}

	T out_para[4];

	if (!ZQ_Calibration::stickCalib_closed_form_solution(n_views, len_ac, len_bc, X2, out_para))
	{
		printf("fail\n");
		delete[]X2;
		delete[]phi;
		delete[]theta;
		return 0;
	}
	m_printf_mxn_f(standard_intrinsic_para, 1, 4);
	m_printf_mxn_f(out_para, 1, 4);
	delete[]X2;
	delete[]phi;
	delete[]theta;
	return 0;
}

template<class T>
int test_stickCalib()
{
	srand(time(NULL));
	for (int pass = 0; pass < 100; pass ++)
	{
		const int n_pts = 5;
		T len_pts_to_A[10] = { 0, 10, 20, 30, 40, 5, 15, 25, 35, 45 };
		int n_views = 60;
		T* X2 = new T[n_views * n_pts * 2];
		T A_pos[3] = { -10, -5, 50 };
		T* theta_phi = new T[n_views * 2];
		T standard_intrinsic_para[4] = { 801.234, 810.43, 512.98, 510.78 };
		double m_pi = atan(1.0) * 4;
		T A[9] =
		{
			standard_intrinsic_para[0], 0, standard_intrinsic_para[2],
			0, standard_intrinsic_para[1], standard_intrinsic_para[3],
			0, 0, 1
		};
		T R[9] =
		{
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		};
		T t[3] = { 0, 0, 0 };

		for (int i = 0; i < n_views; i++)
		{
			double theta = (0.1 + m_noise(0.8))*m_pi;
			double phi = (0.5 + m_noise(0.3))*m_pi;
			theta_phi[i * 2 + 0] = theta;
			theta_phi[i * 2 + 1] = phi;
			T dir[3] =
			{
				sin(theta)*cos(phi),
				sin(theta)*sin(phi),
				cos(theta)
			};
			T X3[n_pts * 3];
			for (int pp = 0; pp < n_pts; pp++)
			{
				X3[pp * 3 + 0] = A_pos[0] + len_pts_to_A[pp] * dir[0];
				X3[pp * 3 + 1] = A_pos[1] + len_pts_to_A[pp] * dir[1];
				X3[pp * 3 + 2] = A_pos[2] + len_pts_to_A[pp] * dir[2];
			};
			ZQ_Calibration::proj_no_distortion(n_pts, A, R, t, X3, X2 + n_pts * 2 * i, 0);
		}

		double noise_scale = 0.2;
		for (int i = 0; i < n_views * n_pts * 2; i++)
		{
			X2[i] += m_noise(noise_scale) - 0.5*noise_scale;
		}

		T out_para[4];
		T out_A_pos[3];
		T* out_theta_phi = new T[n_views * 2];
		double avg_err_square;
		double eps = 1e-9;
		int max_iter = 300;
		if (!ZQ_Calibration::stickCalib_estimate_no_distortion_init(1200, 1200, n_views, n_pts, len_pts_to_A, X2, max_iter, out_para, out_A_pos, out_theta_phi))
		{
			printf("init fail\n");
			delete[]X2;
			delete[]theta_phi;
			delete[]out_theta_phi;
			return 0;
		}
		m_printf_mxn_f(standard_intrinsic_para, 1, 4);
		m_printf_mxn_f(out_para, 1, 4);
		//m_printf_mxn_f(theta_phi, n_views,2 );
		printf("\n");
		//m_printf_mxn_f(out_theta_phi, n_views, 2);
		if (!ZQ_Calibration::stickCalib_estimate_no_distortion_with_init(n_views, n_pts, len_pts_to_A, X2, max_iter, out_para, out_A_pos, out_theta_phi, avg_err_square, eps))
		{
			printf("fail\n");
			delete[]X2;
			delete[]theta_phi;
			delete[]out_theta_phi;
			return 0;
		}
		m_printf_mxn_f(standard_intrinsic_para, 1, 4);
		m_printf_mxn_f(out_para, 1, 4);
		printf("avg_err_square = %f\n", avg_err_square);
		delete[]X2;
		delete[]theta_phi;
		delete[]out_theta_phi;
	}
	return 0;
}

template<class T, const bool zAxis_in>
int test_CamCalib()
{
	int width = 1024;
	int height = 1024;
	float x3_noise_scale = 0.001;
	float x2_noise_scale = 0.01;
	//srand(time(0));
	int n_cams = 10;
	T* p = new T[10 + n_cams * 6];
	T* fc = p;
	T* cc = p + 2;
	T* alpha_c = p + 4;
	T* kc = p + 5;
	
	fc[0] = 800;
	fc[1] = 800;
	cc[0] = 500;
	cc[1] = 489;
	alpha_c[0] = 0.001;
	kc[0] = 0.1;
	kc[1] = -0.1;
	kc[2] = 0.0;
	kc[3] = 0.0;
	kc[4] = 0;
	for (int cc = 0; cc < n_cams; cc++)
	{
		T* r = p + 10 + cc * 6;
		T* t = p + 10 + cc * 6 + 3;
		switch (cc)
		{
		case 0:
			r[0] = 0.2;
			r[1] = 0;
			break;
		case 1:
			r[0] = -0.1;
			r[1] = 0;
			break;
		case 2:
			r[0] = 0;
			r[1] = 0.3;
			break;
		default:
			r[0] = m_noise(0.4) - 0.2;
			r[1] = m_noise(0.4) - 0.2;
			break;
		}
		r[2] = m_noise(0.0);
		t[0] = m_noise(0.0);
		t[1] = m_noise(0.0);
		t[2] = 5 + m_noise(1);
		if (!zAxis_in)
			t[2] = -t[2];
	}


	int M_point = 4;
	int N_point = 6;
	int num_of_pts = M_point*N_point;
	T* X3 = new T[num_of_pts * 3 * n_cams];
	T* X3_noise = new T[num_of_pts * 3];

	for (int i = 0; i < M_point; i++)
	{
		for (int j = 0; j < N_point; j++)
		{
			X3[(i*N_point + j) * 3 + 0] = i - N_point / 2;
			X3[(i*N_point + j) * 3 + 1] = j - M_point / 2;
			X3[(i*N_point + j) * 3 + 2] = 0;
		}
	}

	for (int i = 0; i < num_of_pts * 3; i++)
	{
		double scale = x3_noise_scale;
		X3_noise[i] = X3[i] + m_noise(scale) - 0.5*scale;
	}
	for (int cc = 1; cc < n_cams; cc++)
		memcpy(X3 + num_of_pts * 3 * cc, X3, sizeof(T)*num_of_pts * 3);

	T* X2 = new T[num_of_pts * 2 * n_cams];
	T* X2_1 = new T[num_of_pts * 2 * n_cams];

	T A[9] = 
	{
		fc[0], 0, cc[0],
		0, fc[1], cc[1],
		0,0,1
	};
	for (int vv = 0; vv < n_cams; vv++)
	{
		T* rT = p + 10 + vv * 6;
		T R[9];
		ZQ_Rodrigues::ZQ_Rodrigues_r2R(rT, R);
		if (!ZQ_CameraCalibration::project_points_fun(num_of_pts, X3_noise, rT, fc, cc, kc, alpha_c[0], X2 + num_of_pts*2*vv, zAxis_in))
		{
			printf("project fail!\n");
			return -1;
		}
	}
	
	for (int i = 0; i < num_of_pts * 2 * n_cams; i++)
	{
		double scale = x2_noise_scale;
		X2[i] += m_noise(scale) - 0.5*scale;
	}

	int max_iter = 30;
	T* out_p = new T[10 + n_cams * 6];
	memset(out_p, 0, sizeof(T)*(10 + n_cams * 6));


	bool* active_images = new bool[n_cams];
	for (int i = 0; i < n_cams; i++)
		active_images[i] = true;
	FILE* out1 = fopen("init.txt", "w");
	for (int i = 0; i < 10; i++)
		fprintf(out1, "%f ", p[i]);
	fprintf(out1, "\n");
	for (int i = 0; i < n_cams * 6; i++)
	{
		fprintf(out1, "%f ", p[10 + i]);
		if ((i + 1) % 6 == 0)
			fprintf(out1, "\n");
	}
	fclose(out1);

	// init fc, cc, alpha_c, kc
	double init_fovY = 35.0 / 45.0*atan(1.0);
	out_p[0] = out_p[1] = height*0.5 / tan(0.5*init_fovY);
	out_p[2] = width*0.5;
	out_p[3] = height*0.5;
	memset(out_p + 4, 0, sizeof(T)* 6);
	ZQ_DImage<T> R(n_cams * 9, 1);
	T*& R_data = R.data();
	double avg_err_square;
	double tol_E = 3;

	printf("truth:\n");
	m_printf_mxn_f(p, 2, 5);
	m_printf_mxn_f(p+10, n_cams, 6);
	printf("\n\n");
	
	memset(out_p, 0, sizeof(T)*(10 + n_cams * 6));
	ZQ_CameraCalibration::CalibrateCamera(n_cams, num_of_pts, width, height, X2, X3, out_p, out_p + 2, out_p + 5, out_p[4], out_p + 10, active_images, ZQ_CameraCalibration::CALIB_F2_C_ALPHA_K5, zAxis_in, max_iter, tol_E, false);

	printf("result1:\n");
	m_printf_mxn_f(out_p, 2, 5);
	m_printf_mxn_f(out_p + 10, n_cams, 6);
	printf("\n\n");

	memset(out_p, 0, sizeof(T)*(10 + n_cams * 6));
	ZQ_CameraCalibration::CalibrateCamera(n_cams, num_of_pts, width, height, X2, X3, out_p, out_p + 2, out_p + 5, out_p[4], out_p + 10, active_images, ZQ_CameraCalibration::CALIB_F2_C_ALPHA_K5, zAxis_in, max_iter, tol_E, true);

	printf("result2:\n");
	m_printf_mxn_f(out_p, 2, 5);
	m_printf_mxn_f(out_p + 10, n_cams, 6);
	printf("\n\n");
	delete[]out_p;
	delete[]p;
	delete[]X2;
	delete[]X3;
	delete[]X3_noise;

	return 0;
}

template<class T, const bool zAxis_in>
int test_proj()
{
	T fc[2] = { 1600, 1600 };
	T cc[2] = { 512, 512 };
	T kc[5] = { 0.1, -0.1, 0, 0, 0 };
	T alpha_c = 0.0001;
	T rT[6] = { 0.1, 0, 0, 0, 0, 50 };
	T X3[3] = { -1, -1, 0 };
	
	T x[2];
	T dxdrT[12];
	T dxdf[4];
	T dxdc[4];
	T dxdk[10];
	T dxdalpha[2];
	ZQ_CameraCalibration::project_points_fun(1, X3, rT, fc, cc, kc, alpha_c, x, zAxis_in);
	m_printf_mxn_f(x, 2, 1);
	printf("\n\n");
	ZQ_CameraCalibration::project_points_jac(1, X3, rT, fc, cc, kc, alpha_c, dxdrT, dxdf, dxdc, dxdk, dxdalpha, zAxis_in);
	m_printf_mxn_f(dxdrT, 2, 6);
	printf("\n\n");
	m_printf_mxn_f(dxdf, 2, 2);
	printf("\n\n");
	m_printf_mxn_f(dxdc, 2, 2);
	printf("\n\n");
	m_printf_mxn_f(dxdk, 2, 5);
	printf("\n\n");
	m_printf_mxn_f(dxdalpha, 2, 1);
	printf("\n\n");
	return 0;
}