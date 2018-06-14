#define _USE_UMFPACK 1
#include "ZQ_CameraCalibration.h"
#include "ZQ_StereoCalibration.h"
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

template<class T, const bool zAxis_in>
int test_binocalib_with_known_intrinsic();

template<class T, const bool zAxis_in>
int test_binocalib();

int main()
{
	/*float will return false, may be precision problem*/
	//test_binocalib_with_known_intrinsic<float, true>();
	//test_binocalib_with_known_intrinsic<float, false>();
	test_binocalib_with_known_intrinsic<double,true>();
	test_binocalib_with_known_intrinsic<double,false>();
	test_binocalib<double, true>();
	test_binocalib<double, false>();
	return 0;
}



template<class T, const bool zAxis_in>
int test_binocalib_with_known_intrinsic()
{
	double proj_noise_scale = 0.5;
	T left_fc[2] = { 500, 500 };
	T left_cc[2] = { 512, 512 };
	T left_alpha_c = 0.001;
	T left_kc[5] = { 0.01, -0.01, 0.001, 0.001, 0.01 };

	T right_fc[2] = { 600, 600 };
	T right_cc[2] = { 512, 512 };
	T right_alpha_c = 0.001;
	T right_kc[5] = { 0.02, -0.02, 0.002, 0.002, 0.02 };

	int n_views = 20;
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




template<class T, const bool zAxis_in>
int test_binocalib()
{
	double proj_noise_scale = 0.5;
	T left_fc[2] = { 500, 500 };
	T left_cc[2] = { 512, 512 };
	T left_alpha_c = 0.001;
	T left_kc[5] = { 0.01, -0.01, 0.001, 0.001, 0.01 };

	T right_fc[2] = { 600, 600 };
	T right_cc[2] = { 512, 512 };
	T right_alpha_c = 0.001;
	T right_kc[5] = { 0.02, -0.02, 0.002, 0.002, 0.02 };

	

	T in_left_fc[2] = { left_fc[0] * (m_noise(0.02) + 0.99), left_fc[1] * (m_noise(0.02) + 0.99) };
	T in_left_cc[2] = { left_cc[0] * (m_noise(0.02) + 0.99), left_cc[1] * (m_noise(0.02) + 0.99) };
	T in_left_alpha_c = left_alpha_c;
	T in_left_kc[5];
	memcpy(in_left_kc, left_kc, sizeof(T) * 5);
	T in_right_fc[2] = { right_fc[0] * (m_noise(0.02) + 0.99), right_fc[1] * (m_noise(0.02) + 0.99) };
	T in_right_cc[2] = { right_cc[0] * (m_noise(0.02) + 0.99), right_cc[1] * (m_noise(0.02) + 0.99) };
	T in_right_alpha_c = right_alpha_c;
	T in_right_kc[5];
	memcpy(in_right_kc, right_kc, sizeof(T) * 5);

	T out_left_fc[2];
	memcpy(out_left_fc, in_left_fc, sizeof(T) * 2);
	T out_left_cc[2];
	memcpy(out_left_cc, in_left_cc, sizeof(T) * 2);
	T out_left_alpha_c = in_left_alpha_c;
	T out_left_kc[5];
	memcpy(out_left_fc, in_left_fc, sizeof(T) * 5);
	T out_right_fc[2];
	memcpy(out_right_fc, in_right_fc, sizeof(T) * 2);
	T out_right_cc[2];
	memcpy(out_right_cc, in_right_cc, sizeof(T) * 2);
	T out_right_alpha_c = in_right_alpha_c;
	T out_right_kc[5];
	memcpy(out_right_fc, in_right_fc, sizeof(T) * 5);

	int n_views = 20;
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
	if (!ZQ_StereoCalibration::CalibrateBinocularCamera2<T>(n_views, num_of_pts, X3, left_X2, right_X2, out_right_to_left_rT, out_right_rT, out_left_fc, out_left_cc, out_left_kc, out_left_alpha_c,
		out_right_fc, out_right_cc, out_right_kc, out_right_alpha_c, ZQ_CameraCalibration::CALIB_F2_C_ALPHA_K5, zAxis_in, 0, 0, 0, 0, max_iter, true, true))
	{
		printf("failed \n");
		return -1;
	}
	clock_t t2 = clock();
	printf("cost: %.3f secs\n", 0.001*(t2 - t1));
	printf("\n\n\n");
	printf("truth  left: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", left_fc[0], left_fc[1], left_cc[0], left_cc[1], left_alpha_c,
		left_kc[0], left_kc[1], left_kc[2], left_kc[3], left_kc[4]);
	printf("input  left: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", in_left_fc[0], in_left_fc[1], in_left_cc[0], in_left_cc[1], in_left_alpha_c,
		in_left_kc[0], in_left_kc[1], in_left_kc[2], in_left_kc[3], in_left_kc[4]);
	printf("output left: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", out_left_fc[0], out_left_fc[1], out_left_cc[0], out_left_cc[1], out_left_alpha_c,
		out_left_kc[0], out_left_kc[1], out_left_kc[2], out_left_kc[3], out_left_kc[4]);
	printf("truth  right: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", right_fc[0], right_fc[1], right_cc[0], right_cc[1], right_alpha_c,
		right_kc[0], right_kc[1], right_kc[2], right_kc[3], right_kc[4]);
	printf("input  right: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", in_right_fc[0], in_right_fc[1], in_right_cc[0], in_right_cc[1], in_right_alpha_c,
		in_right_kc[0], in_right_kc[1], in_right_kc[2], in_right_kc[3], in_right_kc[4]);
	printf("output right: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", out_right_fc[0], out_right_fc[1], out_right_cc[0], out_right_cc[1], out_right_alpha_c,
		out_right_kc[0], out_right_kc[1], out_right_kc[2], out_right_kc[3], out_right_kc[4]);
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

