#define _USE_UMFPACK 1
#include "ZQ_CameraCalibrationMono.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

using namespace ZQ;

template<class T>
void m_printf_mxn_f(const T* A, int m, int n)
{
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
			printf("%13.4f", A[i*n + j]);
		printf("\n");
	}
}


double m_noise(double scale)
{
	return rand() % 10001 / 10000.0*scale;
}

template<class T, const bool zAxis_in>
int test_CamCalib();


int main()
{
	test_CamCalib<float, true>();
	test_CamCalib<float, false>();
	test_CamCalib<double, true>();
	test_CamCalib<double, false>();
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
		if (!ZQ_CameraProjection::project_points_fun(num_of_pts, X3_noise, rT, fc, cc, kc, alpha_c[0], X2 + num_of_pts * 2 * vv, zAxis_in))
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
	/*FILE* out1 = 0;
	fopen_s(&out1,"init.txt", "w");
	for (int i = 0; i < 10; i++)
		fprintf(out1, "%f ", p[i]);
	fprintf(out1, "\n");
	for (int i = 0; i < n_cams * 6; i++)
	{
		fprintf(out1, "%f ", p[10 + i]);
		if ((i + 1) % 6 == 0)
			fprintf(out1, "\n");
	}
	fclose(out1);*/

	// init fc, cc, alpha_c, kc
	double init_fovY = 35.0 / 45.0*atan(1.0);
	out_p[0] = out_p[1] = height*0.5 / tan(0.5*init_fovY);
	out_p[2] = width*0.5;
	out_p[3] = height*0.5;
	memset(out_p + 4, 0, sizeof(T) * 6);
	ZQ_DImage<T> R(n_cams * 9, 1);
	T*& R_data = R.data();
	double avg_err_square;
	double tol_E = 3;

	printf("truth:\n");
	m_printf_mxn_f(p, 2, 5);
	m_printf_mxn_f(p + 10, n_cams, 6);
	printf("\n\n");

	memset(out_p, 0, sizeof(T)*(10 + n_cams * 6));
	ZQ_CameraCalibrationMono::CalibrateCamera(n_cams, num_of_pts, width, height, X2, X3, out_p, out_p + 2, out_p + 5, out_p[4], out_p + 10, active_images, 
		ZQ_CameraCalibrationMono::CALIB_F2_C_ALPHA_K5, zAxis_in, max_iter, tol_E, false);

	printf("result1:\n");
	m_printf_mxn_f(out_p, 2, 5);
	m_printf_mxn_f(out_p + 10, n_cams, 6);
	printf("\n\n");

	memset(out_p, 0, sizeof(T)*(10 + n_cams * 6));
	ZQ_CameraCalibrationMono::CalibrateCamera(n_cams, num_of_pts, width, height, X2, X3, out_p, out_p + 2, out_p + 5, out_p[4], out_p + 10, active_images, 
		ZQ_CameraCalibrationMono::CALIB_F2_C_ALPHA_K5, zAxis_in, max_iter, tol_E, true);

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
