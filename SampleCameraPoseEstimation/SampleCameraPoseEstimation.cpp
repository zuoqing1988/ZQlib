#include "ZQ_CameraCalibrationMono.h"
#include "ZQ_CameraPoseEstimation.h"
#include <time.h>

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
int test_posit_coplanar_robust();


int main()
{
	test_posit_coplanar_robust<double, true>();
	test_posit_coplanar_robust<double, false>();

	// float is not accurate
	test_posit_coplanar_robust<float, true>();
	test_posit_coplanar_robust<float, false>();

	return 0;
}

template<class T, const bool zAxis_in>
int test_posit_coplanar_robust()
{
	int seed = time(0);
	printf("%d\n", seed);
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
		for (int i = 0; i < M; i++)
		{
			for (int j = 0; j < N; j++)
			{
				X3[(i*N + j) * 3 + 0] = j - N / 2;
				X3[(i*N + j) * 3 + 1] = i - M / 2;
				X3[(i*N + j) * 3 + 2] = 0;
			}
		}

		double z_shift = 20;
		double x_shift = (cc[0] / fc[0])*z_shift - N / 2;
		double y_shift = (cc[1] / fc[1])*z_shift - M / 2;

		T rT[6] =
		{
			m_noise(0.5) - 0.25,
			m_noise(0.5) - 0.25,
			m_noise(0.4) - 0.2,
			m_noise(x_shift*.8) - x_shift*.4,
			m_noise(y_shift*.8) - y_shift*.4,
			m_noise(z_shift*0.2) + z_shift

		};
		if (!zAxis_in)
			rT[5] = -rT[5];

		ZQ_DImage<T> X2_im(num_of_pts * 2, 1);
		T*& X2 = X2_im.data();

		ZQ_CameraProjection::project_points_fun(num_of_pts, X3, rT, fc, cc, kc, alpha_c, X2, zAxis_in);

		for (int i = 0; i < num_of_pts * 2; i++)
		{
			double scale = 0.2;
			X2[i] += m_noise(scale) - 0.5*scale;
		}

		int max_iter = 10;
		int max_iter_levmar = 50;
		double tol_E = 0.1;
		T out_rT[6] = { 0,0,0,0,0,100 };
		T out_rT2[6] = { 0, 0, 0, 0, 0, 100 };
		bool active_images[1] = { true };
		double avg_E;

		ZQ_CameraPoseEstimation::PositCoplanarRobust(num_of_pts, X3, X2, fc, cc, kc, alpha_c, max_iter, max_iter_levmar, tol_E, out_rT, avg_E, zAxis_in);
		ZQ_CameraCalibrationMono::_compute_extrinsic_param(1, num_of_pts, X2, X3, fc, cc, kc, alpha_c, out_rT2, active_images, max_iter, 1e6, false, zAxis_in);

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

		if ((vv + 1) % 100 == 0)
			printf("vv=%d\n", vv + 1);

	}

	return 0;

}
