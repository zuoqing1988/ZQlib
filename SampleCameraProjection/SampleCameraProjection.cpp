#include "ZQ_CameraProjection.h"
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

template<class T, const bool zAxis_in>
int test_proj();

int main()
{
	test_proj<float, true>();
	test_proj<float, false>();
	test_proj<double, true>();
	test_proj<double, false>();

	return 0;
}


template<class T, const bool zAxis_in>
int test_proj()
{
	T fc[2] = { 1600, 1600 };
	T cc[2] = { 512, 512 };
	T kc[5] = { 0.1, -0.1, 0, 0, 0 };
	T alpha_c = 0.0001;
	T rT[6] = { 0, 0, 0, 0, 0, 0 };
	T X3[3] = { -1, -1, -50 };

	if (zAxis_in)
		X3[2] = -X3[2];

	T x[2];
	T dxdrT[12];
	T dxdf[4];
	T dxdc[4];
	T dxdk[10];
	T dxdalpha[2];
	ZQ_CameraProjection::project_points_fun(1, X3, rT, fc, cc, kc, alpha_c, x, zAxis_in);
	m_printf_mxn_f(x, 2, 1);
	printf("\n");
	ZQ_CameraProjection::project_points_jac(1, X3, rT, fc, cc, kc, alpha_c, dxdrT, dxdf, dxdc, dxdk, dxdalpha, zAxis_in);
	m_printf_mxn_f(dxdrT, 2, 6);
	m_printf_mxn_f(dxdf, 2, 2);
	m_printf_mxn_f(dxdc, 2, 2);
	m_printf_mxn_f(dxdk, 2, 5);
	m_printf_mxn_f(dxdalpha, 2, 1);
	printf("\n");
	return 0;
}