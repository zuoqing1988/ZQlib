#include "ZQ_Rodrigues.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace ZQ;

template<class T>
void test_dRdr();

template<class T>
void test_drdR();


template<class T>
void m_printf_mxn_f(const T* A, int m, int n);

double m_rand()
{
	return rand() % 2000001 / 1000000.0 - 1.0;
}

int main()
{
	//srand(time(0));

	//test_dRdr<float>();
	//test_drdR<float>();
	test_dRdr<double>();
	//test_drdR<double>();
	return 0;
}

template<class T>
void test_dRdr()
{
	printf("test dRdr\n");
	// test of the Jacobians :
	
	T r[3] = { 1e-6, 1e-6, 1e-6 }; 
	T dr[3] = { 1e-6, 1e-6, 1e-6 };
	//T r[3] = { m_rand(), m_rand(), m_rand() };
	//T dr[3] = { m_rand()*1e-6, m_rand()*1e-6, m_rand()*1e-6 };
	T r_plus_dr[3];
	ZQ_MathBase::VecPlus(3, r, dr, r_plus_dr);

	T R1[9], dR1[27];
	T R2[9];
	ZQ_Rodrigues::ZQ_Rodrigues_r2R(r, R1, dR1);
	ZQ_Rodrigues::ZQ_Rodrigues_r2R(r_plus_dr, R2);

	T R2a[9], dR1_mul_dr[9];
	ZQ_MathBase::MatrixMul(dR1, dr, 9, 3, 1, dR1_mul_dr);
	ZQ_MathBase::VecPlus(9, R1, dR1_mul_dr, R2a);

	T R2_minus_R1[9], R2_minus_R2a[9];
	ZQ_MathBase::VecMinus(9, R2, R1, R2_minus_R1);
	ZQ_MathBase::VecMinus(9, R2, R2a, R2_minus_R2a);
	double gain = ZQ_MathBase::NormVector_L2(9, R2_minus_R1) / ZQ_MathBase::NormVector_L2(9, R2_minus_R2a);
	printf("gain = %f\n", gain);
}

template<class T>
void test_drdR()
{
	printf("test drdR\n");
	T r[3] = { m_rand(), m_rand(), m_rand() };
	T R[9];
	ZQ_Rodrigues::ZQ_Rodrigues_r2R(r, R);

	T dr[3] = { m_rand()*1e-4, m_rand()*1e-4, m_rand()*1e-4 };
	T r_plus_dr[3];
	ZQ_MathBase::VecPlus(3, r, dr, r_plus_dr);
	T R_[9];
	ZQ_Rodrigues::ZQ_Rodrigues_r2R(r_plus_dr, R_);
	T dR[9];
	ZQ_MathBase::VecMinus(9, R_, R,dR);
	
	T rc[3], drdR[27];
	ZQ_Rodrigues::ZQ_Rodrigues_R2r(R, rc, drdR);
	T rc2[3], R_plus_dR[9];
	ZQ_MathBase::VecPlus(9, R, dR, R_plus_dR);
	ZQ_Rodrigues::ZQ_Rodrigues_R2r(R_plus_dR, rc2);

	T drdR_mul_dR[3];
	ZQ_MathBase::MatrixMul(drdR, dR, 3, 9, 1, drdR_mul_dR);
	T r_app[3];
	ZQ_MathBase::VecPlus(3,rc, drdR_mul_dR, r_app);
	
	T rc2_minus_rc[3], rc2_minus_r_app[3];
	ZQ_MathBase::VecMinus(3, rc2, rc, rc2_minus_rc);
	ZQ_MathBase::VecMinus(3, rc2, r_app, rc2_minus_r_app);
	double gain = ZQ_MathBase::NormVector_L2(3,rc2_minus_rc) / ZQ_MathBase::NormVector_L2(3,rc2_minus_r_app);

	printf("gain = %f\n", gain);

	/*	% %% OTHER BUG : (FIXED NOW!!!)

		omu = randn(3, 1);
	omu = omu / norm(omu)
		om = pi*omu;
	[R, dR] = rodrigues(om);
	[om2] = rodrigues(R);
	[om om2]

	% %% NORMAL OPERATION

		om = randn(3, 1);
	[R, dR] = rodrigues(om);
	[om2] = rodrigues(R);
	[om om2]

	return

		% Test: norm(om) = pi

		u = randn(3, 1);
	u = u / sqrt(sum(u. ^ 2));
	om = pi*u;
	R = rodrigues(om);

	R2 = rodrigues(rodrigues(R));

	norm(R - R2)*/
}


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