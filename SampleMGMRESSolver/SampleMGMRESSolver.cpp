#include "ZQ_MGMRESSolver.h"
#include "ZQ_SparseMatrix.h"
#include <iostream>
using namespace ZQ;
using namespace std;

int main()
{
	double AA[9] = 
	{
		1,1,0,
		2,1,0,
		3,0,2
	};
	ZQ_SparseMatrix<double> At_mat(3, 3);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (AA[i * 3 + j] != 0)
				At_mat.AddTo(j, i, AA[i * 3 + j]);
		}
	}
	taucs_ccs_matrix* At = At_mat.ExportCCS(TAUCS_DOUBLE);
	double x[3] = { 0 };
	double b[3] = { 1, 2, 3 };
	ZQ_MGMRESSolver::MGMResSolve(At, b, x, 1, 10, 1e-9, true);
	printf("%f %f %f\n", x[0], x[1], x[2]);
	return 0;
}
int main1()
{
# define N 20
# define NZ_NUM 3 * N - 2

	double a[NZ_NUM];
	int ia[NZ_NUM];
	int itr_max;
	int ja[NZ_NUM];
	int mr;
	int n = N;
	int nz_num = NZ_NUM;
	double rhs[N];
	double tol_abs;
	double tol_rel;
	double x_error;
	double x_estimate[N];
	double x_exact[N];

	cout << "\n";
	cout << "TEST01\n";
	cout << "  Test MGMRES_ST on the simple -1,2-1 matrix.\n";
	//
	//  Set the matrix.
	//  Note that we use zero based index values in IA and JA.
	//
	
	int k = 0;

	for (int i = 0; i < n; i++)
	{
		if (0 < i)
		{
			ia[k] = i;
			ja[k] = i - 1;
			a[k] = -1.0;
			k = k + 1;
		}

		ia[k] = i;
		ja[k] = i;
		a[k] = 2.0;
		k = k + 1;

		if (i < n - 1)
		{
			ia[k] = i;
			ja[k] = i + 1;
			a[k] = -1.0;
			k = k + 1;
		}

	}
	//
	//  Set the right hand side:
	//
	for (int i = 0; i < n - 1; i++)
	{
		rhs[i] = 0.0;
	}
	rhs[N - 1] = (double)(n + 1);
	//
	//  Set the exact solution.
	//
	for (int i = 0; i < n; i++)
	{
		x_exact[i] = (double)(i + 1);
	}

	for (int test = 1; test <= 3; test++)
	{
		//
		//  Set the initial solution estimate.
		//
		for (int i = 0; i < n; i++)
		{
			x_estimate[i] = 0.0;
		}

		x_error = 0.0;
		for (int i = 0; i < n; i++)
		{
			x_error = x_error + pow(x_exact[i] - x_estimate[i], 2);
		}
		x_error = sqrt(x_error);

		if (test == 1)
		{
			itr_max = 1;
			mr = 20;
		}
		else if (test == 2)
		{
			itr_max = 2;
			mr = 10;
		}
		else if (test == 3)
		{
			itr_max = 5;
			mr = 4;
		}
		tol_abs = 1.0E-08;
		tol_rel = 1.0E-08;

		cout << "\n";
		cout << "  Test " << test << "\n";
		cout << "  Matrix order N = " << n << "\n";
		cout << "  Inner iteration limit = " << mr << "\n";
		cout << "  Outer iteration limit = " << itr_max << "\n";
		cout << "  Initial X_ERROR = " << x_error << "\n";

		ZQ_MGMRESSolver::mgmres_st(n, nz_num, ia, ja, a, x_estimate, rhs, itr_max, mr, tol_abs, tol_rel,true);

		x_error = 0.0;
		for (int i = 0; i < n; i++)
		{
			x_error = x_error + pow(x_exact[i] - x_estimate[i], 2);
		}
		x_error = sqrt(x_error);

		cout << "  Final X_ERROR = " << x_error << "\n";
	}
	return 0;
# undef N
# undef NZ_NUM


}