#include "ZQ_SparseMatrix.h"
#include "ZQ_PCGSolver.h"

using namespace ZQ;

template<class T>
void test()
{
	int flag;
	if(strcmp(typeid(T).name(),"float") == 0)
		flag = TAUCS_SINGLE;
	else if(strcmp(typeid(T).name(),"double") == 0)
		flag = TAUCS_DOUBLE;
	else
		return;

	ZQ_SparseMatrix<T> sp_m(3,2);
	sp_m.SetValue(0,0,1);
	sp_m.SetValue(0,1,2);
	sp_m.SetValue(1,0,2);
	sp_m.SetValue(1,1,3);
	sp_m.SetValue(2,0,-3);
	sp_m.SetValue(2,1,1);
	

	T b[3] = {0,1,2};

	T x0[3] = {0,0};
	T x1[3];
	taucs_ccs_matrix* A = sp_m.ExportCCS(flag);
	int max_iter = 1000;
	double tol = 1e-9;
	int it1;
	ZQ_PCGSolver::PCG_sparse_unsquare(A,b,x0,max_iter,tol,x1,it1,true);

	printf("x1=(%f,%f)\n",x1[0],x1[1]);

}

int main()
{
	test<float>();
	test<double>();
	return 0;

}