#include "ZQ_PCGSolver.h"

#include <vector>
#include <map>

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

	int nRow = 3;
	std::vector<std::map<int,T>> cols(2);
	cols[0].insert(std::make_pair(0,1));
	cols[0].insert(std::make_pair(1,2));
	cols[0].insert(std::make_pair(2,-3));
	cols[1].insert(std::make_pair(0,2));
	cols[1].insert(std::make_pair(1,3));
	cols[1].insert(std::make_pair(2,1));

	T b[3] = {0,1,2};
	
	T Atb[2];
	T l[2] = {-0.1,-0.1};
	T u[2] = {0.1,5};
	T x0[3] = {0,0};
	T x1[3];
	T x2[3];
	T x3[3];
	taucs_ccs_matrix* A = ZQ_TaucsBase::ZQ_taucs_ccs_CreateMatrixFromColumns(cols,nRow,flag);
	taucs_ccs_matrix* At = ZQ_TaucsBase::ZQ_taucs_ccs_matrixTranspose(A);
	taucs_ccs_matrix* AtA = ZQ_TaucsBase::ZQ_taucs_ccs_mul2NonSymmetricMatrices(At,A);
	ZQ_TaucsBase::ZQ_taucs_ccs_matrix_time_vec(At,b,Atb);
	int max_iter = 1000;
	double tol = 1e-9;
	int it1,it2,it3;
	ZQ_PCGSolver::PCG(AtA,Atb,x0,max_iter,tol,x1,it1,true);
	ZQ_PCGSolver::PCG_sparse_unsquare(A,b,x0,max_iter,tol,x2,it2,true);

	double val;
	int exit_code;
	ZQ_PCGSolver::PCG_BQP(AtA,Atb,x0,l,u,max_iter,tol,tol,x3,val,it3,exit_code,true);
	printf("x1=(%f,%f)\n",x1[0],x1[1]);
	printf("x2=(%f,%f)\n",x2[0],x2[1]);
	printf("x3=(%f,%f)\n",x3[0],x3[1]);

}
int main()
{
	test<double>();
	test<float>();
	
	return 0;
}