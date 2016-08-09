#include "ZQ_LSQRSolver.h"
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

	T x0[3] = {0,0};
	T x1[3];
	taucs_ccs_matrix* A = ZQ_TaucsBase::ZQ_taucs_ccs_CreateMatrixFromColumns(cols,nRow,flag);
	int max_iter = 1000;
	double tol = 1e-9;
	int it1;
	ZQ_LSQRSolver::LSQRSolve(A,b,x0,max_iter,tol,x1,it1,true);
	printf("x1=(%f,%f)\n",x1[0],x1[1]);

}

int main()
{
	test<float>();
	test<double>();
	return 0;

}
