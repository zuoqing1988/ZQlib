#include "ZQ_TaucsBase.h"
#include <typeinfo>
#include <map>
#include <vector>

using namespace ZQ;
template<class T>
void test()
{
	srand(0);
	int m = 10, n = 5;
	std::vector<std::map<int, T>> cols;
	for(int i = 0;i < n;i++)
	{
		std::map<int, T> one_col;
		for(int j = 0;j < m;j++)
		{
			if(rand()%5 == 0)
				one_col.insert(std::make_pair(j,rand()%5+1));
		}
		cols.push_back(one_col);
	}
	int flag;
	if(strcmp(typeid(T).name(),"float") == 0)
		flag = TAUCS_SINGLE;
	else if(strcmp(typeid(T).name(),"double") == 0)
		flag = TAUCS_DOUBLE;
	else
		return;

	taucs_ccs_matrix* A = ZQ_TaucsBase::ZQ_taucs_ccs_CreateMatrixFromColumns(cols,m,flag);
	taucs_ccs_matrix* At = ZQ_TaucsBase::ZQ_taucs_ccs_matrixTranspose(A);
	taucs_ccs_matrix* AtA = ZQ_TaucsBase::ZQ_taucs_ccs_mul2NonSymmetricMatrices(At,A);
	taucs_ccs_matrix* As = ZQ_TaucsBase::ZQ_taucs_ccs_scaleMatrix(A,2);

	cols.clear();
	for(int i = 0;i < n;i++)
	{
		std::map<int, T> one_col;
		for(int j = 0;j < m;j++)
		{
			if(rand()%5 == 0)
				one_col.insert(std::make_pair(j,rand()%5+1));
		}
		cols.push_back(one_col);
	}

	taucs_ccs_matrix* B = ZQ_TaucsBase::ZQ_taucs_ccs_CreateMatrixFromColumns(cols,m,flag);
	taucs_ccs_matrix* C = ZQ_TaucsBase::ZQ_taucs_ccs_add2NonSymmetricMatrices(A,B);
	taucs_ccs_matrix* D = ZQ_TaucsBase::ZQ_taucs_ccs_mul2NonSymmetricMatrices(At,B);

	T* b = new T[n];
	for(int i = 0;i < n;i++)
		b[i] = i+1;
	T* Ab = new T[m];
	ZQ_TaucsBase::ZQ_taucs_ccs_matrix_time_vec(A,b,Ab);
	T* bAt = new T[m];
	ZQ_TaucsBase::ZQ_taucs_ccs_vec_time_matrix(b,At,bAt);

	printf("type %s:\n",typeid(T).name());
	printf("Ab:\n");
	for(int i = 0;i < n;i++)
	{
		printf("%f ",Ab[i]);
	}
	printf("\nbtAt:\n");
	for(int i = 0;i < n;i++)
	{
		printf("%f ",bAt[i]);
	}
	printf("\n");

	ZQ_TaucsBase::ZQ_taucs_ccs_free(A);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(As);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(At);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(AtA);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(B);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(C);
	ZQ_TaucsBase::ZQ_taucs_ccs_free(D);

	delete []Ab;
	delete []bAt;
	delete []b;

	

}

int main()
{
	test<float>();
	test<double>();

	return 0;
}