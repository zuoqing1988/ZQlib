#include "ZQ_SVD.h"

using namespace ZQ;
int main()
{
	ZQ_Matrix<float> A(3,2),invA(2,3),U(3,2),SS(2,2),V(2,2),b(3,5),x(2,5);
	float* A_ptr = A.GetDataPtr();
	float* U_ptr = U.GetDataPtr();
	float* S_ptr = SS.GetDataPtr();
	float* V_ptr = V.GetDataPtr();
	float* b_ptr = b.GetDataPtr();
	float* x_ptr = x.GetDataPtr();
	for(int i = 0;i < 2*3;i++)
		A_ptr[i] = rand()%10;
	for(int i = 0;i < 2*5;i++)
		b_ptr[i] = rand()%10;

	ZQ_SVD::Decompose(A,U,SS,V);
	ZQ_SVD::Invert(A,invA);
	ZQ_SVD::Solve(A,x,b);

	return 0;
}