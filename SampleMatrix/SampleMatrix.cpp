#include "ZQ_Matrix.h"
using namespace ZQ;
int main()
{
	ZQ_Matrix<float> m1(2,3),m2(3,4),m3(2,4);
	float* ptr1 = m1.GetDataPtr();
	float* ptr2 = m2.GetDataPtr();
	for(int i = 0;i < 2*3;i++)
		ptr1[i] = rand()%10;
	for(int i = 0;i < 3*4;i++)
		ptr2[i] = rand()%10;

	m3 = m1*m2;
	
	ZQ_Matrix<float> m1_t = m1.GetTransposeMatrix();

	ZQ_Matrix<float>* m4 = m1.Clone();

	ZQ_Matrix<float> m5(2,4);
	ZQ_Matrix<float>::MatrixMul(m1,m2,m5);

	return 0;

}