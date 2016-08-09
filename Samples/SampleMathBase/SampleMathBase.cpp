#include "ZQ_MathBase.h"
#include <stdio.h>
#include <stdlib.h>

using namespace ZQ;

void test_length2()
{
	int n = 10;
	float* v = new float[n];
	for(int i = 0;i < n;i++)
		v[i] = i;
	float length2 = ZQ_MathBase::Length2(n,v);
	float normL2 = ZQ_MathBase::NormVector_L2(n,v);
	float normL1 = ZQ_MathBase::NormVector_L1(n,v);
	float normlinf = ZQ_MathBase::NormVector_Linf(n,v);
	delete []v;
	printf("length2 = %f, normL2 = %f, normL1 = %f, normLinf = %f\n",length2,normL2,normL1,normlinf);
}

void test_product()
{
	int n = 10;
	float* v1 = new float[n];
	float* v2 = new float[n];
	for(int i = 0;i < n;i++)
	{
		v1[i] = rand()%10;
		v2[i] = rand()%10;
	}
	float dotv1v2 = ZQ_MathBase::DotProduct(n,v1,v2);
	printf("dotproduct = %f\n",dotv1v2);
	delete []v1;
	delete []v2;

	int sp_idx1[3] = {1,3,5};
	float sp_v1[3] = {1,2,3};
	int sp_idx2[3] = {2,5,7};
	float sp_v2[3] = {1,2,3};
	float dot_sp_v1v2 = ZQ_MathBase::DotProductSparse(n,3,sp_idx1,sp_v1,3,sp_idx2,sp_v2);
	printf("dot_sparse=%f\n",dot_sp_v1v2);

	float cro_v1[3] = {1,1,2};
	float cro_v2[3] = {2,1,1};
	float cro_v3[3];
	ZQ_MathBase::CrossProduct(cro_v1,cro_v2,cro_v3);
	printf("v3=[%f %f %f]\n",cro_v3[0],cro_v3[1],cro_v3[2]);
}

void test_binarysearch()
{
	float values[10] = 
	{
		1,3,4,6,7,8,10,12,14,50
	};
	float val1 = 2;
	float val2 = 14;
	int idx1 = ZQ_MathBase::BinarySearch(10,values,val1,true);
	int idx2 = ZQ_MathBase::BinarySearch(10,values,val2,true);
	printf("idx1=%d, idx2=%d\n",idx1,idx2);
}

void test_vec()
{
	int n = 10;
	float* v1 = new float[n];
	float* v2 = new float[n];
	for(int i = 0;i < n;i++)
	{
		v1[i] = rand()%10;
		v2[i] = rand()%10+1;
	}
	float* v3 = new float[n];
	ZQ_MathBase::VecPlus(n,v1,v2,v3);
	ZQ_MathBase::VecMinus(n,v1,v2,v3);
	ZQ_MathBase::VecMul(n,v1,v2,v3);
	ZQ_MathBase::VecDiv(n,v1,v2,v3);
	delete []v1;
	delete []v2;
	delete []v3;
}

void test_sign()
{
	int sn = ZQ_MathBase::Sign(-10);
}

void test_rem()
{
	float x = 10, y = -2.1f;
	printf("rem(%f,%f)=%f\n",x,y,ZQ_MathBase::Rem(x,y));


}

void test_find_minmax()
{
	int n = 10;
	float* v = new float[n];
	for(int i = 0;i < n;i++)
		v[i] = rand()%100;
	int min_id,max_id;
	float min,max;
	ZQ_MathBase::FindMin(n,v,min,min_id);
	ZQ_MathBase::FindMax(n,v,max,max_id);
	delete []v;
}

void test_mat()
{
	int m = 10, n = 20, k = 30;
	float* A = new float[m*n];
	float* B = new float[n*k];
	float* C = new float[m*k];
	for(int i = 0;i < m*n;i++)
		A[i] = rand()%10;
	for(int i = 0;i < n*k;i++)
		B[i] = rand()%10;
	ZQ_MathBase::MatrixMul(A,B,m,n,k,C);
	delete []A;
	delete []B;
	delete []C;

	float m1[6] = {1,2,3,4,5,6};
	float m2[6];
	ZQ_MathBase::MatrixTranspose(m1,2,3,m2);

	float m3[4] = {1,2,3,4};
	float m4[4];
	ZQ_MathBase::MatrixInverse(m3,2,m4);
	printf("m4=\n[%f %f\n%f %f]\n",m4[0],m4[1],m4[2],m4[3]);

	float m5[4];
	ZQ_MathBase::MatrixMul(m3,m4,2,2,2,m5);
	printf("m5=\n[%f %f\n%f %f]\n",m5[0],m5[1],m5[2],m5[3]);

	ZQ_MathBase::MatrixIdentity(m5,2);


}

void test_svd()
{
	int m = 10;
	int n = 20;
	int sdim = __min(m,n);
	float* Af = new float[m*n];
	float* Uf = new float[m*sdim];
	float* Sf = new float[sdim*sdim];
	float* Vf = new float[sdim*n];

	for(int i = 0;i < m*n;i++)
		Af[i] = rand()%100;

	if(!ZQ_MathBase::SVD_Decompose(Af,m,n,Uf,Sf,Vf))
	{
		printf("svd fail\n");
	}
	else
		printf("svd success\n");
	delete []Af;
	delete []Uf;
	delete []Sf;
	delete []Vf;
	
}

int main()
{
	test_length2();
	test_product();
	test_binarysearch();
	test_vec();
	test_rem();
	test_find_minmax();
	test_mat();
	test_svd();
	
	return 0;

}
