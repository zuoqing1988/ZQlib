#include "ZQ_Kahansum.h"
#include <stdlib.h>
#include <stdio.h>

template<class T>
T sum(const T* input, const int n)
{
	T result = 0;
	for(int i = 0;i < n;i++)
		result += input[i];
	return result;
}

typedef float BaseType ;
void main()
{
	int num = 1e8;
	BaseType * input = new BaseType[num];
	for(int i = 0;i < num;i++)
		input[i] = 1;

	BaseType sum1 = ZQ::ZQ_KahanSum(input,num);
	BaseType sum2 = sum(input,num);

	printf("sum1 = %f\nsum2=%f\n",sum1,sum2);
}
