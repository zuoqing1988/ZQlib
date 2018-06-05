#include "ZQ_WeightedMedian.h"
#include <stdio.h>
#include <stdlib.h>

using namespace ZQ;

void main()
{
	int num = 100;
	float* vals = new float[num];
	float* weights = new float[num];
	for (int i = 0; i < num; i++)
	{
		vals[i] = rand() % 1001 / 1000.0;
		weights[i] = rand() % 1001 / 1000.0 + 0.1;
	}
	for (int i = 0; i < num; i++)
	{
		printf("%12.5f %12.5f\n", vals[i], weights[i]);
	}

	float out_v;
	ZQ_WeightedMedian::FindMedian(vals, weights, num, out_v);

	printf("%f\n", out_v);

	
}