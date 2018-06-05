#include "ZQ_Kmeans.h"
#include <stdio.h>

using namespace ZQ;

template<class T>
void test()
{
	/*const char* infile = "input.txt";
	FILE* in = fopen(infile , "r");
	if (in == 0)
	{
		printf("failed to open file %s\n", infile);
		return;
	}
	int num = 0, dim = 0;
	fscanf(in, "%d%d", &num, &dim);
	T* pts = new T[num*dim];
	for (int i = 0; i < num; i++)
	{
		for (int j = 0; j < dim; j++)
		{
			float val;
			fscanf(in, "%f", &val);
			pts[i*dim + j] = val;
		}
	}
	fclose(in);*/

	int num = 100, dim = 3;
	T* pts = new T[num*dim];
	for (int i = 0; i < num*dim; i++)
	{
		pts[i] = rand() % 101 / 50.0 - 1;
	}

	int* idx = new int[num];
	int k = 50;
	T* out_centers = new T[k*dim];
	memset(out_centers, 0, sizeof(T)*k*dim);
	if (!ZQ_Kmeans<T>::Kmeans(num, dim, k, pts, idx, out_centers))
	{
		printf("failed to run Kmeans\n");
		delete[]out_centers;
		delete[]idx;
		delete[]pts;
		return ;
	}
	int* sum_kid = new int[k];
	memset(sum_kid, 0, sizeof(int)*k);
	for (int i = 0; i < num; i++)
		sum_kid[idx[i]]++;

	for (int i = 0; i < k; i++)
	{
		printf("%d ", sum_kid[i]);
		for (int j = 0; j < dim; j++)
		{
			printf("%f ", out_centers[i*dim + j]);
		}
		printf("\n");
	}
	printf("\n=====================================\n");
	for (int i = 0; i < num; i++)
	{
		printf("%d ", idx[i]);
		for (int j = 0; j < dim; j++)
		{
			printf("%f ", pts[i*dim + j]);
		}
		printf("\n");
	}
	delete[]out_centers;
	delete[]idx;
	delete[]pts;
}
int main()
{
	test<double>();
	return 0;
}