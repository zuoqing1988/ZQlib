#include "ZQ_ICP.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace ZQ;

template<class T>
void RotateX(T angle, T R[9])
{
	T tmpR[] =
	{
		1,0,0,
		0,cos(angle),-sin(angle),
		0,sin(angle),cos(angle)
	};
	memcpy(R, tmpR, sizeof(T) * 9);
}

template<class T>
void RotateY(T angle, T R[9])
{
	T tmpR[] =
	{
		cos(angle),0,-sin(angle),
		0,1,0,
		sin(angle),0,cos(angle)
	};
	memcpy(R, tmpR, sizeof(T) * 9);
}

template<class T>
void RotateZ(T angle, T R[9])
{
	T tmpR[] =
	{
		cos(angle),-sin(angle),0,
		sin(angle),cos(angle),0,
		0,0,1
	};
	memcpy(R, tmpR, sizeof(T) * 9);
}

template<class T>
void Rotate(T an_x, T an_y, T an_z, T R[9])
{
	T R_x[9], R_y[9], R_z[9], R_xy[9];
	RotateX(an_x, R_x);
	RotateY(an_y, R_y);
	RotateZ(an_z, R_z);
	ZQ_MathBase::MatrixMul(R_x, R_y, 3, 3, 3, R_xy);
	ZQ_MathBase::MatrixMul(R_xy, R_z, 3, 3, 3, R);
}

double Noise(double scale)
{
	return rand() % 10001 / 10000.0*scale;
}

template<class T>
bool RandomSelect(int dim, const T* full_data, int nFull, T* data, int nData)
{
	if (full_data == 0 || data == 0 || nFull <= 0 || nData <= 0)
		return false;
	int* idx = new int[nFull];
	for (int i = 0; i < nFull; i++)
		idx[i] = i;
	for (int i = 0; i < nData; i++)
	{
		int cur_i = rand() % (nFull - i) + i;
		if (cur_i != i)
		{
			int tmp = idx[i];
			idx[i] = idx[cur_i];
			idx[cur_i] = tmp;
		}
	}
	for (int i = 0; i < nData; i++)
	{
		memcpy(data + i*dim, full_data + idx[i] * dim, sizeof(T)*dim);
	}
	delete[]idx;
	return true;
}

template<class T>
void test()
{
	const double pi = 4 * atan(1.0);
	T an_x = 0.2*pi, an_y = 0.1*pi, an_z = 0.04*pi;
	T t[3] = { 0,0,10 };
	T R[9];
	Rotate(an_x, an_y, an_z, R);

	int N = 8000;
	int M = 2000;
	double scale = 10;
	double noise_scale = 1;
	T* full_data = new T[N * 3];
	T* model = new T[N * 3];
	for (int i = 0; i < N; i++)
	{
		full_data[i * 3 + 0] = Noise(scale) + scale * 2;
		full_data[i * 3 + 1] = Noise(scale) + scale * 2;
		full_data[i * 3 + 2] = Noise(scale) + scale * 2;
		ZQ_MathBase::MatrixMul(R, full_data + i * 3, 3, 3, 1, model + i * 3);
		model[i * 3 + 0] += t[0] + Noise(noise_scale);
		model[i * 3 + 1] += t[1] + Noise(noise_scale);
		model[i * 3 + 2] += t[2] + Noise(noise_scale);
	}

	T* data = new T[M * 3];
	RandomSelect<T>(3, full_data, N, data, M);

	int maxIter = 200;
	double thresh = 1e-5;
	T RR[9], tt[9];

	/*FILE* out1 = fopen("model.txt","w");
	FILE* out2 = fopen("data.txt","w");
	for(int i = 0;i < 3;i++)
	{
	for(int j = 0;j < N;j++)
	fprintf(out1,"%12.7f",model[i*N+j]);
	fprintf(out1,"\n");
	}
	for(int i = 0;i < 3;i++)
	{
	for(int j = 0;j < M;j++)
	fprintf(out2,"%12.7f",data[i*M+j]);
	fprintf(out2,"\n");
	}
	fclose(out1);
	fclose(out2);*/

	ZQ_ICP::IterativeClosestPoint<T>(model, N, data, M, maxIter, thresh, RR, tt);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			printf("%12.5f", R[i * 3 + j]);
		}
		printf("%12.5f\n", t[i]);
	}
	printf("\n");
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			printf("%12.5f", RR[i * 3 + j]);
		}
		printf("%12.5f\n", tt[i]);
	}
	printf("\n\n");

	delete[]full_data;
	delete[]data;
	delete[]model;
}

int main()
{
	test<float>();
	test<double>();
}

