#include "ZQ_GMM.h"
#include <iostream>
#include <time.h>

using namespace std;
using namespace ZQ;

bool loaddata(const string& filename, double* pts, int nPts, int dim);
double noise(double scale);

int main()
{
	int nPts = 80;
	int dim = 2;
	int k = 4;
	double* pts = new double[nPts*dim];
	double* out_prob = new double[nPts*k];
	double* out_mu = new double[k*dim];
	double* out_sigma = new double[k*dim*dim];
	double* out_weight = new double[k * 1];
	if (!loaddata("testSet.txt", pts, nPts, dim))
	{
		cout << "failed to load data!\n";
		return EXIT_FAILURE;
	}

	double init_centers[8] = 
	{
		2.5367 + noise(1), 2.9367 + noise(1),
		2.7433 + noise(1), -2.906 + noise(1),
		-3.5393 + noise(1), -2.8933 + noise(1),
		-2.4603 + noise(1), 2.7875 + noise(1)
	};
	srand(time(0));
	//if (!ZQ_GMM<double>::GMM(nPts, dim, k, pts, out_prob, out_mu, out_sigma, out_weight))
	if (!ZQ_GMM<double>::GMM_with_init(nPts,dim,k,pts,init_centers,out_prob,out_mu,out_sigma,out_weight,1e-15))
	{
		cout << "failed to run GMM!\n";
		return EXIT_FAILURE;
	}
	for (int pp = 0; pp < nPts; pp++)
	{
		for (int kk = 0; kk < k; kk++)
			printf("%12.5f", out_prob[pp*k + kk]);
		printf("\n");
	}
	for (int i = 0; i < k; i++)
	{
		printf("%12.5f: %12.5f%12.5f\n", out_weight[i], out_mu[i * 2 + 0], out_mu[i * 2 + 1]);
	}

	for (int pp = 0; pp < nPts; pp++)
	{
		int id = 0;
		double max_val = out_prob[pp*k + 0];
		for (int kk = 1; kk < k; kk++)
		{
			if (max_val < out_prob[pp*k + kk])
			{
				max_val = out_prob[pp*k + kk];
				id = kk;
			}
		}
		cout << id+1 << " ";
		if (pp % 20 == 19)
			cout << "\n";

	}

	int* idx = new int[nPts];
	double* out_centers = new double[k*dim];
	ZQ_Kmeans<double>::Kmeans_with_init(nPts, dim, k, pts, init_centers, idx, out_centers);
	for (int pp = 0; pp < nPts; pp++)
	{
		cout << idx[pp] + 1 << " ";
		if (pp % 20 == 19)
			cout << "\n";

	}

	delete[]pts;
	delete[]out_mu;
	delete[]out_sigma;
	delete[]out_weight;
	delete[]out_prob;
	delete[]out_centers;
	delete[]idx;
	return EXIT_SUCCESS;
}

bool loaddata(const string& filename, double* pts, int nPts, int dim)
{
	FILE* in = NULL;
	fopen_s(&in, filename.c_str(), "rb");
	if (in == 0)
	{
		return false;
	}

	for (int i = 0; i < nPts*dim; i++)
		fscanf_s(in, "%lf", pts + i);

	fclose(in);
	return true;
}

double noise(double scale)
{
	return rand() % 2001 / 1000.0*scale;
}