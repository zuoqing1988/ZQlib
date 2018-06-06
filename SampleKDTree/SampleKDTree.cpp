#include "ZQ_KDTree.h"
#include <omp.h>
using namespace ZQ;
int main()
{
	int npts = 5000000;
	int ndim = 3;
	int range = 100;
	float* data = new float[npts*ndim];
	for(int i = 0;i < npts*ndim;i++)
		data[i] = rand()%range;

	ZQ_KDTree<float> kd_tree;
	double t1 = omp_get_wtime();
	kd_tree.BuildKDTree(data,npts,ndim,10);
	double t2 = omp_get_wtime();
	printf("build kdtree cost: %.3f seconds\n", t2 - t1);
	//if(!kd_tree.Check())
	//{
	//	printf("error\n");
	//}

	float* cur_pt = new float[ndim];
	for(int i = 0;i < ndim;i++)
		cur_pt[i] = rand()%range;

	int k = 50;
	int* out_idx = new int[k];
	int* out_idx_ann = new int[k];
	float* out_dis2 = new float[k];
	float* out_dis2_ann = new float[k];
	double t3 = omp_get_wtime();
	kd_tree.BruteForceSearch(cur_pt,k,out_idx,out_dis2);
	double t4 = omp_get_wtime();
	printf("bruteforce search cost: %.3f seconds\n", t4 - t3);

	//for(int i = 0;i < k;i++)
	//{
	//	printf("%12d:%12.5f\n",out_idx[i],out_dis2[i]);
	//}
	//printf("\n\n");
	double t5 = omp_get_wtime();
	kd_tree.AnnSearch(cur_pt,k,out_idx_ann,out_dis2_ann,0);
	double t6 = omp_get_wtime();
	printf("ann search cost: %.3f seconds\n", t6 - t5);

	float radius = range/10.0;
	int fr_k = 0;
	double t7 = omp_get_wtime();
	if(!kd_tree.AnnFixRadiusSearchCountReturnNum(cur_pt,radius,fr_k))
		printf("fr_cnt failed\n");
	printf("fr_k = %d\n",fr_k);
	double t8 = omp_get_wtime();
	printf("ann fixed radius search return num cost: %.3f seconds\n", t8 - t7);

	int* out_fr_idx = new int[fr_k];
	float* out_fr_dis2 = new float[fr_k];
	double t9 = omp_get_wtime();
	if(!kd_tree.AnnFixRadiusSearch(cur_pt,radius,fr_k,out_fr_idx,out_fr_dis2))
		printf("fr_search failed\n");
	double t10 = omp_get_wtime();
	printf("ann fixed radius search cost: %.3f seconds\n", t10 - t9);

	/*printf("radius2 = %f\n",radius*radius);
	for(int i = 0;i < fr_k;i++)
	{
		printf("%12d:%12.3f\n",out_fr_idx[i],out_fr_dis2[i]);
	}*/


	delete []out_idx;
	delete []out_idx_ann;
	delete []out_fr_idx;
	delete []out_dis2;
	delete []out_dis2_ann;
	delete []out_fr_dis2;
	delete []data;
	return 0;
}