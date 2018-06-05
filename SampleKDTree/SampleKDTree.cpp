#include "ZQ_KDTree.h"
#include <time.h>
using namespace ZQ;
int main()
{
	int npts = 50000;
	int ndim = 3;
	int range = 100;
	float* data = new float[npts*ndim];
	for(int i = 0;i < npts*ndim;i++)
		data[i] = rand()%range;

	ZQ_KDTree<float> kd_tree;
	clock_t t1 = clock();
	kd_tree.BuildKDTree(data,npts,ndim,10);

	clock_t t2 = clock();
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
	clock_t t3 = clock();


	kd_tree.BruteForceSearch(cur_pt,k,out_idx,out_dis2);

	clock_t t4 = clock();
	//for(int i = 0;i < k;i++)
	//{
	//	printf("%12d:%12.5f\n",out_idx[i],out_dis2[i]);
	//}
	//printf("\n\n");
	clock_t t5 = clock();

	kd_tree.AnnSearch(cur_pt,k,out_idx_ann,out_dis2_ann,0);

	float radius = range/10.0;
	int fr_k = 0;
	if(!kd_tree.AnnFixRadiusSearchCountReturnNum(cur_pt,radius,fr_k))
		printf("fr_cnt failed\n");
	printf("fr_k = %d\n",fr_k);

	int* out_fr_idx = new int[fr_k];
	float* out_fr_dis2 = new float[fr_k];
	if(!kd_tree.AnnFixRadiusSearch(cur_pt,radius,fr_k,out_fr_idx,out_fr_dis2))
		printf("fr_search failed\n");
	clock_t t6 = clock();

	printf("radius2 = %f\n",radius*radius);
	for(int i = 0;i < fr_k;i++)
	{
		printf("%12d:%12.3f\n",out_fr_idx[i],out_fr_dis2[i]);
	}






	printf("build: %f\n",0.001*(t2-t1));
	printf("brute: %f\n",0.001*(t4-t3));
	printf("ann  : %f\n",0.001*(t6-t5));
	delete []out_idx;
	delete []out_idx_ann;
	delete []out_fr_idx;
	delete []out_dis2;
	delete []out_dis2_ann;
	delete []out_fr_dis2;
	delete []data;
	return 0;
}