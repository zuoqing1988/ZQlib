#include "ZQ_BitonicSort.h"
#include "ZQ_MergeSort.h"
#include <stdio.h>
#include <stdlib.h>

using namespace ZQ;

class mType 
{
public:
	int idx;
	float val;
	bool operator>(const mType& other)
	{
		return val > other.val;
	}
};


int main()
{
	static const int n = 8;
	mType a[n],b[n],c[n];
	float af[n],bf[n],cf[n];
	int ai[n],bi[n],ci[n];
	for(int i = 0;i < n;i++)
	{
		a[i].idx = i;
		a[i].val = rand()%10/7.0;
		b[i].idx = i;
		b[i].val = a[i].val;
		c[i].idx = i;
		c[i].val = a[i].val;
		af[i] = a[i].val;
		bf[i] = a[i].val;
		cf[i] = a[i].val;
		ai[i] = i;
		bi[i] = i;
		ci[i] = i;
	}
	ZQ_BitonicSort::Sort(a,8,true);
	ZQ_BitonicSort::Sort_Recursive(b,8,0,true);
	ZQ_MergeSort::MergeSort(c,8,true);

	ZQ_BitonicSort::Sort(af,ai,8,true);
	ZQ_BitonicSort::Sort_Recursive(bf,bi,8,0,true);
	ZQ_MergeSort::MergeSort(cf,ci,8,true);
	
	for(int i = 0;i < n;i++)
		printf("%7.2f ",a[i].val);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",a[i].idx);
	printf("\n");

	for(int i = 0;i < n;i++)
		printf("%7.2f ",b[i].val);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",b[i].idx);
	printf("\n");

	for(int i = 0;i < n;i++)
		printf("%7.2f ",c[i].val);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",c[i].idx);
	printf("\n");

	for(int i = 0;i < n;i++)
		printf("%7.2f ",af[i]);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",ai[i]);
	printf("\n");

	for(int i = 0;i < n;i++)
		printf("%7.2f ",bf[i]);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",bi[i]);
	printf("\n");

	for(int i = 0;i < n;i++)
		printf("%7.2f ",cf[i]);
	printf("\n");
	for(int i = 0;i < n;i++)
		printf("%7d ",ci[i]);
	printf("\n");
	
	return 0;
}
