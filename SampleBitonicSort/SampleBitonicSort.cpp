#include "ZQ_BitonicSort.h"
#include "ZQ_MergeSort.h"
#include "ZQ_QuickSort.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
using namespace ZQ;

class mType 
{
public:
	int idx;
	float val;
	bool operator>(const mType& other) const
	{
		return val > other.val;
	}
	bool operator>=(const mType& other) const
	{
		return val >= other.val;
	}
	bool operator<(const mType& other) const
	{
		return val < other.val;
	}
	bool operator<=(const mType& other) const
	{
		return val <= other.val;
	}
};

template<class mType>
bool check(const mType* vals, int n, bool ascenddir)
{
	for (int i = 0; i < n - 1; i++)
	{
		if (ascenddir)
		{
			if (vals[i] > vals[i + 1])
				return false;
		}
		else
		{
			if (vals[i] < vals[i + 1])
				return false;
		}
	}
	return true;
}


int main()
{
	srand(time(0));
	static const int n = 64;
	mType a[n],b[n],c[n],d[n];
	float af[n],bf[n],cf[n],df[n];
	int ai[n],bi[n],ci[n],di[n];
	for(int i = 0;i < n;i++)
	{
		a[i].idx = i;
		a[i].val = rand()%10/7.0;
		b[i].idx = i;
		b[i].val = a[i].val;
		c[i].idx = i;
		c[i].val = a[i].val;
		d[i].idx = i;
		d[i].val = a[i].val;
		af[i] = a[i].val;
		bf[i] = a[i].val;
		cf[i] = a[i].val;
		df[i] = a[i].val;
		ai[i] = i;
		bi[i] = i;
		ci[i] = i;
		di[i] = i;
	}

	bool ascenddir = true;
	ZQ_BitonicSort::Sort(a, n, ascenddir);
	ZQ_BitonicSort::Sort_Recursive(b, n, 0, ascenddir);
	ZQ_MergeSort::MergeSort(c, n, ascenddir);
	ZQ_QuickSort::QuickSort(d, n, ascenddir);

	ZQ_BitonicSort::Sort(af, ai, n, ascenddir);
	ZQ_BitonicSort::Sort_Recursive(bf, bi, n, 0, ascenddir);
	ZQ_MergeSort::MergeSort(cf, ci, n, ascenddir);
	ZQ_QuickSort::QuickSort(df, di, n, ascenddir);

	printf("%s\n", check(a, n, ascenddir) ? "true":"false");
	printf("%s\n", check(b, n, ascenddir) ? "true":"false");
	printf("%s\n", check(c, n, ascenddir) ? "true" : "false");
	printf("%s\n", check(d, n, ascenddir) ? "true" : "false");
	printf("%s\n", check(af, n, ascenddir) ? "true" : "false");
	printf("%s\n", check(bf, n, ascenddir) ? "true" : "false");
	printf("%s\n", check(cf, n, ascenddir) ? "true" : "false");
	printf("%s\n", check(df, n, ascenddir) ? "true" : "false");

	float out;
	int k = 3;
	printf("%dth max:%f\n", k, ascenddir ? df[n - 1 - k] : df[k]);
	ZQ_QuickSort::FindKthMax(df, n, k, out);
	printf("found : %f\n", out);
	
	/*for(int i = 0;i < n;i++)
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

	for (int i = 0; i < n; i++)
		printf("%7.2f ", d[i].val);
	printf("\n");
	for (int i = 0; i < n; i++)
		printf("%7d ", d[i].idx);
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

	for (int i = 0; i < n; i++)
		printf("%7.2f ", df[i]);
	printf("\n");
	for (int i = 0; i < n; i++)
		printf("%7d ", di[i]);
	printf("\n");*/
	
	return 0;
}