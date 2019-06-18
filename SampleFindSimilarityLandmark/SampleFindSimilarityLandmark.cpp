#include "ZQ_FindSimilarityLandmark.h"
#include <stdio.h>
using namespace ZQ;

template<class T>
int test()
{
	T coord5point[10] =
	{
		30.2946, 51.6963,
		65.5318, 51.5014,
		48.0252, 71.7366,
		33.5493, 92.3655,
		62.7299, 92.2041
	};

	T found_coord[10] =
	{
		20, 20,
		50, 20,
		30, 30,
		23, 40,
		48, 42
	};

	T transform[6];
	ZQ_FindSimilarityLandmark::FindSimilarityLandmark(5, found_coord, coord5point, transform);
	printf("transform = \n");
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			printf("%12.5f", transform[i * 3 + j]);
		}
		printf("\n");
	}
	return 0;
}

int main()
{
	test<float>();
	test<double>();
	return 0;
}