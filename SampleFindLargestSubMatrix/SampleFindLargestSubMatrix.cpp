#include "ZQ_FindLargestRectInHistgram.h"
#include "ZQ_FindLargestSubMatrix.h"

using namespace ZQ;

int main()
{
	int in_w = 5;
	int in_h = 5;
	bool flag[25] = 
	{
		1,0,0,1,0,
		0,1,1,1,1,
		0,1,1,0,1,
		1,1,1,1,1,
		0,1,0,0,1
	};
	int off_x, off_y, w, h;
	ZQ_FindLargestSubMatrix::FindLargestSubMatrix(flag, in_w, in_h, off_x, off_y, w, h);
	printf("%d,%d,%d,%d\n", off_x, off_y, w, h);
	return 0;
}

int main2()
{
	int len = 7;
	unsigned int hist[7] = { 2, 1, 4, 5, 1, 3, 3 };
	int off, w, h;
	long long area;
	ZQ_FindLargestRectInHistgram::FindLargestRectInHistgram(len, hist, off, w, h, area);
	
	printf("%d,%d,%ld\n", off, w, area);
	return 0;
}