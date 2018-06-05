#include "ZQ_MergeImage.h"

using namespace ZQ;

void main(const int argc, const char** argv)
{
	ZQ_MergeImageOptions opt;
	if(!opt.HandleParas(argc-1,argv+1))
	{
		printf("failed to handle args\n");
		return ;
	}

	if(!ZQ_MergeImage::Go(opt))
	{
		"go failed\n";
		return ;
	}
}
