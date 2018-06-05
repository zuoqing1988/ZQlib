#include "ZQ_BilateralTextureFilter.h"
#include <time.h>
#include "ZQ_ImageIO.h"
#define ZQ_LINK_OPENCV_VERSION_2413
#include "ZQ_Link_OpenCV_Lib.h"


using namespace ZQ;
typedef float BaseType;
typedef ZQ_DImage<BaseType> DImage;

int main()
{
	const char* inputfile = "input2.png";
	const char* outputfile = "output2.png";



	DImage input, output;

	if (!ZQ_ImageIO::loadImage(input, inputfile, 1))
	{
		printf("failed to load %s\n", inputfile);
		return EXIT_FAILURE;
	}

	clock_t t1 = clock();
	ZQ_BilateralTextureFilter::BilateralTextureFilter(input, output, 3, 3);
	clock_t t2 = clock();
	printf("time:%f\n", 0.001*(t2 - t1));

	if (!ZQ_ImageIO::saveImage(output, outputfile))
	{
		printf("failed to save %s\n", outputfile);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int main2(/*const int argc, const char** argv*/)
{
	/*if(argc < 3)
	return;
	const char* inputfile = argv[1];
	const char* outputfile = argv[2];*/
	const char* inputfile = "input2.png";
	const char* outputfile = "output2.png";
	const char* opt_args[] = {
		"half_patch_size", "1",
		"sigma_alpha", "15",
		"fsize", "2",
		"sigma_s", "2",
		"sigma_v", "0.05"
	};


	DImage input, output;

	if (!ZQ_ImageIO::loadImage(input, inputfile, 1))
	{
		printf("failed to load %s\n", inputfile);
		return EXIT_FAILURE;
	}



	ZQ_BilateralTextureFilterOptions opt;
	//if(!opt.HandleParas(argc-3,argv+3))
	if (!opt.HandleParas(sizeof(opt_args) / sizeof(const char*), opt_args))
	{
		printf("failed to handle args\n");
		opt.showArgs();
		return EXIT_FAILURE;
	}

	clock_t t1 = clock();
	DImage tmp = input;
	int nOuterIter = 3;
	for (int it = 0; it < nOuterIter; it++)
	{
		if (!ZQ_BilateralTextureFilter::BilateralTextureFilter(tmp, output, opt))
		{
			printf("failed to bilateral filtering\n");
			return EXIT_FAILURE;
		}
		tmp = output;
	}
	
	clock_t t2 = clock();
	printf("time:%f\n", 0.001*(t2 - t1));

	if (!ZQ_ImageIO::saveImage(output, outputfile))
	{
		printf("failed to save %s\n", outputfile);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}