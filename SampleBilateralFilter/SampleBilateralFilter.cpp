#include "ZQ_BilateralFilter.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;
typedef float BaseType;
typedef ZQ_DImage<BaseType> DImage;

int main(/*const int argc, const char** argv*/)
{
	/*if(argc < 3)
		return EXIT_FAILURE;
	const char* inputfile = argv[1];
	const char* outputfile = argv[2];*/
	const char* inputfile = "input2.jpg";
	const char* outputfile = "output2.jpg";
	const char* opt_args[] = {
		"fsize","5",
		"sigma_s","2",
		"sigma_v","0.3"
	};

	
	DImage input,output;

	if(!ZQ_ImageIO::loadImage(input,inputfile,1))
	{
		printf("failed to load %s\n",inputfile);
		return EXIT_FAILURE;
	}



	ZQ_BilateralFilterOptions opt;
	//if(!opt.HandleParas(argc-3,argv+3))
	if(!opt.HandleParas(sizeof(opt_args)/sizeof(const char*),opt_args))
	{
		printf("failed to handle args\n");
		opt.showArgs();
		return EXIT_FAILURE;
	}

	clock_t t1 = clock();
	if(!ZQ_BilateralFilter::BilateralFilter(input,output,opt))
	{
		printf("failed to bilateral filtering\n");
		return EXIT_FAILURE;
	}
	clock_t t2 = clock();
	printf("time:%f\n",0.001*(t2-t1));

	if(!ZQ_ImageIO::saveImage(output,outputfile))
	{
		printf("failed to save %s\n",outputfile);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}