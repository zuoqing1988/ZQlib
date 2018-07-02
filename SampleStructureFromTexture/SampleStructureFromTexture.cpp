#include "ZQ_StructureFromTexture.h"
#include "ZQ_StructureFromTextureOptions.h"
#include "ZQ_ImageIO.h"
#include <time.h>

using namespace ZQ;

template<class T>
int test(const int argc, const char** argv)
{
	if(argc < 3)
		return EXIT_FAILURE;
	const char* inputfile = argv[1];
	const char* outputfile = argv[2];

	ZQ_DImage<T> input,output;

	if(!ZQ_ImageIO::loadImage(input,inputfile,1))
	{
		printf("failed to load %s\n",inputfile);
		return EXIT_FAILURE;
	}


	ZQ_StructureFromTextureOptions opt;
	if(!opt.HandleParas(argc-3,argv+3))
	{
		printf("failed to handle args\n");
		opt.showArgs();
		return EXIT_FAILURE;
	}

	clock_t t1 = clock();
	if(!ZQ_StructureFromTexture::StructureFromTexture(input,output,opt))
	{
		printf("failed to extract structure from texture\n");
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

int main(const int argc, const char** argv)
{
	/*Sample args, copy the following to a .bat file
	SampleStructureFromTexture input2.jpg  output2_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input2.jpg  output2_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input2.jpg  output2_wei_wls40.jpg  weight 1.0 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_WLS   fsize 5 sigma 3  nSolverIteration 10 nOuterIteration 40
	*/
	
	return test<double>(argc,argv);
}