#include "ZQ_StructureFromTexture.h"
#include "ZQ_StructureFromTextureOptions.h"
#include "ZQ_ImageIO.h"
#include <time.h>

using namespace ZQ;


template<class T>
void test(const int argc, const char** argv)
{
	if(argc < 3)
		return;
	const char* inputfile = argv[1];
	const char* outputfile = argv[2];

	ZQ_DImage<T> input,output;

	if(!ZQ_ImageIO::loadImage(input,inputfile,1))
	{
		printf("failed to load %s\n",inputfile);
		return;
	}


	ZQ_StructureFromTextureOptions opt;
	if(!opt.HandleParas(argc-3,argv+3))
	{
		printf("failed to handle args\n");
		opt.showArgs();
		return;
	}

	clock_t t1 = clock();
	if(!ZQ_StructureFromTexture::StructureFromTexture(input,output,opt))
	{
		printf("failed to extract structure from texture\n");
		return;
	}
	clock_t t2 = clock();
	printf("time:%f\n",0.001*(t2-t1));

	if(!ZQ_ImageIO::saveImage(output,outputfile))
	{
		printf("failed to save %s\n",outputfile);
		return;
	}
}

void main(const int argc, const char** argv)
{
	/*Sample args, copy the following to a .bat file
	SampleStructureFromTexture input1.jpg  output1_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input2.jpg  output2_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input3.jpg  output3_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input4.jpg  output4_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input5.jpg  output5_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input6.jpg  output6_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input7.jpg  output7_wei_mix40.jpg  weight 0.01 sigma 3 fsize 5 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input8.jpg  output8_wei_mix40.jpg  weight 0.01 sigma 3 fsize 5 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input9.jpg  output9_wei_mix40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input10.jpg output10_wei_mix40.jpg weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV_MIX nSolverIteration 10 nOuterIteration 40

	SampleStructureFromTexture input1.jpg  output1_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input2.jpg  output2_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input3.jpg  output3_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input4.jpg  output4_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input5.jpg  output5_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input6.jpg  output6_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input7.jpg  output7_wei_rtv40.jpg  weight 0.01 sigma 3 fsize 5 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input8.jpg  output8_wei_rtv40.jpg  weight 0.01 sigma 3 fsize 5 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input9.jpg  output9_wei_rtv40.jpg  weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40
	SampleStructureFromTexture input10.jpg output10_wei_rtv40.jpg weight 0.01 methodType PENALTY_GRADIENT_WEIGHT penaltyweighttype WEIGHT_RTV     nSolverIteration 10 nOuterIteration 40

	*/


	test<double>(argc,argv);
}