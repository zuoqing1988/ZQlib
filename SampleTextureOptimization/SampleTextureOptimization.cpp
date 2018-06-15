#include "ZQ_TextureSynthesisProbe.h"
#include "ZQ_TextureOptimization.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include <time.h>

using namespace ZQ;
typedef ZQ_DImage<float> DImage;

int main()
{
	const char* input_file = "input.jpg";
	const char* ctrl_file = "ctrl.jpg";
	const char* output_file1 = "to_no_ctrl.jpg";
	const char* output_file = "to_ctrl.jpg";
	DImage input,ctrl;
	if(!ZQ_ImageIO::loadImage(input,input_file,1))
	{
		printf("failed to load image %s\n",input_file);
		return EXIT_FAILURE;
	}

	if(!ZQ_ImageIO::loadImage(ctrl,ctrl_file,1))
	{
		printf("failed to load image %s\n",ctrl_file);
		return EXIT_FAILURE;
	}

	int nChannels = input.nchannels();
	DImage output1(256,256,nChannels),output;

	ZQ_TextureOptimization to;
	to.SetSource(input);

	int borderWidth = 16;
	float reduce_factor = 0.5;
	int max_level = 6;

	float grad_weight = 0.07;
	float ctrl_weight = 0.03;
	int to_max_iter = 5;
	int sor_iter = 20;
	clock_t t1 = clock();
	to.SynthesisWithoutControl(borderWidth,reduce_factor,max_level,grad_weight,to_max_iter,sor_iter,output1);
	clock_t t2 = clock();
	to.SynthesisWithControl(ctrl,borderWidth,reduce_factor,max_level,grad_weight,ctrl_weight,to_max_iter,sor_iter,output);
	clock_t t3 = clock();

	printf("%f %f\n",0.001*(t2-t1),0.001*(t3-t2));

	ZQ_ImageIO::saveImage(output1,output_file1);
	ZQ_ImageIO::saveImage(output,output_file);
	return EXIT_SUCCESS;
}