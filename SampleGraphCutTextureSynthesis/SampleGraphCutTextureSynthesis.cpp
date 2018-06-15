#include "ZQ_GraphCutForTexture.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include <stdio.h>
#include <stdlib.h>

using namespace ZQ;

int main(int argc, const char** argv)
{
	if (argc != 6)
	{
		printf(".exe infile outfile out_width out_height rand_seed\n");
		return -1;
	}
	const char* infile = argv[1];
	const char* outfile = argv[2];
	const int width = atoi(argv[3]);
	const int height = atoi(argv[4]);
	const int rand_seed = atoi(argv[5]);

	srand(rand_seed);
	ZQ_DImage<float> in_img, out_img;
	
	if (!ZQ_ImageIO::loadImage(in_img, infile, 1))
	{
		printf("failed to load %s\n", infile);
		return -1;
	}

	int in_width = in_img.width();
	int in_height = in_img.height();
	int in_nChannels = in_img.nchannels();
	out_img.allocate(width, height, in_nChannels);
	ZQ_GraphCutForTexture<float> sampler;
	if (!sampler.SetInputTexture(in_img.data(), in_width, in_height, in_nChannels))
	{
		printf("failed to set input!\n");
		return -1;
	}
	if (!sampler.RandomSynthesis(width, height, 64))
	{
		printf("failed to random synthesis!\n");
		return -1;
	}
	printf("random synthesis done!\n");
	if (!sampler.Optimize(1000))
	{
		printf("failed to optimize!\n");
		return -1;
	}
	printf("optimize done!\n");
	if (!sampler.ExportOutputTexture(out_img.data()))
	{
		printf("failed to export!\n");
		return -1;
	}

	if (!ZQ_ImageIO::saveImage(out_img, outfile))
	{
		printf("failed to save %s\n", outfile);
		return -1;
	}
	printf("done! %s saved!\n",outfile);
	return 0;
}