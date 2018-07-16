#define _USE_UMFPACK 1
#include "ZQ_ImageMattingGUI.h"
using namespace ZQ;

typedef double BaseType;
int main(int argc, const char** argv)
{
	if (argc != 5)
	{
		printf("%s in_image_file out_fore_file out_back_file out_alpha_file\n", argv[0]);
		return EXIT_FAILURE;
	}

	const char* in_image_file = argv[1];
	const char* out_fore_file = argv[2];
	const char* out_back_file = argv[3];
	const char* out_alpha_file = argv[4];
	ZQ_DImage<BaseType> ori_img, fore, back, alpha;
	if (!ZQ_ImageIO::loadImage(ori_img, in_image_file, 1))
	{
		printf("failed to load image %s\n", in_image_file);
		return EXIT_FAILURE;
	}
	
	if (!ZQ_ImageMattingGUI<BaseType>::GetInstance()->Run(ori_img, fore, back, alpha))
	{
		printf("failed to run image matting\n");
		return EXIT_FAILURE;
	}

	int width = fore.width();
	int height = fore.height();
	int nChannels = fore.nchannels();
	ZQ_DImage<BaseType> fore_mul_alpha(width, height, 3);
	ZQ_DImage<BaseType> back_mul_1_alpha(width, height, 3);
	BaseType*& fore_mul_alpha_data = fore_mul_alpha.data();
	BaseType*& back_mul_1_alpha_data = back_mul_1_alpha.data();
	BaseType*& fore_data = fore.data();
	BaseType*& back_data = back.data();
	BaseType*& alpha_data = alpha.data();
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			int offset = h*width + w;
			float cur_alpha = alpha_data[offset];
			float cur_1_alpha = 1 - cur_alpha;
			fore_mul_alpha_data[offset * 3 + 0] = fore_data[offset * 3 + 0] * cur_alpha;
			fore_mul_alpha_data[offset * 3 + 1] = fore_data[offset * 3 + 1] * cur_alpha;
			fore_mul_alpha_data[offset * 3 + 2] = fore_data[offset * 3 + 2] * cur_alpha;
			back_mul_1_alpha_data[offset * 3 + 0] = back_data[offset * 3 + 0] * cur_1_alpha;
			back_mul_1_alpha_data[offset * 3 + 1] = back_data[offset * 3 + 1] * cur_1_alpha;
			back_mul_1_alpha_data[offset * 3 + 2] = back_data[offset * 3 + 2] * cur_1_alpha;
		}
	}

	ZQ_ImageIO::saveImage(alpha, out_alpha_file);
	ZQ_ImageIO::saveImage(fore_mul_alpha, out_fore_file);
	ZQ_ImageIO::saveImage(back_mul_1_alpha, out_back_file);
	return EXIT_SUCCESS;
}