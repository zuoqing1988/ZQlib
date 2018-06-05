#define _USE_UMFPACK 1

#include "ZQ_ClosedFormImageMattingOptions.h"
#include "ZQ_ClosedFormImageMatting.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;

typedef double BaseType;

int main(int argc, const char** argv)
{
	if (argc < 6)
	{
		printf(".exe image tri_map out_alpha out_fore out_back [opts]\n");
		return -1;
	}
	ZQ_ClosedFormImageMattingOptions opt;
	if (!opt.HandleArgs(argc - 6, argv + 6))
	{
		return -1;
	}
	const char* imagefile = argv[1]; 
	const char* trimapfile = argv[2];
	const char* alphafile = argv[3];
	const char* forefile = argv[4];
	const char* backfile = argv[5];
	ZQ_DImage<BaseType> im1, consts_vals;
	if (!ZQ_ImageIO::loadImage(im1, imagefile, 1))
	{
		printf("failed to load %s\n", imagefile);
		return -1;
	}

	if (!ZQ_ImageIO::loadImage(consts_vals, trimapfile, 0))
	{
		printf("failed to load %s\n", trimapfile);
		return -1;
	}

	int width = im1.width();
	int height = im1.height();
	
	if (!consts_vals.matchDimension(width, height, 1))
	{
		printf("dimension does not match!\n");
		return -1;
	}

	ZQ_DImage<bool> consts_map(width, height);
	bool*& consts_map_data = consts_map.data();
	BaseType*& consts_vals_data = consts_vals.data();
	for (int i = 0; i < width*height; i++)
	{
		consts_map_data[i] = consts_vals_data[i] == 1 || consts_vals_data[i] == 0;
	}

	ZQ_DImage<BaseType> alpha(width,height);

	if (!ZQ_ClosedFormImageMatting::Coarse2FineSolveAlpha(im1, consts_map, consts_vals, alpha, opt.max_level_for_c2f, opt.consts_thresh_for_c2f, opt.epsilon,opt.win_size,opt.display))
	{
		printf("failed to Solve Alpha!\n");
		return -1;
	}
	
	alpha.imclamp(0, 1);
	if (!ZQ_ImageIO::saveImage(alpha, alphafile))
	{
		printf("failed to save %s\n", alphafile);
		return -1;
	}

	ZQ_DImage<BaseType> fore, back;
	
	if (opt.solve_fb_mode == ZQ_ClosedFormImageMattingOptions::SOLVE_FORE_BACK_4DIR)
	{
		if (!ZQ_ClosedFormImageMatting::SolveForeBack_4dir(im1, alpha, fore, back, opt.max_iter_for_solve_fb, opt.display))
		{
			printf("failed to Solve Foreground and Background!\n");
			return -1;
		}
	}
	else if (opt.solve_fb_mode == ZQ_ClosedFormImageMattingOptions::SOLVE_FORE_BACK_2DIR)
	{
		if (!ZQ_ClosedFormImageMatting::SolveForeBack_2dir(im1, alpha, fore, back, opt.max_iter_for_solve_fb, opt.display))
		{
			printf("failed to Solve Foreground and Background!\n");
			return -1;
		}
	}
	else
	{
		if (!ZQ_ClosedFormImageMatting::SolveForeBack_ori_paper(im1, alpha, fore, back, opt.max_iter_for_solve_fb, opt.display))
		{
			printf("failed to Solve Foreground and Background!\n");
			return -1;
		}
	}
	
	
	ZQ_DImage<BaseType> alpha2,alpha3;
	alpha2.assemble(alpha, alpha);
	alpha3.assemble(alpha, alpha2);
	fore.Multiplywith(alpha3);
	ZQ_DImage<BaseType> alpha3_1(alpha3);
	alpha3_1.Multiplywith(-1);
	alpha3_1.Addwith(1);
	back.Multiplywith(alpha3_1);
	if (!ZQ_ImageIO::saveImage(fore, forefile))
	{
		printf("failed to save %s\n", forefile);
		return -1;
	}
	if (!ZQ_ImageIO::saveImage(back, backfile))
	{
		printf("failed to save %s\n", backfile);
		return -1;
	}
	return 0;
}