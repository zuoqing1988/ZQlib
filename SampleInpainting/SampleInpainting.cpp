
#include "ZQ_ImageIO.h"
#include "ZQ_Inpainting.h"

using namespace ZQ;
typedef double BaseType;
typedef ZQ_DImage<BaseType> DImage;


void main(const int argc, const char** argv)
{
	//const char* infile = "input5.png";
	//const char* maskfile = "mask5.png";
	//const char* outfile = "output5.png";

	if(argc < 4)
	{
		printf(".exe infile maskfile outfile [opt]\n");
		return ;
	}
	const char* infile = argv[1];
	const char* maskfile = argv[2];
	const char* outfile = argv[3];

	DImage img,mask,outimg;

	if(!ZQ_ImageIO::loadImage(img,infile,1))
	{
		printf("failed to load %s\n",infile);
		return;
	}

	if(!ZQ_ImageIO::loadImage(mask,maskfile,0))
	{
		printf("failed to load %s\n",maskfile);
		return;
	}

	ZQ_InpaintingOptions opt;
	if(!opt.HandleParas(argc-4,argv+4))
	{
		opt.showArgs();
		return;
	}

	if(!ZQ_Inpainting::Inpainting(img,mask,outimg,opt))
	{
		printf("failed to inpainting\n");
		return;
	}


	if(!ZQ_ImageIO::saveImage(outimg,outfile))
	{
		printf("failed to save %s\n",outfile);
		return;
	}	

	cv::Mat cv_src = cv::imread(infile);
	cv::Mat cv_mask = cv::imread(maskfile, 0);
	cv::Mat cv_dst;
	cv::inpaint(cv_src, cv_mask, cv_dst, 3, CV_INPAINT_TELEA);
	cv::imwrite("outcv.png", cv_dst);

}
