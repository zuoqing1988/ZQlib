#define ZQLIB_USE_OPENMP
#include "ZQ_OpticalFlow.h"
#include "ZQ_ImageIO.h"
#include <stdio.h>
#include <time.h>
#include <iostream>

using namespace ZQ;

typedef ZQ_DImage<float> DImage;

void main_Pair(const ZQ_OpticalFlowOptions& opt, const char* in_fold, const char* prefix, const char* suffix, const int image_num, const int base_id,const char* out_fold);

void main_Seq(const ZQ_OpticalFlowOptions& opt, const char* in_fold, const char* prefix, const char* suffix, const int image_num, const int base_id,const char* out_fold);



void main()
{
	const char* opt_argv[] = {
		"methodtype", "HS_L2",
		"nAltIter", "3",
		"nAdmmIter", "20",
		"nOuterIter", "10",
		"nInnerIter", "3",
		"nPoissonIter", "30",
		"nAdvectIter", "3",
		"nSORIter", "30",
		"initType","ADMM_AS_INIT",
		"alpha","0.06",
		"beta","0",
		"gamma","0.1",
		"lambda","0.01",
		"display",
		"cubic",
		"useOMP"
	};

	ZQ_OpticalFlowOptions opt;
	if(!opt.HandleParas(sizeof(opt_argv)/sizeof(char*),opt_argv))
	{
		printf("option args error\n");
		return;
	}

	const char* input_fold = "INPUT";
	const char* prefix = "par_";
	const char* suffix = "png";
	const int image_num = 4;
	const int base_id = 0;
	
	const char* HS_L2_fold = "HS_L2";
	const char* HS_DL1_fold = "HS_DL1";
	const char* HS_L1_fold = "HS_L1";
	const char* ADMM_L2_fold = "ADMM_L2";
	const char* ADMM_DL1_fold = "ADMM_DL1";
	const char* ONEDIR_INC_L2_fold = "ONEDIR_INC_L2";
	const char* ONEDIR_INC_DL1_fold = "ONEDIR_INC_DL1";
	const char* ONEDIR_DEC_L2_fold = "ONEDIR_DEC_L2";
	const char* ONEDIR_DEC_DL1_fold = "ONEDIR_DEC_DL1";
	const char* TWODIR_INC_L2_fold = "TWODIR_INC_L2";
	const char* TWODIR_INC_DL1_fold = "TWODIR_INC_DL1";
	const char* TWODIR_DEC_L2_fold = "TWODIR_DEC_L2";
	const char* TWODIR_DEC_DL1_fold = "TWODIR_DEC_DL1";

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_HS_L2;
	opt.alpha = 0.06;
	opt.beta = 0.003;
	std::cout << "METHOD_HS_L2: omp = " << opt.use_omp << "\n";
	main_Pair(opt,input_fold,prefix,suffix,image_num,base_id,HS_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_HS_DL1;
	opt.alpha = 0.2;
	opt.beta = 0.01;
	std::cout << "METHOD_HS_DL1: omp = " << opt.use_omp << "\n";
	main_Pair(opt,input_fold,prefix,suffix,image_num,base_id,HS_DL1_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_HS_L1;
	opt.alpha = 0.012;
	opt.nInnerFixedPointIterations = 2;
	opt.beta = 0.001;
	std::cout << "METHOD_HS_L1: omp = " << opt.use_omp << "\n";
	main_Pair(opt,input_fold,prefix,suffix,image_num,base_id,HS_L1_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ADMM_L2;
	opt.alpha = 0.06;
	opt.nOuterFixedPointIterations = 5;
	std::cout << "METHOD_ADMM_L2: omp = " << opt.use_omp << "\n";
	main_Pair(opt,input_fold,prefix,suffix,image_num,base_id,ADMM_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ADMM_DL1;
	opt.alpha = 0.2;
	opt.nOuterFixedPointIterations = 5;
	opt.nInnerFixedPointIterations = 2;
	std::cout << "METHOD_ADMM_DL1: omp = " << opt.use_omp << "\n";
	main_Pair(opt,input_fold,prefix,suffix,image_num,base_id,ADMM_DL1_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ONEDIR_INC_L2;
	opt.alpha = 0.06;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 3;
	std::cout << "METHOD_ONEDIR_INC_L2: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,ONEDIR_INC_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ONEDIR_INC_DL1;
	opt.alpha = 0.2;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 5;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_ONEDIR_INC_DL1: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,ONEDIR_INC_DL1_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ONEDIR_DEC_L2;
	opt.alpha = 0.06;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_ONEDIR_DEC_L2: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,ONEDIR_DEC_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_ONEDIR_DEC_DL1;
	opt.alpha = 0.2;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_ONEDIR_DEC_DL1: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,ONEDIR_DEC_DL1_fold);


	opt.methodType = ZQ_OpticalFlowOptions::METHOD_TWODIR_INC_L2;
	opt.alpha = 0.06;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_TWODIR_INC_L2: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,TWODIR_INC_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_TWODIR_INC_DL1;
	opt.alpha = 0.2;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_TWODIR_INC_DL1: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,TWODIR_INC_DL1_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_TWODIR_DEC_L2;
	opt.alpha = 0.06;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_TWODIR_DEC_L2: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,TWODIR_DEC_L2_fold);

	opt.methodType = ZQ_OpticalFlowOptions::METHOD_TWODIR_DEC_DL1;
	opt.alpha = 0.2;
	opt.nAlterations = 3;
	opt.nOuterFixedPointIterations = 3;
	opt.nInnerFixedPointIterations = 1;
	std::cout << "METHOD_TWODIR_DEC_DL1: omp = " << opt.use_omp << "\n";
	main_Seq(opt,input_fold,prefix,suffix,image_num,base_id,TWODIR_DEC_DL1_fold);
}


void main_Pair(const ZQ_OpticalFlowOptions& opt, const char* in_fold, const char* prefix, const char* suffix, const int image_num, const int base_id,const char* out_fold)
{
	if(image_num < 2)
	{
		printf("error: image num < 2\n");
		return ;
	}

	char buf[2000];

	std::vector<DImage> Images;
	for(int i = 0;i < image_num;i++)
	{
		DImage Im;
		sprintf_s(buf,"%s\\%s%d.%s",in_fold,prefix,i+base_id,suffix);
		bool flag = ZQ_ImageIO::loadImage(Im,buf,0);
		if(!flag)
		{
			printf("fail to load image %s\n",buf);
			return ;
		}
		Images.push_back(Im);
	}


	double max_rad = 12;
	bool user_input = true;

	std::vector<DImage> u(image_num-1),v(image_num-1),warpIm(image_num-1);

	ZQ_OpticalFlowOptions opt1 = opt;
	opt1.use_omp = false;
	double start = omp_get_wtime();
#pragma omp parallel for 
	for(int i = 0;i < image_num-1;i++)
	{
		switch(opt.methodType)
		{
		case ZQ_OpticalFlowOptions::METHOD_HS_L2:
			ZQ_OpticalFlow::Coarse2Fine_HS_L2(u[i],v[i],warpIm[i],Images[i],Images[i+1],opt1);
			break;
		case ZQ_OpticalFlowOptions::METHOD_HS_DL1:
			ZQ_OpticalFlow::Coarse2Fine_HS_DL1(u[i],v[i],warpIm[i],Images[i],Images[i+1],opt1);
			break;
		case ZQ_OpticalFlowOptions::METHOD_HS_L1:
			ZQ_OpticalFlow::Coarse2Fine_HS_L1(u[i],v[i],warpIm[i],Images[i],Images[i+1],opt1);
			break;
		case ZQ_OpticalFlowOptions::METHOD_ADMM_L2:
			ZQ_OpticalFlow::Coarse2Fine_ADMM_L2(u[i],v[i],warpIm[i],Images[i],Images[i+1],opt1);
			break;
		case ZQ_OpticalFlowOptions::METHOD_ADMM_DL1:
			ZQ_OpticalFlow::Coarse2Fine_ADMM_DL1(u[i],v[i],warpIm[i],Images[i],Images[i+1],opt1);
			break;
		}

	}	

	double end = omp_get_wtime();

#pragma omp parallel for 
	for(int i = 0;i < image_num-1;i++)
	{
		DImage flow;
		flow.assemble(u[i],v[i]);
		sprintf_s(buf,"%s\\flow%d_%d.di2",out_fold,i+base_id,i+1+base_id);
		flow.saveImage(buf);


		int wheelSize = 64;
		int color_type = 1;
		IplImage* flow_img =  ZQ_ImageIO::SaveFlowToColorImage(u[i],v[i],user_input,max_rad,wheelSize,color_type);

		sprintf_s(buf,"%s\\flow%d_%d.%s",out_fold,i+base_id,i+1+base_id,suffix);
		cvSaveImage(buf,flow_img);
		cvReleaseImage(&flow_img);

		DImage warp,other;
		warpIm[i].separate(1,warp,other);
		sprintf_s(buf,"%s\\warp%d_%d.%s",out_fold,i+1+base_id,i+base_id,suffix);
		ZQ_ImageIO::saveImage(warp,buf);

	}

	printf("total_cost = %f \n",(end-start) );

	return ;
}


void main_Seq(const ZQ_OpticalFlowOptions& opt, const char* in_fold, const char* prefix, const char* suffix, const int image_num, const int base_id,const char* out_fold)
{
	if(image_num < 3)
	{
		printf("error: image num < 3\n");
		return ;
	}
	
	char buf[2000];

	std::vector<DImage> Images;
	for(int i = 0;i < image_num;i++)
	{
		DImage Im;
		sprintf_s(buf,"%s\\%s%d.%s",in_fold,prefix,i+base_id,suffix);
		bool flag = ZQ_ImageIO::loadImage(Im,buf,0);
		if(!flag)
		{
			printf("fail to load image %s\n",buf);
			return ;
		}
		Images.push_back(Im);

	}
	
	
	double max_rad = 12;
	bool user_input = true;

	std::vector<DImage> u,v,warpIm;

	clock_t start = clock();

	switch(opt.methodType)
	{

	case ZQ_OpticalFlowOptions::METHOD_ONEDIR_INC_L2:
		ZQ_OpticalFlow::Coarse2Fine_OneDir_Inc_L2(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_ONEDIR_INC_DL1:
		ZQ_OpticalFlow::Coarse2Fine_OneDir_Inc_DL1(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_ONEDIR_DEC_L2:
		ZQ_OpticalFlow::Coarse2Fine_OneDir_Dec_L2(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_ONEDIR_DEC_DL1:
		ZQ_OpticalFlow::Coarse2Fine_OneDir_Dec_DL1(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_TWODIR_INC_L2:
		ZQ_OpticalFlow::Coarse2Fine_TwoDir_Inc_L2(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_TWODIR_INC_DL1:
		ZQ_OpticalFlow::Coarse2Fine_TwoDir_Inc_DL1(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_TWODIR_DEC_L2:
		ZQ_OpticalFlow::Coarse2Fine_TwoDir_Dec_L2(u,v,warpIm,Images,opt);
		break;
	case ZQ_OpticalFlowOptions::METHOD_TWODIR_DEC_DL1:
		ZQ_OpticalFlow::Coarse2Fine_TwoDir_Dec_DL1(u,v,warpIm,Images,opt);
		break;
	}
	
	clock_t end = clock();

	float total_time = 0.001*(end-start);
	
	for(int i = 0;i < image_num-1;i++)
	{
		
		
		DImage flow;
		flow.assemble(u[i],v[i]);
		sprintf_s(buf,"%s\\flow%d_%d.di2",out_fold,i+base_id,i+1+base_id);
		flow.saveImage(buf);
		
			
		int wheelSize = 64;
		int color_type = 1;
		IplImage* flow_img =  ZQ_ImageIO::SaveFlowToColorImage(u[i],v[i],user_input,max_rad,wheelSize,color_type);

		sprintf_s(buf,"%s\\flow%d_%d.%s",out_fold,i+base_id,i+1+base_id,suffix);
		cvSaveImage(buf,flow_img);
		cvReleaseImage(&flow_img);
		
		DImage warp,other;
		warpIm[i].separate(1,warp,other);
		sprintf_s(buf,"%s\\warp%d_%d.%s",out_fold,i+1+base_id,i+base_id,suffix);
		ZQ_ImageIO::saveImage(warp,buf);

	}

	printf("total_time = %f\n", total_time);
	return ;
}
