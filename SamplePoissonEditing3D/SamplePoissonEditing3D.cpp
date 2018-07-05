#include "ZQ_DoubleImage.h"
#include "ZQ_DoubleImage3D.h"
#include "ZQ_ImageIO.h"
#include "ZQ_PoissonEditing.h"
#include "ZQ_PoissonEditing3D.h"
#include <vector>
#include <stdio.h>
#include <string.h>

using namespace ZQ;

template<class T>
void test(const int argc, const char** argv)
{
	typedef T BaseType;
	typedef ZQ_DImage<BaseType> DImage;
	typedef ZQ_DImage3D<BaseType> DImage3D;

	if(argc != 8)
	{
		printf(" args error\n");
		return;
	}
	int num = atoi(argv[1]);
	const char* input_fold = argv[2];
	const char* input_prefix = "";
	const char* input_suffix = argv[3];
	const char* copyin_fold = argv[4];
	const char* copyin_prefix = "";
	const char* copyin_suffix = argv[5];
	const char* mask_file = argv[6];
	const char* output_fold = argv[7];

	std::vector<DImage> input_images;
	std::vector<DImage> copyin_images;
	DImage mask;

	char buf[500];
	for(int i = 0;i < num;i++)
	{
		DImage tmp;
		sprintf(buf,"%s\\%s%d.%s",input_fold,input_prefix,i,input_suffix);
		if(!ZQ_ImageIO::loadImage(tmp,buf,0))
		{
			printf("failed to load %s\n",buf);
			return;
		}
		input_images.push_back(tmp);
	}

	for(int i = 0;i < num;i++)
	{
		DImage tmp;
		sprintf(buf,"%s\\%s%d.%s",copyin_fold,copyin_prefix,i,copyin_suffix);
		if(!ZQ_ImageIO::loadImage(tmp,buf,0))
		{
			printf("failed to load %s\n",buf);
			return;
		}
		copyin_images.push_back(tmp);
	}

	if(!ZQ_ImageIO::loadImage(mask,mask_file,0))
	{
		printf("failed to load %s\n",mask_file);
		return;
	}

	ZQ_PoissonEditingOptions opt;
	opt.type = ZQ_PoissonEditingOptions::METHOD_NAIVE;
	opt.grad_scale = 1;
	opt.nSORIteration = 500;
	opt.display = true;

	DImage tmp_output;
	ZQ_PoissonEditing::PoissonEditing(mask,copyin_images[0],input_images[0],tmp_output,opt);
	input_images[0] = tmp_output;
	ZQ_PoissonEditing::PoissonEditing(mask,copyin_images[num-1],input_images[num-1],tmp_output,opt);
	input_images[num-1] = tmp_output;

	int width = mask.width();
	int height = mask.height();

	DImage3D input3D(width,height,num);
	DImage3D copyin3D(width,height,num);
	DImage3D output3D(width,height,num);
	DImage3D mask3D(width,height,num);
	BaseType*& input3D_data = input3D.data();
	BaseType*& copyin3D_data = copyin3D.data();
	BaseType*& mask3D_data = mask3D.data();
	for(int i = 0;i < num;i++)
	{
		for(int pp = 0;pp < width*height;pp++)
		{
			input3D_data[i*width*height+pp] = input_images[i].data()[pp];
			copyin3D_data[i*width*height+pp] = copyin_images[i].data()[pp];
		}
	}
	for(int i = 1;i < num-1;i++)
	{
		for(int pp = 0;pp < width*height;pp++)
		{
			mask3D_data[i*width*height+pp] = mask.data()[pp];
		}
	}

	ZQ_PoissonEditing3DOptions opt3d;
	opt3d.nSORIteration = 500;
	opt3d.display = true;
	opt3d.grad_scale = 1;

	if(!ZQ_PoissonEditing3D::PoissonEditing(mask3D,copyin3D,input3D,output3D,opt3d))
	{
		printf("poisson edting 3d fail\n");
		return;
	}

	BaseType*& output_data = output3D.data();
	for(int i = 0;i < num;i++)
	{
		DImage tmp(width,height);
		for(int pp = 0;pp < width*height;pp++)
			tmp.data()[pp] = output_data[i*width*height+pp];

		sprintf(buf,"%s\\%s%d.%s",output_fold,input_prefix,i,input_suffix);
		if(!ZQ_ImageIO::saveImage(tmp,buf))
		{
			printf("failed to save %s\n",buf);
		}
	}
	return;
}

int main(/*int argc, const char** argv*/)
{
	const char* m_argv1[] = 
	{
		"SamplePoissonEditing3D.exe",
		"10",
		"input1",
		"png",
		"copyin1",
		"png",
		"mask1.png",
		"output1"
	};
	const char* m_argv2[] =
	{
		"SamplePoissonEditing3D.exe",
		"200",
		"input2",
		"jpg",
		"copyin2",
		"jpg",
		"mask2.png",
		"output2"
	};
	int m_argc1 = sizeof(m_argv1) / sizeof(char*);
	int m_argc2 = sizeof(m_argv2) / sizeof(char*);
	test<float>(m_argc1,m_argv1);
	test<float>(m_argc2, m_argv2);
	//test<double>(m_argc,m_argv);
	return EXIT_SUCCESS;
}
