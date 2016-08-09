#include "ZQ_BlendTwoImages.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_CompressedImage.h"
#include "ZQ_ImageIO.h"
#include <time.h>

using namespace ZQ;


class blend_options
{
public:
	blend_options(){reset();}
	~blend_options(){}

	int skip;
	float radius;
	float radius_scale;
	int number_of_neighbor;
	int iterations;
	bool various_neighbor_num;
	bool cubic;
	int blend_mode;

	void reset()
	{
		skip = 1;
		radius = 1.5;
		iterations = 30;
		radius_scale = 1.5;
		number_of_neighbor = 5;
		various_neighbor_num = false;
		cubic = false;
	}

	bool handle_args(const int argc, const char** argv)
	{
		int k = 0;
		while(k < argc)
		{
			if(_strcmpi(argv[k],"skip") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for skip\n");
					return false;
				}
				skip = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"radius") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for radius\n");
					return false;
				}
				radius = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"radius_scale") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for radius_scale\n");
					return false;
				}
				radius_scale = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"number_of_neighbor") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for number_of_neighbor\n");
					return false;
				}
				number_of_neighbor = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"iterations") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for iterations\n");
					return false;
				}
				iterations = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"various_neighbor_num") == 0)
			{
				various_neighbor_num = true;
			}
			else if(_strcmpi(argv[k],"cubic") == 0)
			{
				cubic = true;
			}
			else if(_strcmpi(argv[k],"blend_mode") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need the value for blend_mode\n");
					return false;
				}
				blend_mode = atoi(argv[k]);
			}
			k++;
		}
		return true;
	}

	bool check_valid()
	{
		if(skip < 1)
		{
			printf("skip must be >= 1\n");
			return false;
		}
		if(radius < 0)
		{
			printf("radius must > 0\n");
			return false;
		}
		if(iterations < 1)
		{
			printf("iterations must be >= 1\n");
			return false;
		}
		if(number_of_neighbor <= 1)
		{
			printf("number_of_neighbor must be > 1\n");
			return false;
		}
	
		return true;
	}
};


template<class T>
void test(int argc, const char** argv)
{
	if(argc < 6)
	{
		printf(" imgfile1 imgfile2 flowfile weight1 outfile [options]\n");
		return;
	}

	blend_options opt;
	opt.handle_args(argc-6,argv+6);
	if(!opt.check_valid())
	{
		return;
	}

	const char* imgfile1 = argv[1];
	const char* imgfile2 = argv[2];
	const char* flowfile = argv[3];
	double weight1 = atof(argv[4]);
	const char* outfile = argv[5];
	ZQ_DImage<T> img1,img2,out_img;
	bool flag1 = ZQ_ImageIO::loadImage(img1,imgfile1,1);
	bool flag2 = ZQ_ImageIO::loadImage(img2,imgfile2,1);
	if(!flag1)
	{
		printf("failed to load image %s\n",imgfile1);
		return ;
	}
	if(!flag2)
	{
		printf("failed to load image %s\n",imgfile2);
		return ;
	}

	if(!img1.matchDimension(img2))
	{
		printf("two images' dimensions don't match\n");
		return ;
	}

	int width = img1.width();
	int height = img1.height();
	int nChannels = img1.nchannels();

	out_img.allocate(width,height,nChannels);


	ZQ_DImage<T> flow,u,v;

	clock_t t1 = clock();

	//if(!ZQ_CompressedImage::LoadCompressedImage(flowfile,flow))
	if(!flow.loadImage(flowfile))
	{
		printf("failed to load flow %s\n",flowfile);
		return;
	}

	clock_t t2 = clock();
	
	if(flow.width() != width || flow.height() != height || flow.nchannels() != 2)
	{
		printf("flow's dimension does not match with the images\n");
		return;
	}

	flow.separate(1,u,v);

	if(!ZQ_BlendTwoImages::BlendTwoImages<T>(
		width,height,nChannels,img1.data(),img2.data(),u.data(),v.data(),weight1,out_img.data(),opt.skip,opt.iterations,opt.number_of_neighbor,opt.radius_scale,opt.cubic,opt.blend_mode))
	{
		printf("blend images fail\n");
		return;
	}

	if(!ZQ_ImageIO::saveImage(out_img,outfile))
	{
		printf("save image %s fail\n",outfile);
		return ;
	}

	clock_t t3 = clock();
	printf("blend images cost %.3f seconds\n",0.001*(t3-t2));

}


/*Sample args,
SampleBlendTwoImages par_0.png par_3.png flow0_3.di2 0.5 blend.png iteration 5000 skip 4
*/
int main(int argc, const char** argv)
{
	//test<float>(argc,argv);
	test<double>(argc,argv);
	return 0;
}
