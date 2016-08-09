#include "ZQ_BlendTwoImages3D.h"
#include "ZQ_DoubleImage3D.h"
#include <time.h>
#include <stdio.h>

using namespace ZQ;


class blend_options
{
public:
	blend_options(){reset();}
	~blend_options(){}

	int skip;
	float radius;
	int iterations;

	void reset()
	{
		skip = 1;
		radius = 1.5;
		iterations = 30;
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
		return true;
	}
};

template<class T>
void test(const int argc, const char** argv)
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

	clock_t start = clock();

	const char* imgfile1 = argv[1];
	const char* imgfile2 = argv[2];
	const char* flowfile = argv[3];
	double weight1 = atof(argv[4]);
	const char* outfile = argv[5];
	ZQ_DImage3D<T> img1,img2,flow;
	if(!img1.loadImage(imgfile1))
	{
		printf("failed to load file %s\n",imgfile1);
		return;
	}

	if(!img2.loadImage(imgfile2))
	{
		printf("failed to load file %s\n",imgfile2);
		return;
	}

	if(!flow.loadImage(flowfile))
	{
		printf("failed to load file %s\n",flowfile);
		return ;
	}

	int width = img1.width();
	int height = img1.height();
	int depth = img1.depth();
	int nChannels = img1.nchannels();

	if(!img2.matchDimension(width,height,depth,nChannels))
	{
		printf("two images' dimensions don' match\n");
		return;
	}

	if(!flow.matchDimension(width,height,depth,3))
	{
		printf("invalid flow dimension\n");
		return ;
	}

	ZQ_DImage3D<T> u(width,height,depth),v(width,height,depth) ,w(width,height,depth);

	T* im1_data = img1.data();
	T* im2_data = img2.data();
	T* flow_data = flow.data();
	T* u_data = u.data();
	T* v_data = v.data();
	T* w_data = w.data();

	for(int i = 0;i < width*height*depth;i++)
	{
		u_data[i] = flow_data[i*3+0];
		v_data[i] = flow_data[i*3+1];
		w_data[i] = flow_data[i*3+2];
	}


	ZQ_DImage3D<T> output(width,height,depth,nChannels);
	T* out_data = output.data();

	ZQ_BlendTwoImages3D::BlendTwoImages<T>(width,height,depth,nChannels,im1_data,im2_data,u_data,v_data,w_data,weight1,opt.skip,opt.radius,opt.iterations,out_data);

	if(!output.saveImage(outfile))
	{
		printf("failed to save file %s\n",outfile);
		return ;
	}

	clock_t end = clock();
	printf("time: %.3f seconds\n", 0.001*(end-start));

}

int main(int argc, const char** argv)
{
	test<float>(argc,argv);
	test<double>(argc,argv);
	return 0;
}
