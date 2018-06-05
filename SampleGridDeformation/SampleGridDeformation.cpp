#include "ZQ_GridDeformation.h"
#include "ZQ_GridDeformationOptions.h"
#include "SampleGridDeformationSceneDescription.h"
#include "cv.h"
#include "highgui.h"

using namespace ZQ;

template<class T>
void test(int argc, const char** argv)
{
	SampleGridDeformationSceneDescription<T> scenedecs;
	if(argc < 4)
	{
		printf(".exe outfile angle_alpha iteration [options for scene]\n");
		return;
	}

	const char* outfile = argv[1];
	float angle_alpha = atof(argv[2]);
	int iteration = atoi(argv[3]);
	
	
	if(!scenedecs.LoadFromArgs(argc-4,(const char**)(argv+4)))
	{
		return;
	}

	int width = scenedecs.GetWidth();
	int height = scenedecs.GetHeight();
	const bool* nouseful_flag = scenedecs.GetNousefulFlag();
	const bool* fixed_flag = scenedecs.GetFixedFlag();
	const T* init_coord = scenedecs.GetInitCoord();

	T* out_coord = new T[(width*height*2)];
	
	ZQ_GridDeformation<T> deform;
	ZQ_GridDeformationOptions opt;
	opt.methodType = ZQ_GridDeformationOptions::METHOD_LINE_ANGLE_DISTANCE_ENERGY;
	opt.line_weight = 0;
	opt.angle_weight = 1;
	opt.distance_weight = 10;
	opt.distance = 1;
	opt.FPIteration = 5;
	opt.iteration = iteration;

	deform.BuildMatrix(width,height,nouseful_flag,fixed_flag,opt);
	deform.Deformation(init_coord,out_coord);
	

	float scale = 10;
	int b_width = 100;
	int b_height = 100;
	int b_shift_x = (b_width/2+0.5)*scale;
	int b_shift_y = (b_height/2+0.5)*scale;
	int backwidth = b_width*scale;
	int backheight = b_height*scale;
	IplImage* background = cvCreateImage(cvSize(backwidth,backheight),IPL_DEPTH_8U,3);
	cvZero(background);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width-1;j++)
		{
			if(!nouseful_flag[i*width+j] && !nouseful_flag[i*width+j+1])
				cvLine(background,cvPoint(out_coord[(i*width+j)*2]*scale+b_shift_x,out_coord[(i*width+j)*2+1]*scale+b_shift_y),
				cvPoint(out_coord[(i*width+j+1)*2]*scale+b_shift_x,out_coord[(i*width+j+1)*2+1]*scale+b_shift_y),cvScalar(0,200,0));
		}
		if(opt.methodType == ZQ_GridDeformationOptions::METHOD_LINE_ANGLE_DISTANCE_ENERGY_XLOOP)
		{
			if(!nouseful_flag[i*width+0] && !nouseful_flag[i*width+width-1])
				cvLine(background,cvPoint(out_coord[(i*width+0)*2]*scale+b_shift_x,out_coord[(i*width+0)*2+1]*scale+b_shift_y),
				cvPoint(out_coord[(i*width+width-1)*2]*scale+b_shift_x,out_coord[(i*width+width-1)*2+1]*scale+b_shift_y),cvScalar(0,200,0));
		}
	}
	for(int i = 0;i < height-1;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(!nouseful_flag[i*width+j] && !nouseful_flag[i*width+j+width])
				cvLine(background,cvPoint(out_coord[(i*width+j)*2]*scale+b_shift_x,out_coord[(i*width+j)*2+1]*scale+b_shift_y),
				cvPoint(out_coord[(i*width+j+width)*2]*scale+b_shift_x,out_coord[(i*width+j+width)*2+1]*scale+b_shift_y),cvScalar(0,200,0));
		}
	}
	

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(!nouseful_flag[i*width+j])
				cvCircle(background,cvPoint(out_coord[(i*width+j)*2]*scale+b_shift_x,out_coord[(i*width+j)*2+1]*scale+b_shift_y),2,cvScalar(0,0,200),2);
		}
	}

	cvSaveImage(outfile,background);
	
	
	delete []out_coord;

	cvReleaseImage(&background);
}


int main(/*int argc, const char** argv*/)
{
	
	const char* m_argv[] = 
	{
		"SampleGridDeformation.exe",
		"deform.png",
		"5",
		"10000",
		"resolution",
		"4",
		"4",
		"fixed",
		"0",
		"0",
		"10",
		"10",
		"fixed",
		"3",
		"0",
		"13",
		"10",
		"fixed",
		"0",
		"3",
		"10",
		"13",
		"fixed",
		"3",
		"3",
		"15",
		"15"
	};

	int m_argc = sizeof(m_argv) / sizeof(char*);

	//test<float>(m_argc,m_argv);
	test<double>(m_argc,m_argv);
	return 0;
}