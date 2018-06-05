#include "ZQ_InsertFrames.h"
#include "windows.h"
#include "process.h"
#include <time.h>

using namespace ZQ;

class _insert_frames_arg
{
public:
	ZQ_InsertFramesOptions opt;
	bool done_flag;
};

void _insert_frames(void* arg)
{
	_insert_frames_arg* opt = (_insert_frames_arg*)arg;
	ZQ_InsertFrames::InsertFrames<float>((*opt).opt);
	opt->done_flag = true;
}

int main(const int argc, const char** argv)
{
	clock_t t1 = clock();
	ZQ_InsertFramesOptions opt;
	if (!opt.HandleArgs(argc - 1, argv + 1))
	{
		return 0;
	}

	if (opt.nCores > opt.frameNum - 1)
		opt.nCores = opt.frameNum - 1;

	if (opt.nCores > 1)
	{
		
		int each_frame_num = (opt.frameNum -1 + opt.nCores - 1) / opt.nCores;
		_insert_frames_arg* tmp_args = new _insert_frames_arg[opt.nCores];
		for (int i = 0; i < opt.nCores; i++)
		{
			tmp_args[i].opt = opt;
			tmp_args[i].opt.baseId = opt.baseId + each_frame_num * i;
			if (i == opt.nCores - 1)
			{
				tmp_args[i].opt.frameNum = opt.frameNum - each_frame_num*i;
			}
			else
			{
				tmp_args[i].opt.frameNum = each_frame_num + 1;
			}
			tmp_args[i].opt.outBaseId = opt.outBaseId + each_frame_num*i*opt.speedUp;
			tmp_args[i].done_flag = false;
		}

		for (int i = 0; i < opt.nCores - 1;i++)
			_beginthread(_insert_frames, 0, &tmp_args[i]);
		_insert_frames(&tmp_args[opt.nCores - 1]);

		bool all_done;
		do{
			Sleep(10);
			all_done = true;
			for (int i = 0; i < opt.nCores; i++)
			{
				if (!tmp_args[i].done_flag)
				{
					all_done = false;
					break;
				}
			}
		} while (!all_done);
		delete[]tmp_args;
	}
	else
	{
		ZQ_InsertFrames::InsertFrames<float>(opt);
	}
	
	clock_t t2 = clock();
	printf("cost time:%f seconds\n", 0.001*(t2 - t1));
	return 0;
}


int main2()
{
	/************* 读取图像 ****************/
	//ZQ_DImage<float> 是自定义图像格式，排列顺序和opencv类似，但是图像亮度是0.0-1.0
	//找到文件ZQ_ImageIO.h 中loadImage函数，可以查看是如何从IplImage 转成ZQ_DImage<float>
	ZQ_DImage<float> im1, im2;
	ZQ_DImage<float> bgr1, bgr2, out_im;
	ZQ_ImageIO::loadImage(im1, "1.jpg",0);	//读取灰度图，用于光流
	ZQ_ImageIO::loadImage(im2, "2.jpg",0);	
	ZQ_ImageIO::loadImage(bgr1, "1.jpg", 1);	//读取彩色图，用于插帧
	ZQ_ImageIO::loadImage(bgr2, "2.jpg", 1);
	int width = im1.width();
	int height = im1.height();
	float weight1 = 0.5;	//插帧时第一帧图像的权重，取值范围(0.0, 1.0)
	float scale = 0.25;	//估计光流时可以缩小了进行估计，建议取0.25或者0.5
	float scale_back = 1.0 / scale;
	im1.imresize(scale);
	im2.imresize(scale);

	/************** L2 使用示例  ************/
	/* L2 方法固定的参数适应性极强，计算量也非常小，使用默认参数，两帧图像间插入7帧，单线程约6秒（包括读写文件）
	*/
	ZQ_DImage<float> fw_u_L2, fw_v_L2, warpIm_L2;
	ZQ_OpticalFlowOptions opt_L2;	//默认参数已是最佳经验参数，一般不要改动，除非经验特别丰富
	ZQ_OpticalFlow::Coarse2Fine_HS_L2(fw_u_L2, fw_v_L2, warpIm_L2, im1, im2, opt_L2);
	fw_u_L2.imresize(width, height);	//注意放大回原来尺度
	fw_v_L2.imresize(width, height);
	fw_u_L2.Multiplywith(scale_back);	//注意光流的值也要放大
	fw_v_L2.Multiplywith(scale_back);
	if (!ZQ_InsertFrames::InsertOneFrameWithFlow_Simple(bgr1, bgr2, fw_u_L2, fw_v_L2, weight1, out_im))
	{
		printf("failed to run InsertOneFrameWithFlow_Simple!\n");
		return 0;
	}
	ZQ_ImageIO::saveImage(out_im, "out_L2.jpg");

	/*************** L1 使用示例 ***************/
	/* L1 方法参数繁多，而且适应性插，计算量非常大，使用默认参数，两帧图像间插入7帧，单线程约196秒（包括读写文件）
	*/
	ZQ_DImage<float> fw_u_L1, fw_v_L1, bw_u_L1, bw_v_L1, warpIm_L1;
	ZQ_OpticalFlowOptions opt_L1;
	ZQ_OpticalFlowOptions::GetDefaultOptions_HS_L1(opt_L1);//默认参数已是最佳经验参数，一般不要改动，除非经验特别丰富
	ZQ_OpticalFlow::Coarse2Fine_HS_L1(fw_u_L1, fw_v_L1, warpIm_L1, im1, im2, opt_L1);//估计im1 到im2的光流
	ZQ_OpticalFlow::Coarse2Fine_HS_L1(bw_u_L1, bw_v_L1, warpIm_L1, im2, im1, opt_L1);//估计im2 到im1的光流
	fw_u_L1.imresize(width, height);	//注意放大回原来尺度
	fw_v_L1.imresize(width, height);
	fw_u_L1.Multiplywith(scale_back);	//注意光流的值也要放大
	fw_v_L1.Multiplywith(scale_back);
	bw_u_L1.imresize(width, height);	//注意放大回原来尺度
	bw_v_L1.imresize(width, height);
	bw_u_L1.Multiplywith(scale_back);	//注意光流的值也要放大
	bw_v_L1.Multiplywith(scale_back);
	if (!ZQ_InsertFrames::InsertOneFrameWithFlow_Complex(bgr1, bgr2, fw_u_L1, fw_v_L1, bw_u_L1, bw_v_L1, weight1, out_im))//注意有默认参数，以及这个函数有重载，一般情况不要改动参数
	{
		printf("failed to run InsertOneFrameWithFlow_Complex!\n");
		return 0;
	}
		
	ZQ_ImageIO::saveImage(out_im, "out_L1.jpg");
	
	return 0;
}