#include "ZQ_MotionFieldInputTexture.h"
#include "ZQ_ImageIO.h"
#include "ZQ_TextureOptimization.h"
#include "ZQ_MotionFieldSynthesisOptions.h"
#include "ZQ_GraphCutForTexture.h"
#include "ZQ_Advection.h"

using namespace ZQ;
typedef ZQ_DImage<float> DImage;

void Advection(const DImage& u, const DImage& v, const float dt, const DImage& input, DImage& output);
bool texture_to_flow(const ZQ_MotionFieldSynthesisOptions& opt);
bool generate_detailflow(const ZQ_MotionFieldSynthesisOptions& opt);
bool advect_density_static(const ZQ_MotionFieldSynthesisOptions& opt);
bool advect_density_dynamic(const ZQ_MotionFieldSynthesisOptions& opt);

void test_main(const int argc, const char** argv)
{
	ZQ_MotionFieldSynthesisOptions opt;
	if(!opt.HandleArgs(argc-1,argv+1))
	{
		return;
	}
	if(!opt.CheckValid())
	{
		return ;
	}
	
	if(opt.type == ZQ_MotionFieldSynthesisOptions::TYPE_TEXTURE_TO_FLOW)
		texture_to_flow(opt);
	else if(opt.type == ZQ_MotionFieldSynthesisOptions::TYPE_SYNTHESIS_FIELD)
		generate_detailflow(opt);
	else if(opt.type == ZQ_MotionFieldSynthesisOptions::TYPE_ADVECTION_STATIC)
		advect_density_static(opt);
	else if(opt.type == ZQ_MotionFieldSynthesisOptions::TYPE_ADVECTION_DYNAMIC)
		advect_density_dynamic(opt);
}

void main()
{
	const char* argv_texture_to_flow[] =
	{
		"samplemotionfieldsynthesis.exe",
		"type","texture_to_flow",
		"texture_file","tex1.png",
		"output_flowfile","flow1.di2"
	};
	int argc_texture_to_flow = sizeof(argv_texture_to_flow) / sizeof(char*);
	//test_main(argc_texture_to_flow, argv_texture_to_flow);

	const char* argv_synthesis_field[] =
	{
		"samplemotionfieldsynthesis.exe",
		"type","synthesis_field",
		"texture_file","tex1.png",
		"original_fold","original",
		"original_prefix","coarse_",
		"original_suffix","di2",
		"base_id","0",
		"frame_count","20",
		"synthesis_fold","synthesis",
		"synthesis_prefix","detail_",
		"synthesis_suffix","di2"
	};
	int argc_synthesis_field = sizeof(argv_synthesis_field) / sizeof(char*);
	test_main(argc_synthesis_field, argv_synthesis_field);

	const char* argv_advection_static[] =
	{
		"samplemotionfieldsynthesis.exe",
		"type","advection_static",
		"synthesis_fold","synthesis",
		"synthesis_prefix","detail_",
		"synthesis_suffix","di2",
		"frame_count","20",
		"base_id","0",
		"input_density_fold","original",
		"input_density_prefix","density_",
		"input_density_suffix","png",
		"output_density_fold","density1",
		"output_density_prefix","density_",
		"output_density_suffix","png",
		"original_fold","original",
		"original_prefix","coarse_",
		"original_suffix","di2",
		"dt","1"
	};
	int argc_advection_static = sizeof(argv_advection_static) / sizeof(char*);
	test_main(argc_advection_static, argv_advection_static);
}

bool texture_to_flow(const ZQ_MotionFieldSynthesisOptions& opt)
{
	const char* tex_file = opt.texture_file;
	const char* output_flowfile = opt.output_flowfile;
	DImage tex;
	if(!ZQ_ImageIO::loadImage(tex,tex_file,0))
	{
		printf("failed to load %s : tex_type: image (.png, .jpg, .bmp, etc)\n",tex_file);
		return false;
	}

	DImage tmpfield;
	ZQ_MotionFieldInputTexture input;
	if(opt.smoothSynthesis)
	{
		DImage smoothDetail;
		tex.GaussianSmoothing(smoothDetail,opt.smoothSigma,3);
		input.SetTexture(smoothDetail);
		input.TranslateToMotionField(tmpfield);
	}
	else
	{
		input.SetTexture(tex);
		input.TranslateToMotionField(tmpfield);
	}
	const static int BUF_LEN = 200;
	char buf[BUF_LEN];

	sprintf_s(buf, BUF_LEN,"%s.png",output_flowfile);
	DImage detail_u,detail_v;
	tmpfield.separate(1,detail_u,detail_v);
	IplImage* detail_img = ZQ_ImageIO::SaveFlowToColorImage(detail_u,detail_v,false,0,64,1,false);
	cvSaveImage(buf,detail_img);
	cvReleaseImage(&detail_img);

	tmpfield.saveImage(output_flowfile);
	return true;
}

bool generate_detailflow(const ZQ_MotionFieldSynthesisOptions& opt)
{
	const char* tex_file = opt.texture_file;

	DImage tex;

	if(opt.tex_vector_field)
	{
		if(!tex.loadImage(tex_file))
		{
			printf("failed to load %s : tex_type: vector_field (.di2 or .di3)\n",tex_file);
			return false;
		}
	}
	else
	{
		if(!ZQ_ImageIO::loadImage(tex,tex_file,0))
		{
			printf("failed to load %s : tex_type : image (.png,.jpg, .bmp, etc.)\n",tex_file);
			return false;
		}
	}
	

	const char* original_fold = opt.original_fold;
	const char* original_prefix = opt.original_prefix;
	const char* original_suffix = opt.original_suffix;
	const char* synthesis_fold = opt.synthesis_fold;
	const char* synthesis_prefix = opt.synthesis_prefix;
	const char* synthesis_suffix = opt.synthesis_suffix;

	int fineWidth = opt.fine_width;
	int fineHeight = opt.fine_height;

	const static int BUF_LEN = 200;
	char buf[BUF_LEN];
	
	DImage detailflow(fineWidth,fineHeight,1);
	DImage advectedDetail(fineWidth,fineHeight,1);

	if(opt.tex_vector_field)
	{
		detailflow.allocate(fineWidth,fineHeight,2);
		advectedDetail.allocate(fineWidth,fineHeight,2);
	}
	

	ZQ_TextureOptimization to;
	int borderWidth = opt.border_width;
	float reduce_factor = opt.reduce_factor;
	int max_level = opt.max_level;
	int to_max_iter = opt.to_iter;
	int sor_iter = opt.sor_iter;
	float grad_weight = opt.grad_weight;
	float ctrl_weight = opt.ctrl_weight;
	float search_probe = opt.search_probe;

	to.SetSource(tex);
	to.SetSearchProbe(search_probe);

	int base_id = opt.base_id;
	int frame_count = opt.frame_count;
	for(int fr = base_id; fr < base_id+frame_count;fr++)
	{
		printf("fr = %d\n",fr);
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",original_fold,original_prefix,fr,original_suffix);
		
		DImage coarseFlow,coarseU,coarseV;
		if(!coarseFlow.loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}
		coarseFlow.separate(1,coarseU,coarseV);

		if(fr == 0)
		{
			ZQ_GraphCutForTexture<float> g;
			g.SetInputTexture(tex.data(),tex.width(),tex.height(),tex.nchannels());
			g.TextureSynthesis(detailflow.width(),detailflow.height(),opt.border_width*2,opt.border_width,opt.search_probe);

			g.ExportOutputTexture(detailflow.data());

			for(int patch_size = opt.border_width*2; patch_size >= 4;)				
			{
				g.Optimize(30,patch_size);
				patch_size *= 0.75;
			}
			
			//to.SynthesisWithoutControl(borderWidth,reduce_factor,max_level,grad_weight,to_max_iter,sor_iter,detailflow);
		}
		else
		{
			to.SynthesisWithControl(advectedDetail,borderWidth,reduce_factor,max_level,grad_weight,ctrl_weight,to_max_iter,sor_iter,detailflow);
		}

		const float dt = 0.03;
		Advection(coarseU,coarseV,dt,detailflow,advectedDetail);

		if(opt.tex_vector_field)
		{
			sprintf_s(buf, BUF_LEN,"%s\\%s%d.png",synthesis_fold,synthesis_prefix,fr);
			DImage detail_u,detail_v;
			detailflow.separate(1,detail_u,detail_v);
			IplImage* detail_img = ZQ_ImageIO::SaveFlowToColorImage(detail_u,detail_v,false,0,64,1,false);
			cvSaveImage(buf,detail_img);
			cvReleaseImage(&detail_img);

			sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",synthesis_fold,synthesis_prefix,fr,synthesis_suffix);
			detailflow.saveImage(buf);

		}
		else
		{
			sprintf_s(buf, BUF_LEN,"%s\\tex_%d.png",synthesis_fold,fr);
			ZQ_ImageIO::saveImage(detailflow,buf);

			DImage tmpfield;
			ZQ_MotionFieldInputTexture input;
			if(opt.smoothSynthesis)
			{
				DImage smoothDetail;
				detailflow.GaussianSmoothing(smoothDetail,opt.smoothSigma,3);
				input.SetTexture(smoothDetail);
				input.TranslateToMotionField(tmpfield);
			}
			else
			{
				input.SetTexture(detailflow);
				input.TranslateToMotionField(tmpfield);
			}

			sprintf_s(buf, BUF_LEN,"%s\\%s%d.png",synthesis_fold,synthesis_prefix,fr);
			DImage detail_u,detail_v;
			tmpfield.separate(1,detail_u,detail_v);
			IplImage* detail_img = ZQ_ImageIO::SaveFlowToColorImage(detail_u,detail_v,false,0,64,1,false);
			cvSaveImage(buf,detail_img);
			cvReleaseImage(&detail_img);

			sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",synthesis_fold,synthesis_prefix,fr,synthesis_suffix);
			tmpfield.saveImage(buf);
		}
	}

	return true;
}

bool advect_density_static(const ZQ_MotionFieldSynthesisOptions& opt)
{
	int N = opt.advectionN;
	int frame_count = opt.frame_count;
	const char* synthesis_fold = opt.synthesis_fold;
	const char* synthesis_prefix = opt.synthesis_prefix;
	const char* synthesis_suffix = opt.synthesis_suffix;
	const char* original_fold = opt.original_fold;
	const char* original_prefix = opt.original_prefix;
	const char* original_suffix = opt.original_suffix;
	const char* input_density_fold = opt.input_density_fold;
	const char* input_density_prefix = opt.input_density_prefix;
	const char* input_density_suffix = opt.input_density_suffix;
	const char* output_density_fold = opt.output_density_fold;
	const char* output_density_prefix = opt.output_density_prefix;
	const char* output_density_suffix = opt.output_density_suffix;


	int start_fr = opt.base_id;
	std::vector<DImage> flows(N);

	const static int BUF_LEN = 2000;
	char buf[BUF_LEN];

	const float coarse_weight = opt.coarse_field_weight;
	const float detail_weight = opt.detail_field_weight;
	const float dt = opt.dt;

	for(int i = start_fr;i < start_fr+frame_count;i++)
	{
		printf("frame [%d]\n",i);
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",original_fold,original_prefix,i,original_suffix);
		DImage flow;
		if(!flow.loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		DImage detailflow;
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",synthesis_fold,synthesis_prefix,i,synthesis_suffix);
		if(!detailflow.loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		int width = detailflow.width();
		int height = detailflow.height();
		if(!flow.matchDimension(width,height,2))
			flow.imresize(width,height);

		flow.Multiplywith(coarse_weight);
		flow.Addwith(detailflow,detail_weight);

		DImage in_density,out_density;
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",input_density_fold,input_density_prefix,i,input_density_suffix);
		if(!ZQ_ImageIO::loadImage(in_density,buf,0))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		in_density.imresize(width,height);


		DImage cur_u,cur_v;
		flow.separate(1,cur_u,cur_v);

		sprintf_s(buf, BUF_LEN,"%s\\flow_%d.png",output_density_fold,i);
		IplImage* flow_img = ZQ_ImageIO::SaveFlowToColorImage(cur_u,cur_v,true,1,64,1,false);
		cvSaveImage(buf,flow_img);
		cvReleaseImage(&flow_img);

		for(int j = 0;j < N;j++)
		{
			Advection(cur_u,cur_v,dt,in_density,out_density);
			in_density = out_density;
		}

		sprintf_s(buf, BUF_LEN, "%s\\%s%d.%s",output_density_fold,output_density_prefix,i,output_density_suffix);
		if(!ZQ_ImageIO::saveImage(out_density,buf))
		{
			printf("failed to save %s\n",buf);
			return false;
		}
	}

	return true;
}

bool advect_density_dynamic(const ZQ_MotionFieldSynthesisOptions& opt)
{
	int N = opt.advectionN;
	int M = opt.frame_count-N;
	const char* synthesis_fold = opt.synthesis_fold;
	const char* synthesis_prefix = opt.output_density_prefix;
	const char* synthesis_suffix = opt.synthesis_suffix;
	const char* original_fold = opt.original_fold;
	const char* original_prefix = opt.original_prefix;
	const char* original_suffix = opt.original_suffix;
	const char* input_density_fold = opt.input_density_fold;
	const char* input_density_prefix = opt.input_density_prefix;
	const char* input_density_suffix = opt.input_density_suffix;
	const char* output_density_fold = opt.output_density_fold;
	const char* output_density_prefix = opt.output_density_prefix;
	const char* output_density_suffix = opt.output_density_suffix;

	int start_fr = opt.base_id;
	std::vector<DImage> flows(N);
	const static int BUF_LEN = 2000;
	char buf[BUF_LEN];

	const float coarse_weight = opt.coarse_field_weight;
	const float detail_weight = opt.detail_field_weight;
	const float dt = opt.dt;

	for(int i = start_fr;i < start_fr+N-1;i++)
	{
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",original_fold,original_prefix,i,original_suffix);
		int off = i%N;
		if(!flows[off].loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		DImage detailflow;
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",synthesis_fold,synthesis_prefix,i,synthesis_suffix);
		if(!detailflow.loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		int width = detailflow.width();
		int height = detailflow.height();
		if(!flows[off].matchDimension(width,height,2))
			flows[off].imresize(width,height);

		flows[off].Multiplywith(coarse_weight);
		flows[off].Addwith(detailflow,detail_weight);
	}

	for(int i = start_fr;i < start_fr+M;i++)
	{
		printf("frame [%d]\n",i);
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",original_fold,original_prefix,i+N-1,original_suffix);
		int off = (i+N-1)%N;
		if(!flows[off].loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		DImage detailflow;
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",synthesis_fold,synthesis_prefix,i+N-1,synthesis_suffix);
		if(!detailflow.loadImage(buf))
		{
			printf("failed to load %s\n",buf);
			return false;
		}

		int width = detailflow.width();
		int height = detailflow.height();
		if(!flows[off].matchDimension(width,height,2))
			flows[off].imresize(width,height);
		
		
		flows[off].Multiplywith(coarse_weight);
		flows[off].Addwith(detailflow,detail_weight);

		DImage in_density,out_density;
		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",input_density_fold,input_density_prefix,i,input_density_suffix);
		
		if(!ZQ_ImageIO::loadImage(in_density,buf,0))
		{
			printf("failed to load %s\n",buf);
			return false;
		}
		
		in_density.imresize(width,height);


		for(int j = 0;j < N;j++)
		{
			int cur_off = (i+j)%N;
			DImage cur_u,cur_v;
			flows[cur_off].separate(1,cur_u,cur_v);

			if(j == 0)
			{
				sprintf_s(buf, BUF_LEN, "%s\\flow_%d.png",output_density_fold,i);
				IplImage* flow_img = ZQ_ImageIO::SaveFlowToColorImage(cur_u,cur_v,true,1,64,1,false);
				cvSaveImage(buf,flow_img);
				cvReleaseImage(&flow_img);
			}
			Advection(cur_u,cur_v,dt,in_density,out_density);
			
			in_density = out_density;
		}

		sprintf_s(buf, BUF_LEN,"%s\\%s%d.%s",output_density_fold,output_density_prefix,i,output_density_suffix);
		
		if(!ZQ_ImageIO::saveImage(out_density,buf))
		{
			printf("failed to save %s\n",buf);
			return false;
		}
	}

	return true;
}

void Advection(const DImage& u, const DImage& v, const float dt, const DImage& input, DImage& output)
{
	int width = u.width();
	int height = u.height();
	float voxel_len = 1.0/width;
	int inputWidth = input.width();
	int inputHeight = input.height();
	int nChannels = input.nchannels();

	float detail_voxel_xlen = 1.0/inputWidth;
	float detail_voxel_ylen = 1.0/inputHeight;
	
	int nPts = inputWidth*inputHeight;
	float* in_pos = new float[nPts*2];
	float* out_pos = new float[nPts*2];

	for(int h = 0;h < inputHeight;h++)
	{
		for(int w = 0;w < inputWidth;w++)
		{
			float y = (h+0.5)*detail_voxel_ylen;
			float x = (w+0.5)*detail_voxel_xlen;
			in_pos[(h*inputWidth+w)*2+0] = x;
			in_pos[(h*inputWidth+w)*2+1] = y;
		}
	}

	ZQ::ZQ_Advection::ZQ_BactTraceAdvection(u.data(),v.data(),0,width,height,voxel_len,voxel_len,dt,30,nPts,in_pos,out_pos);

	const float* input_data = input.data();
	
	if(!output.matchDimension(inputWidth,inputHeight,nChannels))
		output.allocate(inputWidth,inputHeight,nChannels);

	float* output_data = output.data();
	for(int h = 0;h < inputHeight;h++)
	{
		for(int w = 0;w < inputWidth;w++)
		{
			float x = out_pos[(h*inputWidth+w)*2+0]/detail_voxel_xlen-0.5;
			float y = out_pos[(h*inputWidth+w)*2+1]/detail_voxel_ylen-0.5;

			ZQ::ZQ_ImageProcessing::BilinearInterpolate(input_data,inputWidth,inputHeight,nChannels,x,y,output_data+(h*inputWidth+w)*nChannels,false);
		}
	}

	delete []in_pos;
	delete []out_pos;
}