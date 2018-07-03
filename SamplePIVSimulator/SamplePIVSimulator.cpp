#include "ZQ_PIVSimulator.h"
#include "ZQ_PoissonSolver.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;
using namespace ZQ_PoissonSolver;

typedef ZQ_PIVMovingObject::BaseType BaseType;
typedef ZQ_DImage<BaseType> DImage;

cv::Mat ParImageToIplImage(DImage& img);

void main_Regular(int idx);

void main(int argc, char** argv)
{
	if(argc != 2)
		main_Regular(0);
	else
		main_Regular(atoi(argv[1]));
}
void main_Regular(int idx)
{
	if(idx > 18 || idx < 0)
		return;
	int boarder_size = 16;
	int center_width = 256;
	int center_height = 256;
	int width = center_width+boarder_size*2;
	int height = center_height+boarder_size*2;
	bool cut_boarder = true;

	int data_index = idx;

	const static int BUF_LEN = 200;
	char out_flow_fold[BUF_LEN] = {0};
	char out_par_fold[BUF_LEN] = {0};
	int par_num = 0;
	int vort_num = 0;
	double max_vort = 1;
	double min_vort = 0.6;
	double max_vort_radius = 20;
	double min_vort_radius = 15;

	bool use_peroid_coord = false;
	
	double base_vel_u = 0;
	double base_vel_v = 0;
	int skip_frames = 0;
	int coarse_len = 16;

	ZQ_PIVMovingObject* mvobj = 0;

	DImage par_mask(width,height);


	switch(data_index)
	{
	case 0:
		strcpy_s(out_flow_fold, BUF_LEN,"flow0");
		strcpy_s(out_par_fold, BUF_LEN,"par0");
		srand(1000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.8;
		min_vort = 1.2;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
	
		boarder_size = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;

		coarse_len = 32;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				if(x < coarse_len || x >= width - coarse_len)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;

	case 1:
		strcpy_s(out_flow_fold, BUF_LEN,"flow1");
		strcpy_s(out_par_fold, BUF_LEN,"par1");
		srand(2000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 48;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				if(x >= width - coarse_len)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;

	case 2:
		strcpy_s(out_flow_fold, BUF_LEN,"flow2");
		strcpy_s(out_par_fold, BUF_LEN,"par2");
		srand(2000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 16;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if(cx%3 != 0)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;

	case 3:
		strcpy_s(out_flow_fold, BUF_LEN,"flow3");
		strcpy_s(out_par_fold, BUF_LEN,"par3");
		srand(3000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 16;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if(cx%2 == 0)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;
	case 4:
		strcpy_s(out_flow_fold, BUF_LEN,"flow4");
		strcpy_s(out_par_fold, BUF_LEN,"par4");
		srand(4000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 64;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if(cx%2 == 0)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;
	case 5:
		strcpy_s(out_flow_fold, BUF_LEN,"flow5");
		strcpy_s(out_par_fold, BUF_LEN,"par5");
		srand(5000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 32;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if(cx%2 == 0)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;
	case 6:
		strcpy_s(out_flow_fold, BUF_LEN,"flow6");
		strcpy_s(out_par_fold, BUF_LEN,"par6");
		srand(6000);
		par_num = 10000;
		vort_num = 40;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 32;
		par_mask.allocate(width,height);
		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if(cy%2 == 0)
					par_mask.data()[y*width+x] = 1;
			}
		}
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;
	case 7:
		strcpy_s(out_flow_fold, BUF_LEN,"flow7");
		strcpy_s(out_par_fold, BUF_LEN,"par7");
		srand(7000);
		par_num = 10000;
		vort_num = 20;
		max_vort = 1.8;
		min_vort = 1.2;
		max_vort_radius = 30;
		min_vort_radius = 20;
		use_peroid_coord = true;
		coarse_len = 64;
		par_mask.allocate(width,height);
		/*for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				int cy = y/coarse_len;
				int cx = x/coarse_len;

				if((cy+cx)%2 == 0)
					par_mask.data()[y*width+x] = 1;
			}
		}*/
		base_vel_u = 5;
		base_vel_v = 1;
		skip_frames = 20;
		break;
	case 8:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow8");
		strcpy_s(out_par_fold, BUF_LEN,"par8");
		srand(9000);
		par_num = 10000;
		vort_num = 30;
		max_vort = 1.6;
		min_vort = 1.2;
		max_vort_radius = 30;
		min_vort_radius = 20;
		use_peroid_coord = true;

		mvobj = new ZQ_PIVMovingObject(48,48,ZQ_PIVMovingObject::ZQ_PIV_MOVOB_CIRCLE_STATIC,"wenli.di2");

		/*has_occupy = true;
		occupy = new bool[width*height];
		memset(occupy,0,sizeof(bool)*width*height);

		for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{
				double dis = sqrt((y-height*0.5)*(y-height*0.5)+(x-width*0.5)*(x-width*0.5));
				if(dis < (height+width)*0.05)
					occupy[y*width+x] = true;
			}
		}*/

		coarse_len = 32;
		par_mask.allocate(width,height);
		/*for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
			{

				if(occupy[y*width+x])
					par_mask.data()[y*width+x] = 1;
			}
		}*/
		base_vel_u = 4;
		base_vel_v = 0;
		skip_frames = 20;

		break;

	case 9:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow9");
		strcpy_s(out_par_fold, BUF_LEN,"par9");
		srand(8000);
		par_num = 10000;
		vort_num = 30;
		max_vort = 1.4;
		min_vort = 1.0;
		max_vort_radius = 30;
		min_vort_radius = 20;
		use_peroid_coord = true;

		mvobj = new ZQ_PIVMovingObject(48,48, ZQ_PIVMovingObject::ZQ_PIV_MOVOB_RECT_UPDOWN,"wenli.di2");
		

		coarse_len = 32;
		par_mask.allocate(width,height);
		
		base_vel_u = 4;
		base_vel_v = 0;
		skip_frames = 20;

		break;

	case 10:

		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN, "flow10");
		strcpy_s(out_par_fold, BUF_LEN, "par10");
		srand(10000);
		par_num = 10000;
		vort_num = 30;
		max_vort = 1.4;
		min_vort = 1.0;
		max_vort_radius = 30;
		min_vort_radius = 20;
		use_peroid_coord = true;

		mvobj = new ZQ_PIVMovingObject(48,48, ZQ_PIVMovingObject::ZQ_PIV_MOVOB_CIRCLE_CIRCULAR,"wenli.di2");


		coarse_len = 32;
		par_mask.allocate(width,height);

		base_vel_u = 0;
		base_vel_v = 0;
		skip_frames = 20;
		break;

	case 11:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow11");
		strcpy_s(out_par_fold, BUF_LEN, "par11");
		srand(11000);
		par_num = 5000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		par_mask.allocate(width,height);
		
		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;
	case 12:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow12");
		strcpy_s(out_par_fold, BUF_LEN,"par12");
		srand(1240);
		par_num = 5000;
		vort_num = 30;
		max_vort = 2.5;
		min_vort = 1.6;
		max_vort_radius = 20;
		min_vort_radius = 10;
		use_peroid_coord = true;
		par_mask.allocate(width,height);

		base_vel_u = 0;
		base_vel_v = 0;
		skip_frames = 20;
		break;
	case 13:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow13");
		strcpy_s(out_par_fold, BUF_LEN,"par13");
		srand(1300);
		par_num = 5000;
		vort_num = 10;
		max_vort = 1.5;
		min_vort = 0.6;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		par_mask.allocate(width,height);

		base_vel_u = 1;
		base_vel_v = 1;
		skip_frames = 10;
		break;

	case 14:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN, "flow14");
		strcpy_s(out_par_fold, BUF_LEN, "par14");
		srand(14000);
		par_num = 5000;
		vort_num = 10;
		max_vort = 1.0;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 10;
		use_peroid_coord = true;

		mvobj = new ZQ_PIVMovingObject(48,48, ZQ_PIVMovingObject::ZQ_PIV_MOVOB_CIRCLE_STATIC,"wenli.di2");
	
		par_mask.allocate(width,height);
		
		base_vel_u = 2;
		base_vel_v = 0;
		skip_frames = 20;

		break;

	case 15:
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow15");
		strcpy_s(out_par_fold, BUF_LEN,"par15");
		srand(15000);
		par_num = 5000;
		vort_num = 20;
		max_vort = 1.0;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 10;
		use_peroid_coord = true;

		mvobj = new ZQ_PIVMovingObject(48,48, ZQ_PIVMovingObject::ZQ_PIV_MOVOB_RECT_STATIC,"wenli.di2");

		par_mask.allocate(width,height);

		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 50;

		break;
	case 16:	// the same field as "case18" ,but more particles
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN, "flow16");
		strcpy_s(out_par_fold, BUF_LEN,"par16");
		srand(11000);
		par_num = 20000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		par_mask.allocate(width,height);

		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;

	case 17:	// the same field as "case18" ,but less particles
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN, "flow17");
		strcpy_s(out_par_fold, BUF_LEN, "par17");
		srand(11000);
		par_num = 2000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		par_mask.allocate(width,height);

		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;

	case 18:	// the same field as "case16" and "case17" 
		boarder_size = 16;
		center_width = 256;
		center_height = 256;
		width = center_width+boarder_size*2;
		height = center_height+boarder_size*2;
		cut_boarder = true;

		strcpy_s(out_flow_fold, BUF_LEN,"flow18");
		strcpy_s(out_par_fold, BUF_LEN,"par18");
		srand(11000);
		par_num = 5000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 20;
		use_peroid_coord = true;
		par_mask.allocate(width,height);

		base_vel_u = 5;
		base_vel_v = 0;
		skip_frames = 20;
		break;
	}

	/*suggest values*/
	/*
	par_num = width*height/3;
	vort_num = 30;
	*/

	char cmdbuf[500];
	sprintf_s(cmdbuf,"if not exist \"%s\" mkdir \"%s\"",out_par_fold,out_par_fold);
	system(cmdbuf);
	sprintf_s(cmdbuf,"if not exist \"%s\" mkdir \"%s\"",out_flow_fold,out_flow_fold);
	system(cmdbuf);
	
	DImage vor_img(width-1,height-1);
	DImage macu_img(width+1,height);
	DImage macv_img(width,height+1);
	DImage u_img(width,height);
	DImage v_img(width,height);
	DImage par_img(width,height);

	BaseType*& vort_pData = vor_img.data();
	BaseType*& macu_pData = macu_img.data();
	BaseType*& macv_pData = macv_img.data();
	BaseType*& u_pData = u_img.data();
	BaseType*& v_pData = v_img.data();


	for(int vor_it = 0; vor_it < vort_num; vor_it++)
	{
		double intensity = (rand()%2-0.5)*2.0*(rand()%101/100.0*(max_vort-min_vort)+min_vort);
		double radius = rand()%101/100.0*(max_vort_radius-min_vort_radius)+min_vort_radius;
		double posx = rand()%width;
		double posy = rand()%height;
		ZQ_PIVSimulator::DrawOneParticle(vor_img,posx,posy,intensity,radius,true);
	}

	ZQ_PoissonSolver::ReconstructCurlField(width,height,vort_pData,macu_pData,macv_pData,100,false);
	ZQ_PoissonSolver::MACtoRegularGrid(width,height,macu_pData,macv_pData,u_pData,v_pData);

	if(use_peroid_coord)
	{
		ZQ_PoissonSolver::RegularGridtoMAC(width,height,u_pData,v_pData,macu_pData,macv_pData,true);

		ZQ_PoissonSolver::SolveOpenPoissonSOR_MACGrid(macu_pData,macv_pData,width,height,1000,false);

		ZQ_PoissonSolver::MACtoRegularGrid(width,height,macu_pData,macv_pData,u_pData,v_pData);
	}
	

	for(int i = 0;i < width*height;i++)
	{
		u_pData[i] += base_vel_u;
		v_pData[i] += base_vel_v;
	}


	ZQ_PIVSimulator piv_simu(width,height,mvobj);

	piv_simu.RandomInit(par_num,u_img,v_img,par_mask);


	int frame = 0;
	char buf[200];

	DImage flow;

	DImage cut_u_img(center_width,center_height),cut_v_img(center_width,center_height);
	DImage cut_par_img(center_width,center_height);

	for(int i = 0;i < skip_frames;i++)
	{
		printf("skip frame [%3d/%3d]...\n",i,skip_frames);
		piv_simu.RunOneFrame(1.0,use_peroid_coord,false);
	}

	do 
	{
		printf("frame [%3d] ...\n",frame);
		
		if(cut_boarder)
		{
			piv_simu.ExportVelocity(u_img,v_img);

			for(int h = 0; h < center_height;h++)
			{
				for(int w = 0;w < center_width;w++)
				{
					cut_u_img.data()[h*center_width+w] = u_img.data()[(h+boarder_size)*width+w+boarder_size];
					cut_v_img.data()[h*center_width+w] = v_img.data()[(h+boarder_size)*width+w+boarder_size];
				}
			}

			cv::Mat show_flow_img = ZQ_ImageIO::SaveFlowToColorImage(cut_u_img,cut_v_img,true,12,64,1);
			sprintf_s(buf,"%s\\flow_%d.png",out_flow_fold,frame);
			cv::imwrite(buf,show_flow_img);
			
			flow.assemble(cut_u_img,cut_v_img);
			sprintf_s(buf,"%s\\flow_%d.di2",out_flow_fold,frame);
			flow.saveImage(buf);

			par_img.reset();
			piv_simu.ExportParticleImage(par_img);

			for(int h = 0;h < center_height;h++)
			{
				for(int w = 0;w < center_width;w++)
				{

					cut_par_img.data()[h*center_width+w] = par_img.data()[(h+boarder_size)*width+w+boarder_size];
				}
			}

			cv::Mat show_par_img = ParImageToIplImage(cut_par_img);
			sprintf_s(buf,"%s\\par_%d.png",out_par_fold,frame);
			cv::imwrite(buf,show_par_img);
			cv::namedWindow("show");
			cv::imshow("show", show_par_img);
			cv::waitKey(10);
		}
		else
		{
			piv_simu.ExportVelocity(u_img,v_img);

			cv::Mat show_flow_img = ZQ_ImageIO::SaveFlowToColorImage(u_img,v_img,true,12,64,1);
			sprintf_s(buf,"%s\\flow_%d.png",out_flow_fold,frame);
			cv::imwrite(buf,show_flow_img);
			
			flow.assemble(u_img,v_img);
			sprintf_s(buf,"%s\\flow_%d.di2",out_flow_fold,frame);
			flow.saveImage(buf);

			par_img.reset();
			piv_simu.ExportParticleImage(par_img);

			cv::Mat show_par_img = ParImageToIplImage(par_img);
			sprintf_s(buf,"%s\\par_%d.png",out_par_fold,frame);
			cv::imwrite(buf,show_par_img);
			cv::namedWindow("show");
			cv::imshow("show", show_par_img);
			cv::waitKey(10);
		}
		
		piv_simu.RunOneFrame(1.0,use_peroid_coord,true);
		frame++;

	} while (frame < 400);

	if(mvobj)
		delete mvobj;
}


cv::Mat ParImageToIplImage(DImage& img)
{
	int width = img.width();
	int height = img.height();
	int nChannels = img.nchannels();

	if(width <= 0 || height <= 0 || nChannels != 1)
		return cv::Mat();

	BaseType*& pData = img.data();
	cv::Mat image = cv::Mat(height, width, CV_MAKETYPE(8, 1));
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			image.ptr<uchar>(i)[j] = pData[i*width + j] * 255;
		}
	}
	return image;
}