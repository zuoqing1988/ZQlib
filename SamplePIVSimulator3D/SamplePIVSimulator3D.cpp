#include "ZQ_PIVSimulator3D.h"
#include "ZQ_PIVSimulator.h"
#include "ZQ_PoissonSolver.h"
#include "ZQ_DoubleImage.h"

using namespace ZQ;
using namespace ZQ_PoissonSolver3D;
using namespace ZQ_PoissonSolver;

typedef ZQ_PIVMovingObject3D::BaseType BaseType;
typedef ZQ_DImage3D<BaseType> DImage3D;
typedef ZQ_DImage<BaseType> DImage;

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
	if(idx > 11 || idx < 0)
		return;
	
	int boarder_size = 16;
	int center_width = 64;
	int center_height = 64;
	int center_depth = 64;
	int width = center_width+boarder_size*2;
	int height = center_height+boarder_size*2;
	int depth = center_depth+boarder_size*2;
	bool cut_boarder = true;

	int data_index = idx;
	const static int BUF_LEN = 200;
	char out_flow_fold[BUF_LEN] = {0};
	char out_par_fold[BUF_LEN] = {0};
	int par_num = 0;
	double max_radius = 3.0;
	double min_radius = 2.0;
	double max_density = 0.8;
	double min_density = 0.6;
	int vort_num = 0;
	double max_vort = 1;
	double min_vort = 0.6;
	double max_vort_radius = 20;
	double min_vort_radius = 15;

	bool use_peroid_coord = false;

	double base_vel_u = 0;
	double base_vel_v = 0;
	double base_vel_w = 0;
	int skip_frames = 0;

	int coarse_len = 16;

	ZQ_PIVMovingObject3D* mvobj = 0;

	DImage3D par_mask(width,height,depth);

	switch(data_index)
	{
	case 0:
		strcpy_s(out_flow_fold, BUF_LEN,"flow3D0");
		strcpy_s(out_par_fold, BUF_LEN,"par3D0");
		srand(1000);
		par_num = 1000;
		vort_num = 10;
		max_vort = 0.8;
		min_vort = 0.5;
		max_vort_radius = 10;
		min_vort_radius = 5;
		use_peroid_coord = true;

		boarder_size = 8;
		center_width = 128;
		center_height = 64;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		coarse_len = 16;
		par_mask.allocate(width,height,depth);
		
		
		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 1:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D1");
		strcpy_s(out_par_fold, BUF_LEN,"par3D1");
		srand(2000);
		par_num = 1000;
		vort_num = 20;
		max_vort = 0.8;
		min_vort = 0.5;
		max_vort_radius = 10;
		min_vort_radius = 8;
		use_peroid_coord = true;
		
		boarder_size = 8;
		center_width = 128;
		center_height = 64;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);
		
		
		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 2:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D2");
		strcpy_s(out_par_fold, BUF_LEN,"par3D2");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		max_radius = 2;
		min_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CYLINDER_STATIC,0);

		base_vel_u = 5;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 3:
		strcpy_s(out_flow_fold, BUF_LEN,"flow3D3");
		strcpy_s(out_par_fold, BUF_LEN,"par3D3");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		min_radius = 2;
		max_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);
		
		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CIRCLE_STATIC,0);

		base_vel_u = 5;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;
	case 4:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D4");
		strcpy_s(out_par_fold, BUF_LEN, "par3D4");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		max_radius = 2;
		min_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CYLINDER_STATIC,0);

		base_vel_u = 3;
		base_vel_v = 1;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 5:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D5");
		strcpy_s(out_par_fold, BUF_LEN, "par3D5");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		max_radius = 2;
		min_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CIRCLE_STATIC,0);

		base_vel_u = 3;
		base_vel_v = 1;
		base_vel_w = 0;
		skip_frames = 20;
		break;
	case 6:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D6");
		strcpy_s(out_par_fold, BUF_LEN, "par3D6");
		srand(3000);
		par_num = 2000;
		vort_num = 20;
		max_vort = 0.8;
		min_vort = 0.3;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CYLINDER_STATIC,0);

		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 7:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D7");
		strcpy_s(out_par_fold, BUF_LEN,"par3D7");
		srand(3000);
		par_num = 3000;
		vort_num = 20;
		max_vort = 0.8;
		min_vort = 0.3;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CIRCLE_STATIC,0);

		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 8:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D8");
		strcpy_s(out_par_fold, BUF_LEN, "par3D8");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		min_radius = 2;
		max_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		coarse_len = 16;
		par_mask.allocate(width,height,depth);
		for(int k = 0;k < depth;k++)
		{
			for(int j = 0;j < height;j++)
			{
				for(int i = 0;i < width;i++)
				{
					if((i/coarse_len)%3 == 0)
					{
						par_mask.data()[k*height*width+j*width+i] = 1;
					}
				}
			}
		}

		base_vel_u = 3;
		base_vel_v = 1;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 9:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D9");
		strcpy_s(out_par_fold, BUF_LEN, "par3D9");
		srand(3000);
		par_num = 8000;
		vort_num = 20;
		max_vort = 1.6;
		min_vort = 0.8;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		min_radius = 2;
		max_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		coarse_len = 16;
		par_mask.allocate(width,height,depth);
		for(int k = 0;k < depth;k++)
		{
			for(int j = 0;j < height;j++)
			{
				for(int i = 0;i < width;i++)
				{
					if((i/coarse_len)%2 == 0)
					{
						par_mask.data()[k*height*width+j*width+i] = 1;
					}
				}
			}
		}

		base_vel_u = 3;
		base_vel_v = 1;
		base_vel_w = 0;
		skip_frames = 20;
		break;
	case 10:
		strcpy_s(out_flow_fold, BUF_LEN, "flow3D10");
		strcpy_s(out_par_fold, BUF_LEN, "par3D10");
		srand(3000);
		par_num = 20000;
		vort_num = 20;
		max_vort = 0.8;
		min_vort = 0.3;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		min_radius = 2;
		max_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		mvobj = new ZQ_PIVMovingObject3D(24,24,depth, ZQ_PIVMovingObject3D::ZQ_PIV_MOVOB_CYLINDER_STATIC,0);

		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	case 11:
		strcpy_s(out_flow_fold, BUF_LEN,"flow3D11");
		strcpy_s(out_par_fold, BUF_LEN,"par3D11");
		srand(3000);
		par_num = 20000;
		vort_num = 20;
		max_vort = 0.8;
		min_vort = 0.3;
		max_vort_radius = 20;
		min_vort_radius = 15;
		use_peroid_coord = true;

		min_radius = 2;
		max_radius = 2;

		boarder_size = 8;
		center_width = 128;
		center_height = 128;
		center_depth = 64;
		width = center_width + 2*boarder_size;
		height = center_height + 2*boarder_size;
		depth = center_depth + 2*boarder_size;

		par_mask.allocate(width,height,depth);

		base_vel_u = 3;
		base_vel_v = 0;
		base_vel_w = 0;
		skip_frames = 20;
		break;

	}

	/*suggest values*/
	/*
	par_num = width*height/3;
	vort_num = 30;
	*/

	DImage3D u_img(width,height,depth);
	DImage3D v_img(width,height,depth);
	DImage3D w_img(width,height,depth);
	DImage3D par_img(width,height,depth);

	bool has_flow = false;
	/*
	FILE* in = fopen(buf,"rb");
	
	if(in != 0)
	{
		fclose(in);
		ZQ_DImage3D tmpflow;
		if(has_flow = tmpflow.loadImage(buf))
		{
			tmpflow.separate(1,u_img,v_img);
			v_img.separate(1,tmpflow,w_img);
			v_img.copyData(tmpflow);
		}
	}*/
	
	if(!has_flow)
	{
		DImage vor2d(width-1,height-1);
		DImage macu2d(width+1,height);
		DImage macv2d(width,height+1);
		DImage u2d(width,height);
		DImage v2d(width,height);

		BaseType*& vort_pData = vor2d.data();
		BaseType*& macu_pData = macu2d.data();
		BaseType*& macv_pData = macv2d.data();
		BaseType*& u_pData = u2d.data();
		BaseType*& v_pData = v2d.data();


		//XOY
		for(int vor_it = 0; vor_it < vort_num; vor_it++)
		{
			double intensity = (rand()%2-0.5)*2.0*(rand()%101/100.0*(max_vort-min_vort)+min_vort);
			double radius = rand()%101/100.0*(max_vort_radius-min_vort_radius)+min_vort_radius;
			double posx = rand()%(width-2*(int)max_vort_radius) + max_vort_radius;
			double posy = rand()%(height-2*(int)max_vort_radius) + max_vort_radius;
			ZQ_PIVSimulator::DrawOneParticle(vor2d,posx,posy,intensity,radius);
		}

		ZQ_PoissonSolver::ReconstructCurlField(width,height,vort_pData,macu_pData,macv_pData,100,false);

		ZQ_PoissonSolver::MACtoRegularGrid(width,height,macu_pData,macv_pData,u_pData,v_pData);

		for(int z = 0;z < depth;z++)
		{
			for(int p = 0;p < height*width;p++)
			{
				u_img.data()[z*height*width+p] = u_pData[p];
				v_img.data()[z*height*width+p] = v_pData[p];
				w_img.data()[z*height*width+p] = 0;
			}
		}

		//XOZ
		float xoz_scale = 0.1;
		vor2d.allocate(width-1,depth-1);
		macu2d.allocate(width+1,depth);
		macv2d.allocate(width,depth+1);
		u2d.allocate(width,depth);
		v2d.allocate(width,depth);

		for(int vor_it = 0; vor_it < vort_num; vor_it++)
		{
			double intensity = 0.2*(rand()%2-0.5)*2.0*(rand()%101/100.0*(max_vort-min_vort)+min_vort);
			double radius = rand()%101/100.0*(max_vort_radius-min_vort_radius)+min_vort_radius;
			double posx = rand()%(width-2*(int)max_vort_radius) + max_vort_radius;
			double posy = rand()%(depth-2*(int)max_vort_radius) + max_vort_radius;
			ZQ_PIVSimulator::DrawOneParticle(vor2d,posx,posy,intensity,radius);
		}

		ZQ_PoissonSolver::ReconstructCurlField(width,depth,vort_pData,macu_pData,macv_pData,100,false);

		ZQ_PoissonSolver::MACtoRegularGrid(width,depth,macu_pData,macv_pData,u_pData,v_pData);

		for(int y = 0;y < height;y++)
		{
			for(int z = 0;z < depth;z++)
			{
				for(int x = 0;x < width;x++)
				{
					u_img.data()[z*height*width+y*width+x] += u_pData[z*width+x]*xoz_scale;
					v_img.data()[z*height*width+y*width+x] += 0;
					w_img.data()[z*height*width+y*width+x] += v_pData[z*width+x]*xoz_scale;
				}
			}
		}

		//YOZ
		float yoz_scale = 0.1;
		vor2d.allocate(height-1,depth-1);
		macu2d.allocate(height+1,depth);
		macv2d.allocate(height,depth+1);
		u2d.allocate(height,depth);
		v2d.allocate(height,depth);

		for(int vor_it = 0; vor_it < vort_num; vor_it++)
		{
			double intensity = 0.2*(rand()%2-0.5)*2.0*(rand()%101/100.0*(max_vort-min_vort)+min_vort);
			double radius = rand()%101/100.0*(max_vort_radius-min_vort_radius)+min_vort_radius;
			double posx = rand()%(height-2*(int)max_vort_radius) + max_vort_radius;
			double posy = rand()%(depth-2*(int)max_vort_radius) + max_vort_radius;
			ZQ_PIVSimulator::DrawOneParticle(vor2d,posx,posy,intensity,radius);
		}

		ZQ_PoissonSolver::ReconstructCurlField(height,depth,vort_pData,macu_pData,macv_pData,100,false);

		ZQ_PoissonSolver::MACtoRegularGrid(height,depth,macu_pData,macv_pData,u_pData,v_pData);

		for(int x = 0;x < width;x++)
		{
			for(int z = 0;z < depth;z++)
			{
				for(int y = 0;y < height;y++)
				{
					u_img.data()[z*height*width+y*width+x] += 0;
					v_img.data()[z*height*width+y*width+x] += u_pData[z*height+y]*yoz_scale;
					w_img.data()[z*height*width+y*width+x] += v_pData[z*height+y]*yoz_scale;
				}
			}
		}

		//ZQ_DImage3D tmpflow;
		//tmpflow.assemble(u_img,v_img,w_img);
		//tmpflow.saveImage(buf);
	}
	
	char cmdbuf[500];
	sprintf_s(cmdbuf,"if not exist \"%s\" mkdir \"%s\"",out_par_fold,out_par_fold);
	system(cmdbuf);
	sprintf_s(cmdbuf,"if not exist \"%s\" mkdir \"%s\"",out_flow_fold,out_flow_fold);
	system(cmdbuf);

	for(int i = 0;i < width*height*depth;i++)
	{
		u_img.data()[i] += base_vel_u;
		v_img.data()[i] += base_vel_v;
		w_img.data()[i] += base_vel_w;
	}

	ZQ_PIVSimulator3D piv_simu(width,height,depth,mvobj);

	piv_simu.RandomInit(par_num,u_img,v_img,w_img,par_mask);


	int frame = 0;
	char buf[200];

	DImage3D flow;

	DImage3D cut_u_img(center_width,center_height,center_depth);
	DImage3D cut_v_img(center_width,center_height,center_depth);
	DImage3D cut_w_img(center_width,center_height,center_depth);
	DImage3D cut_par_img(center_width,center_height,center_depth);

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
			piv_simu.ExportVelocity(u_img,v_img,w_img);

			for(int d = 0;d < center_depth;d++)
			{
				for(int h = 0; h < center_height;h++)
				{
					for(int w = 0;w < center_width;w++)
					{
						cut_u_img.data()[d*center_height*center_width+h*center_width+w] = u_img.data()[(d+boarder_size)*height*width+(h+boarder_size)*width+w+boarder_size];
						cut_v_img.data()[d*center_height*center_width+h*center_width+w] = v_img.data()[(d+boarder_size)*height*width+(h+boarder_size)*width+w+boarder_size];
						cut_w_img.data()[d*center_height*center_width+h*center_width+w] = w_img.data()[(d+boarder_size)*height*width+(h+boarder_size)*width+w+boarder_size];
					}
				}
			}

			flow.assemble(cut_u_img,cut_v_img,cut_w_img);
			sprintf_s(buf,"%s\\flow_%d.di3",out_flow_fold,frame);
			flow.saveImage(buf);

			par_img.reset();
			piv_simu.ExportParticleImage(par_img);

			for(int d = 0;d < center_depth;d++)
			{
				for(int h = 0;h < center_height;h++)
				{
					for(int w = 0;w < center_width;w++)
					{
						cut_par_img.data()[d*center_height*center_width+h*center_width+w] = par_img.data()[(d+boarder_size)*height*width+(h+boarder_size)*width+w+boarder_size];
					}
				}
			}
			
			sprintf_s(buf,"%s\\par_%d.di3",out_par_fold,frame);
			cut_par_img.saveImage(buf);
		}
		else
		{
			piv_simu.ExportVelocity(u_img,v_img,w_img);

			flow.assemble(u_img,v_img,w_img);
			sprintf_s(buf,"%s\\flow_%d.di3",out_flow_fold,frame);
			flow.saveImage(buf);

			par_img.reset();
			piv_simu.ExportParticleImage(par_img);

			
			sprintf_s(buf,"%s\\par_%d.di3",out_par_fold,frame);
			par_img.saveImage(buf);

		}

		piv_simu.RunOneFrame(1.0,use_peroid_coord,true);
		frame++;

	} while (frame < 100);

	if(mvobj)
		delete mvobj;


}
