#include "ZQ_Advection.h"
#include "ZQ_PoissonSolver.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include "opencv\cv.h"
#include "opencv\highgui.h"

using namespace ZQ;
using namespace ZQ_PoissonSolver;
using namespace ZQ_Advection;

#define open_boundary 1
#define closed_boundary 2
#define with_occupy 4
#define without_occupy 8
#define sor_solver 16
#define redblack_solver 32

const int OPEN_POISSON_WITHOUT_OCCUPY	= open_boundary | without_occupy ;
const int OPEN_POISSON_WITH_OCCUPY	= open_boundary | with_occupy;
const int OPEN_POISSON_SOR_WITHOUT_OCCUPY = open_boundary | sor_solver | without_occupy;
const int OPEN_POISSON_SOR_WITH_OCCUPY = open_boundary | sor_solver | with_occupy;
const int OPEN_POISSON_REDBLACK_WITHOUT_OCCUPY = open_boundary | redblack_solver | without_occupy;
const int OPEN_POISSON_REDBLACK_WITH_OCCUPY = open_boundary | redblack_solver | with_occupy;
const int CLOSED_POISSON_WITHOUT_OCCUPY = closed_boundary | without_occupy;
const int CLOSED_POISSON_WITH_OCCUPY = closed_boundary | with_occupy;
const int CLOSED_POISSON_SOR_WITHOUT_OCCUPY = closed_boundary | sor_solver | without_occupy;
const int CLOSED_POISSON_SOR_WITH_OCCUPY = closed_boundary | sor_solver | with_occupy;

#define MyType float

int solver_type = -1;// OPEN_POISSON_WITH_OCCUPY;


bool is_open_boundary = true;
bool add_boundary_occupy_for_closed = true;
bool use_sor_for_open = true;
bool use_redblack_for_sor = false;

bool display_running_info = true;

float buoy_coeff = 5;
float fconf_coeff = 200;
float diffuse_temperature_coeff = 10;
float vel_atten_coeff = 10;
float temp_atten_coeff = 0;
int substeps = 30;
int max_iter = 200;
float source_center_x = 0.5;
float source_center_y = 0.05;
float source_half_xlen = 0.05;
float source_half_ylen = 0.05;
float dt = 0.01;


int width = 256;
int height = 256;
bool* occupy = 0;
MyType* density = 0;
MyType* temperature = 0;
MyType* mac_u = 0;
MyType* mac_v = 0;

taucs_ccs_matrix* A = 0;

void init(int w,int h);

bool initFromImage(const char* file);

void shutdown();

void reinject();
void diffuse(float dt);
void addForce(float dt);
void attenuation(float dt);
void advectVelocity(float dt);
void projection();
void advectScalar(float dt);


void show(float wait_time);

int main()
{
	if(!initFromImage("scene.jpg"))
	{
		init(64,128);
	}
	printf("solver type = %d\n",solver_type);
	for(int i = 0;i < 100000;i++)
	{
		reinject();
		diffuse(dt);
		addForce(dt);
		//show(0);
		attenuation(dt);
		advectVelocity(dt);
		//show(0);
		projection();
		//show(0);
		advectScalar(dt);
		show(10);
	}
	shutdown();
	return EXIT_SUCCESS;
}

void init(int w, int h)
{
	width = w;
	height = h;
	density = new MyType[width*height];
	temperature = new MyType[width*height];
	mac_u = new MyType[(width+1)*height];
	mac_v = new MyType[width*(height+1)];

	memset(density,0,sizeof(MyType)*width*height);
	memset(temperature,0,sizeof(MyType)*width*height);
	memset(mac_u,0,sizeof(MyType)*(width+1)*height);
	memset(mac_v,0,sizeof(MyType)*width*(height+1));

	if(!is_open_boundary)
	{
		if(add_boundary_occupy_for_closed)
		{
			occupy = new bool[width*height];
			memset(occupy,0,sizeof(bool)*width*height);
			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					if(i == 0 || i == height-1 || j == 0 || j == width-1)
						occupy[i*width+j] = true;
				}
			}
			BuildClosedPoisson<MyType>(width,height,occupy,&A,display_running_info);
			solver_type = CLOSED_POISSON_WITH_OCCUPY;
		}
		else
		{
			BuildClosedPoisson<MyType>(width,height,&A,display_running_info);
			solver_type = CLOSED_POISSON_WITHOUT_OCCUPY;
		}
	}
	else
	{
		if(add_boundary_occupy_for_closed)
		{
			occupy = new bool[width*height];
			memset(occupy,0,sizeof(bool)*width*height);
			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					if(/*i == 0 ||*/ i == height-1 || j == 0 || j == width-1)
						occupy[i*width+j] = true;
				}
			}
			if(use_sor_for_open)
			{
				solver_type = use_redblack_for_sor ? OPEN_POISSON_REDBLACK_WITH_OCCUPY : OPEN_POISSON_SOR_WITH_OCCUPY;
			}
			else
			{
				BuildOpenPoisson<MyType>(width,height,occupy,&A,display_running_info);
				solver_type = OPEN_POISSON_WITH_OCCUPY;
			}

		}
		else
		{
			if(use_sor_for_open)
			{
				solver_type = use_redblack_for_sor ? OPEN_POISSON_REDBLACK_WITHOUT_OCCUPY : OPEN_POISSON_SOR_WITHOUT_OCCUPY;
			}
			else
			{
				BuildOpenPoisson<MyType>(width,height,&A,display_running_info);
				solver_type = OPEN_POISSON_WITHOUT_OCCUPY;
			}

		}
	}
}


bool initFromImage(const char* file)
{
	cv::Mat img = cv::imread(file, 0);
	if (img.empty())
		return false;

	width = img.cols;
	height = img.rows;

	density = new MyType[width*height];
	temperature = new MyType[width*height];
	mac_u = new MyType[(width+1)*height];
	mac_v = new MyType[width*(height+1)];

	memset(density,0,sizeof(MyType)*width*height);
	memset(temperature,0,sizeof(MyType)*width*height);
	memset(mac_u,0,sizeof(MyType)*(width+1)*height);
	memset(mac_v,0,sizeof(MyType)*width*(height+1));

	occupy = new bool[width*height];
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
			occupy[i*width+j] = *(img.data + (height-1-i)*img.step[0] + j) > 128;
	}

	if(is_open_boundary)
	{
		if(use_sor_for_open)
		{
			solver_type = use_redblack_for_sor ? OPEN_POISSON_REDBLACK_WITH_OCCUPY : OPEN_POISSON_SOR_WITH_OCCUPY;
		}
		else
		{
			BuildOpenPoisson<MyType>(width,height,occupy,&A,display_running_info);
			solver_type = OPEN_POISSON_REDBLACK_WITH_OCCUPY;
		}

	}
	else
	{
		if(add_boundary_occupy_for_closed)
		{
			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					if(i == 0 || i == height-1 || j == 0 || j == width-1)
						occupy[i*width+j] = true;
				}
			}
		}

		BuildClosedPoisson<MyType>(width,height,occupy,&A,display_running_info);
		solver_type = CLOSED_POISSON_WITH_OCCUPY;
	}


	return true;
}

void shutdown()
{
	delete []density;
	delete []temperature;
	delete []mac_u;
	delete []mac_v;
	density = 0;
	temperature = 0;
	mac_u = 0;
	mac_v = 0;

	if(occupy)
	{
		delete []occupy;
		occupy = 0;
	}

	if(A)
	{
		ZQ_TaucsBase::ZQ_taucs_ccs_free(A);
		A = 0;
	}
}

void reinject()
{
	float cx = source_center_x*width;
	float cy = source_center_y*height;
	float x_half_len = source_half_xlen*width;
	float y_half_len = source_half_ylen*height;

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(fabs(j-cx) <= x_half_len && fabs(i-cy) <= y_half_len)
			{
				density[i*width+j] = 0.8;
				temperature[i*width+j] = 20;
			}
		}
	}
}

void diffuse(float dt)
{
	ZQ::ZQ_DImage<MyType> temp(width,height);
	MyType*& temp_ptr = temp.data();
	memcpy(temp_ptr,temperature,sizeof(MyType)*width*height);
	temp.GaussianSmoothing(4,4);

	MyType diffuse_temp_coeff = exp(-dt*diffuse_temperature_coeff);

	for(int i = 0;i < width*height;i++)
		temperature[i] = temperature[i]* diffuse_temp_coeff + (1-diffuse_temp_coeff)*temp_ptr[i];
}

void addForce(float dt)
{
	float voxel_len = 1.0/width;

	MyType* force = new MyType[width*height*2];
	memset(force,0,sizeof(MyType)*width*height*2);

	ZQ_DImage<MyType> u(width,height),v(width,height);
	ZQ_DImage<MyType> vortVector(width,height);
	ZQ_DImage<MyType> vortScale(width,height);
	ZQ_DImage<MyType> vort_gx(width,height);
	ZQ_DImage<MyType> vort_gy(width,height);

	MyType* uPtr = u.data();
	MyType* vPtr = v.data();
	MyType* vortVectorPtr = vortVector.data();
	MyType* vortScalePtr = vortScale.data();
	MyType* vort_gxPtr = vort_gx.data();
	MyType* vort_gyPtr = vort_gy.data();

	MACtoRegularGrid(width,height,mac_u,mac_v,uPtr,vPtr);

	/*calculate vorticity vector and scale*/
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(i == 0 || i == height-1 || j == 0 || j == width-1)
			{
			}
			else
			{
				vortVectorPtr[i*width+j] = (vPtr[i*width+j+1] - vPtr[i*width+j-1] - uPtr[(i+1)*width+j] + uPtr[(i-1)*width+j])/(2.0f*voxel_len);
			}
		}
	}

	//vortVector.GaussianSmoothing(4,4);

	for(int i = 0;i < height*width;i++)
		vortScalePtr[i] = fabs(vortVectorPtr[i]);


	/*calculate gradients of vorticity scale*/
	vortScale.dx(vort_gx,true);
	vortScale.dy(vort_gy,true);
	for(int i = 0;i < height*width;i++)
	{
		float len = sqrt(vort_gxPtr[i]*vort_gxPtr[i] + vort_gyPtr[i]*vort_gyPtr[i]);
		if(len != 0)
		{
			vort_gxPtr[i] /= len;
			vort_gyPtr[i] /= len;
		}
	}


	/*fconf = eta * deltat *(gradVort X vortVector)*/
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			int offset = i*width+j;
			force[offset*2+0] += fconf_coeff * voxel_len * vort_gyPtr[offset]*vortVectorPtr[offset];
			force[offset*2+1] += fconf_coeff * voxel_len * vort_gxPtr[offset]*vortVectorPtr[offset];
		}
	}


	////debug show
	//ZQ::ZQ_DImage<MyType> flow(width,height,2),uu(width,height),vv(width,height);
	//MyType*& flow_ptr = flow.data();
	//for(int i = 0;i < height;i++)
	//{
	//	for(int j = 0;j < width;j++)
	//	{
	//		flow_ptr[(i*width+j)*2+0] = force[((height-1-i)*width+j)*2+0];
	//		flow_ptr[(i*width+j)*2+1] = force[((height-1-i)*width+j)*2+1];
	//	}
	//}
	//flow.separate(1,uu,vv);
	//IplImage* showimg = ZQ::ZQ_ImageIO::SaveFlowToColorImage(uu,vv,false,0,64,1);
	//cvNamedWindow("force_flow");
	//cvShowImage("force_flow",showimg);
	//cvWaitKey(0);
	//cvReleaseImage(&showimg);

	//buoyancy
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			int offset = i*width+j;
			force[offset*2+1] += buoy_coeff*temperature[offset];
		}
	}

	if(occupy == 0)
	{
		// update mac_u
		for(int i = 0;i < height;i++)
		{
			for(int j = 0;j <= width;j++)
			{
				if(j == 0)
				{
					mac_u[i*(width+1)+j] += dt * force[(i*width+j)*2+0];
				}
				else if(j == width)
				{
					mac_u[i*(width+1)+j] += dt * force[(i*width+j-1)*2+0];
				}
				else
				{
					mac_u[i*(width+1)+j] += dt * 0.5 * (force[(i*width+j)*2+0]+force[(i*width+j-1)*2+0]);
				}
			}
		}

		//update mac_v
		for(int i = 0;i <= height;i++)
		{
			for(int j = 0;j < width;j++)
			{
				if(i == 0)
				{
					mac_v[i*width+j] += dt * force[(i*width+j)*2+1];
				}
				else if(i == height)
				{
					mac_v[i*width+j] += dt * force[((i-1)*width+j)*2+1];
				}
				else
				{
					mac_v[i*width+j] += dt * 0.5* (force[(i*width+j)*2+1]+force[((i-1)*width+j)*2+1]);
				}
			}
		}
	}
	else
	{
		// update mac_u
		for(int i = 0;i < height;i++)
		{
			for(int j = 0;j <= width;j++)
			{
				if(j == 0)
				{
					if(!occupy[i*width+j])
						mac_u[i*(width+1)+j] += dt * force[(i*width+j)*2+0];
				}
				else if(j == width)
				{
					if(!occupy[i*width+j-1])
						mac_u[i*(width+1)+j] += dt * force[(i*width+j-1)*2+0];
				}
				else
				{
					if(!occupy[i*width+j] && !occupy[i*width+j-1])
						mac_u[i*(width+1)+j] += dt * 0.5 * (force[(i*width+j)*2+0]+force[(i*width+j-1)*2+0]);
				}
			}
		}

		//update mac_v
		for(int i = 0;i <= height;i++)
		{
			for(int j = 0;j < width;j++)
			{
				if(i == 0)
				{
					if(!occupy[i*width+j])
						mac_v[i*width+j] += dt * force[(i*width+j)*2+1];
				}
				else if(i == height)
				{
					if(!occupy[(i-1)*width+j])
						mac_v[i*width+j] += dt * force[((i-1)*width+j)*2+1];
				}
				else
				{
					if(!occupy[i*width+j] && !occupy[(i-1)*width+j])
						mac_v[i*width+j] += dt * 0.5* (force[(i*width+j)*2+1]+force[((i-1)*width+j)*2+1]);
				}
			}
		}
	}

	if(!is_open_boundary)
	{
		for(int i = 0;i < height;i++)
		{
			mac_u[i*(width+1)+0] = 0;
			mac_u[i*(width+1)+width] = 0;
		}

		for(int j = 0;j < width;j++)
		{
			mac_v[0*width+j] = 0;
			mac_v[height*width+j] = 0;
		}
	}

	delete []force;
}

void attenuation(float dt)
{
	float attenTemp = exp(-dt * temp_atten_coeff);
	for(int i = 0;i < height*width;i++)
	{
		temperature[i] *= attenTemp;
	}

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(i == 0 || i == height-1 || j == 0 || j == width-1)
			{
				temperature[i*width+j] = 0;
				density[i*width+j] = 0;
			}
		}
	}

	float attenVel = exp(-dt * vel_atten_coeff);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j <= width;j++)
		{
			if(occupy == 0)
			{
				mac_u[i*(width+1)+j] *= attenVel;
			}
			else
			{
				if(j == 0 && occupy[i*width+j])
				{

				}
				else if(j == width && occupy[i*width+j-1])
				{

				}
				else if((j > 0 && j < width) && (occupy[i*width+j] || occupy[i*width+j-1]))
				{

				}
				else
					mac_u[i*(width+1)+j] *= attenVel;
			}
		}
	}

	for(int i = 0;i <= height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(occupy == 0)
			{
				mac_v[i*width+j] *= attenVel;
			}
			else
			{
				if(i == 0 && occupy[i*width+j])
				{

				}
				else if(i == height && occupy[(i-1)*width+j])
				{

				}
				else if(i > 0 && i < height && (occupy[i*width+j] || occupy[(i-1)*width+j]))
				{

				}
				else
					mac_v[i*width+j] *= attenVel;
			}
		}
	}
}


void advectVelocity(float dt)
{
	float voxel_len = 1.0/width;

	MyType* out_mac_u = new MyType[(width+1)*height];
	MyType* out_mac_v = new MyType[width*(height+1)];
	memcpy(out_mac_u,mac_u,sizeof(MyType)*(width+1)*height);
	memcpy(out_mac_v,mac_v,sizeof(MyType)*width*(height+1));

	int nPts = (width+1)*height;
	MyType* in_pos = new MyType[nPts*2];
	MyType* out_pos = new MyType[nPts*2];
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j <= width;j++)
		{
			int offset = i*(width+1)+j;
			in_pos[offset*2+0] = j*voxel_len;
			in_pos[offset*2+1] = (i+0.5f)*voxel_len;
		}
	}

	ZQ_BacktraceAdvection_MACGrid(mac_u,mac_v,occupy,width,height,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j <= width;j++)
		{
			if(j == 0 && occupy && occupy[i*width+j])
				continue;
			if(j == width && occupy && occupy[i*width+j-1])
				continue;
			if(j > 0 && j < width && occupy && (occupy[i*width+j] || occupy[i*width+j-1]))
				continue;

			int offset = i*(width+1)+j;
			float cur_x = out_pos[offset*2+0]/voxel_len;
			float cur_y = out_pos[offset*2+1]/voxel_len;
			ZQ::ZQ_ImageProcessing::BilinearInterpolate(mac_u,width+1,height,1,cur_x,cur_y-0.5f,out_mac_u+offset,false);
		}
	}

	delete []in_pos;
	delete []out_pos;

	nPts = width*(height+1);
	in_pos = new MyType[nPts*2];
	out_pos = new MyType[nPts*2];
	for(int i = 0;i <= height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			int offset = i*width+j;
			in_pos[offset*2+0] = (j+0.5f)*voxel_len;
			in_pos[offset*2+1] = i*voxel_len;
		}
	}

	ZQ_BacktraceAdvection_MACGrid(mac_u,mac_v,occupy,width,height,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int i = 0;i <= height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(i == 0 && occupy &&  occupy[i*width+j])
				continue;
			if(i == height && occupy &&occupy[(i-1)*width+j])
				continue;
			if(i > 0 && i < height && occupy &&(occupy[i*width+j] || occupy[(i-1)*width+j]))
				continue;

			int offset = i*width+j;
			float cur_x = out_pos[offset*2+0]/voxel_len;
			float cur_y = out_pos[offset*2+1]/voxel_len;
			ZQ::ZQ_ImageProcessing::BilinearInterpolate(mac_v,width,height+1,1,cur_x-0.5f,cur_y,out_mac_v+offset,false);
		}
	}

	delete []in_pos;
	delete []out_pos;

	memcpy(mac_u,out_mac_u,sizeof(MyType)*(width+1)*height);
	memcpy(mac_v,out_mac_v,sizeof(MyType)*width*(height+1));
	delete []out_mac_u;
	delete []out_mac_v;
}

void projection()
{

	switch(solver_type)
	{
	case OPEN_POISSON_WITHOUT_OCCUPY:
		SolveOpenPoisson_MACGrid(mac_u,mac_v,width,height,A,max_iter,display_running_info);
		break;
	case OPEN_POISSON_WITH_OCCUPY:
		SolveOpenPoisson_MACGrid(mac_u,mac_v,width,height,occupy,A,max_iter,display_running_info);
		break;
	case OPEN_POISSON_SOR_WITHOUT_OCCUPY:
		SolveOpenPoissonSOR_MACGrid(mac_u,mac_v,width,height,max_iter,display_running_info);
		break;
	case OPEN_POISSON_SOR_WITH_OCCUPY:
		SolveOpenPoissonSOR_MACGrid(mac_u,mac_v,width,height,occupy,max_iter,display_running_info);
		break;
	case OPEN_POISSON_REDBLACK_WITHOUT_OCCUPY:
		SolveOpenPoissonRedBlack_MACGrid(mac_u,mac_v,width,height,max_iter,display_running_info);
		break;
	case OPEN_POISSON_REDBLACK_WITH_OCCUPY:
		SolveOpenPoissonRedBlack_MACGrid(mac_u,mac_v,width,height,occupy,max_iter,display_running_info);
		break;
	case CLOSED_POISSON_WITHOUT_OCCUPY:
		SolveClosedPoisson_MACGrid(mac_u,mac_v,width,height,A,max_iter,display_running_info);
		break;
	case CLOSED_POISSON_WITH_OCCUPY:
		SolveClosedPoisson_MACGrid(mac_u,mac_v,width,height,occupy,A,max_iter,display_running_info);
		break;
	default:
		printf("unknown solver\n");
		break;
	}
}

void advectScalar(float dt)
{
	float voxel_len = 1.0/width;
	MyType* out_density = new MyType[width*height];
	MyType* out_temperature = new MyType[width*height];
	memcpy(out_density,density,sizeof(MyType)*width*height);
	memcpy(out_temperature,temperature,sizeof(MyType)*width*height);

	int nPts = width*height;
	MyType* in_pos = new MyType[nPts*2];
	MyType* out_pos = new MyType[nPts*2];

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			int offset = i*width+j;
			in_pos[offset*2+0] = (j+0.5f)*voxel_len;
			in_pos[offset*2+1] = (i+0.5f)*voxel_len;
		}
	}

	ZQ_BacktraceAdvection_MACGrid(mac_u,mac_v,occupy,width,height,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			if(occupy && occupy[i*width+j])
				continue;

			int offset = i*width+j;
			float cur_x = out_pos[offset*2+0]/voxel_len;
			float cur_y = out_pos[offset*2+1]/voxel_len;
			ZQ::ZQ_ImageProcessing::BilinearInterpolate(density,width,height,1,cur_x-0.5,cur_y-0.5,out_density+offset,false);
			ZQ::ZQ_ImageProcessing::BilinearInterpolate(temperature,width,height,1,cur_x-0.5,cur_y-0.5,out_temperature+offset,false);
		}
	}

	delete []in_pos;
	delete []out_pos;

	memcpy(density,out_density,sizeof(MyType)*width*height);
	memcpy(temperature,out_temperature,sizeof(MyType)*width*height);
	delete []out_density;
	delete []out_temperature;
}

void show(float wait_time)
{
	cv::Mat show_img = cv::Mat(height, width, CV_MAKETYPE(8, 3));
	
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			int offset = i*width+j;
			if(occupy && occupy[offset])
			{
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 0] = 0;
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 1] = 255;
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 2] = 0;
			}
			else
			{
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 0] = density[offset] * 255;
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 1] = density[offset] * 255;
				show_img.ptr<uchar>(height - 1 - i)[j * 3 + 2] = density[offset] * 255;
			}	
		}
	}

	const float max_div = 0.1;
	const float divshow_scale = 255.0/max_div;
	cv::Mat div_img = cv::Mat(height, width, CV_MAKETYPE(8, 3));
	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			float cur_div = mac_u[i*(width+1)+j+1] - mac_u[i*(width+1)+j] + mac_v[(i+1)*width+j] - mac_v[i*width+j];
			cur_div *= divshow_scale;
			if (cur_div > 0)
			{
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 0] = 0;
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 1] = cur_div;
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 2] = 0;
			}
			else
			{
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 0] = 0;
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 1] = 0;
				div_img.ptr<uchar>(height - 1 - i)[j * 3 + 2] = -cur_div;
			}
		}
	}


	ZQ_DImage<MyType> uu(width,height),vv(width,height);

	for(int i = 0;i < height;i++)
	{
		for(int j = 0;j < width;j++)
		{
			uu.data()[(height-1-i)*width+j] = 0.5*( mac_u[i*(width+1)+j] + mac_u[i*(width+1)+j+1]);
			vv.data()[(height-1-i)*width+j] = -0.5*( mac_v[i*width+j] + mac_v[(i+1)*width+j]);
		}
	}
	cv::Mat flow_img = ZQ::ZQ_ImageIO::SaveFlowToColorImage(uu,vv,false,0,64,1,display_running_info);
	
	
	cv::Mat scaled_img;
	float scale = 4;
	cv::resize(flow_img, scaled_img, cv::Size(), scale, scale);
	cv::namedWindow("flow");
	cv::imshow("flow", scaled_img);

	cv::resize(div_img, scaled_img, cv::Size(), scale, scale);
	cv::namedWindow("div");
	cv::imshow("div",scaled_img);

	cv::resize(show_img, scaled_img, cv::Size(), scale, scale);
	cv::imshow("show",scaled_img);
	cvWaitKey(wait_time);
}
