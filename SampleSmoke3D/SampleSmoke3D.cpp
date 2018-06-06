#include "ZQ_PoissonSolver3D.h"
#include "ZQ_DoubleImage3D.h"
#include "ZQ_Advection3D.h"
#include "ZQ_CPURaycasting.h"
#include "opencv\cv.h"
#include "opencv\highgui.h"

using namespace ZQ;
using namespace ZQ_PoissonSolver3D;
using namespace ZQ_Advection3D;

#define OPEN_POISSON_WITHOUT_OCCUPY					0
#define OPEN_POISSON_WITH_OCCUPY					1
#define OPEN_POISSON_SOR_WITHOUT_OCCUPY				2
#define OPEN_POISSON_SOR_WITH_OCCUPY				3
#define OPEN_POISSON_REDBLACK_WITHOUT_OCCUPY		4
#define OPEN_POISSON_REDBLACK_WITH_OCCUPY			5
#define CLOSED_POISSON_WITHOUT_OCCUPY				6
#define CLOSED_POISSON_WITH_OCCUPY					7

#define MyType float

int solver_type = OPEN_POISSON_WITHOUT_OCCUPY;


bool is_open_boundary = true;
bool add_boundary_occupy_for_closed = true;
bool use_sor_for_open = true;
bool use_redblack_for_sor = true;

bool display_running_info = true;

float buoy_coeff = 1;
float fconf_coeff = 50;
float diffuse_temperature_coeff = 0;
float vel_atten_coeff = 3;
float temp_atten_coeff = 0;
int substeps = 10;
int max_iter = 20;
float source_center_x = 0.5;
float source_center_y = 0.05;
float source_center_z = 0.5;
float source_half_xlen = 0.1;
float source_half_ylen = 0.03;
float source_half_zlen = 0.1;
float dt = 0.03;


int width = 32;
int height = 32;
int depth = 32;
bool* occupy = 0;
MyType* density = 0;
MyType* temperature = 0;
MyType* mac_u = 0;
MyType* mac_v = 0;
MyType* mac_w = 0;

taucs_ccs_matrix* A = 0;

unsigned int win_width = 400;
unsigned int win_height = 400;

float* volumeData = 0;
ZQ_CPURaycasting m_CPURaycast;


void init(int w,int h, int depth);
bool initFromImage(const char* file);
void initRaycating();
void shutdown();

void reinject();
void diffuse(float dt);
void addForce(float dt);
void attenuation(float dt);
void advectVelocity(float dt);
void projection();
void advectScalar(float dt);


void show(float wait_time);

void main()
{
	//if(!initFromImage("scene.jpg"))
	{
		init(32,32,32);
	}

	printf("solver type = %d\n",solver_type);

	for(int i = 0;i < 100000;i++)
	{
		printf("frame [%d]\n",i);
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
}

void init(int w, int h, int d)
{
	width = w;
	height = h;
	depth = d;
	density = new MyType[width*height*depth];
	temperature = new MyType[width*height*depth];
	mac_u = new MyType[(width+1)*height*depth];
	mac_v = new MyType[width*(height+1)*depth];
	mac_w = new MyType[width*height*(depth+1)];

	memset(density,0,sizeof(MyType)*width*height*depth);
	memset(temperature,0,sizeof(MyType)*width*height*depth);
	memset(mac_u,0,sizeof(MyType)*(width+1)*height*depth);
	memset(mac_v,0,sizeof(MyType)*width*(height+1)*depth);
	memset(mac_w,0,sizeof(MyType)*width*height*(depth+1));

	if(!is_open_boundary)
	{
		if(add_boundary_occupy_for_closed)
		{
			occupy = new bool[width*height*depth];
			memset(occupy,0,sizeof(bool)*width*height*depth);
			for(int k = 0;k < depth;k++)
			{
				for(int j = 0;j < height;j++)
				{
					for(int i = 0;i < width;i++)
					{
						if(k == 0 || k == depth-1 || j == 0 || j == height-1 || i == 0 || i == width-1)
							occupy[k*height*width+j*width+i] = true;
					}
				}
			}

			BuildClosedPoisson<MyType>(width,height,depth,occupy,&A,display_running_info);
			solver_type = CLOSED_POISSON_WITH_OCCUPY;
		}
		else
		{
			BuildClosedPoisson<MyType>(width,height,depth,&A,display_running_info);
			solver_type = CLOSED_POISSON_WITHOUT_OCCUPY;
		}
	}
	else
	{
		if(add_boundary_occupy_for_closed)
		{
			occupy = new bool[width*height*depth];
			memset(occupy,0,sizeof(bool)*width*height*depth);
			for(int k = 0;k < depth;k++)
			{
				for(int j = 0;j < height;j++)
				{
					for(int i = 0;i < width;i++)
					{
						if(k == 0 || k == depth-1 || /*j == 0 ||*/ j == height-1 || i == 0 || i == width-1)
							occupy[k*height*width+j*width+i] = true;
					}
				}
			}

			if(use_sor_for_open)
			{
				solver_type = use_redblack_for_sor ? OPEN_POISSON_REDBLACK_WITH_OCCUPY : OPEN_POISSON_SOR_WITH_OCCUPY;
			}
			else
			{
				BuildOpenPoisson<MyType>(width,height,depth,occupy,&A,display_running_info);
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
				BuildOpenPoisson<MyType>(width,height,depth,&A,display_running_info);
				solver_type = OPEN_POISSON_WITHOUT_OCCUPY;
			}

		}
	}

	initRaycating();
}


bool initFromImage(const char* file)
{
	IplImage* img = cvLoadImage(file,0);
	if(img == 0)
		return false;

	width = img->width;
	height = img->height;
	depth = width;

	density = new MyType[width*height*depth];
	temperature = new MyType[width*height*depth];
	mac_u = new MyType[(width+1)*height*depth];
	mac_v = new MyType[width*(height+1)*depth];
	mac_w = new MyType[width*height*(depth+1)];

	memset(density,0,sizeof(MyType)*width*height*depth);
	memset(temperature,0,sizeof(MyType)*width*height*depth);
	memset(mac_u,0,sizeof(MyType)*(width+1)*height*depth);
	memset(mac_v,0,sizeof(MyType)*width*(height+1)*depth);
	memset(mac_w,0,sizeof(MyType)*width*height*(depth+1));

	occupy = new bool[width*height*depth];
	for(int j = 0;j < height;j++)
	{
		for(int i = 0;i < width;i++)
		{
			bool cur_occ = cvGetReal2D(img,height-1-j,i) > 128;
			for(int k = 0;k < depth;k++)
				occupy[k*height*width+j*width+i] = cur_occ;
		}
	}

	if(is_open_boundary)
	{
		if(add_boundary_occupy_for_closed)
		{
			for(int k = 0;k < depth;k++)
			{
				for(int j = 0;j < height;j++)
				{
					for(int i = 0;i < width;i++)
					{
						if(k == 0 || k == depth-1 || /*j == 0 ||*/ j == height-1 || i == 0 || i == width-1)
							occupy[k*height*width+j*width+i] = true;
					}
				}
			}
		}
		if(use_sor_for_open)
		{
			solver_type = use_redblack_for_sor ? OPEN_POISSON_REDBLACK_WITH_OCCUPY : OPEN_POISSON_SOR_WITH_OCCUPY;
		}
		else
		{
			BuildOpenPoisson<MyType>(width,height,depth,occupy,&A,display_running_info);
			solver_type = OPEN_POISSON_REDBLACK_WITH_OCCUPY;
		}

	}
	else
	{
		if(add_boundary_occupy_for_closed)
		{
			for(int k = 0;k < depth;k++)
			{
				for(int j = 0;j < height;j++)
				{
					for(int i = 0;i < width;i++)
					{
						if(k == 0 || k == depth-1 || j == 0 || j == height-1 || i == 0 || i == width-1)
							occupy[k*height*width+j*width+i] = true;
					}
				}
			}
		}

		BuildClosedPoisson<MyType>(width,height,depth,occupy,&A,display_running_info);
		solver_type = CLOSED_POISSON_WITH_OCCUPY;
	}


	initRaycating();
	return true;
}

void initRaycating()
{
	const float densityScale = 10.0f;
	const float opacityScale = 0.0001f;

	const float pi = 3.1415926f;


	ZQ_Vec3D boxmin(-1,-1,-1),boxmax(1,1,1);

	float worldMatrix[16] = {   
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	float viewMatrix[16] = {   
		cos(0.0f),	0.0f,	sin(0.0f),	0.0f,
		0.0f,		1.0f,	0.0f,		0.0f,
		-sin(0.0f), 0.0f,	cos(0.0f),	-16.0f,
		0.0f,		0.0f,	0.0f,		1.0f
	};

	float fovy = pi/6;
	float focal_len = win_height/tan(fovy*0.5);
	m_CPURaycast.SetWindowSize(win_width,win_height);
	m_CPURaycast.SetInnerPara(win_width/2,win_height/2,focal_len,focal_len);
	m_CPURaycast.SetVolumeData(density,width,height,depth);
	m_CPURaycast.SetVolumeDensityScale(densityScale);
	m_CPURaycast.SetOpacityScale(opacityScale);
	m_CPURaycast.SetVolumeBoundingBox(boxmin,boxmax);
	m_CPURaycast.SetViewMatrix(viewMatrix);
	m_CPURaycast.SetWorldMatrix(worldMatrix);

}

void shutdown()
{
	delete []density;
	delete []temperature;
	delete []mac_u;
	delete []mac_v;
	delete []mac_w;
	density = 0;
	temperature = 0;
	mac_u = 0;
	mac_v = 0;
	mac_w = 0;

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
	float cz = source_center_z*depth;
	float x_half_len = source_half_xlen*width;
	float y_half_len = source_half_ylen*height;
	float z_half_len = source_half_zlen*depth;

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(fabs(i-cx) <= x_half_len && fabs(j-cy) <= y_half_len && fabs(k-cz) <= z_half_len)
				{
					density[k*height*width+j*width+i] = 0.8;
					temperature[k*height*width+j*width+i] = 20;
				}
			}
		}
	}
}

void diffuse(float dt)
{
	ZQ::ZQ_DImage3D<MyType> temp(width,height,depth);
	MyType*& temp_ptr = temp.data();
	memcpy(temp_ptr,temperature,sizeof(MyType)*width*height*depth);
	temp.GaussianSmoothing(4,4);

	MyType diffuse_temp_coeff = exp(-dt*diffuse_temperature_coeff);

	for(int i = 0;i < width*height*depth;i++)
		temperature[i] = temperature[i]* diffuse_temp_coeff + (1-diffuse_temp_coeff)*temp_ptr[i];
}

void addForce(float dt)
{
	float voxel_len = 1.0/width;

	MyType* force = new MyType[width*height*depth*3];
	memset(force,0,sizeof(MyType)*width*height*depth*3);

	ZQ_DImage3D<MyType> u(width,height,depth),v(width,height,depth),w(width,height,depth);
	ZQ_DImage3D<MyType> vortVector(width,height,depth,3);
	ZQ_DImage3D<MyType> vortScale(width,height,depth);
	ZQ_DImage3D<MyType> vort_gx(width,height,depth);
	ZQ_DImage3D<MyType> vort_gy(width,height,depth);
	ZQ_DImage3D<MyType> vort_gz(width,height,depth);

	MyType* uPtr = u.data();
	MyType* vPtr = v.data();
	MyType* wPtr = w.data();
	MyType* vortVectorPtr = vortVector.data();
	MyType* vortScalePtr = vortScale.data();
	MyType* vort_gxPtr = vort_gx.data();
	MyType* vort_gyPtr = vort_gy.data();
	MyType* vort_gzPtr = vort_gz.data();

	MACtoRegularGrid(width,height,depth,mac_u,mac_v,mac_w,uPtr,vPtr,wPtr);

	/*calculate vorticity vector and scale*/
	int KSLICE = height*width;
	int JSLICE = width;
	int ISLICE = 1;
	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(k== 0 || k == depth-1 || j == 0 || j == height-1 || i == 0 || i == width-1)
				{
				}
				else
				{
					int offset = k*height*width+j*width+i;

					vortVectorPtr[offset*3+2] = (vPtr[offset+ISLICE] - vPtr[offset-ISLICE] - uPtr[offset+JSLICE] + uPtr[offset-JSLICE])/(2.0f*voxel_len);
					vortVectorPtr[offset*3+0] = (wPtr[offset+JSLICE] - wPtr[offset-JSLICE] - vPtr[offset+KSLICE] + vPtr[offset-KSLICE])/(2.0f*voxel_len);
					vortVectorPtr[offset*3+1] = (uPtr[offset+KSLICE] - uPtr[offset-KSLICE] - wPtr[offset+ISLICE] + wPtr[offset-ISLICE])/(2.0f*voxel_len);
				}
			}
		}
	}


	//vortVector.GaussianSmoothing(4,4);

	for(int i = 0;i < height*width*depth;i++)
		vortScalePtr[i] = sqrt(vortVectorPtr[i*3+0]*vortVectorPtr[i*3+0]+vortVectorPtr[i*3+1]*vortVectorPtr[i*3+1]+vortVectorPtr[i*3+2]*vortVectorPtr[i*3+2]);


	/*calculate gradients of vorticity scale*/
	vortScale.dx(vort_gx,true);
	vortScale.dy(vort_gy,true);
	vortScale.dz(vort_gz,true);
	for(int i = 0;i < height*width*depth;i++)
	{
		float len = sqrt(vort_gxPtr[i]*vort_gxPtr[i] + vort_gyPtr[i]*vort_gyPtr[i] + vort_gzPtr[i]*vort_gzPtr[i]);
		if(len != 0)
		{
			vort_gxPtr[i] /= len;
			vort_gyPtr[i] /= len;
			vort_gzPtr[i] /= len;
		}
	}


	/*fconf = eta * deltat *(gradVort X vortVector)*/
	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int offset = k*height*width+j*width+i;
				force[offset*3+0] += fconf_coeff * voxel_len * vort_gyPtr[offset]*vortVectorPtr[offset*3+2];
				force[offset*3+1] += fconf_coeff * voxel_len * vort_gzPtr[offset]*vortVectorPtr[offset*3+0];
				force[offset*3+2] += fconf_coeff * voxel_len * vort_gxPtr[offset]*vortVectorPtr[offset*3+1];
			}
		}
	}

	//buoyancy
	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int offset = k*height*width+j*width+i;
				force[offset*3+1] += buoy_coeff*temperature[offset];
			}
		}
	}


	if(occupy == 0)
	{
		// update mac_u
		for(int k = 0;k < depth;k++)
		{
			for(int j = 0;j < height;j++)
			{
				for(int i = 1;i < width;i++)
				{
					mac_u[k*height*(width+1)+j*(width+1)+i] += dt * 0.5 * (force[(k*height*width+j*width+i)*3+0]+force[(k*height*width+j*width+i-1)*3+0]);
				}
				mac_u[k*height*(width+1)+j*(width+1)+0] += dt * force[(k*height*width+j*width+0)*3+0];
				mac_u[k*height*(width+1)+j*(width+1)+width] += dt * force[(k*height*width+j*width+width-1)*3+0];
			}
		}


		//update mac_v
		for(int k = 0;k < depth;k++)
		{
			for(int i = 0;i < width;i++)
			{
				for(int j = 1;j < height;j++)
				{
					mac_v[k*(height+1)*width+j*width+i] += dt * 0.5* (force[(k*height*width+j*width+i)*3+1]+force[(k*height*width+(j-1)*width+i)*3+1]);
				}
				mac_v[k*(height+1)*width+0*width+i] += dt * force[(k*height*width+0*width+i)*3+1];
				mac_v[k*(height+1)*width+height*width+i] += dt * force[(k*height*width+(height-1)*width+i)*3+1];
			}
		}

		//update mac_w
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				for(int k = 1;k < depth;k++)
				{
					mac_w[k*height*width+j*width+i] += dt * 0.5 * (force[(k*height*width+j*width+i)*3+2]+force[((k-1)*height*width+j*width+i)*3+2]);
				}
				mac_w[0*height*width+j*width+i] += dt * force[(0*height*width+j*width+i)*3+2];
				mac_w[depth*height*width+j*width+i] += dt * force[((depth-1)*height*width+j*width+i)*3+2];
			}
		}
	}
	else
	{
		// update mac_u
		for(int k = 0;k < depth;k++)
		{
			for(int j = 0;j < height;j++)
			{
				for(int i = 1;i < width;i++)
				{
					if(!occupy[k*height*width+j*width+i] && !occupy[k*height*width+j*width+i-1])
						mac_u[k*height*(width+1)+j*(width+1)+i] += dt * 0.5 * (force[(k*height*width+j*width+i)*3+0]+force[(k*height*width+j*width+i-1)*3+0]);
				}
				if(!occupy[k*height*width+j*width+0])
					mac_u[k*height*(width+1)+j*(width+1)+0] += dt * force[(k*height*width+j*width+0)*3+0];
				if(!occupy[k*height*width+j*width+width-1])
					mac_u[k*height*(width+1)+j*(width+1)+width] += dt * force[(k*height*width+j*width+width-1)*3+0];
			}
		}


		//update mac_v
		for(int k = 0;k < depth;k++)
		{
			for(int i = 0;i < width;i++)
			{
				for(int j = 1;j < height;j++)
				{
					if(!occupy[k*height*width+j*width+i] && !occupy[k*height*width+(j-1)*width+i])
						mac_v[k*(height+1)*width+j*width+i] += dt * 0.5* (force[(k*height*width+j*width+i)*3+1]+force[(k*height*width+(j-1)*width+i)*3+1]);
				}
				if(!occupy[k*height*width+0*width+i])
					mac_v[k*(height+1)*width+0*width+i] += dt * force[(k*height*width+0*width+i)*3+1];
				if(!occupy[k*height*width+(height-1)*width+i])
					mac_v[k*(height+1)*width+height*width+i] += dt * force[(k*height*width+(height-1)*width+i)*3+1];
			}
		}

		//update mac_w
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				for(int k = 1;k < depth;k++)
				{
					if(!occupy[k*height*width+j*width+i] && !occupy[(k-1)*height*width+j*width+i])
						mac_w[k*height*width+j*width+i] += dt * 0.5 * (force[(k*height*width+j*width+i)*3+2]+force[((k-1)*height*width+j*width+i)*3+2]);
				}
				if(!occupy[0*height*width+j*width+i])
					mac_w[0*height*width+j*width+i] += dt * force[(0*height*width+j*width+i)*3+2];
				if(!occupy[(depth-1)*height*width+j*width+i])
					mac_w[depth*height*width+j*width+i] += dt * force[((depth-1)*height*width+j*width+i)*3+2];
			}
		}
	}

	if(!is_open_boundary)
	{

		for(int k = 0;k < depth;k++)
		{
			for(int j = 0;j < height;j++)
			{
				mac_u[k*height*(width+1)+j*(width+1)+0] = 0;
				mac_u[k*height*(width+1)+j*(width+1)+width] = 0;
			}
		}

		for(int k = 0;k < depth;k++)
		{
			for(int i = 0;i < width;i++)
			{
				mac_v[k*(height+1)*width+0*width+i] = 0;
				mac_v[k*(height+1)*width+height*width+i] = 0;
			}
		}

		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				mac_w[0*height*width+j*width+i] = 0;
				mac_w[depth*height*width+j*width+i] = 0;
			}
		}
	}

	delete []force;
}

void attenuation(float dt)
{
	float attenTemp = exp(-dt * temp_atten_coeff);
	for(int i = 0;i < width*height*depth;i++)
	{
		temperature[i] *= attenTemp;
	}

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(k == 0 || k == depth-1 || j == 0 || j == height-1 || i == 0 || i == width-1)
				{
					temperature[k*height*width+j*width+i] = 0;
					density[k*height*width+j*width+i] = 0;
				}
			}
		}
	}


	float attenVel = exp(-dt * vel_atten_coeff);

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i <= width;i++)
			{
				if(occupy == 0)
				{
					mac_u[k*height*(width+1)+j*(width+1)+i] *= attenVel;
				}
				else
				{
					if(i == 0 && occupy[k*height*width+j*width+i])
					{

					}
					else if(i == width && occupy[k*height*width+j*width+i-1])
					{

					}
					else if((i > 0 && i < width) && (occupy[k*height*width+j*width+i] || occupy[k*height*width+j*width+i-1]))
					{

					}
					else
						mac_u[k*height*(width+1)+j*(width+1)+i] *= attenVel;
				}
			}
		}
	}


	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j <= height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(occupy == 0)
				{
					mac_v[k*(height+1)*width+j*width+i] *= attenVel;
				}
				else
				{
					if(j == 0 && occupy[k*height*width+j*width+i])
					{

					}
					else if(j == height && occupy[k*height*width+(j-1)*width+i])
					{

					}
					else if(j > 0 && j < height && (occupy[k*height*width+j*width+i] || occupy[k*height*width+(j-1)*width+i]))
					{

					}
					else
						mac_v[k*(height+1)*width+j*width+i] *= attenVel;
				}
			}
		}
	}

	for(int k = 0;k <= depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(occupy == 0)
				{
					mac_w[k*height*width+j*width+i] *= attenVel;
				}
				else
				{
					if(k == 0 && occupy[k*height*width+j*width+i])
					{

					}
					else if(k == depth && occupy[(k-1)*height*width+j*width+i])
					{

					}
					else if(k > 0 && k < depth && (occupy[k*height*width+j*width+i] || occupy[(k-1)*height*width+j*width+i]))
					{

					}
					else
						mac_w[k*height*width+j*width+i] *= attenVel;
				}
			}
		}
	}

}


void advectVelocity(float dt)
{
	float voxel_len = 1.0/width;

	MyType* out_mac_u = new MyType[(width+1)*height*depth];
	MyType* out_mac_v = new MyType[width*(height+1)*depth];
	MyType* out_mac_w = new MyType[width*height*(depth+1)];
	memcpy(out_mac_u,mac_u,sizeof(MyType)*(width+1)*height*depth);
	memcpy(out_mac_v,mac_v,sizeof(MyType)*width*(height+1)*depth);
	memcpy(out_mac_w,mac_w,sizeof(MyType)*width*height*(depth+1));

	int nPts = (width+1)*height*depth;
	MyType* in_pos = new MyType[nPts*3];
	MyType* out_pos = new MyType[nPts*3];
	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i <= width;i++)
			{
				int offset = k*height*(width+1)+j*(width+1)+i;
				in_pos[offset*3+0] = i*voxel_len;
				in_pos[offset*3+1] = (j+0.5f)*voxel_len;
				in_pos[offset*3+2] = (k+0.5f)*voxel_len;
			}
		}
	}


	ZQ_BactTraceAdvection_MACGrid(mac_u,mac_v,mac_w,occupy,width,height,depth,voxel_len,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i <= width;i++)
			{
				if(i == 0 && occupy && occupy[k*height*width+j*width+i])
					continue;
				if(i == width && occupy && occupy[k*height*width+j*width+i-1])
					continue;
				if(i > 0 && i < width && occupy && (occupy[k*height*width+j*width+i] || occupy[k*height*width+j*width+i-1]))
					continue;

				int offset = k*height*(width+1)+j*(width+1)+i;
				float cur_x = out_pos[offset*3+0]/voxel_len;
				float cur_y = out_pos[offset*3+1]/voxel_len;
				float cur_z = out_pos[offset*3+2]/voxel_len;
				ZQ_ImageProcessing3D::TricubicInterpolate(mac_u,width+1,height,depth,1,cur_x,cur_y-0.5f,cur_z-0.5f,out_mac_u+offset,false);
			}
		}
	}


	delete []in_pos;
	delete []out_pos;

	nPts = width*(height+1)*depth;
	in_pos = new MyType[nPts*3];
	out_pos = new MyType[nPts*3];
	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j <= height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int offset = k*(height+1)*width+j*width+i;
				in_pos[offset*3+0] = (i+0.5f)*voxel_len;
				in_pos[offset*3+1] = j*voxel_len;
				in_pos[offset*3+2] = (k+0.5f)*voxel_len;
			}
		}
	}


	ZQ_BactTraceAdvection_MACGrid(mac_u,mac_v,mac_w,occupy,width,height,depth,voxel_len,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j <= height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(j == 0 && occupy &&  occupy[k*height*width+j*width+i])
					continue;
				if(j == height && occupy &&occupy[k*height*width+(j-1)*width+i])
					continue;
				if(j > 0 && j < height && occupy &&(occupy[k*height*width+j*width+i] || occupy[k*height*width+(j-1)*width+i]))
					continue;

				int offset = k*(height+1)*width+j*width+i;
				float cur_x = out_pos[offset*3+0]/voxel_len;
				float cur_y = out_pos[offset*3+1]/voxel_len;
				float cur_z = out_pos[offset*3+2]/voxel_len;
				ZQ_ImageProcessing3D::TrilinearInterpolate(mac_v,width,height+1,depth,1,cur_x-0.5f,cur_y,cur_z-0.5f,out_mac_v+offset,false);
			}
		}
	}


	delete []in_pos;
	delete []out_pos;

	nPts = width*height*(depth+1);
	in_pos = new MyType[nPts*3];
	out_pos = new MyType[nPts*3];
	for(int k = 0;k <= depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int offset = k*height*width+j*width+i;
				in_pos[offset*3+0] = (i+0.5f)*voxel_len;
				in_pos[offset*3+1] = (j+0.5f)*voxel_len;
				in_pos[offset*3+2] = k*voxel_len;
			}
		}
	}


	ZQ_BactTraceAdvection_MACGrid(mac_u,mac_v,mac_w,occupy,width,height,depth,voxel_len,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int k = 0;k <= depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(k == 0 && occupy &&  occupy[k*height*width+j*width+i])
					continue;
				if(k == depth && occupy &&occupy[(k-1)*height*width+j*width+i])
					continue;
				if(k > 0 && k < height && occupy &&(occupy[k*height*width+j*width+i] || occupy[(k-1)*height*width+j*width+i]))
					continue;

				int offset = k*height*width+j*width+i;
				float cur_x = out_pos[offset*3+0]/voxel_len;
				float cur_y = out_pos[offset*3+1]/voxel_len;
				float cur_z = out_pos[offset*3+2]/voxel_len;
				ZQ_ImageProcessing3D::TrilinearInterpolate(mac_w,width,height,depth+1,1,cur_x-0.5f,cur_y-0.5f,cur_z,out_mac_w+offset,false);
			}
		}
	}


	delete []in_pos;
	delete []out_pos;

	memcpy(mac_u,out_mac_u,sizeof(MyType)*(width+1)*height*depth);
	memcpy(mac_v,out_mac_v,sizeof(MyType)*width*(height+1)*depth);
	memcpy(mac_w,out_mac_w,sizeof(MyType)*width*height*(depth+1));
	delete []out_mac_u;
	delete []out_mac_v;
	delete []out_mac_w;
}

void projection()
{

	switch(solver_type)
	{
	case OPEN_POISSON_WITHOUT_OCCUPY:
		SolveOpenPoisson_MACGrid(mac_u,mac_v,mac_w,width,height,depth,A,max_iter,display_running_info);
		break;
	case OPEN_POISSON_WITH_OCCUPY:
		SolveOpenPoisson_MACGrid(mac_u,mac_v,mac_w,width,height,depth,occupy,A,max_iter,display_running_info);
		break;
	case OPEN_POISSON_SOR_WITHOUT_OCCUPY:
		SolveOpenPoissonSOR_MACGrid(mac_u,mac_v,mac_w,width,height,depth,max_iter,display_running_info);
		break;
	case OPEN_POISSON_SOR_WITH_OCCUPY:
		SolveOpenPoissonSOR_MACGrid(mac_u,mac_v,mac_w,width,height,depth,occupy,max_iter,display_running_info);
		break;
	case OPEN_POISSON_REDBLACK_WITHOUT_OCCUPY:
		SolveOpenPoissonRedBlack_MACGrid(mac_u,mac_v,mac_w,width,height,depth,max_iter,display_running_info);
		break;
	case OPEN_POISSON_REDBLACK_WITH_OCCUPY:
		SolveOpenPoissonRedBlack_MACGrid(mac_u,mac_v,mac_w,width,height,depth,occupy,max_iter,display_running_info);
		break;
	case CLOSED_POISSON_WITHOUT_OCCUPY:
		SolveClosedPoisson_MACGrid(mac_u,mac_v,mac_w,width,height,depth,A,max_iter,display_running_info);
		break;
	case CLOSED_POISSON_WITH_OCCUPY:
		SolveClosedPoisson_MACGrid(mac_u,mac_v,mac_w,width,height,depth,occupy,A,max_iter,display_running_info);
		break;
	default:
		printf("unknown solver\n");
		break;
	}
}

void advectScalar(float dt)
{
	float voxel_len = 1.0/width;
	MyType* out_density = new MyType[width*height*depth];
	MyType* out_temperature = new MyType[width*height*depth];
	memcpy(out_density,density,sizeof(MyType)*width*height*depth);
	memcpy(out_temperature,temperature,sizeof(MyType)*width*height*depth);

	int nPts = width*height*depth;
	MyType* in_pos = new MyType[nPts*3];
	MyType* out_pos = new MyType[nPts*3];

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				int offset = k*height*width+j*width+i;
				in_pos[offset*3+0] = (i+0.5f)*voxel_len;
				in_pos[offset*3+1] = (j+0.5f)*voxel_len;
				in_pos[offset*3+2] = (k+0.5f)*voxel_len;
			}
		}
	}


	ZQ_BactTraceAdvection_MACGrid(mac_u,mac_v,mac_w,occupy,width,height,depth,voxel_len,voxel_len,voxel_len,dt,substeps,nPts,in_pos,out_pos);

	for(int k = 0;k < depth;k++)
	{
		for(int j = 0;j < height;j++)
		{
			for(int i = 0;i < width;i++)
			{
				if(occupy && occupy[k*height*width+j*width+i])
					continue;

				int offset = k*height*width+j*width+i;
				float cur_x = out_pos[offset*3+0]/voxel_len;
				float cur_y = out_pos[offset*3+1]/voxel_len;
				float cur_z = out_pos[offset*3+2]/voxel_len;
				ZQ_ImageProcessing3D::TrilinearInterpolate(density,width,height,depth,1,cur_x-0.5,cur_y-0.5,cur_z-0.5f,out_density+offset,false);
				ZQ_ImageProcessing3D::TrilinearInterpolate(temperature,width,height,depth,1,cur_x-0.5,cur_y-0.5,cur_z-0.5f,out_temperature+offset,false);
			}
		}
	}


	delete []in_pos;
	delete []out_pos;

	memcpy(density,out_density,sizeof(MyType)*width*height*depth);
	memcpy(temperature,out_temperature,sizeof(MyType)*width*height*depth);
	delete []out_density;
	delete []out_temperature;
}

void show(float wait_time)
{
	float* renderBuffer = new float[win_height*win_width*4];
	ZQ_CPURaycasting::ColorFormat color_fmt = ZQ_CPURaycasting::COLOR_BGRA;

	m_CPURaycast.RenderToBuffer(renderBuffer,50,color_fmt);

	IplImage* rc_im = cvCreateImage(cvSize(win_width,win_height),IPL_DEPTH_8U,4);
	for(int h = 0;h < win_height;h++)
	{
		for(int w = 0;w < win_width;w++)
		{	
			int offset = h*win_width+w;
			cvSet2D(rc_im,h,w,cvScalar(renderBuffer[4*offset+0]*255,renderBuffer[4*offset+1]*255,renderBuffer[4*offset+2]*255,renderBuffer[4*offset+3]*255));
		}
	}
	delete []renderBuffer;

	if(cvGetWindowHandle("ZQ_RayCasting") == 0)
		cvNamedWindow("ZQ_RayCasting");
	cvShowImage("ZQ_RayCasting",rc_im);

	cvWaitKey(wait_time);
	cvReleaseImage(&rc_im);
}
