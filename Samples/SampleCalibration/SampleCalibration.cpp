#include "ZQ_Calibration.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>


void m_printf_mxn(const double* A, int m, int n)
{
	for(int i = 0;i < m;i++)
	{
		for(int j = 0;j < n;j++)
			printf("%12.7f",A[i*n+j]);
		printf("\n");
	}
}

double m_noise(double scale)
{
	return rand()%10001/10000.0*scale;
}

int main1()
{
	//srand(time(0));
	int n_cams = 20;
	double* p = new double[7+n_cams*6];
	p[0] = 120;
	p[1] = 90;
	p[2] = 1.001;
	p[3] = 512;
	p[4] = 512;
	p[5] = 1e-8;
	p[6] = 0;//1e-11;
	for(int cc = 0;cc < n_cams;cc++)
	{
		double* r = p+7+cc*6;
		double* t = p+7+cc*6+3;
		r[0] = m_noise(0.2);
		r[1] = m_noise(0.2);
		r[2] = m_noise(0.2);
		t[0] = m_noise(1);
		t[1] = m_noise(1);
		t[2] = m_noise(20)+80;
	}

	int M_point = 11;
	int N_point = 11;
	int num_of_pts = M_point*N_point;
	double* X3 = new double[M_point*N_point*3];
	double* X3_noise = new double[num_of_pts*3];

	for(int i = 0;i < M_point;i++)
	{
		for(int j = 0;j < N_point;j++)
		{
			X3[(i*N_point+j)*3+0] = i-N_point/2;
			X3[(i*N_point+j)*3+1] = j-M_point/2;
			X3[(i*N_point+j)*3+2] = 0;
		}
	}

	for(int i = 0;i < num_of_pts*3;i++)
	{
		double scale = 0.01;
		X3_noise[i] = X3[i] + m_noise(scale)-0.5*scale;
	}

	double* X2 = new double[num_of_pts*2*n_cams];

	double A[9] = {
		p[0],p[2],p[3],
		0,p[1],p[4],
		0,0,1
	};

	double eps = 1e-9;

	for(int cc = 0;cc < n_cams;cc++)
	{
		double R[9];
		double* T = p+7+cc*6+3;
		ZQ::ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(p+7+cc*6,R);

		ZQ::ZQ_Calibration::proj_distortion_k2(num_of_pts,A,R,T,p+3,p+5,X3_noise,X2+num_of_pts*2*cc,eps);
	}

	for(int i = 0;i < num_of_pts*2*n_cams;i++)
	{
		double scale = 0.05;
		X2[i] += m_noise(scale)-0.5*scale;
	}

	int max_iter = 1000;
	double* out_p = new double[7+n_cams*6];
	memset(out_p,0,sizeof(double)*(7+n_cams*6));

	double* out_p2 = new double[7+n_cams*6];
	memset(out_p2,0,sizeof(double)*(7+n_cams*6));



	int n_cams1 = 10;
	int n_cams2 = 20;

	ZQ::ZQ_Calibration::calib_estimate_k_int_rT_init(n_cams1,num_of_pts,X3,X2,max_iter,out_p,out_p+7,eps);
	ZQ::ZQ_Calibration::calib_estimate_k_int_rT_init(n_cams2,num_of_pts,X3,X2,max_iter,out_p2,out_p2+7,eps);
	

	ZQ::ZQ_Calibration::calib_estimate_no_distortion_with_init(n_cams1,num_of_pts,X3,X2,max_iter,out_p,out_p+7);
	ZQ::ZQ_Calibration::calib_estimate_no_distortion_with_init(n_cams2,num_of_pts,X3,X2,max_iter,out_p2,out_p2+7);

	m_printf_mxn(p,1,5);
	m_printf_mxn(out_p,1,5);
	m_printf_mxn(out_p2,1,5);

	ZQ::ZQ_Calibration::calib_estimate_k_int_rT_with_init(n_cams1,num_of_pts,X3,X2,max_iter,out_p+5,out_p,out_p+7);
	ZQ::ZQ_Calibration::calib_estimate_k_int_rT_with_init(n_cams2,num_of_pts,X3,X2,max_iter,out_p2+5,out_p2,out_p2+7);

	m_printf_mxn(p,1,7);
	m_printf_mxn(out_p,1,7);
	m_printf_mxn(out_p2,1,7);


	delete []out_p;
	delete []out_p2;
	delete []p;
	delete []X2;
	delete []X3;
	delete []X3_noise;
	return 0;

}

int main2()
{
	
	srand(time(0));

	for(int cc = 0;cc < 100;cc++)
	{


		clock_t t1 = clock();
		double intrinsic_para[5] = {1200,1200,1.00,512,512};

		double A[9] =
		{
			intrinsic_para[0],intrinsic_para[2],intrinsic_para[3],
			0,intrinsic_para[1],intrinsic_para[4],
			0,0,1
		};

		double rT[6] = 
		{
			m_noise(0.2),
			m_noise(0.2),
			m_noise(0.2),
			m_noise(1),
			m_noise(1),
			m_noise(20)+80

		};

		int num_of_pts = 100;
		double* X3 = new double[num_of_pts*3];
		for(int i = 0;i < num_of_pts;i++)
		{
			X3[i*3+0] = m_noise(10)-5;
			X3[i*3+1] = m_noise(10)-5;
			X3[i*3+2] = m_noise(10)-5;
		}
		
		double* X2 = new double[num_of_pts*2];

		double eps = 1e-16;


		double R[9];
		double* T = rT+3;
		ZQ::ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(rT,R);

		ZQ::ZQ_Calibration::proj_no_distortion(num_of_pts,A,R,T,X3,X2,eps);

		for(int i = 0;i < num_of_pts*2;i++)
		{
			double scale = 0.02;
			X2[i] += m_noise(scale)-0.5*scale;
		}

		int max_iter = 5000;
		double out_rT[6] = {0,0,0,0,0,100};


		ZQ::ZQ_Calibration::posit_no_coplanar(num_of_pts,X3,X2,max_iter,intrinsic_para,out_rT);
		double avg_err;
		ZQ::ZQ_Calibration::pose_estimate_no_distortion_with_init(num_of_pts,X3,X2,max_iter,intrinsic_para,out_rT,avg_err,eps);
		clock_t t2 = clock();

		printf("cost time: %f seconds\n",0.001*(t2-t1));

		m_printf_mxn(rT,1,6);
		m_printf_mxn(out_rT,1,6);
		delete []X2;
		delete []X3;

	}
	return 0;

}


int main()
{

	int seed = time(0);
	printf("%d\n",seed);
	srand(seed);

	int posit_suc = 0;
	int optmi_count = 0;
	int max_cams = 1000;
	for(int cc = 0;cc < max_cams;cc++)
	{


		clock_t t1 = clock();
		double intrinsic_para[5] = {1200,1200,1.00,0,0};

		double A[9] =
		{
			intrinsic_para[0],intrinsic_para[2],intrinsic_para[3],
			0,intrinsic_para[1],intrinsic_para[4],
			0,0,1
		};

		int M = 7;
		int N = 7;
		int num_of_pts = M*N;
		double* X3 = new double[num_of_pts*3];
		for(int i = 0;i < M;i++)
		{
			for(int j = 0;j < N;j++)
			{
				X3[(i*N+j)*3+0] = j - N/2;
				X3[(i*N+j)*3+1] = i - M/2;
				X3[(i*N+j)*3+2] = 0;
			}
		}

		double z_shift = 20;
		double x_shift = (intrinsic_para[3]/intrinsic_para[0])*z_shift-N/2;
		double y_shift = (intrinsic_para[4]/intrinsic_para[1])*z_shift-M/2;

		double rT[6] = 
		{
			m_noise(0.5)-0.25,
			m_noise(0.5)-0.25,
			m_noise(0.4)-0.2,
			m_noise(x_shift*.8)-x_shift*.4,
			m_noise(y_shift*.8)-y_shift*.4,
			m_noise(z_shift*0.2)+z_shift

		};
		//double rT[6] = {0.0480800,   0.0321400,   0.0288200,   6.0000000,  6.0000000,  23.7360000};
		//double rT[6] = {0.2171500,  -0.1415500,  -0.1765600,   0.1130400,   1.1109600,  23.3328000};
		


		double* X2 = new double[num_of_pts*2];
		double eps = 1e-16;


		double R[9];
		double* T = rT+3;
		ZQ::ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(rT,R);
		

		ZQ::ZQ_Calibration::proj_no_distortion(num_of_pts,A,R,T,X3,X2,eps);

		for(int i = 0;i < num_of_pts*2;i++)
		{
			double scale = 0.2;
			X2[i] += m_noise(scale)-0.5*scale;
		}
		
		int max_iter = 500;
		double tol_E = 1;
		double out_rT[6] = {0,0,0,0,0,100};


		double avg_E;
		if(!ZQ::ZQ_Calibration::posit_coplanar_robust(num_of_pts,X3,X2,100,max_iter,tol_E,intrinsic_para,out_rT,avg_E,eps))
		{
			printf("failed\n");
		}
		
		if(avg_E > tol_E )
		{
			printf("cc=%d\n",cc);
			m_printf_mxn(rT,1,6);
			m_printf_mxn(out_rT,1,6);
		}
		
		
		delete []X2;
		delete []X3;
		if((cc+1)%100==0)
			printf("cc=%d\n",cc+1);

	}
	
	return 0;

}

