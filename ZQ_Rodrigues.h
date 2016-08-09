#ifndef _ZQ_RODRIGUES_H_
#define _ZQ_RODRIGUES_H_

#include "ZQ_SVD.h"

namespace ZQ
{
	class ZQ_Rodrigues
	{
	public:
		template<class T>
		static bool ZQ_Rodrigues_r2R_fun(const T r[3], T R[9]);

		template<class T>
		static bool ZQ_Rodrigues_r2R_jac(const T r[3], T dRdr[27]);

		template<class T>
		static bool ZQ_Rodrigues_R2r_fun(const T R[9], T r[3]);

	};
	

	template<class T>
	bool ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(const T r[3], T R[9])
	{
		double x = r[0];
		double y = r[1];
		double z = r[2];

		double x2 = x*x;
		double y2 = y*y;
		double z2 = z*z;
		double xyz2 = x2+y2+z2;

		double w2 = 1.0-xyz2;

		bool ret_flag = true;

		if(x2 > 1 || y2 > 1 || z2 > 1 || w2 < 0)
			ret_flag = false;

		if(w2 < 0)
		{
			x2 /= xyz2;
			y2 /= xyz2;
			z2 /= xyz2;
			w2 = 0;
		}
		

		double w = sqrt(w2);

		R[0] = w2+x2-y2-z2;
		R[1] = 2*(x*y-z*w);
		R[2] = 2*(x*z+y*w);
		R[3] = 2*(x*y+z*w);
		R[4] = w2+y2-x2-z2;
		R[5] = 2*(y*z-x*w);
		R[6] = 2*(x*z-y*w);
		R[7] = 2*(y*z+x*w);
		R[8] = w2+z2-x2-y2;
		return ret_flag;
	}

	template<class T>
	bool ZQ_Rodrigues::ZQ_Rodrigues_r2R_jac(const T r[3], T dRdr[27])
	{
		const double eps = 1e-6;

		double x = r[0];
		double y = r[1];
		double z = r[2];

		double x2 = x*x;
		double y2 = y*y;
		double z2 = z*z;

		if(x2 > 1 || y2 > 1 || z2 > 1 || x2+y2+z2 > 1)
			return false;

		double w2 = 1.0-x2-y2-z2;
		double w = sqrt(w2);

		double dRdv[36] = {
			2*w,  2*x, -2*y, -2*z,
			-2*z,  2*y,  2*x, -2*z,
			2*y,  2*z,  2*w,  2*x,
			2*z,  2*y,  2*x,  2*w,
			2*w, -2*x,  2*y, -2*z,
			-2*x, -2*w,  2*z,  2*y,
			-2*y,  2*z, -2*w,  2*x,
			2*x,  2*w,  2*z,  2*y,
			2*w, -2*x, -2*y,  2*z
		};

		double dvdr[12] = {
			0,0,0,
			1,0,0,
			0,1,0,
			0,0,1
		};

		if(w >= eps)
		{
			dvdr[0] = -x/w;
			dvdr[1] = -y/w;
			dvdr[2] = -z/w;
		}


		memset(dRdr,0,sizeof(T)*27);
		for(int i = 0;i < 9;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				for(int k = 0;k < 4;k++)
					dRdr[i*3+j] += dRdv[i*4+k]*dvdr[k*3+j];
			}
		}
		return true;
	}

	template<class T>
	bool ZQ_Rodrigues::ZQ_Rodrigues_R2r_fun(const T R[9], T r[3])
	{
		ZQ_Matrix<double> Rm(3,3),U(3,3),S(3,3),V(3,3);
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
				Rm.SetData(i,j,R[i*3+j]);
		}

		if(!ZQ_SVD::Decompose(Rm,U,S,V))
			return false;


		V.Transpose();
		Rm = U*V;

		T RR[9];
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				bool flag;
				RR[i*3+j] = Rm.GetData(i,j,flag);
			}
		}

		for(int i = 0;i < 9;i++)
			RR[i] = __max(-1.0,__min(1.0,RR[i]));

		double costheta = (RR[0]+RR[4]+RR[8]-1)/2.0;
		double theta = acos(costheta);


		const double eps = 1e-6;

		if(sin(theta) > eps) //means: a = cos(theta/2) > 0
		{
			double w2 = (RR[0]+RR[4]+RR[8]+1.0)/4;
			double w = sqrt(w2);
			double xw = (RR[7]-RR[5])/4;
			double yw = (RR[2]-RR[6])/4;
			double zw = (RR[3]-RR[1])/4;

			r[0] = xw/w;
			r[1] = yw/w;
			r[2] = zw/w;
		}
		else
		{
			if(costheta > 0)//means: angle = 0
			{
				double w2 = (RR[0]+RR[4]+RR[8]+1.0)/4;
				double w = sqrt(w2);
				double xw = (RR[7]-RR[5])/4;
				double yw = (RR[2]-RR[6])/4;
				double zw = (RR[3]-RR[1])/4;

				r[0] = xw/w;
				r[1] = yw/w;
				r[2] = zw/w;
			}
			else //means: angle = pi
			{
				double x_abs = sqrt((RR[0]+1)*0.5);
				double y_abs = sqrt((RR[4]+1)*0.5);
				double z_abs = sqrt((RR[8]+1)*0.5);

				int xy_sign = (RR[1] > eps) - (RR[1] < -eps);
				int yz_sign = (RR[5] > eps) - (RR[5] < -eps);
				int xz_sign = (RR[6] > eps) - (RR[6] < -eps);

				int hash_vec[11] = {0, -1, -3, -9, 9, 3, 1, 13, 5, -7, -11};
				int signs_mat[11][3] = 
				{
					{1,1,1},
					{1,0,-1},
					{0,1,-1}, 
					{1,-1,0}, 
					{1,1,0}, 
					{0,1,1}, 
					{1,0,1}, 
					{1,1,1}, 
					{1,1,-1},
					{1,-1,-1},
					{1,-1,1}
				};

				int hash_val = xy_sign*9 + yz_sign*3 + xz_sign;

				int hash_idx = 0;
				for(int i = 0;i < 11;i++)
				{
					if(hash_val == hash_vec[i])
					{
						hash_idx = i;
						break;
					}
				}

				r[0] = x_abs * signs_mat[hash_idx][0];
				r[1] = y_abs * signs_mat[hash_idx][1];
				r[2] = z_abs * signs_mat[hash_idx][2];
			}
		}
		return true;

	}

}


#endif
