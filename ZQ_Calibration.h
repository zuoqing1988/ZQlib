#ifndef _ZQ_CALIBRATION_H_
#define _ZQ_CALIBRATION_H_
#include "ZQ_Rodrigues.h"
#include "ZQ_SVD.h"
#include "ZQ_LevMar.h"
#include <vector>

namespace ZQ
{
	class ZQ_Calibration
	{
	public:
		/* left hand coordinates*/
		template<class T>
		static void proj_no_distortion(int n, const T A[9], const T R[9], const T t[3], const T* X3, T* X2, double eps = 1e-9);

		template<class T>
		static void distortion_k2(int n, const T center[2], const T k[2], const T* inX2, T* outX2);

		/* left hand coordinates*/
		template<class T>
		static void proj_distortion_k2(int n, const T A[9], const T R[9], const T t[3], const T center[2], const T k[2], const T* X3, T* X2, double eps = 1e-9);

	private:
		
		template<class T>
		class Calib_Data_Header 
		{
		public:
			const T* X3;
			const T* X2;
			int n_cams;
			int n_pts;
			double eps;
			const T* k;
			const T* intrinsic_para;
			const T* rT;
		};

		template<class T>
		class Posit_Coplanar_Node
		{
		public:
			T kk[3];
			T R[9],tt[3],rr[3];
			double Z0;
			double Error;
		};

	private:
		/*
		refer to the paper:
		A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
		compute the camera intrinsic parameter with at least  3 checkboard images. 
		left hand coordinates.
		*/

		template<class T>
		static bool _estimate_H_func(const T* x, T* fx, int m, int n, const void* data);

		template<class T>
		static bool _estimate_H_jac(const T* p, T* jx, int m, int n, const void* data);

		template<class T>
		static bool _estimate_H(T* H, int n_pts, const T* X3, const T* X2, int max_iter, double eps = 1e-9);
		
		template<class T>
		static void _get_v_i_j(const T* H, int i, int j, T* vij);

		template<class T>
		static void _get_v_no_distortion(const T* H, T* row1, T* row2);

		template<class T>
		static bool _compute_b(const T* VV, int m, int n, T* b);

		template<class T>
		static bool _compute_A_from_b(const T* b, T* A);

		template<class T>
		static bool _compute_RT_from_AH(const T* H, const T* A, T* R, T* tt);

	public:
		/*
		refer to the paper:
		A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
		compute the camera intrinsic parameter with at least  3 checkboard images. 
		left hand coordinates.
		*/
		template<class T>
		static bool calib_estimate_k_int_rT_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter, T* intrinsic_para, T* rT, double eps  = 1e-9);

	private:
		/*calibrate camera: intrinsic parameter,
		based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/
		template<class T>
		static bool _calib_estimate_no_distortion_func(const T* p, T* hx, int m, int n, const void* data);

		template<class T>
		static bool _calib_estimate_no_distortion_jac(const T* p, T* jx, int m, int n, const void* data);

	public:
		/*calibrate camera: intrinsic parameter,
		based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/
		
		template<class T>
		static bool calib_estimate_no_distortion_with_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter,T* intrinsic_para, T* rT, double eps = 1e-9);

	private:
		/*calibrate camera: intrinsic parameter and distortion (k1,k2),
		based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/
		template<class T>
		static bool _calib_estimate_k_int_rT_func(const T* p, T* hx, int m, int n, const void* data);
		
		template<class T>
		static bool _calib_estimate_k_int_rT_jac(const T* p, T* jx, int m, int n, const void* data);

	public:
		/*calibrate camera: intrinsic parameter and distortion (k1,k2),
		based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/

		template<class T>
		static bool calib_estimate_k_int_rT_with_init(int n_cams, int n_pts,const T* X3, const T* X2, int max_iter, T* k,  T* intrinsic_para, T* rT, double eps = 1e-9);

		template<class T>
		static bool calib_estimate_k_int_rT_without_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter, T* k, T* intrinsic_para, T* rT, double eps = 1e-9);

	private:
		/*pose estimation based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/
		template<class T>
		static bool _pose_estimate_no_distortion_func(const T* p, T* hx, int m, int n, const void* data);
		
		template<class T>
		static bool _pose_estimate_no_distortion_jac(const T* p, T* jx, int m, int n, const void* data);
		
	public:
		/*pose estimation based on Lev-Mar optimization, need good initialization.
		left hand coordinates.
		*/

		template<class T>
		static bool pose_estimate_no_distortion_with_init(int n_pts, const T* X3, const T* X2, int max_iter, const T* intrinsic_para, T* rT, double& avg_err_square, double eps = 1e-9);

	public:
		/*
		refer to the paper: 
		Model-Based Object Pose in 25 Lines of Codes. Daniel F. DeMenthon and Larry S. Davis. IJCV,1995.
		left hand coordinates.
		intrinsic_para[0-4]: fx, fy, shear, u0, v0.  with no distortion.
		rT[0-5]: rx, ry, rz, Tx, Ty, Tz.  (rx,ry,rz,rw) is a quaternion.   
		*/
		template<class T>
		static bool posit_no_coplanar(int n_pts, const T* X3, const T* X2, int max_iter, const T* intrinsic_para, T* rT);

		/*
		refer to the paper:
		iterative pose estimation using coplanar points. Denis Oberkampf, Daniel F. DeMenthon, Larry  S. Davis. CVPR, 1993. 
		left hand coordinates.
		intrinsic_para[0-4]: fx, fy, shear, u0, v0. with no distortion.
		rT[0-5]: rx, ry, rz, Tx, Ty, Tz.  (rx,ry,rz,rw) is a quaternion.
		*/
		template<class T>
		static bool posit_coplanar(int n_pts, const T* X3, const T* X2, int max_iter, double tol_avg_E, const T* intrinsic_para, T* rT, T* reproj_err_square, double eps = 1e-9);

		/*
		left hand coordinates.
		The base idea is to use the method proposed in the paper:
		iterative pose estimation using coplanar points. Denis Oberkampf, Daniel F. DeMenthon, Larry  S. Davis. CVPR, 1993. 
		However, I find it cannot make sure the method always converge to the optimal solution.
		But the translation seems to be near the optimal one according to my observations.
		So I choose 9 rotations to run Lev-Mar solvers to find a best solution.
		If all choices do not give a solution with avg_E < tol_avg_E, the best solution of the 9 will be returned,
		otherwise, the first one satisfying avg_E < tol_avg_E will be returned.
		*/
		template<class T>
		static bool posit_coplanar_robust(int n_pts, const T* X3, const T* X2, int max_iter_posit, int max_iter_levmar, double tol_E, const T* intrinsic_para, T* rT, double& avg_E, double eps = 1e-9);

	};

	/*left hand coordinates.*/
	template<class T>
	void ZQ_Calibration::proj_no_distortion(int n, const T A[], const T R[], const T t[], const T* X3, T* X2, double eps /* = 1e-6 */)
	{
		for(int i = 0;i < n;i++)
		{
			double tmp_x3[3] = 
			{
				R[0]*X3[i*3+0] + R[1]*X3[i*3+1] + R[2]*X3[i*3+2] + t[0],
				R[3]*X3[i*3+0] + R[4]*X3[i*3+1] + R[5]*X3[i*3+2] + t[1],
				R[6]*X3[i*3+0] + R[7]*X3[i*3+1] + R[8]*X3[i*3+2] + t[2]
			};

			double tmp_x2[3] = 
			{
				A[0]*tmp_x3[0] + A[1]*tmp_x3[1] + A[2]*tmp_x3[2],
				A[3]*tmp_x3[0] + A[4]*tmp_x3[1] + A[5]*tmp_x3[2],
				A[6]*tmp_x3[0] + A[7]*tmp_x3[1] + A[8]*tmp_x3[2]
			};

			
			X2[i*2+0] = tmp_x2[0]*tmp_x2[2]/(tmp_x2[2]*tmp_x2[2]+eps*eps);
			X2[i*2+1] = tmp_x2[1]*tmp_x2[2]/(tmp_x2[2]*tmp_x2[2]+eps*eps);
		}
	}

	template<class T>
	void ZQ_Calibration::distortion_k2(int n, const T center[2], const T k[2], const T* inX2, T* outX2)
	{
		double u0 = center[0];
		double v0 = center[1];

		for(int i = 0;i < n;i++)
		{
			double X = inX2[i*2+0]-u0;
			double Y = inX2[i*2+1]-v0;
			double R2 = X*X+Y*Y;
			outX2[i*2+0] = inX2[i*2+0] + X*(k[0]*R2+k[1]*R2*R2);
			outX2[i*2+1] = inX2[i*2+1] + Y*(k[0]*R2+k[1]*R2*R2);
		}
	}


	/*left hand coordinates.*/
	template<class T>
	void ZQ_Calibration::proj_distortion_k2(int n, const T A[9], const T R[9], const T t[3], const T center[2], const T k[2], const T* X3, T* X2, double eps /* = 1e-6 */)
	{
		proj_no_distortion(n,A,R,t,X3,X2,eps);
		distortion_k2(n,center,k,X2,X2);
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_estimate_H_func(const T* p, T* hx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;
		const double eps = ptr->eps;

		for(int i = 0;i < N;i++)
		{
			double tmp_x3[3] = 
			{
				p[0]*X3[i*3+0] + p[1]*X3[i*3+1] + p[2]*X3[i*3+2],
				p[3]*X3[i*3+0] + p[4]*X3[i*3+1] + p[5]*X3[i*3+2],
				p[6]*X3[i*3+0] + p[7]*X3[i*3+1] + p[8]*X3[i*3+2]
			};

			double tmp_x2[2] =
			{
				tmp_x3[0]*tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps),
				tmp_x3[1]*tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)
			};


			hx[i*2+0] = tmp_x2[0] - X2[i*2+0];
			hx[i*2+1] = tmp_x2[1] - X2[i*2+1];
		}
		return true;
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_estimate_H_jac(const T* p, T* jx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const int N = ptr->n_pts;

		const double eps = ptr->eps;

		//n === N*2, m===9
		memset(jx,0,sizeof(T)*m*n);
		for(int i = 0;i < N;i++)
		{

			double tmp_x3[3] = 
			{
				p[0]*X3[i*3+0] + p[1]*X3[i*3+1] + p[2]*X3[i*3+2],
				p[3]*X3[i*3+0] + p[4]*X3[i*3+1] + p[5]*X3[i*3+2],
				p[6]*X3[i*3+0] + p[7]*X3[i*3+1] + p[8]*X3[i*3+2]
			};

			jx[(2*i+0)*m+0] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+0];
			jx[(2*i+0)*m+1] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+1];
			jx[(2*i+0)*m+2] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+2];
			jx[(2*i+0)*m+6] = -tmp_x3[0]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+0];
			jx[(2*i+0)*m+7] = -tmp_x3[0]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+1];
			jx[(2*i+0)*m+8] = -tmp_x3[0]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+2];
			jx[(2*i+1)*m+3] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+0];
			jx[(2*i+1)*m+4] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+1];
			jx[(2*i+1)*m+5] = tmp_x3[2]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+2];
			jx[(2*i+1)*m+6] = -tmp_x3[1]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+0];
			jx[(2*i+1)*m+7] = -tmp_x3[1]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+1];
			jx[(2*i+1)*m+8] = -tmp_x3[1]/(tmp_x3[2]*tmp_x3[2]+eps*eps)*X3[i*3+2];
		}
		return true;

	}

	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_estimate_H(T* H, int n_pts, const T* X3, const T* X2, int max_iter, double eps /* = 1e-9*/)
	{


		H[1] = H[2] = H[3] = H[5] = H[6] = H[7] = 0;
		H[0] = H[4] = 0;
		H[8] = 1;


		///
		Calib_Data_Header<T> data;
		data.n_pts = n_pts;
		data.X3 = X3;
		data.X2 = X2;
		data.eps = eps;

		ZQ_LevMarOptions opts;
		opts.tol_max_jte = 1e-45;
		opts.tol_dx_square = 1e-45;
		opts.tol_e_square = 1e-45;
		ZQ_LevMarReturnInfos infos;
		T* hx = new T[n_pts*2];
		memset(hx,0,sizeof(T)*n_pts*2);

		if(!ZQ_LevMar::ZQ_LevMar_Der<T>(_estimate_H_func<T>,_estimate_H_jac<T>,H,hx,9,n_pts*2,max_iter,opts,infos,&data))
		{
			delete []hx;
			return false;
		}
		delete []hx;
		return true;
	}

	
	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images. 
	left hand coordinates.
	*/
	template<class T>
	void ZQ_Calibration::_get_v_i_j(const T* H, int i, int j, T* vij)
	{
		vij[0] = H[i]*H[j];
		vij[1] = H[1*3+i]*H[j] + H[i]*H[1*3+j];
		vij[2] = H[2*3+i]*H[j] + H[i]*H[2*3+j];
		vij[3] = H[1*3+i]*H[1*3+j];
		vij[4] = H[2*3+i]*H[1*3+j]+H[1*3+i]*H[2*3+j];
		vij[5] = H[2*3+i]*H[2*3+j];
	}

	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images. 
	left hand coordinates.
	*/
	template<class T>
	void ZQ_Calibration::_get_v_no_distortion(const T* H, T* row1, T* row2)
	{
		double v11[6],v22[6],v12[6];
		_get_v_i_j(H,0,0,v11);
		_get_v_i_j(H,0,1,v12);
		_get_v_i_j(H,1,1,v22);
		memcpy(row1,v12,sizeof(double)*6);
		for(int i = 0;i < 6;i++)
			row2[i] = v11[i]-v22[i];
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images. 
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_compute_b(const T* VV, int m, int n, T* b)
	{
		ZQ_Matrix<T> VVmat(m,n),U(m,n),S(n,n),V(n,n);
		for(int i = 0;i < m;i++)
		{
			for(int j = 0;j < n;j++)
				VVmat.SetData(i,j,VV[i*n+j]);
		}

		if(!ZQ_SVD::Decompose(VVmat,U,S,V))
			return false;
		for(int i = 0;i < n;i++)
		{
			bool flag;
			b[i] = V.GetData(i,n-1,flag);
		}
		return true;
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images. 
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_compute_A_from_b(const T* b, T* A)
	{
		double B[9] = 
		{
			b[0], b[1], b[2],
			b[1], b[3], b[4],
			b[2], b[4], b[5]
		};

		if(B[0*3+0]*B[1*3+1]-B[0*3+1]*B[0*3+1] == 0)
			return false;
		double v0 = (B[0*3+1]*B[0*3+2]-B[0*3+0]*B[1*3+2])/(B[0*3+0]*B[1*3+1]-B[0*3+1]*B[0*3+1]);
		if(B[0] == 0)
			return false;
		double lambda = B[2*3+2] - (B[0*3+2]*B[0*3+2]+v0*(B[0*3+1]*B[0*3+2]-B[0*3+0]*B[1*3+2]))/B[0];
		if(B[0*3+0] == 0)
			return false;
		if(lambda/B[0*3+0] < 0)
			return false;
		double alpha = sqrt((lambda/B[0*3+0]));
		if(B[0*3+0]*B[1*3+1]-B[0*3+1]*B[0*3+1] == 0)
			return false;
		if(lambda*B[0*3+0]/(B[0*3+0]*B[1*3+1]-B[0*3+1]*B[0*3+1]) < 0)
			return false;
		double beta = sqrt(lambda*B[0*3+0]/(B[0*3+0]*B[1*3+1]-B[0*3+1]*B[0*3+1]));
		if(lambda == 0)
			return false;
		double gamma = -B[0*3+1]*alpha*alpha*beta/lambda;
		if(beta == 0)
			return false;
		double u0 = gamma*v0/beta-B[0*3+2]*alpha*alpha/lambda;

		A[0] = alpha;
		A[1] = gamma;
		A[2] = u0;
		A[3] = 0;
		A[4] = beta;
		A[5] = v0;
		A[6] = 0;
		A[7] = 0;
		A[8] = 1;

		return true;
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images. 
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_compute_RT_from_AH(const T* H, const T* A, T* R, T* tt)
	{
		ZQ_Matrix<double> Amat(3,3),Hmat(3,3),RTmat(3,3);
		for(int i = 0;i < 3;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				Amat.SetData(i,j,A[i*3+j]);
				Hmat.SetData(i,j,H[i*3+j]);
			}
		}

		if(!ZQ_SVD::Solve(Amat,RTmat,Hmat))
			return false;

		double r1[3],r2[3],t[3],r3[3];
		bool flag;
		r1[0] = RTmat.GetData(0,0,flag);
		r1[1] = RTmat.GetData(1,0,flag);
		r1[2] = RTmat.GetData(2,0,flag);
		r2[0] = RTmat.GetData(0,1,flag);
		r2[1] = RTmat.GetData(1,1,flag);
		r2[2] = RTmat.GetData(2,1,flag);
		t[0] = RTmat.GetData(0,2,flag);
		t[1] = RTmat.GetData(1,2,flag);
		t[2] = RTmat.GetData(2,2,flag);
		double scale1 = sqrt(r1[0]*r1[0]+r1[1]*r1[1]+r1[2]*r1[2]);
		double scale2 = sqrt(r2[0]*r2[0]+r2[1]*r2[1]+r2[2]*r2[2]);

		if(t[2] < 0)
		{
			scale1 = -scale1;
			scale2 = -scale2;
		}

		if(scale1 != 0)
		{
			r1[0] /= scale1;
			r1[1] /= scale1;
			r1[2] /= scale1;

			t[0] /= scale1;
			t[1] /= scale1;
			t[2] /= scale1;
		}
		else
		{
			return false;
		}

		if(scale2 != 0)
		{
			r2[0] /= scale2;
			r2[1] /= scale2;
			r2[2] /= scale2;
		}
		else
		{
			return false;
		}

		r3[0] = r1[1]*r2[2]-r1[2]*r2[1];
		r3[1] = -r1[0]*r2[2]+r1[2]*r2[0];
		r3[2] = r1[0]*r2[1]-r1[1]*r2[0];


		R[0] = r1[0]; R[1] = r2[0]; R[2] = r3[0];
		R[3] = r1[1]; R[4] = r2[1]; R[5] = r3[1];
		R[6] = r1[2]; R[7] = r2[2]; R[8] = r3[2];
		tt[0] = t[0]; tt[1] = t[1]; tt[2] = t[2];

		return true;
	}


	/*
	refer to the paper:
	A flexible new technique for camera calibration[J]. Zhang Z. PAMI 2000.
	compute the camera intrinsic parameter with at least  3 checkboard images.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::calib_estimate_k_int_rT_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter, T* intrinsic_para, T* rT, double eps /* = 1e-9 */ )
	{
		if(n_cams < 3)
			return false;

		T* H = new T[9*n_cams];
		T* M = new T[3*n_pts];
		for(int i = 0;i < n_pts;i++)
		{
			M[i*3+0] = X3[i*3+0];
			M[i*3+1] = X3[i*3+1];
			M[i*3+2] = 1;
		}

		for(int cc = 0;cc < n_cams;cc++)
		{
			if(!_estimate_H(H+cc*9,n_pts,M,X2+n_pts*2*cc,max_iter,eps))
			{
				delete []H;
				delete []M;
				return false;
			}
		}

		double* V = new double[2*n_cams*6];
		for(int cc = 0;cc < n_cams;cc++)
		{
			_get_v_no_distortion(H+cc*9,V+(cc*2)*6,V+(cc*2+1)*6);
		}


		double b[6];
		if(!_compute_b(V,2*n_cams,6,b))
		{
			delete []H;
			delete []M;
			delete []V;
			return false;
		}

		double rec_A[9];
		if(!_compute_A_from_b(b,rec_A))
		{
			delete []H;
			delete []M;
			delete []V;
			return false;
		}

		for(int cc = 0; cc < n_cams;cc++)
		{
			double rec_R[9],rec_T[3];
			if(!_compute_RT_from_AH(H+cc*9,rec_A,rec_R,rec_T))
			{
				delete []H;
				delete []M;
				delete []V;
				return false;
			}
			if(!ZQ_Rodrigues::ZQ_Rodrigues_R2r_fun(rec_R,rT+cc*6))
			{
				delete []H;
				delete []M;
				delete []V;
				return false;
			}
			memcpy(rT+cc*6+3,rec_T,sizeof(double)*3);
		}


		intrinsic_para[0] = rec_A[0];
		intrinsic_para[1] = rec_A[4];
		intrinsic_para[2] = rec_A[1];
		intrinsic_para[3] = rec_A[2];
		intrinsic_para[4] = rec_A[5];

		delete []H;
		delete []M;
		delete []V;
		return true;
	}

	/*calibrate camera: intrinsic parameter,
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_calib_estimate_no_distortion_func(const T* p, T* hx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;
		const int n_cams = ptr->n_cams;

		const double eps = ptr->eps;

		//intrinsic_num = 5, ext_num = 6 * n_cams
		int intrinsic_num = 5;
		double alpha = p[0];
		double beta = p[1];
		double gamma = p[2];
		double u0 = p[3];
		double v0 = p[4];

		T A[9] = {
			alpha, gamma, u0,
			0,beta,v0,
			0,0,1
		};

		T* tmp_X2 = new T[N*2];

		for(int cc = 0;cc < n_cams;cc++)
		{
			const T* r = p+5 + cc*6;
			const T* t = p+5 + cc*6 + 3;
			T R[9];
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(r,R))
			{
				//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
				delete []tmp_X2;
				return false;
			}

			proj_no_distortion(N,A,R,t,X3,tmp_X2,eps);

			for(int i = 0;i < N*2;i++)
				hx[N*2*cc+i] = tmp_X2[i] - X2[N*2*cc+i];
		}
		delete []tmp_X2;
		return true;
	}

	
	/*calibrate camera: intrinsic parameter,
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_calib_estimate_no_distortion_jac(const T* p, T* jx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;
		const int n_cams = ptr->n_cams;

		const double eps = ptr->eps;

		memset(jx,0,sizeof(T)*m*n);

		//
		double alpha = p[0];
		double beta = p[1];
		double gamma = p[2];
		double u0 = p[3];
		double v0 = p[4];

		T A[9] = {
			alpha, gamma, u0,
			0,beta,v0,
			0,0,1
		};

		T dAdint[54] = {
			1,0,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,0,
			0,1,0,0,0,
			0,0,0,0,1,
			0,0,0,0,0,
			0,0,0,0,0,
			0,0,0,0,0
		};

		//intrinsic_num = 5, ext_num = 6 * n_cams
		for(int cc = 0;cc < n_cams;cc++)
		{
			T dRdr[27],R[9];
			const T *tt = p+5+cc*6+3;
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_jac(p+5+6*cc,dRdr))
			{
				//printf("error: Rodrigues r to R failed: (%d)%s\n",__LINE__,__FILE__);
				return false;
			}
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(p+5+6*cc,R))
			{
				//printf("error: Rodrigues r to R failed: (%d)%s\n",__LINE__,__FILE__);
				return false;
			}

			int cur_row_shift = N*2*cc;
			for(int i = 0;i < N;i++)
			{
				int cur_row0 = cur_row_shift + i*2+0;
				int cur_row1 = cur_row_shift + i*2+1;
				T cur_X3[3] = {X3[i*3+0],X3[i*3+1],X3[i*3+2]};

				//var1 = R*X3+T: dvar1dR,dvar1dT
				T var1[3] = {
					R[0]*cur_X3[0]+R[1]*cur_X3[1]+R[2]*cur_X3[2] + tt[0],
					R[3]*cur_X3[0]+R[4]*cur_X3[1]+R[5]*cur_X3[2] + tt[1],
					R[6]*cur_X3[0]+R[7]*cur_X3[1]+R[8]*cur_X3[2] + tt[2]
				};

				T dvar1dR[27] = {
					cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,0,0,0,
					0,0,0,cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,
					0,0,0,0,0,0,cur_X3[0],cur_X3[1],cur_X3[2]
				};
				T dvar1dT[9] = 
				{
					1,0,0,
					0,1,0,
					0,0,1
				};

				//
				T dvar1drT[18] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 3;jjj++)
					{
						for(int kkk = 0;kkk < 9;kkk++)
							dvar1drT[iii*6+jjj] += dvar1dR[iii*9+kkk]*dRdr[kkk*3+jjj];
						dvar1drT[iii*6+jjj+3] = dvar1dT[iii*3+jjj];
					}
				}


				//var2 = A*var1: dvar2dA, dvar2dvar1
				T var2[3] = {
					A[0]*var1[0]+A[1]*var1[1]+A[2]*var1[2],
					A[3]*var1[0]+A[4]*var1[1]+A[5]*var1[2],
					A[6]*var1[0]+A[7]*var1[1]+A[8]*var1[2]
				};

				T dvar2dA[27] = {
					var1[0],var1[1],var1[2],0,0,0,0,0,0,
					0,0,0,var1[0],var1[1],var1[2],0,0,0,
					0,0,0,0,0,0,var1[0],var1[1],var1[2]
				};
				T dvar2dvar1[9] = {
					A[0],A[1],A[2],
					A[3],A[4],A[5],
					A[6],A[7],A[8]
				};
				T dvar2drT[18] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 6;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dvar2drT[iii*6+jjj] += dvar2dvar1[iii*3+kkk]*dvar1drT[kkk*6+jjj];
					}
				}
				T dvar2dint[15] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 5;jjj++)
					{
						for(int kkk = 0;kkk < 9;kkk++)
							dvar2dint[iii*5+jjj] += dvar2dA[iii*9+kkk]*dAdint[kkk*5+jjj];
					}
				}


				//X2 = [var2(1)/var2(3);var2(2)/var2(3)]
				T cur_X2[2] = {
					var2[0]*var2[2]/(var2[2]*var2[2]+eps*eps),
					var2[1]*var2[2]/(var2[2]*var2[2]+eps*eps)
				};

				T dX2dvar2[6] = 
				{
					var2[2]/(var2[2]*var2[2]+eps*eps),0,-var2[0]/(var2[2]*var2[2]+eps*eps),
					0,var2[2]/(var2[2]*var2[2]+eps*eps),-var2[1]/(var2[2]*var2[2]+eps*eps)
				};

				T dX2dint[10] = {0};
				T dX2drT[12] = {0};
				for(int iii = 0;iii < 2;iii++)
				{
					for(int jjj = 0;jjj < 5;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dX2dint[iii*5+jjj] += dX2dvar2[iii*3+kkk]*dvar2dint[kkk*5+jjj];
					}
				}

				for(int iii = 0;iii < 2;iii++)
				{
					for(int jjj = 0;jjj < 6;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dX2drT[iii*6+jjj] += dX2dvar2[iii*3+kkk]*dvar2drT[kkk*6+jjj];
					}
				}

				for(int iii = 0;iii < 5;iii++)
				{
					jx[cur_row0*m+iii] = dX2dint[iii];
					jx[cur_row1*m+iii] = dX2dint[5+iii];
				}

				for(int iii = 0;iii < 6;iii++)
				{
					jx[cur_row0*m+5+cc*6+iii] = dX2drT[iii];
					jx[cur_row1*m+5+cc*6+iii] = dX2drT[6+iii];
				}
			}
		}
		return true;
	}


	/*calibrate camera: intrinsic parameter,
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::calib_estimate_no_distortion_with_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter,T* intrinsic_para, T* rT, double eps /* = 1e-9 */)
	{

		///
		Calib_Data_Header<T> data;
		data.n_pts = n_pts;
		data.n_cams = n_cams;
		data.X3 = X3;
		data.X2 = X2;
		data.eps = eps;

		ZQ_LevMarOptions opts;
		ZQ_LevMarReturnInfos infos;
		opts.tol_max_jte = 1e-16;
		opts.tol_dx_square = 1e-16;
		opts.tol_e_square = 1e-16;

		T* hx = new double[n_pts*2*n_cams];
		memset(hx,0,sizeof(T)*n_pts*2*n_cams);
		T* p = new T[5+n_cams*6];
		memcpy(p,intrinsic_para,sizeof(T)*5);
		memcpy(p+5,rT,sizeof(T)*6*n_cams);
		
		if(!ZQ_LevMar::ZQ_LevMar_Der<T>(_calib_estimate_no_distortion_func<T>,_calib_estimate_no_distortion_jac<T>,p,hx,5+n_cams*6,n_pts*2*n_cams,max_iter,opts,infos,&data))
		{
			delete []hx;
			delete []p;
			return false;
		}

		memcpy(intrinsic_para,p,sizeof(T)*5);
		memcpy(rT,p+5,sizeof(T)*6*n_cams);
		return true;
		
	}


	/*calibrate camera: intrinsic parameter and distortion (k1,k2),
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_calib_estimate_k_int_rT_func(const T* p, T* hx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;
		const int n_cams = ptr->n_cams;

		const double eps = ptr->eps;

		//intrinsic_num = 5, k_num = 2, ext_num = 6 * n_cams
		int intrinsic_num = 5;
		double alpha = p[0];
		double beta = p[1];
		double gamma = p[2];
		double u0 = p[3];
		double v0 = p[4];

		const T* k = p+5;

		T A[9] = {
			alpha, gamma, u0,
			0,beta,v0,
			0,0,1
		};

		T* tmp_X2 = new T[N*2];

		for(int cc = 0;cc < n_cams;cc++)
		{
			const T* r = p+7 + cc*6;
			const T* t = p+7 + cc*6 + 3;
			T R[9];
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(r,R))
			{
				//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
				delete []tmp_X2;
				return false;
			}

			proj_distortion_k2(N,A,R,t,p+3,k,X3,tmp_X2,eps);

			for(int i = 0;i < N*2;i++)
				hx[N*2*cc+i] = tmp_X2[i] - X2[N*2*cc+i];
		}
		delete []tmp_X2;
		return true;

	}


	/*calibrate camera: intrinsic parameter and distortion (k1,k2),
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_calib_estimate_k_int_rT_jac(const T* p, T* jx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;
		const int n_cams = ptr->n_cams;

		const double eps = ptr->eps;

		memset(jx,0,sizeof(T)*m*n);

		//
		double alpha = p[0];
		double beta = p[1];
		double gamma = p[2];
		double u0 = p[3];
		double v0 = p[4];

		const T* k = p+5;

		T A[9] = {
			alpha, gamma, u0,
			0,beta,v0,
			0,0,1
		};

		T dAdint[54] = {
			1,0,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,0,
			0,1,0,0,0,
			0,0,0,0,1,
			0,0,0,0,0,
			0,0,0,0,0,
			0,0,0,0,0
		};

		//intrinsic_num = 5, ext_num = 6 * n_cams
		for(int cc = 0;cc < n_cams;cc++)
		{
			T dRdr[27],R[9];
			const T *tt = p+7+cc*6+3;
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_jac(p+7+6*cc,dRdr))
			{
				//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
				return false;
			}
			if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(p+7+6*cc,R))
			{
				//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
				return false;
			}

			int cur_row_shift = N*2*cc;
			for(int i = 0;i < N;i++)
			{
				int cur_row0 = cur_row_shift + i*2+0;
				int cur_row1 = cur_row_shift + i*2+1;
				T cur_X3[3] = {X3[i*3+0],X3[i*3+1],X3[i*3+2]};

				//var1 = R*X3+T: dvar1dR,dvar1dT
				T var1[3] = {
					R[0]*cur_X3[0]+R[1]*cur_X3[1]+R[2]*cur_X3[2] + tt[0],
					R[3]*cur_X3[0]+R[4]*cur_X3[1]+R[5]*cur_X3[2] + tt[1],
					R[6]*cur_X3[0]+R[7]*cur_X3[1]+R[8]*cur_X3[2] + tt[2]
				};

				T dvar1dR[27] = {
					cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,0,0,0,
					0,0,0,cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,
					0,0,0,0,0,0,cur_X3[0],cur_X3[1],cur_X3[2]
				};
				T dvar1dT[9] = 
				{
					1,0,0,
					0,1,0,
					0,0,1
				};

				//
				T dvar1drT[18] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 3;jjj++)
					{
						for(int kkk = 0;kkk < 9;kkk++)
							dvar1drT[iii*6+jjj] += dvar1dR[iii*9+kkk]*dRdr[kkk*3+jjj];
						dvar1drT[iii*6+jjj+3] = dvar1dT[iii*3+jjj];
					}
				}


				//var2 = A*var1: dvar2dA, dvar2dvar1
				T var2[3] = {
					A[0]*var1[0]+A[1]*var1[1]+A[2]*var1[2],
					A[3]*var1[0]+A[4]*var1[1]+A[5]*var1[2],
					A[6]*var1[0]+A[7]*var1[1]+A[8]*var1[2]
				};

				T dvar2dA[27] = {
					var1[0],var1[1],var1[2],0,0,0,0,0,0,
					0,0,0,var1[0],var1[1],var1[2],0,0,0,
					0,0,0,0,0,0,var1[0],var1[1],var1[2]
				};
				T dvar2dvar1[9] = {
					A[0],A[1],A[2],
					A[3],A[4],A[5],
					A[6],A[7],A[8]
				};
				T dvar2drT[18] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 6;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dvar2drT[iii*6+jjj] += dvar2dvar1[iii*3+kkk]*dvar1drT[kkk*6+jjj];
					}
				}
				T dvar2dint[15] = {0};
				for(int iii = 0;iii < 3;iii++)
				{
					for(int jjj = 0;jjj < 5;jjj++)
					{
						for(int kkk = 0;kkk < 9;kkk++)
							dvar2dint[iii*5+jjj] += dvar2dA[iii*9+kkk]*dAdint[kkk*5+jjj];
					}
				}


				//X2 = [var2(1)/var2(3);var2(2)/var2(3)]
				T cur_X2[2] = {
					var2[0]*var2[2]/(var2[2]*var2[2]+eps*eps),
					var2[1]*var2[2]/(var2[2]*var2[2]+eps*eps)
				};

				T dX2dvar2[6] = 
				{
					var2[2]/(var2[2]*var2[2]+eps*eps),0,-var2[0]/(var2[2]*var2[2]+eps*eps),
					0,var2[2]/(var2[2]*var2[2]+eps*eps),-var2[1]/(var2[2]*var2[2]+eps*eps)
				};

				T dX2dint[10] = {0};
				T dX2drT[12] = {0};
				for(int iii = 0;iii < 2;iii++)
				{
					for(int jjj = 0;jjj < 5;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dX2dint[iii*5+jjj] += dX2dvar2[iii*3+kkk]*dvar2dint[kkk*5+jjj];
					}
				}

				for(int iii = 0;iii < 2;iii++)
				{
					for(int jjj = 0;jjj < 6;jjj++)
					{
						for(int kkk = 0;kkk < 3;kkk++)
							dX2drT[iii*6+jjj] += dX2dvar2[iii*3+kkk]*dvar2drT[kkk*6+jjj];
					}
				}

				// var3 = x-u: 
				// var4 = var3^2
				// var5 = y-v
				// var6 = var5^2;
				// var7 = var4+var6
				// var8 = var7^2
				double var3 = cur_X2[0] - u0;
				double var4 = var3*var3;
				double var5 = cur_X2[1] - v0;
				double var6 = var5*var5;
				double var7 = var4+var6;
				double var8 = var7*var7;

				//
				T dudint[5] = {0,0,0,1,0};
				T dvdint[5] = {0,0,0,0,1};
				T dvar3dint[5];
				T dvar4dint[5];
				T dvar5dint[5];
				T dvar6dint[5];
				T dvar7dint[5];
				T dvar8dint[5];

				const T* dvar3drT = dX2drT;
				const T* dvar5drT = dX2drT+6;

				T dvar4drT[6];
				T dvar6drT[6];
				T dvar7drT[6];
				T dvar8drT[6];

				for(int iii = 0;iii < 5;iii++)
				{
					dvar3dint[iii] = dX2dint[iii] - dudint[iii];
					dvar4dint[iii] = 2*var3*dvar3dint[iii];
					dvar5dint[iii] = dX2dint[5+iii] - dvdint[iii];
					dvar6dint[iii] = 2*var5*dvar5dint[iii];
					dvar7dint[iii] = dvar4dint[iii]+dvar6dint[iii];
					dvar8dint[iii] = 2*var7*dvar7dint[iii];
				}

				for(int iii = 0;iii < 6;iii++)
				{
					dvar4drT[iii] = 2*var3*dvar3drT[iii];
					dvar6drT[iii] = 2*var5*dvar5drT[iii];
					dvar7drT[iii] = dvar4drT[iii] + dvar6drT[iii];
					dvar8drT[iii] = 2*var7*dvar7drT[iii];
				}

				T dxbardint[5];
				T dybardint[5];
				T dxbardrT[6];
				T dybardrT[6];
				for(int iii = 0;iii < 5;iii++)
				{
					dxbardint[iii] = dX2dint[iii]   + dvar3dint[iii]*(k[0]*var7+k[1]*var8) + var3*(k[0]*dvar7dint[iii]+k[1]*dvar8dint[iii]);
					dybardint[iii] = dX2dint[5+iii] + dvar5dint[iii]*(k[0]*var7+k[1]*var8) + var5*(k[0]*dvar7dint[iii]+k[1]*dvar8dint[iii]);
				}
				for(int iii = 0;iii < 6;iii++)
				{
					dxbardrT[iii] = dX2drT[iii]   + dvar3drT[iii]*(k[0]*var7+k[1]*var8) + var3*(k[0]*dvar7drT[iii]+k[1]*dvar8drT[iii]);
					dybardrT[iii] = dX2drT[iii+6] + dvar5drT[iii]*(k[0]*var7+k[1]*var8) + var5*(k[0]*dvar7drT[iii]+k[1]*dvar8drT[iii]);
				}

				double x_u0 = cur_X2[0] - u0;
				double y_v0 = cur_X2[1] - v0;
				double dXbardk1 = x_u0*(x_u0*x_u0+y_v0*y_v0);
				double dXbardk2 = x_u0*((x_u0*x_u0+y_v0*y_v0)*(x_u0*x_u0+y_v0*y_v0));
				double dYbardk1 = y_v0*(x_u0*x_u0+y_v0*y_v0);
				double dYbardk2 = y_v0*((x_u0*x_u0+y_v0*y_v0)*(x_u0*x_u0+y_v0*y_v0));

				jx[cur_row0*m+5] = dXbardk1;
				jx[cur_row0*m+6] = dXbardk2;
				jx[cur_row1*m+5] = dYbardk1;
				jx[cur_row1*m+6] = dYbardk2;


				for(int iii = 0;iii < 5;iii++)
				{
					jx[cur_row0*m+iii] = dxbardint[iii];
					jx[cur_row1*m+iii] = dybardint[iii];
				}

				for(int iii = 0;iii < 6;iii++)
				{
					jx[cur_row0*m+7+cc*6+iii] = dxbardrT[iii];
					jx[cur_row1*m+7+cc*6+iii] = dybardrT[iii];
				}
			}
		}
		return true;
	}


	/*calibrate camera: intrinsic parameter and distortion (k1,k2),
	based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::calib_estimate_k_int_rT_with_init(int n_cams, int n_pts,const T* X3, const T* X2, int max_iter, T* k,  T* intrinsic_para, T* rT, double eps /* = 1e-9 */)
	{
		///
		Calib_Data_Header<T> data;
		data.n_pts = n_pts;
		data.n_cams = n_cams;
		data.X3 = X3;
		data.X2 = X2;
		data.eps = eps;
		

		ZQ_LevMarOptions opts;
		ZQ_LevMarReturnInfos infos;
		opts.tol_max_jte = 1e-64;
		opts.tol_dx_square = 1e-64;
		opts.tol_e_square = 1e-64;

		T* hx = new double[n_pts*2*n_cams];
		memset(hx,0,sizeof(T)*n_pts*2*n_cams);
		T* p = new T[7+n_cams*6];
		memcpy(p,intrinsic_para,sizeof(T)*5);
		memcpy(p+5,k,sizeof(T)*2);
		memcpy(p+7,rT,sizeof(T)*6*n_cams);

		if(!ZQ_LevMar::ZQ_LevMar_Der<T>(_calib_estimate_k_int_rT_func<T>,_calib_estimate_k_int_rT_jac<T>,p,hx,7+n_cams*6,n_pts*n_cams*2,max_iter,opts,infos,&data))
		{
			delete []hx;
			delete []p;
			return false;
		}

		memcpy(intrinsic_para,p,sizeof(T)*5);
		memcpy(k,p+5,sizeof(T)*2);
		memcpy(rT,p+7,sizeof(T)*6*n_cams);
		delete []p;
		delete []hx;
		return true;
	}

	/*calibrate camera: intrinsic parameter and distortion (k1,k2),
	based on Lev-Mar optimization, do not need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::calib_estimate_k_int_rT_without_init(int n_cams, int n_pts, const T* X3, const T* X2, int max_iter, T* k, T* intrinsic_para, T* rT, double eps /* = 1e-9 */)
	{

		if(!calib_estimate_k_int_rT_init(n_cams,n_pts,X3,X2,max_iter,intrinsic_para,rT,eps))
			return false;

		k[0] = k[1] = 0;

		if(!calib_estimate_k_int_rT_with_init(n_cams,n_pts,X3,X2,max_iter,k,intrinsic_para,rT,eps))
			return false;
		return true; 
	}


	/*pose estimation based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_pose_estimate_no_distortion_func(const T* p, T* hx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		double eps = ptr->eps;
		int N = ptr->n_pts;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;

		const T* intrinsic_para = ptr->intrinsic_para;

		T A[9] = {
			intrinsic_para[0],intrinsic_para[2],intrinsic_para[3],
			0,intrinsic_para[1],intrinsic_para[4],
			0,0,1
		};

		const T* r = p;
		const T* tt = p+3;

		T R[9];
		T* tmp_X2 = new T[N*2];
		if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(r,R))
		{
			//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
			delete []tmp_X2;
			return false;
		}

		proj_no_distortion(N,A,R,tt,X3,tmp_X2,eps);

		for(int i = 0;i < N;i++)
		{
			hx[i*2+0] = tmp_X2[i*2+0] - X2[i*2+0];
			hx[i*2+1] = tmp_X2[i*2+1] - X2[i*2+1];
		}

		delete []tmp_X2;
		return true;
	}

	/*pose estimation based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::_pose_estimate_no_distortion_jac(const T* p, T* jx, int m, int n, const void* data)
	{
		const Calib_Data_Header<T>* ptr = (const Calib_Data_Header<T>*)data;
		const T* X3 = ptr->X3;
		const T* X2 = ptr->X2;
		const int N = ptr->n_pts;

		const double eps = ptr->eps;
		const T* intrinsic_para = ptr->intrinsic_para;

		memset(jx,0,sizeof(T)*m*n);

		//
		T A[9] = {
			intrinsic_para[0],intrinsic_para[2],intrinsic_para[3],
			0,intrinsic_para[1],intrinsic_para[4],
			0,0,1
		};

		
		T dRdr[27],R[9];
		const T *tt = p+3;
		if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_jac(p,dRdr))
		{
			//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
			return false;
		}
		if(!ZQ_Rodrigues::ZQ_Rodrigues_r2R_fun(p,R))
		{
			//printf("error: Rodrigues r to R failed:(%d)%s\n",__LINE__,__FILE__);
			return false;
		}

		for(int i = 0;i < N;i++)
		{
			int cur_row0 = i*2+0;
			int cur_row1 = i*2+1;
			T cur_X3[3] = {X3[i*3+0],X3[i*3+1],X3[i*3+2]};

			//var1 = R*X3+T: dvar1dR,dvar1dT
			T var1[3] = {
				R[0]*cur_X3[0]+R[1]*cur_X3[1]+R[2]*cur_X3[2] + tt[0],
				R[3]*cur_X3[0]+R[4]*cur_X3[1]+R[5]*cur_X3[2] + tt[1],
				R[6]*cur_X3[0]+R[7]*cur_X3[1]+R[8]*cur_X3[2] + tt[2]
			};

			T dvar1dR[27] = {
				cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,0,0,0,
				0,0,0,cur_X3[0],cur_X3[1],cur_X3[2],0,0,0,
				0,0,0,0,0,0,cur_X3[0],cur_X3[1],cur_X3[2]
			};
			T dvar1dT[9] = 
			{
				1,0,0,
				0,1,0,
				0,0,1
			};

			//
			T dvar1drT[18] = {0};
			for(int iii = 0;iii < 3;iii++)
			{
				for(int jjj = 0;jjj < 3;jjj++)
				{
					for(int kkk = 0;kkk < 9;kkk++)
						dvar1drT[iii*6+jjj] += dvar1dR[iii*9+kkk]*dRdr[kkk*3+jjj];
					dvar1drT[iii*6+jjj+3] = dvar1dT[iii*3+jjj];
				}
			}


			//var2 = A*var1: dvar2dA, dvar2dvar1
			T var2[3] = {
				A[0]*var1[0]+A[1]*var1[1]+A[2]*var1[2],
				A[3]*var1[0]+A[4]*var1[1]+A[5]*var1[2],
				A[6]*var1[0]+A[7]*var1[1]+A[8]*var1[2]
			};
			
			T dvar2dvar1[9] = {
				A[0],A[1],A[2],
				A[3],A[4],A[5],
				A[6],A[7],A[8]
			};
			T dvar2drT[18] = {0};
			for(int iii = 0;iii < 3;iii++)
			{
				for(int jjj = 0;jjj < 6;jjj++)
				{
					for(int kkk = 0;kkk < 3;kkk++)
						dvar2drT[iii*6+jjj] += dvar2dvar1[iii*3+kkk]*dvar1drT[kkk*6+jjj];
				}
			}

			//X2 = [var2(1)/var2(3);var2(2)/var2(3)]
			T cur_X2[2] = {
				var2[0]*var2[2]/(var2[2]*var2[2]+eps*eps),
				var2[1]*var2[2]/(var2[2]*var2[2]+eps*eps)
			};

			T dX2dvar2[6] = 
			{
				var2[2]/(var2[2]*var2[2]+eps*eps),0,-var2[0]/(var2[2]*var2[2]+eps*eps),
				0,var2[2]/(var2[2]*var2[2]+eps*eps),-var2[1]/(var2[2]*var2[2]+eps*eps)
			};

			T dX2drT[12] = {0};
			

			for(int iii = 0;iii < 2;iii++)
			{
				for(int jjj = 0;jjj < 6;jjj++)
				{
					for(int kkk = 0;kkk < 3;kkk++)
						dX2drT[iii*6+jjj] += dX2dvar2[iii*3+kkk]*dvar2drT[kkk*6+jjj];
				}
			}


			for(int iii = 0;iii < 6;iii++)
			{
				jx[cur_row0*m+iii] = dX2drT[iii];
				jx[cur_row1*m+iii] = dX2drT[6+iii];
			}
		}
		return true;
	}

	/*pose estimation based on Lev-Mar optimization, need good initialization.
	left hand coordinates.
	*/
	template<class T>
	bool ZQ_Calibration::pose_estimate_no_distortion_with_init(int n_pts, const T* X3, const T* X2, int max_iter, const T* intrinsic_para, T* rT, double& avg_err_square, double eps /* = 1e-9 */)
	{
		Calib_Data_Header<T> data;
		data.n_pts = n_pts;
		data.eps = eps;
		data.intrinsic_para = intrinsic_para;
		data.X3 = X3;
		data.X2 = X2;

		ZQ_LevMarOptions opts;
		ZQ_LevMarReturnInfos infos;
		opts.tol_e_square = 1e-16;
		opts.tol_max_jte = 1e-16;
		opts.tol_dx_square = 1e-16;

		double* hx = new double[n_pts*2];
		memset(hx,0,sizeof(double)*n_pts*2);

		if(!ZQ_LevMar::ZQ_LevMar_Der<T>(_pose_estimate_no_distortion_func<T>,_pose_estimate_no_distortion_jac<T>,rT,hx,6,n_pts*2,max_iter,opts,infos,&data))
		{
			delete []hx;
			return false;
		}
		avg_err_square = infos.final_e_square/n_pts;
		delete []hx;
		return true;
	}


	/*
	refer to the paper: 
	Model-Based Object Pose in 25 Lines of Codes. Daniel F. DeMenthon and Larry S. Davis. IJCV,1995.
	left hand coordinates.
	intrinsic_para[0-4]: fx, fy, shear, u0, v0. with no distortion.
	rT[0-5]: rx, ry, rz, Tx, Ty, Tz.  (rx,ry,rz,rw) is a quaternion.
	*/
	template<class T>
	bool ZQ_Calibration::posit_no_coplanar(int n_pts, const T* X3, const T* X2, int max_iter, const T* intrinsic_para, T* rT)
	{
		
		double focal_len = intrinsic_para[0];
		if(focal_len == 0)
			return false;
		double scale_y = intrinsic_para[1]/focal_len;


		T* A = new T[(n_pts-1)*3];
		T* B = new T[3*(n_pts-1)];
		T* X = new T[n_pts*2];
		for(int i = 0;i < n_pts-1;i++)
		{
			A[i*3+0] = X3[(i+1)*3+0]-X3[0];
			A[i*3+1] = X3[(i+1)*3+1]-X3[1];
			A[i*3+2] = X3[(i+1)*3+2]-X3[2];
		}

	
		for(int i = 0;i < n_pts;i++)
		{
			X[i*2+0] = X2[i*2+0]-intrinsic_para[3];
			X[i*2+1] = (X2[i*2+1]-intrinsic_para[4])*scale_y;
		}

		ZQ_Matrix<double> Amat(n_pts-1,3),Bmat(3,n_pts-1);
		for(int i = 0;i < n_pts-1;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				Amat.SetData(i,j,A[i*3+j]);
			}
		}

		ZQ_SVD::Invert(Amat,Bmat);
		for(int i = 0;i < n_pts-1;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				bool flag;
				B[j*(n_pts-1)+i] = Bmat.GetData(j,i,flag);
			}
		}

		T* epsilon = new T[n_pts-1];
		T* xx = new T[n_pts-1];
		T* yy = new T[n_pts-1];

		memset(epsilon,0,sizeof(T)*(n_pts-1));

		double Z0 = 0;

		double last_ii[3],last_jj[3];
		double tolX = 1e-16;

		int it;
		for(it = 0;it < max_iter;it++)
		{
			for(int i = 0;i < n_pts-1;i++)
			{
				xx[i] = X[(i+1)*2+0]*(1+epsilon[i]) - X[0];
				yy[i] = X[(i+1)*2+1]*(1+epsilon[i]) - X[1];
			}

			double II[3] = {0},JJ[3] = {0};
			for(int i = 0;i < 3;i++)
			{
				for(int j = 0;j < n_pts-1;j++)
				{
					II[i] += B[i*(n_pts-1)+j]*xx[j];
					JJ[i] += B[i*(n_pts-1)+j]*yy[j];
				}
			}

			double lenII = sqrt(II[0]*II[0]+II[1]*II[1]+II[2]*II[2]);
			double lenJJ = sqrt(JJ[0]*JJ[0]+JJ[1]*JJ[1]+JJ[2]*JJ[2]);

			if(lenII == 0 || lenJJ == 0)
			{
				delete []A;
				delete []B;
				delete []X;
				delete []epsilon;
				delete []xx;
				delete []yy;
				return false;
			}
			double ii[3] = {II[0]/lenII,II[1]/lenII,II[2]/lenII};
			double jj[3] = {JJ[0]/lenJJ,JJ[1]/lenJJ,JJ[2]/lenJJ};
			
			
			if(it != 0)
			{
				double delta_ii = 0,delta_jj=0;
				for(int i = 0;i < 3;i++)
				{
					delta_ii += (last_ii[i]-ii[i])*(last_ii[i]-ii[i]);
					delta_jj += (last_jj[i]-jj[i])*(last_jj[i]-jj[i]);
				}
				if(delta_ii < tolX*tolX && delta_jj < tolX*tolX)
					break;
			}

			memcpy(last_ii,ii,sizeof(double)*3);
			memcpy(last_jj,jj,sizeof(double)*3);

			double s = 0.5*(lenII+lenJJ);
			Z0 = focal_len/s;
			double kk[3] = {
				ii[1]*jj[2]-ii[2]*jj[1],
				ii[2]*jj[0]-ii[0]*jj[2],
				ii[0]*jj[1]-ii[1]*jj[0]
			};
			for(int i = 0;i < n_pts-1;i++)
				epsilon[i] = (A[i*3+0]*kk[0]+A[i*3+1]*kk[1]+A[i*3+2]*kk[2])/Z0;


		}
		//printf("it = %d\n",it);

		
		double kk[3] = {
			last_ii[1]*last_jj[2]-last_ii[2]*last_jj[1],
			last_ii[2]*last_jj[0]-last_ii[0]*last_jj[2],
			last_ii[0]*last_jj[1]-last_ii[1]*last_jj[0]
		};
		
		double len_kk = sqrt(kk[0]*kk[0]+kk[1]*kk[1]+kk[2]*kk[2]);
		if(len_kk == 0)
		{
			delete []A;
			delete []B;
			delete []X;
			delete []epsilon;
			delete []xx;
			delete []yy;
			return false;
		}

		kk[0] /= len_kk;
		kk[1] /= len_kk;
		kk[2] /= len_kk;

		double jj[3] = 
		{
			kk[1]*last_ii[2]-kk[2]*last_ii[1],
			kk[2]*last_ii[0]-kk[0]*last_ii[2],
			kk[0]*last_ii[1]-kk[1]*last_ii[0]
		};

		double O[3] = {
			X3[0]-Z0/focal_len*(X[0]*last_ii[0] + X[1]*jj[0] + focal_len*kk[0]),
			X3[1]-Z0/focal_len*(X[0]*last_ii[1] + X[1]*jj[1] + focal_len*kk[1]),
			X3[2]-Z0/focal_len*(X[0]*last_ii[2] + X[1]*jj[2] + focal_len*kk[2])
		};
		ZQ_Matrix<double> Mmat(4,4),invM(4,4);
		for(int i = 0;i < 3;i++)
		{
			Mmat.SetData(i,0,last_ii[i]);
			Mmat.SetData(i,1,jj[i]);
			Mmat.SetData(i,2,kk[i]);
			Mmat.SetData(i,3,O[i]);
		}
		Mmat.SetData(3,3,1);

		ZQ_SVD::Invert(Mmat,invM);

		T R[9];
		T* tt = rT+3;
		for(int i = 0;i < 3;i++)
		{
			bool flag;
			for(int j = 0;j < 3;j++)
			{
				R[i*3+j] = invM.GetData(i,j,flag);
			}
			tt[i] = invM.GetData(i,3,flag);
		}
		ZQ_Rodrigues::ZQ_Rodrigues_R2r_fun(R,rT);
		
		
		delete []A;
		delete []B;
		delete []X;
		delete []epsilon;
		delete []xx;
		delete []yy;
		return true;

	}

	

	/*
	refer to the paper:
	iterative pose estimation using coplanar feature points. Denis Oberkampf, Daniel F. DeMenthon, Larry  S. Davis. CVIU, 1995. 
	left hand coordinates.
	intrinsic_para[0-4]: fx, fy, shear, u0, v0. with no distortion.
	rT[0-5]: rx, ry, rz, Tx, Ty, Tz.  (rx,ry,rz,rw) is a quaternion.
	*/
	template<class T>
	bool ZQ_Calibration::posit_coplanar(int n_pts, const T* X3, const T* X2, int max_iter, double tol_E, const T* intrinsic_para, T* rT, T* reproj_err_square, double eps /* = 1e-9 */)
	{
		double focal_len = intrinsic_para[0];
		if(focal_len == 0)
			return false;
		double scale_y = intrinsic_para[1]/focal_len;

		T int_A[9] = {
			intrinsic_para[0],intrinsic_para[2],intrinsic_para[3],
			0,intrinsic_para[1],intrinsic_para[4],
			0,0,1
		};

		double tol_Error = n_pts*tol_E*tol_E;


		T* A = new T[n_pts*3];
		T* B = new T[3*n_pts];
		T* X = new T[n_pts*2];
		for(int i = 0;i < n_pts;i++)
		{
			A[i*3+0] = X3[i*3+0]-X3[0];
			A[i*3+1] = X3[i*3+1]-X3[1];
			A[i*3+2] = X3[i*3+2]-X3[2];
		}


		for(int i = 0;i < n_pts;i++)
		{
			X[i*2+0] = X2[i*2+0]-intrinsic_para[3];
			X[i*2+1] = (X2[i*2+1]-intrinsic_para[4])*scale_y;
		}

		ZQ_Matrix<double> Amat(n_pts,3),Bmat(3,n_pts);
		for(int i = 0;i < n_pts;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				Amat.SetData(i,j,A[i*3+j]);
			}
		}

		if(!ZQ_SVD::Invert(Amat,Bmat))
		{
			delete []A;
			delete []B;
			delete []X;
			return false;
		}
		for(int i = 0;i < n_pts;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				bool flag;
				B[j*n_pts+i] = Bmat.GetData(j,i,flag);
			}
		}

		ZQ_Matrix<double> Umat(n_pts,3),Smat(3,3),Vmat(3,3);
		double center[3] = {0};
		for(int i = 0;i < n_pts;i++)
		{
			center[0] += X3[i*3+0];
			center[1] += X3[i*3+1];
			center[2] += X3[i*3+2];
		}
		center[0] /= n_pts;
		center[1] /= n_pts;
		center[2] /= n_pts;
		for(int i = 0;i < n_pts;i++)
		{
			for(int j = 0;j < 3;j++)
			{
				Amat.SetData(i,j,A[i*3+j] - center[j]);
			}
		}
		if(!ZQ_SVD::Decompose(Amat,Umat,Smat,Vmat))
		{
			delete []A;
			delete []B;
			delete []X;
			return false;
		}

		double u[3];
		for(int i = 0;i < 3;i++)
		{
			bool flag;
			u[i] = Vmat.GetData(i,2,flag);
		}
		if(u[2] < 0)
		{
			u[0] = -u[0];
			u[1] = -u[1];
			u[2] = -u[2];
		}
		

		std::vector<Posit_Coplanar_Node<T>> last_candidates;
		std::vector<Posit_Coplanar_Node<T>> cur_candidates;
		Posit_Coplanar_Node<T> node;
		last_candidates.push_back(node);
		int it = 0;
		double diverge_ratio = 1.1;
		int selection_thresh = 8;
		
		do{
			//printf("%d ",it);
			bool has_find_solution = false;
			cur_candidates.clear();
			for(int cc = 0;cc < last_candidates.size();cc++)
			{
				Posit_Coplanar_Node<T> cur_node = last_candidates[cc];
				Posit_Coplanar_Node<T> new_node[2];
				
				double I0[3] = {0}, J0[3] = {0};
				for(int i = 0;i < n_pts;i++)
				{
					double epsilon;
					if(it == 0)
						epsilon = 0;
					else
						epsilon = 1.0/cur_node.Z0*(A[i*3+0]*cur_node.kk[0]+A[i*3+1]*cur_node.kk[1]+A[i*3+2]*cur_node.kk[2]);
					double xx = X[i*2+0]*(1.0+epsilon) - X[0];
					double yy = X[i*2+1]*(1.0+epsilon) - X[1];
					I0[0] += B[0*n_pts+i]*xx;
					I0[1] += B[1*n_pts+i]*xx;
					I0[2] += B[2*n_pts+i]*xx;
					J0[0] += B[0*n_pts+i]*yy;
					J0[1] += B[1*n_pts+i]*yy;
					J0[2] += B[2*n_pts+i]*yy;
				}

				double I0J0 = I0[0]*J0[0] + I0[1]*J0[1] + I0[2]*J0[2];
				double J02_I02 = (J0[0]*J0[0]+J0[1]*J0[1]+J0[2]*J0[2]) - (I0[0]*I0[0]+I0[1]*I0[1]+I0[2]*I0[2]);

				/*lambda*mu = -I0J0
				lambda^2 - mu^2 = J02-I02
				*/

				double delta = J02_I02*J02_I02+4*I0J0*I0J0;
				double lambda[2],mu[2];

				if(1)
				{
					double q = 0;
					if(J02_I02 <= 0)
						q = 0.5*(J02_I02-sqrt(delta));
					else
						q = 0.5*(J02_I02+sqrt(delta));

					
					if(q >= 0)
					{
						lambda[0] = sqrt(q);
						lambda[1] = -sqrt(q);
						if (lambda[0] == 0.0) 
						{
							mu[0] = 0.0;
							mu[1] = 0.0;
						}
						else
						{
							mu[0] = -I0J0/lambda[0];
							mu[1] = -I0J0/lambda[1];
						}
					}
					else
					{
						lambda[0] = sqrt(-(I0J0*I0J0)/q);
						lambda[1] = -lambda[0];
						if(lambda[0] == 0)
						{
							mu[0] = sqrt(-J02_I02);
							mu[1] = -mu[0];
						}
						else
						{
							mu[0] = -I0J0/lambda[0];
							mu[1] = -mu[0];
						}
					}

				}
				else
				{
					lambda[0] = sqrt(0.5*(J02_I02+sqrt(delta)));
					lambda[1] = -lambda[0];
					if(lambda[0] != 0)
					{
						mu[0] = -I0J0/lambda[0];
						mu[1] = -mu[0];
					}
					else
					{
						mu[0] = mu[1] = 0;
					}
				}
				

				//////////
				double II[2][3] = {
					{I0[0]+lambda[0]*u[0], I0[1]+lambda[0]*u[1], I0[2]+lambda[0]*u[2]},
					{I0[0]+lambda[1]*u[0], I0[1]+lambda[1]*u[1], I0[2]+lambda[1]*u[2]}
				};
				double JJ[2][3] = {
					{J0[0]+mu[0]*u[0], J0[1]+mu[0]*u[1], J0[2]+mu[0]*u[2]},
					{J0[0]+mu[1]*u[0], J0[1]+mu[1]*u[1], J0[2]+mu[1]*u[2]}
				};

				double cur_R[2][9];
				double cur_tt[2][3];
				double cur_Z0[2];
				double cur_kk[2][3];
				double cur_E[2];
				bool discard_flag[2] = {false,false};
				for(int dd = 0;dd < 2;dd++)
				{
					double lenII = sqrt(II[dd][0]*II[dd][0]+II[dd][1]*II[dd][1]+II[dd][2]*II[dd][2]);
					double lenJJ = sqrt(JJ[dd][0]*JJ[dd][0]+JJ[dd][1]*JJ[dd][1]+JJ[dd][2]*JJ[dd][2]);
					if(lenII == 0 || lenJJ == 0)
					{
						delete []A;
						delete []B;
						delete []X;
						return false;
					}
					double ii[3] = {II[dd][0]/lenII,II[dd][1]/lenII,II[dd][2]/lenII};
					double jj[3] = {JJ[dd][0]/lenJJ,JJ[dd][1]/lenJJ,JJ[dd][2]/lenJJ};
					double s = 0.5*(lenII+lenJJ);
					cur_Z0[dd] = focal_len/s;
					cur_kk[dd][0] = ii[1]*jj[2]-ii[2]*jj[1];
					cur_kk[dd][1] = ii[2]*jj[0]-ii[0]*jj[2];
					cur_kk[dd][2] = ii[0]*jj[1]-ii[1]*jj[0];

					double len_kk = sqrt(cur_kk[dd][0]*cur_kk[dd][0]+cur_kk[dd][1]*cur_kk[dd][1]+cur_kk[dd][2]*cur_kk[dd][2]);
					if(len_kk == 0)
					{
						delete []A;
						delete []B;
						delete []X;
						return false;
					}

					cur_kk[dd][0] /= len_kk;
					cur_kk[dd][1] /= len_kk;
					cur_kk[dd][2] /= len_kk;
					jj[0] = cur_kk[dd][1]*ii[2]-cur_kk[dd][2]*ii[1];
					jj[1] = cur_kk[dd][2]*ii[0]-cur_kk[dd][0]*ii[2];
					jj[2] = cur_kk[dd][0]*ii[1]-cur_kk[dd][1]*ii[0];

					double O[3] = {
						X3[0]-cur_Z0[dd]/focal_len*(X[0]*ii[0] + X[1]*jj[0] + focal_len*cur_kk[dd][0]),
						X3[1]-cur_Z0[dd]/focal_len*(X[0]*ii[1] + X[1]*jj[1] + focal_len*cur_kk[dd][1]),
						X3[2]-cur_Z0[dd]/focal_len*(X[0]*ii[2] + X[1]*jj[2] + focal_len*cur_kk[dd][2])
					};
					ZQ_Matrix<double> Mmat(4,4),invM(4,4);
					for(int i = 0;i < 3;i++)
					{
						Mmat.SetData(i,0,ii[i]);
						Mmat.SetData(i,1,jj[i]);
						Mmat.SetData(i,2,cur_kk[dd][i]);
						Mmat.SetData(i,3,O[i]);
					}
					Mmat.SetData(3,3,1);

					ZQ_SVD::Invert(Mmat,invM);

					for(int i = 0;i < 3;i++)
					{
						bool flag;
						for(int j = 0;j < 3;j++)
						{
							cur_R[dd][i*3+j] = invM.GetData(i,j,flag);
						}
						cur_tt[dd][i] = invM.GetData(i,3,flag);
					}
					T* cur_X2 = new T[n_pts*2];
					proj_no_distortion(n_pts,int_A,cur_R[dd],cur_tt[dd],X3,cur_X2,eps);
					cur_E[dd] = 0;
					for(int i = 0;i < n_pts;i++)
					{
						cur_E[dd] += (cur_X2[i*2+0]-X2[i*2+0])*(cur_X2[i*2+0]-X2[i*2+0])+(cur_X2[i*2+1]-X2[i*2+1])*(cur_X2[i*2+1]-X2[i*2+1]);
					}
					delete []cur_X2;
					for(int i = 0;i < n_pts;i++)
					{

						double tmp_pts_Z = cur_R[dd][6]*X3[i*3+0]+cur_R[dd][7]*X3[i*3+1]+cur_R[dd][8]*X3[i*3+2] + cur_tt[dd][2];
						if(tmp_pts_Z < 0)
						{
							discard_flag[dd] = true;
							break;
						}
					}

					
					new_node[dd].Error = cur_E[dd];
					memcpy(new_node[dd].R,cur_R[dd],sizeof(T)*9);
					memcpy(new_node[dd].tt,cur_tt[dd],sizeof(T)*3);
					ZQ_Rodrigues::ZQ_Rodrigues_R2r_fun(cur_R[dd],new_node[dd].rr);
					memcpy(new_node[dd].kk,cur_kk[dd],sizeof(T)*3);
					new_node[dd].Z0 = cur_Z0[dd];

				}/* for dd*/
				

				if(it == 0)
				{
					cur_candidates.push_back(new_node[0]);
					cur_candidates.push_back(new_node[1]);
				}
				else
				{
					if(last_candidates.size() < selection_thresh)
					{
						if(!discard_flag[0]/* && (new_node[0].Error < cur_node.Error*diverge_ratio || new_node[0].Error < tol_Error)*/)
							cur_candidates.push_back(new_node[0]);
						if(!discard_flag[1]/* && (new_node[1].Error < cur_node.Error*diverge_ratio || new_node[1].Error < tol_Error)*/)
							cur_candidates.push_back(new_node[1]);

					}
					else
					{
						if(!discard_flag[0] && !discard_flag[1])
						{
							if(new_node[0].Error < new_node[1].Error)
								cur_candidates.push_back(new_node[0]);
							else
								cur_candidates.push_back(new_node[1]);
						}
						else
						{

							if(!discard_flag[0])
								cur_candidates.push_back(new_node[0]);
							if(!discard_flag[1])
								cur_candidates.push_back(new_node[1]);
						}
					}

					if(new_node[0].Error < tol_Error || new_node[1].Error < tol_Error)
						has_find_solution = true;
				}

			}/*for cc*/

			
			if(cur_candidates.size() == 0)
				break;
			last_candidates = cur_candidates;
			//printf("%d ",last_candidates.size());
			if(has_find_solution)
				break;

			it++;

		}while(it <= max_iter );
		
		//printf("\n");

		//printf("it = %d\n",it);
		if(last_candidates.size() == 0)
		{
			delete []A;
			delete []B;
			delete []X;
			return false;
		}

		double min_error = last_candidates[0].Error;
		int best_idx = 0;
		for(int cc = 1;cc < last_candidates.size();cc++)
		{
			if(last_candidates[cc].Error < min_error)
			{
				best_idx = cc;
				min_error = last_candidates[cc].Error;
			}
		}

		ZQ_Rodrigues::ZQ_Rodrigues_R2r_fun(last_candidates[best_idx].R,rT);
		memcpy(rT+3,last_candidates[best_idx].tt,sizeof(T)*3);

		if(reproj_err_square != 0)
		{
			T* reproj_X2 = new T[n_pts*2];
			proj_no_distortion(n_pts,int_A,last_candidates[best_idx].R,last_candidates[best_idx].tt,X3,reproj_X2,eps);
			for(int i = 0;i < n_pts;i++)
			{
				reproj_err_square[i] = (double)(reproj_X2[i*2+0]-X2[i*2+0])*(reproj_X2[i*2+0]-X2[i*2+0])+(reproj_X2[i*2+1]-X2[i*2+1])*(reproj_X2[i*2+1]-X2[i*2+1]);
			}
			delete []reproj_X2;

		}
		

		delete []A;
		delete []B;
		delete []X;

		return true;
	}


	/*
	left hand coordinates.
	The base idea is to use the method proposed in the paper:
	iterative pose estimation using coplanar points. Denis Oberkampf, Daniel F. DeMenthon, Larry  S. Davis. CVPR, 1993. 
	However, I find it cannot make sure the method always converge to the optimal solution.
	But the translation seems to be near the optimal one according to my observations.
	So I choose 9 rotations to run Lev-Mar solvers to find a best solution.
	If all choices do not give a solution with avg_E < tol_avg_E, the best solution of the 9 will be returned,
	otherwise, the first one satisfying avg_E < tol_avg_E will be returned.
	*/
	template<class T>
	bool ZQ_Calibration::posit_coplanar_robust(int n_pts, const T* X3, const T* X2, int max_iter_posit, int max_iter_levmar, double tol_E_square, const T* intrinsic_para, T* rT, double& avg_E_square, double eps /* = 1e-9 */)
	{
		T* reproj_err_square = new T[n_pts];
		memset(reproj_err_square,0,sizeof(T)*n_pts);
		
		T init_rT[6];
		memcpy(init_rT,rT,sizeof(T)*6);
		if(!ZQ::ZQ_Calibration::posit_coplanar(n_pts,X3,X2,max_iter_posit,tol_E_square,intrinsic_para,init_rT,reproj_err_square,eps))
		{
			delete []reproj_err_square;
			return false;
		}
		avg_E_square = 0;
		for(int i = 0;i < n_pts;i++)
			avg_E_square += reproj_err_square[i]/n_pts;
		delete []reproj_err_square;

		if(avg_E_square <= tol_E_square)
		{	
			memcpy(rT,init_rT,sizeof(T)*6);
			return true;
		}
		


		T rand_rr[10][3] = 
		{
			{init_rT[0],init_rT[1],init_rT[2]},
			{0,0,0},
			{0.2,0.1,0.1},
			{0.2,0.1,-0.1},
			{0.2,-0.1,0.1},
			{0.2,-0.1,0.1},
			{-0.2,0.1,0.1},
			{-0.2,0.1,-0.1},
			{-0.2,-0.1,0.1},
			{-0.2,-0.1,-0.1}
		};

		T tmp_rT[6];
		
		for(int i = 0;i < 10;i++)
		{
			memcpy(tmp_rT,rand_rr[i],sizeof(double)*3);
			memcpy(tmp_rT+3,init_rT+3,sizeof(double)*3);
			double cur_avg_E_square = 0;
			if(!pose_estimate_no_distortion_with_init(n_pts,X3,X2,max_iter_levmar,intrinsic_para,tmp_rT,cur_avg_E_square,eps))
			{
				continue;
			}
			else
			{
				if(cur_avg_E_square < avg_E_square)
				{
					avg_E_square = cur_avg_E_square;
					memcpy(rT,tmp_rT,sizeof(T)*6);
				}
				if(avg_E_square <= tol_E_square)
					break;
			}
		}
	
		return true;
	}

}

#endif
