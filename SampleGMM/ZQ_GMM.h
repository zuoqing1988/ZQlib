#ifndef _ZQ_GMM_H_
#define _ZQ_GMM_H_
#pragma once 

#include "ZQ_MathBase.h"
#include "ZQ_Kmeans.h"
#include "ZQ_SVD.h"
#include <float.h>

namespace ZQ
{
	template<class T>
	class ZQ_GMM
	{
	public:
		/*
		pts :			nPts*dim
		init_centers :	k*dim
		out_probs :			npts*k
		out_mu :			k*dim
		out_sigma :			k*(dim*dim)
		out_weight :		k*1
		*/
		static bool GMM_with_init(int nPts, int dim, int k, const T* pts, const T* init_centers, T* out_prob, T* out_mu, T* out_sigma, T* out_weight, double thresh = 1e-9)
		{
			if (nPts <= 0 || dim <= 0 || k < 1 || k > nPts || pts == 0 || init_centers == 0 
				|| out_prob == 0 || out_mu == 0 || out_sigma == 0 || out_weight == 0)
				return false;

			_init_params(nPts, dim, k, pts, init_centers, out_mu, out_sigma, out_weight);
			double L_prev = -DBL_MAX;

			T* pGamma = new T[nPts*k];
			T* pGamma_sum = new T[nPts];
			T* X_shift = new T[dim];

			while (true)
			{
				/* Estiamtion Step */
				_calc_prob(nPts, dim, k, pts, out_mu, out_sigma, out_prob);

				// new value for pGamma
				for (int pp = 0; pp < nPts; pp++)
				{
					pGamma_sum[pp] = 0;
					for (int kk = 0; kk < k; kk++)
					{
						T tmp = out_prob[pp*k + kk] * out_weight[kk];
						pGamma[pp*k + kk] = tmp;
						pGamma_sum[pp] += tmp;
					}
					for (int kk = 0; kk < k; kk++)
						pGamma[pp*k + kk] /= pGamma_sum[pp];
				}
				
				/* Maximization Step */
				// new value for parameters of each Component
				for (int kk = 0; kk < k; kk++)
				{
					memset(out_mu + kk*dim, 0, sizeof(T)*dim);
					T Nk = 0;
					for (int pp = 0; pp < nPts; pp++)
					{
						for (int d = 0; d < dim; d++)
						{
							out_mu[kk*dim + d] += pGamma[pp*k + kk] * pts[pp*dim + d];
						}
						Nk += pGamma[pp*k + kk];
					}
					for (int d = 0; d < dim; d++)
						out_mu[kk*dim + d] /= Nk;

					out_weight[kk] = Nk/nPts;

					memset(out_sigma + kk*dim*dim, 0, sizeof(T)*dim*dim);
					for (int pp = 0; pp < nPts; pp++)
					{
						for (int d = 0; d < dim; d++)
							X_shift[d] = pts[pp*dim + d] - out_mu[kk*dim + d];
						for (int s = 0; s < dim; s++)
						{
							for (int t = 0; t < dim; t++)
							{
								out_sigma[kk*dim*dim + s*dim + t] += pGamma[pp*k + kk] * X_shift[s] * X_shift[t];
							}
						}
					}
					for (int d = 0; d < dim*dim; d++)
						out_sigma[kk*dim*dim + d] /= Nk;
				}
				

				// check for convergence
				double L = 0;
				for (int pp = 0; pp < nPts; pp++)
					L += log((double)pGamma_sum[pp]);
				if (L - L_prev < thresh)
					break;
				L_prev = L;
			}
			return true;
		}

		/*
		pts :			nPts*dim
		out_probs :			npts*k
		out_mu :			k*dim
		out_sigma :			k*(dim*dim)
		out_weight :		k*1
		out_init_centers :	NULL or k*dim
		*/
		static bool GMM(int nPts, int dim, int k, const T* pts, T* out_prob, T* out_mu, T* out_sigma, T* out_weight, T* out_init_centers = 0, double thresh = 1e-9)
		{
			if (nPts <= 0 || dim <= 0 || k > nPts || pts == 0 || out_prob == 0 || out_mu == 0 || out_sigma == 0 || out_weight == 0)
				return false;

			T* tmp_init_centers = 0;
			if (out_init_centers == 0)
				tmp_init_centers = new T[k*dim];
			else
				tmp_init_centers = out_init_centers;

			if (!ZQ_Kmeans<T>::_select_init_center(nPts, dim, k, pts, tmp_init_centers))
			{
				if (out_init_centers == 0)
					delete[]tmp_init_centers;
				return false;
			}

			if (!GMM_with_init(nPts, dim, k, pts, tmp_init_centers, out_prob, out_mu, out_sigma, out_weight, thresh))
			{
				if (out_init_centers == 0)
					delete[]tmp_init_centers;
				return false;
			}
			if (out_init_centers == 0)
				delete[]tmp_init_centers;
			return true;
		}

	private:
		static void _init_params(int nPts, int dim, int k, const T* pts, const T* init_centers, T* out_mu, T* out_sigma, T* out_weight)
		{
			
			int* counts = new int[k];
			int* idx = new int[nPts];
			T* X_shift = new T[dim];
			memset(counts, 0, sizeof(int)*k);
			memset(out_mu, 0, sizeof(T)*k*dim);
			memset(out_sigma, 0, sizeof(T)*k*dim*dim);
			for (int pp = 0; pp < nPts; pp++)
			{
				int k_id = 0;
				T min_dis = ZQ_Kmeans<T>::_distance2(dim, pts + pp*dim, init_centers);
				for (int kk = 1; kk < k; kk++)
				{
					T cur_dis = ZQ_Kmeans<T>::_distance2(dim, pts + pp*dim, init_centers+kk*dim);
					if (cur_dis < min_dis)
					{
						k_id = kk;
						min_dis = cur_dis;
					}
				}
				idx[pp] = k_id;
				counts[k_id]++;
				for (int d = 0; d < dim; d++)
				{
					out_mu[k_id*dim + d] += pts[pp*dim+d];
				}
			}

			for (int kk = 0; kk < k; kk++)
			{
				for (int d = 0; d < dim; d++)
					out_mu[kk*dim + d] /= counts[kk];
				out_weight[kk] = (double)counts[kk] / nPts;
			}
			
			
			for (int i = 0; i < nPts; i++)
			{
				int k_id = idx[i];
				for (int d = 0; d < dim; d++)
					X_shift[d] = pts[i*dim + d] - out_mu[k_id*dim + d];
				T* sigma = out_sigma + k_id*dim*dim;
				for (int s = 0; s < dim; s++)
				{
					for (int t = 0; t < dim; t++)
						sigma[s*dim + t] += X_shift[s] * X_shift[t];
				}
			}
			for (int kk = 0; kk < k; kk++)
			{
				for (int dd = 0; dd < dim*dim; dd++)
					out_sigma[kk*dim*dim + dd] /= __max(1, counts[kk] - 1);
			}
			memcpy(out_mu, init_centers, sizeof(T)*k*dim);

			delete[]X_shift;
			delete[]idx;
			delete[]counts;
		}

		static void _calc_prob(int nPts, int dim, int k, const T* pts, const T* mu, const T* sigma, T* out_prob)
		{
			ZQ_Matrix<double> Mat(dim, dim), invMat(dim, dim);
			double* MatPtr = Mat.GetDataPtr();
			double* invMatPtr = invMat.GetDataPtr();
			T* X_shift = new T[dim];
			double threshold = 1e-15;
			T* invSigma = new T[dim*dim];
			T* X_invSigma = new T[dim];

			double m_pi = atan(1.0) * 4;
			double coeff1 = pow(2 * m_pi, -0.5*dim);
			for (int kk = 0; kk < k; kk++)
			{
				for (int i = 0; i < dim; i++)
				{
					for (int j = 0; j < dim; j++)
						MatPtr[i*dim + j] = sigma[kk*dim*dim + i*dim + j] + ((i == j) ? threshold : 0);
				}
				ZQ_SVD::Invert(Mat, invMat);

				for (int i = 0; i < dim*dim; i++)
					invSigma[i] = invMatPtr[i];

				double coeff2 = sqrt(__max(0, ZQ_MathBase::Det(dim, invSigma)));
				
				for (int pp = 0; pp < nPts; pp++)
				{
					for (int d = 0; d < dim; d++)
						X_shift[d] = pts[pp*dim + d] - mu[kk*dim + d];
					ZQ_MathBase::MatrixMul(X_shift, invSigma, 1, 2, 2, X_invSigma);
					T tmp = ZQ_MathBase::DotProduct(dim, X_invSigma, X_shift);
					out_prob[pp*k + kk] = coeff1*coeff2*exp(-0.5*tmp);
				}
			}
			
			delete[]X_shift;
			delete[]X_invSigma;
			delete[]invSigma;
		}
	};
}

#endif