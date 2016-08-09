#ifndef _ZQ_IMAGE_PROCESSING_H_
#define _ZQ_IMAGE_PROCESSING_H_
#pragma once

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ZQ_CubicInterpolation.h"

namespace ZQ
{
	namespace ZQ_ImageProcessing
	{
		/*basic functions*/
		template<class T>
		T EnforceRange(const T& x,const int& MaxValue);

		/*bilinear interpolation*/
		template<class T>
		void BilinearInterpolate(const T* pImage,const int width,const int height, const int nChannels, const float x, const float y, T* result, bool use_period_coord);

		/*bicubic interpolation*/
		template<class T>
		void BicubicInterpolate(const T* pImage, const int width, const int height, const int nChannels, const float x, const float y, T* result, bool use_period_coord);


		/*resize image*/
		template<class T>
		void ResizeImage(const T* pSrcImage, T* pDstImage, const int srcWidth, const int srcHeight, const int nChannels, const int dstWidth,const int dstHeight);

		template<class T>
		void ResizeImageBicubic(const T* pSrcImage, T* pDstImage, const int srcWidth, const int srcHeight, const int nChannels, const int dstWidth, const int dstHeight);

		/*resize a flow field on MAC grid
		* U : (width+1)*height,
		* V : width*(height+1) 
		*/
		template<class T>
		void ResizeFlow(const T* pSrcU, const T* pSrcV, T* pDstU, T* pDstV, const int srcWidth, const int srcHeight, const int dstWidth, const int dstHeight);

		template<class T>
		void ResizeFlowBicubic(const T* pSrcU, const T* pSrcV, T* pDstU, T* pDstV, const int srcWidth, const int srcHeight, const int dstWidth, const int dstHeight);

		/*filter, x dimension*/
		template<class T>
		void Hfiltering(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels, const T* pfilter1D, const int fsize);

		/*filter, y dimension*/
		template<class T>
		void Vfiltering(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels, const T* pfilter1D, const int fsize);

		template<class T>
		void Laplacian(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels);

		/*warp image*/
		template<class T>
		void WarpImage(T* pWarpIm2, const T* pIm2, const T* pU, const T* pV, const int width, const int height, const int nChannels,const T* pIm1 = 0);

		template<class T>
		void WarpImageBicubic(T* pWarpIm2, const T* pIm2,const T* pU, const T* pV, const int width, const int height, const int nChannels,  const T* pIm1 = 0);

		/*********************************************************************************/
		/********************************** definitions **********************************/
		/*********************************************************************************/


		template<class T>
		T EnforceRange(const T& x,const int& MaxValue)
		{
			return __min(__max(x,0),MaxValue-1);
		}


		template<class T>
		void BilinearInterpolate(const T* pImage,const int width,const int height, const int nChannels, const float x, const float y, T* result, bool use_period_coord)
		{
			memset(result,0,sizeof(T)*nChannels);

			if(!use_period_coord)
			{
				float fx = EnforceRange(x,width);
				float fy = EnforceRange(y,height);
				int ix = floor(fx);
				int iy = floor(fy);
				float sx = fx-ix;
				float sy = fy-iy;


				for(int i = 0;i <= 1;i++)
				{
					for(int j = 0;j <= 1;j++)
					{
						int u = EnforceRange(ix+j,width);
						int v = EnforceRange(iy+i,height);

						for(int c = 0;c < nChannels;c++)
							result[c] += fabs(1-j-sx)*fabs(1-i-sy)*pImage[(v*width+u)*nChannels+c];
					}
				}
			}
			else
			{
				float shift_x = floor(x/width)*width;
				float shift_y = floor(y/height)*height;
				float xxx = x - shift_x;
				float yyy = y - shift_y;

				int ix = floor(xxx);
				int iy = floor(yyy);
				float sx = xxx - ix;
				float sy = yyy - iy;
				for(int i = 0;i <= 1;i++)
				{
					for(int j = 0;j <= 1;j++)
					{
						int u = (ix+j)%width;
						int v = (iy+i)%height;
						for(int c = 0;c < nChannels;c++)
							result[c] += fabs(1-j-sx)*fabs(1-i-sy)*pImage[(v*width+u)*nChannels+c];
					}
				}
			}	
		}


		template<class T>
		void BicubicInterpolate(const T* pImage, const int width, const int height, const int nChannels, const float x, const float y, T* result, bool use_period_coord)
		{
			memset(result,0,sizeof(T)*nChannels);

			if(!use_period_coord)
			{
				int ix = floor(x);
				int iy = floor(y);
				float sx = x-ix;
				float sy = y-iy;

				T data[16] = {0};
				for(int c = 0;c < nChannels;c++)
				{
					for(int i = 0;i < 4;i++)
					{
						for(int j = 0;j < 4;j++)
						{
							int cur_x = EnforceRange(ix-1+j,width);
							int cur_y = EnforceRange(iy-1+i,height);
							data[i*4+j] = pImage[(cur_y*width+cur_x)*nChannels+c];
						}
					}
					result[c] = ZQ_BicubicInterpolate(data,sx,sy);
				}
			}
			else
			{
				float shift_x = floor(x/width)*width;
				float shift_y = floor(y/height)*height;
				float xxx = x - shift_x;
				float yyy = y - shift_y;

				int ix = floor(xxx);
				int iy = floor(yyy);
				float sx = xxx - ix;
				float sy = yyy - iy;

				T data[16] = {0};
				for(int c = 0;c < nChannels;c++)
				{
					for(int i = 0;i < 4;i++)
					{
						for(int j = 0;j < 4;j++)
						{
							int cur_x = (ix-1+j+width)%width;
							int cur_y = (iy-1+i+height)%height;
							data[i*4+j] = pImage[(cur_y*width+cur_x)*nChannels+c];
						}
					}
					result[c] = ZQ_BicubicInterpolate(data,sx,sy);
				}
			}

		}


		template<class T>
		void ResizeImage(const T* pSrcImage, T* pDstImage, const int srcWidth, const int srcHeight, const int nChannels, const int dstWidth,const int dstHeight)
		{
			memset(pDstImage,0,sizeof(T)*dstWidth*dstHeight*nChannels);

			for(int i = 0;i < dstHeight;i++)
			{
				for(int j = 0;j < dstWidth;j++)
				{
					float coordx = (j+0.5)/dstWidth*srcWidth-0.5;
					float coordy = (i+0.5)/dstHeight*srcHeight-0.5;

					BilinearInterpolate(pSrcImage,srcWidth,srcHeight,nChannels,coordx,coordy,pDstImage+(i*dstWidth+j)*nChannels,false);
				}
			}
		}


		template<class T>
		void ResizeImageBicubic(const T* pSrcImage, T* pDstImage, const int srcWidth, const int srcHeight, const int nChannels, const int dstWidth, const int dstHeight)
		{
			memset(pDstImage,0,sizeof(T)*dstWidth*dstHeight*nChannels);

			T tmpData[16] = {0};

			for(int i = 0;i < dstHeight;i++)
			{
				for(int j = 0;j < dstWidth;j++)
				{
					float coordx = (j+0.5)/dstWidth*srcWidth-0.5;
					float coordy = (i+0.5)/dstHeight*srcHeight-0.5;

					int ix = floor(coordx);
					int iy = floor(coordy);
					float fx = coordx-ix;
					float fy = coordy-iy;


					for(int c = 0;c < nChannels;c++)
					{
						for(int s = 0;s < 4;s++)
						{
							for(int t = 0;t < 4;t++)
							{
								int tmpx = EnforceRange(ix-1+t,srcWidth);
								int tmpy = EnforceRange(iy-1+s,srcHeight);

								tmpData[s*4+t] = pSrcImage[(tmpy*srcWidth+tmpx)*nChannels+c];
							}
						}
						pDstImage[(i*dstWidth+j)*nChannels+c] = ZQ_BicubicInterpolate(tmpData,fx,fy);
					}
				}
			}
		}


		template<class T>
		void ResizeFlow(const T* pSrcU, const T* pSrcV, T* pDstU, T* pDstV, const int srcWidth, const int srcHeight, const int dstWidth, const int dstHeight)
		{
			memset(pDstU,0,sizeof(T)*(dstWidth+1)*dstHeight);
			memset(pDstV,0,sizeof(T)*dstWidth*(dstHeight+1));

			//resize U
			for(int i = 0;i < dstHeight;i++)
			{
				for(int j = 0;j <= dstWidth;j++)
				{
					float coordx = (float)j/dstWidth*srcWidth;
					float coordy = (i+0.5f)/dstHeight*srcHeight - 0.5f;

					BilinearInterpolate(pSrcU,srcWidth+1,srcHeight,1,coordx,coordy,pDstU+i*(dstWidth+1)+j,false);
				}
			}

			//resize V
			for(int i = 0;i <= dstHeight;i++)
			{
				for(int j = 0;j < dstWidth;j++)
				{
					float coordx = (j+0.5f)/dstWidth*srcWidth - 0.5f;
					float coordy = (float)i/dstHeight*srcHeight;

					BilinearInterpolate(pSrcV,srcWidth,srcHeight+1,1,coordx,coordy,pDstV+i*dstWidth+j,false);
				}
			}
		}


		template<class T>
		void ResizeFlowBicubic(const T* pSrcU, const T* pSrcV, T* pDstU, T* pDstV, const int srcWidth, const int srcHeight, const int dstWidth, const int dstHeight)
		{
			memset(pDstU,0,sizeof(T)*(dstWidth+1)*dstHeight);
			memset(pDstV,0,sizeof(T)*dstWidth*(dstHeight+1));

			T tmpData[16] = {0};

			//resize U
			for(int i = 0;i < dstHeight;i++)
			{
				for(int j = 0;j <= dstWidth;j++)
				{
					float coordx = (float)j/dstWidth*srcWidth;
					float coordy = (i+0.5f)/dstHeight*srcHeight-0.5f;

					int ix = floor(coordx);
					int iy = floor(coordy);
					float fx = coordx-ix;
					float fy = coordy-iy;

					for(int s = 0;s < 4;s++)
					{
						for(int t = 0;t < 4;t++)
						{
							int tmpx = EnforceRange(ix-1+t,srcWidth+1);
							int tmpy = EnforceRange(iy-1+s,srcHeight);

							tmpData[s*4+t] = pSrcU[tmpy*(srcWidth+1)+tmpx];
						}
					}
					pDstU[i*(dstWidth+1)+j] = ZQ_BicubicInterpolate(tmpData,fx,fy);

				}
			}

			//resizeV
			for(int i = 0;i <= dstHeight;i++)
			{
				for(int j = 0;j < dstWidth;j++)
				{
					float coordx = (j+0.5f)/dstWidth*srcWidth-0.5f;
					float coordy = (float)i/dstHeight*srcHeight;
					int ix = floor(coordx);
					int iy = floor(coordy);
					float fx = coordx-ix;
					float fy = coordy-iy;

					for(int s = 0;s < 4;s++)
					{
						for(int t = 0;t < 4;t++)
						{
							int tmpx = EnforceRange(ix-1+t,srcWidth);
							int tmpy = EnforceRange(iy-1+s,srcHeight+1);

							tmpData[s*4+t] = pSrcV[tmpy*srcWidth+tmpx];
						}
					}
					pDstV[i*dstWidth+j] = ZQ_BicubicInterpolate(tmpData,fx,fy);
				}
			}
		}


		template<class T>
		void Hfiltering(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels, const T* pfilter1D, const int fsize)
		{
			memset(pDstImage,0,sizeof(T)*width*height*nChannels);

			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					for(int l = -fsize; l <= fsize;l++)
					{
						int jj = EnforceRange(j+l,width);
						for(int c = 0;c < nChannels;c++)
							pDstImage[(i*width+j)*nChannels+c] += pSrcImage[(i*width+jj)*nChannels+c]*pfilter1D[l+fsize];
					}
				}
			}
		}


		template<class T>
		void Vfiltering(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels, const T* pfilter1D, const int fsize)
		{
			memset(pDstImage,0,sizeof(T)*width*height*nChannels);

			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					for(int l = -fsize;l <= fsize;l++)
					{
						int ii = EnforceRange(i+l,height);
						for(int c = 0;c < nChannels;c++)
							pDstImage[(i*width+j)*nChannels+c] += pSrcImage[(ii*width+j)*nChannels+c]*pfilter1D[l+fsize];
					}
				}
			}
		}


		template<class T>
		void Laplacian(const T* pSrcImage, T* pDstImage, const int width, const int height, const int nChannels)
		{
			memset(pDstImage,0,sizeof(T)*width*height*nChannels);

			for(int i = 0;i < height;i++)
			{
				for(int j = 1;j < width-1;j++)
				{
					for(int c = 0;c < nChannels;c++)
						pDstImage[(i*width+j)*nChannels+c] += pSrcImage[(i*width+j+1)*nChannels+c] + pSrcImage[(i*width+j-1)*nChannels+c];
				}
				for(int c = 0;c < nChannels;c++)
					pDstImage[(i*width+0)*nChannels+c] += pSrcImage[(i*width+0)*nChannels+c] + pSrcImage[(i*width+1)*nChannels+c];
				for(int c = 0;c < nChannels;c++)
					pDstImage[(i*width+width-1)*nChannels+c] += pSrcImage[(i*width+width-2)*nChannels+c] + pSrcImage[(i*width+width-1)*nChannels+c];
			}

			for(int j = 0;j < width;j++)
			{
				for(int i = 1;i < height-1;i++)
				{
					for(int c = 0;c < nChannels;c++)
						pDstImage[(i*width+j)*nChannels+c] += pSrcImage[((i+1)*width+j)*nChannels+c] + pSrcImage[((i-1)*width+j)*nChannels+c];
				}
				for(int c = 0;c < nChannels;c++)
					pDstImage[(0*width+j)*nChannels+c] += pSrcImage[(0*width+j)*nChannels+c] + pSrcImage[(1*width+j)*nChannels+c];
				for(int c = 0;c < nChannels;c++)
					pDstImage[((height-1)*width+j)*nChannels+c] += pSrcImage[((height-2)*width+j)*nChannels+c] + pSrcImage[((height-1)*width+j)*nChannels+c];
			}

			for(int i = 0;i < height*width*nChannels;i++)
			{
				pDstImage[i] -= 4*pSrcImage[i];
			}
		}


		template<class T>
		void WarpImage(T* pWarpIm2, const T* pIm2, const T* pU, const T* pV, const int width, const int height, const int nChannels,const T* pIm1 /* = 0 */)
		{
			memset(pWarpIm2,0,sizeof(T)*width*height*nChannels);

			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					int offset = i*width+j;
					float x = j+pU[offset];
					float y = i+pV[offset];
					if(x < 0 || x > width-1 || y < 0 || y > height-1)
					{
						if(pIm1)
						{
							for(int c = 0;c < nChannels;c++)
								pWarpIm2[offset*nChannels+c] = pIm1[offset*nChannels+c];
						}

						continue;
					}
					BilinearInterpolate(pIm2,width,height,nChannels,x,y,pWarpIm2+offset*nChannels,false);
				}
			}
		}


		template<class T>
		void WarpImageBicubic(T* pWarpIm2, const T* pIm2,const T* pU, const T* pV, const int width, const int height, const int nChannels, const T* pIm1 /* = 0 */)
		{
			T tmpData[16] = {0};

			for(int i = 0;i < height;i++)
			{
				for(int j = 0;j < width;j++)
				{
					int offset = i*width+j;
					float x = j + pU[offset];
					float y = i + pV[offset];
					if(x < 0 || x > width-1 || y < 0 || y > height-1)
					{
						if(pIm1)
						{
							for(int c = 0;c < nChannels;c++)
								pWarpIm2[offset*nChannels+c] = pIm1[offset*nChannels+c];
						}

						continue;
					}

					int x0 = floor(x);
					int y0 = floor(y);
					float fx = x-x0;
					float fy = y-y0;

					for(int c = 0;c < nChannels;c++)
					{
						for(int s = 0;s < 4;s++)
						{
							for(int t = 0;t < 4;t++)
							{
								int tmpx = EnforceRange(x0-1+t,width);
								int tmpy = EnforceRange(y0-1+s,height);

								tmpData[s*4+t] = pIm2[(tmpy*width+tmpx)*nChannels+c];
							}
						}
						pWarpIm2[offset*nChannels+c] = ZQ_BicubicInterpolate(tmpData,fx,fy);
					}

				}
			}
		}

	}
}

#endif
