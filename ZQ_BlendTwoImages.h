#ifndef _ZQ_BLEND_TWO_IMAGES_H_
#define _ZQ_BLEND_TWO_IMAGES_H_
#pragma once

#include "ZQ_ScatteredInterpolationRBF.h"
#include "ZQ_ImageProcessing.h"

namespace ZQ
{
	class ZQ_BlendTwoImages
	{
	public:

		template<class T>
		static bool BlendTwoImages(const int width, const int height, const int nChannels, const T* image1, const T* image2, 
			const T* u, const T* v, const float weight1, T* out_image, int skip, int max_iter = 100, int num_of_neighbor = 5, double radius_scale = 1.5, bool cubic = false, const int blend_mode = 0)
		{
			if(image1 == 0 || image2 == 0 || u == 0 || v == 0 || out_image == 0)
				return false;

			memset(out_image,0,sizeof(T)*width*height*nChannels);


			ZQ_ScatteredInterpolationRBF<T> scatter;

			int nPixels = width*height;
			T* coord_for_img = new T[nPixels*2];
			T* vals_four_channels = new T[nPixels*4];

			memset(coord_for_img,0,sizeof(T)*nPixels*2);
			memset(vals_four_channels,0,sizeof(T)*nPixels*4);

			if(skip < 1) skip = 1;
			if(skip > 8) skip = 8;
			//int skip = 4;
			int nPts = 0;

			for(int i = 0;i < height;i+=skip)
			{
				for(int j = 0;j < width;j+=skip)
				{
					int offset = i*width+j;

					T cur_u = u[offset];
					T cur_v = v[offset];

					coord_for_img[nPts*2+0] = j + cur_u*(1-weight1);
					coord_for_img[nPts*2+1] = i + cur_v*(1-weight1);

					vals_four_channels[nPts*4+0] = j;
					vals_four_channels[nPts*4+1] = i;
					vals_four_channels[nPts*4+2] = j+cur_u;
					vals_four_channels[nPts*4+3] = i+cur_v;

					nPts++;
				}
			}

			T* interpolated_four_channels = new T[nPixels*4];
			memset(interpolated_four_channels,0,sizeof(T)*nPixels*4);

			scatter.SetLandmarks(nPts,2,coord_for_img,vals_four_channels,4);
			scatter.SolveCoefficient(num_of_neighbor,radius_scale,max_iter,ZQ_RBFKernel::COMPACT_CPC2);
			scatter.GridData2D(width,height,0,width-1,0,height-1,interpolated_four_channels);

			T* sample_result = new T[nChannels];

			if(cubic)
			{
				for(int i = 0;i < height;i++)
				{
					for(int j = 0;j < width;j++)
					{
						int offset = i*width+j;

						float coord1_x = interpolated_four_channels[offset*4+0];
						float coord1_y = interpolated_four_channels[offset*4+1];
						float coord2_x = interpolated_four_channels[offset*4+2];
						float coord2_y = interpolated_four_channels[offset*4+3];
						bool flag1 = coord1_x >= 0 && coord1_x <= width-1 && coord1_y >= 0 && coord1_y <= height-1; 
						bool flag2 = coord2_x >= 0 && coord2_x <= width-1 && coord2_y >= 0 && coord2_y <= height-1; 
						if(blend_mode == 1)
						{
							if(flag1 && flag2)
							{
								ZQ_ImageProcessing::BicubicInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c]*weight1;
								ZQ_ImageProcessing::BicubicInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c]*(1-weight1);
							}
							else if(flag1)
							{
								ZQ_ImageProcessing::BicubicInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c];
							}
							else if(flag2)
							{
								ZQ_ImageProcessing::BicubicInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c];
							}
						}
						else
						{
							ZQ_ImageProcessing::BicubicInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
							for(int c = 0;c < nChannels;c++)
								out_image[offset*nChannels+c] += sample_result[c]*weight1;
							ZQ_ImageProcessing::BicubicInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
							for(int c = 0;c < nChannels;c++)
								out_image[offset*nChannels+c] += sample_result[c]*(1-weight1);
						}
						
					}
				}
			}
			else
			{
				for(int i = 0;i < height;i++)
				{
					for(int j = 0;j < width;j++)
					{
						int offset = i*width+j;

						float coord1_x = interpolated_four_channels[offset*4+0];
						float coord1_y = interpolated_four_channels[offset*4+1];
						float coord2_x = interpolated_four_channels[offset*4+2];
						float coord2_y = interpolated_four_channels[offset*4+3];
						bool flag1 = coord1_x >= 0 && coord1_x <= width-1 && coord1_y >= 0 && coord1_y <= height-1; 
						bool flag2 = coord2_x >= 0 && coord2_x <= width-1 && coord2_y >= 0 && coord2_y <= height-1; 
						if(blend_mode == 1)
						{
							if(flag1 && flag2)
							{
								ZQ_ImageProcessing::BilinearInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c]*weight1;
								ZQ_ImageProcessing::BilinearInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c]*(1-weight1);
							}
							else if(flag1)
							{
								ZQ_ImageProcessing::BilinearInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c];
							}
							else if(flag2)
							{
								ZQ_ImageProcessing::BilinearInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
								for(int c = 0;c < nChannels;c++)
									out_image[offset*nChannels+c] += sample_result[c];
							}
						}
						else
						{
							ZQ_ImageProcessing::BilinearInterpolate(image1,width,height,nChannels,interpolated_four_channels[offset*4+0],interpolated_four_channels[offset*4+1],sample_result,false);
							for(int c = 0;c < nChannels;c++)
								out_image[offset*nChannels+c] += sample_result[c]*weight1;
							ZQ_ImageProcessing::BilinearInterpolate(image2,width,height,nChannels,interpolated_four_channels[offset*4+2],interpolated_four_channels[offset*4+3],sample_result,false);
							for(int c = 0;c < nChannels;c++)
								out_image[offset*nChannels+c] += sample_result[c]*(1-weight1);
						}
					}
				}

			}
			

			delete []sample_result;
			delete []vals_four_channels;
			delete []interpolated_four_channels;
			delete []coord_for_img;

			return true;
		}
	};
}


#endif
