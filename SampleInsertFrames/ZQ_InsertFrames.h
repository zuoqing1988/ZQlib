#ifndef _ZQ_INSERT_FRAMES_H_
#define _ZQ_INSERT_FRAMES_H_
#pragma once 

#include "ZQ_InsertFramesOptions.h"
#include "ZQ_BlendTwoImages.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include "ZQ_OpticalFlow.h"
#include "ZQ_WeightedMedian.h"
#include <string.h>
#include <stdio.h>

namespace ZQ
{
	class ZQ_InsertFrames
	{	
	public:
		template<class T>
		static bool InsertOneFrameWithFlow_Simple(const ZQ_DImage<T>& im1, const ZQ_DImage<T>& im2, const ZQ_DImage<T>& u1_2, const ZQ_DImage<T>& v1_2, float weight1, ZQ_DImage<T>& out_im)
		{
			int width = im1.width();
			int height = im1.height();
			int nChannels = im1.nchannels();
			if (!im2.matchDimension(width, height, nChannels))
				return false;
			if (!u1_2.matchDimension(width, height, 1))
				return false;
			if (!v1_2.matchDimension(width, height, 1))
				return false;
			if (!out_im.matchDimension(width, height, nChannels))
				out_im.allocate(width, height, nChannels);

			const T*& im1_data = im1.data();
			const T*& im2_data = im2.data();
			const T*& u_data = u1_2.data();
			const T*& v_data = v1_2.data();
			T*& out_data = out_im.data();
			int sample_mode = 0;
			int blend_mode = 0;
			return ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels, im1_data, im2_data, u_data, v_data, weight1, out_data, sample_mode, blend_mode);
		}

		template<class T>
		static bool InsertOneFrameWithFlow_Complex(const ZQ_DImage<T>& im1, const ZQ_DImage<T>& im2, const ZQ_DImage<T>& u1_2, const ZQ_DImage<T>& v1_2, const ZQ_DImage<T>& u2_1, const ZQ_DImage<T>& v2_1, float weight1, ZQ_DImage<T>& out_im, 
			int erosion_size = 2, int final_medfiltSize = 5, float sigma_for_confidence = 0.3)
		{
			ZQ_DImage<T> fw_out_im, fw_confidence, bw_out_im, bw_confidence;
			ZQ_DImage<bool> fw_mask, bw_mask;
			return InsertOneFrameWithFlow_Complex(im1, im2, u1_2, v1_2, u2_1, v2_1, weight1, out_im, fw_out_im, fw_confidence, fw_mask, bw_out_im, bw_confidence, bw_mask, erosion_size, final_medfiltSize, sigma_for_confidence, false);
		}

		template<class T>
		static bool InsertOneFrameWithFlow_Complex(const ZQ_DImage<T>& im1, const ZQ_DImage<T>& im2, const ZQ_DImage<T>& u1_2, const ZQ_DImage<T>& v1_2, const ZQ_DImage<T>& u2_1, const ZQ_DImage<T>& v2_1, float weight1, ZQ_DImage<T>& out_im,
			ZQ_DImage<T>& fw_out_im, ZQ_DImage<T>& fw_confidence, ZQ_DImage<bool>& fw_mask, ZQ_DImage<T>& bw_out_im, ZQ_DImage<T>& bw_confidence, ZQ_DImage<bool>& bw_mask, int erosion_size = 2, int final_medfiltSize = 5, float sigma_for_confidence = 0.3, bool output_debug_info = false)
		{
			int width = im1.width();
			int height = im1.height();
			int nChannels = im1.nchannels();
			if (!im2.matchDimension(width, height, nChannels))
				return false;
			if (!u1_2.matchDimension(width, height, 1))
				return false;
			if (!v1_2.matchDimension(width, height, 1))
				return false;
			if (!u2_1.matchDimension(width, height, 1))
				return false;
			if (!v2_1.matchDimension(width, height, 1))
				return false;

			ZQ_DImage<T> warpIm(width, height, nChannels);
			T*& warpIm_data = warpIm.data();
			const T*& im1_data = im1.data();
			const T*& im2_data = im2.data();
			const T*& fw_u_data = u1_2.data();
			const T*& fw_v_data = v1_2.data();
			const T*& bw_u_data = u2_1.data();
			const T*& bw_v_data = v2_1.data();
			ZQ_ImageProcessing::WarpImage(warpIm_data, im2_data, fw_u_data, fw_v_data, width, height, nChannels, (const T*)0, false);
			if (!fw_confidence.matchDimension(width, height, nChannels))
				fw_confidence.allocate(width, height, nChannels);
			T*& fw_confidence_data = fw_confidence.data();
			for (int pp = 0; pp < width*height; pp++)
			{
				double sum = 0;
				for (int c = 0; c < 3; c++)
				{
					double diff = warpIm_data[pp*nChannels + c] - im1_data[pp*nChannels + c];
					sum += diff*diff;
				}
				fw_confidence_data[pp] = exp(-sum / (sigma_for_confidence*sigma_for_confidence));
			}

			ZQ_ImageProcessing::WarpImage(warpIm_data, im1_data, bw_u_data, bw_v_data, width, height, nChannels, (const T*)0, false);
			if (!bw_confidence.matchDimension(width, height, nChannels))
				bw_confidence.allocate(width, height, nChannels);
			T*& bw_confidence_data = bw_confidence.data();

			for (int pp = 0; pp < width*height; pp++)
			{
				double sum = 0;
				for (int c = 0; c < 3; c++)
				{
					double diff = warpIm_data[pp*nChannels + c] - im2_data[pp*nChannels + c];
					sum += diff*diff;
				}
				bw_confidence_data[pp] = exp(-sum / (sigma_for_confidence*sigma_for_confidence));
			}

			warpIm.clear();

			if (!fw_out_im.matchDimension(width, height, nChannels))
				fw_out_im.allocate(width, height, nChannels);
			if (!fw_mask.matchDimension(width, height, 1))
				fw_mask.allocate(width, height);
				
			T*& fw_out_data = fw_out_im.data();
			bool*& fw_mask_data = fw_mask.data();
			if (!ZQ_BlendTwoImages::BlendTwoImagesByMedFiltWithMask<T>(width, height, nChannels, im1_data, im2_data, fw_u_data, fw_v_data, fw_confidence_data, weight1, fw_out_data, fw_mask_data, 0, 1))
				return false;

			if (!output_debug_info)
			{
				fw_confidence.clear();
			}

			if (!bw_out_im.matchDimension(width, height, nChannels))
				bw_out_im.allocate(width, height, nChannels);
			if (!bw_mask.matchDimension(width, height, 1))
				bw_mask.allocate(width, height);
			T*& bw_out_data = bw_out_im.data();
			bool*& bw_mask_data = bw_mask.data();
			if (!ZQ_BlendTwoImages::BlendTwoImagesByMedFiltWithMask<T>(width, height, nChannels, im2_data, im1_data, bw_u_data, bw_v_data, bw_confidence_data, 1 - weight1, bw_out_data, bw_mask_data, 0, 1))
				return false;
			if (!output_debug_info)
			{
				bw_confidence.clear();
			}

			if (erosion_size >= 2)
			{
				int filter_width = 2 * erosion_size + 1;
				ZQ_DImage<bool> filter2D(filter_width, filter_width);
				bool*& filter2D_data = filter2D.data();
				ZQ_DImage<bool> tmp_mask(fw_mask);
				bool*& tmp_mask_data = tmp_mask.data();
				for (int i = 0; i < filter_width*filter_width; i++)
					filter2D_data[i] = true;
				if (!ZQ_BinaryImageProcessing::Erode(tmp_mask_data, fw_mask_data, width, height, filter2D_data, erosion_size, erosion_size))
					return false;
				tmp_mask = bw_mask;
				if (!ZQ_BinaryImageProcessing::Erode(tmp_mask_data, bw_mask_data, width, height, filter2D_data, erosion_size, erosion_size))
					return false;
			}
			
			ZQ_DImage<bool> merge_mask(width, height, 1);
			bool*& merge_mask_data = merge_mask.data();
			ZQ_DImage<T> merge_out(width, height, nChannels);
			T*& merge_out_data = merge_out.data();
				
			for (int i = 0; i < width*height; i++)
			{
				if (fw_mask_data[i] && bw_mask_data[i])
				{
					merge_mask_data[i] = true;
					for (int c = 0; c < nChannels; c++)
						merge_out_data[i*nChannels + c] = fw_out_data[i*nChannels + c] * weight1 + bw_out_data[i*nChannels + c] * (1 - weight1);
				}
				else if (fw_mask_data[i])
				{
					merge_mask_data[i] = true;
					memcpy(merge_out_data + i*nChannels, fw_out_data + i*nChannels, sizeof(T)*nChannels);
				}
				else if (bw_mask_data[i])
				{
					merge_mask_data[i] = true;
					memcpy(merge_out_data + i*nChannels, bw_out_data + i*nChannels, sizeof(T)*nChannels);
				}
				else
				{
					merge_mask_data[i] = false;
				}
			}
			if (!output_debug_info)
			{
				fw_out_im.clear();
				fw_mask.clear();
				bw_out_im.clear();
				bw_mask.clear();
			}

			if (!out_im.matchDimension(width, height, nChannels))
				out_im.allocate(width, height, nChannels);
			T*& out_im_data = out_im.data();
				
			if (!ZQ_ImageProcessing::MedianFilterWithMask(merge_out_data, out_im_data, width, height, nChannels, final_medfiltSize, merge_mask_data, false))
				return false;
			return true;
		}

		template<class T>
		static bool InsertFrames(const ZQ_InsertFramesOptions& opt)
		{
			switch (opt.methodType)
			{
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_FORWARD: 
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_BIDRECTIONAL:
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_FORWARD:
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_BIDRECTIONAL:
				return InsertFramesSimple<T>(opt);
				break;
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1:
				return InsertFramesL1<T>(opt);
				break;
			default:
				return false;
			}
		}
		
		template<class T>
		static bool InsertFramesSimple(const ZQ_InsertFramesOptions& opt)
		{
			if (opt.speedUp < 2)
			{
				printf("speedUp must be >= 2\n");
				return false;
			}
			if (opt.frameNum < 2)
			{
				//printf("frameNum must be >= 2\n");
				return false;
			}
			float scale = __min(1,__max(0.125,opt.scale));
			
			char filename[ZQ_InsertFramesOptions::MAX_FILENAME_LEN];
			char maskfilename[ZQ_InsertFramesOptions::MAX_FILENAME_LEN];
			ZQ_DImage<T> bgr1, bgr2, mask1, mask2, gray1, gray2;
			ZQ_DImage<T> bgra1, bgra2;
			ZQ_DImage<T> u, v, warpIm2, u1, v1;
			T*& bgr1_data = bgr1.data();
			T*& bgr2_data = bgr2.data();
			T*& mask1_data = mask1.data();
			T*& mask2_data = mask2.data();
			T*& gray1_data = gray1.data();
			T*& gray2_data = gray2.data();
			T*& bgra1_data = bgra1.data();
			T*& bgra2_data = bgra2.data();
			T*& u_data = u.data();
			T*& v_data = v.data();
			T*& u1_data = u1.data();
			T*& v1_data = v1.data();
			int width, height, nChannels;

			int out_num = 0;
			for (int i = 0; i < opt.frameNum - 1; i++)
			{
				if (i == 0)
				{
					sprintf(filename, "%s\\%s%d.%s", opt.inputDir, opt.prefix, opt.baseId + i, opt.suffix);
					if (opt.hasMask)
						sprintf(maskfilename, "%s\\%s%d.%s", opt.inputDir, opt.maskPrefix, opt.baseId + i, opt.suffix);
					if (!_loadImage(bgr1, mask1, filename, maskfilename, opt.hasMask))
					{
						printf("failed to load %d-th frame!\n", i);
						return false;
					}
					width = bgr1.width();
					height = bgr1.height();
					nChannels = bgr1.nchannels();

					_bgr_to_gray(bgr1, gray1);
					
					if (opt.hasMask)
					{
						if (!mask1.matchDimension(width, height, 1))
						{
							printf("dimensions donot match!\n");
							return false;
						}
						for (int i = 0; i < width*height; i++)
						{
							if (mask1_data[i] == 0)
								gray1_data[i] = 0;
						}
						bgra1.assemble(bgr1, mask1);
						bgr1.clear();
						mask1.clear();
					}

					gray1.imresize(scale);
				}
				else
				{
					gray1 = gray2;
					bgra1 = bgra2;
					mask1 = mask2;
					bgr1 = bgr2;
				}
				
				sprintf(filename, "%s\\%s%d.%s", opt.inputDir, opt.prefix, opt.baseId + i + 1, opt.suffix);
				if (opt.hasMask)
					sprintf(maskfilename, "%s\\%s%d.%s", opt.inputDir, opt.maskPrefix, opt.baseId + i + 1, opt.suffix);
				if (!_loadImage(bgr2, mask2, filename, maskfilename, opt.hasMask))
				{
					printf("failed to load %d-th frame!\n", i + 1);
					return false;
				}
				if (!bgr2.matchDimension(width, height, nChannels))
				{
					printf("dimensions donot match!\n");
					return false;
				}
				_bgr_to_gray(bgr2, gray2);
				if (opt.hasMask)
				{
					if (!mask2.matchDimension(width, height, 1))
					{
						printf("dimensions donot match!\n");
						return false;
					}
					for (int i = 0; i < width*height; i++)
					{
						if (mask2_data[i] == 0)
							gray2_data[i] = 0;
					}
					bgra2.assemble(bgr2, mask2);
					bgr2.clear();
					mask2.clear();
				}

				gray2.imresize(scale);
				
				/******************************************/

				bool maxMotionOverHead = false;
				bool has_u1v1 = false;
				_compute_optical_flow(u, v, u1, v1, has_u1v1, gray1, gray2, opt);
				if (opt.enableMaxMotionLimit)
				{
					if (_detect_motion_overhead(u, v, opt.maxMotionLimit*scale))
					{
						maxMotionOverHead = true;
						if (opt.backupMethod == ZQ_InsertFramesOptions::BACKUP_OPTICAL_FLOW)
						{
							ZQ_InsertFramesOptions new_opt = opt;
							new_opt.alpha = opt.backupAlpha;
							bool new_has_u1v1;
							_compute_optical_flow(u, v, u1, v1, new_has_u1v1, gray1, gray2, new_opt);
						}
					}
				}
				
				u.imresize(width, height);
				v.imresize(width, height);
				u.Multiplywith(1.0 / scale);
				v.Multiplywith(1.0 / scale);
				if (opt.exportFlow)
				{
					IplImage* flowimg = ZQ_ImageIO::SaveFlowToColorImage(u, v, false, 0, 64, 0, false);
					if (flowimg)
					{
						sprintf(filename, "%s\\flow%d_%d.png", opt.outputDir, i + opt.baseId, i + 1 + opt.baseId);
						cvSaveImage(filename, flowimg);
						cvReleaseImage(&flowimg);
					}
				}
				if (has_u1v1)
				{
					u1.imresize(width, height);
					v1.imresize(width, height);
					u1.Multiplywith(1.0 / scale);
					v1.Multiplywith(1.0 / scale);
					if (opt.exportFlow)
					{
						IplImage* flowimg = ZQ_ImageIO::SaveFlowToColorImage(u1, v1, false, 0, 64, 0, false);
						if (flowimg)
						{
							sprintf(filename, "%s\\flow%d_%d.png", opt.outputDir, i + 1 + opt.baseId, i + opt.baseId);
							cvSaveImage(filename, flowimg);
							cvReleaseImage(&flowimg);
						}
					}
				}
			
				/*******************************************/

				for (int p = 0; p < opt.speedUp; p++)
				{
					float weight1 = 1.0f - (float)p / opt.speedUp;

					if (maxMotionOverHead && opt.backupMethod == ZQ_InsertFramesOptions::BACKUP_BLEND)
					{
						if (opt.hasMask)
						{
							ZQ_DImage<T> out(bgra1);
							out.Multiplywith(weight1);
							out.Addwith(bgra2, 1 - weight1);
							ZQ_DImage<T> out_bgr, out_mask;
							out.separate(nChannels, out_bgr, out_mask);
							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							sprintf(maskfilename, "%s\\%s%d.%s", opt.outputDir, opt.maskPrefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(out_bgr, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
							if (!ZQ_ImageIO::saveImage(out_mask, maskfilename))
							{
								printf("failed to save %s\n", maskfilename);
								return false;
							}
							else
								printf("save %s success!\n", maskfilename);
						}
						else
						{
							ZQ_DImage<T> out(bgr1);
							out.Multiplywith(weight1);
							out.Addwith(bgr2, 1 - weight1);
							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(out, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
						}
					}
					else if (maxMotionOverHead && opt.backupMethod == ZQ_InsertFramesOptions::BACKUP_REPEAT)
					{
						if (opt.hasMask)
						{
							ZQ_DImage<T> out, out_bgr, out_mask;
							if (p <= opt.speedUp / 2)
								out.copyData(bgra1);
							else
								out.copyData(bgra2);
							out.separate(nChannels, out_bgr, out_mask);
							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							sprintf(maskfilename, "%s\\%s%d.%s", opt.outputDir, opt.maskPrefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(out_bgr, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
							if (!ZQ_ImageIO::saveImage(out_mask, maskfilename))
							{
								printf("failed to save %s\n", maskfilename);
								return false;
							}
							else
								printf("save %s success!\n", maskfilename);
						}
						else
						{
							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(p <= opt.speedUp / 2 ? bgr1 : bgr2, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
						}
					}
					else
					{
						if (opt.hasMask)
						{
							ZQ_DImage<T> out(width, height, nChannels + 1);
							T*& out_data = out.data();
							if (p == 0)
								out.copyData(bgr1);
							else
							{
								switch (opt.methodType)
								{
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_BIDRECTIONAL:
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_BIDRECTIONAL:
									if (p <= opt.speedUp / 2)
										ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels + 1, bgra1_data, bgra2_data, u_data, v_data, weight1, out_data, opt.sampleMode, opt.blendMode);
									else
										ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels + 1, bgra2_data, bgra1_data, u1_data, v1_data, 1 - weight1, out_data, opt.sampleMode, opt.blendMode);
									break;
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_FORWARD:
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_FORWARD:
								default:
									ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels + 1, bgra1_data, bgra2_data, u_data, v_data, weight1, out_data, opt.sampleMode, opt.blendMode);
									break;
								}
							}
							

							ZQ_DImage<T> out_bgr, out_mask;
							out.separate(nChannels, out_bgr, out_mask);
							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							sprintf(maskfilename, "%s\\%s%d.%s", opt.outputDir, opt.maskPrefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(out_bgr, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
							if (!ZQ_ImageIO::saveImage(out_mask, maskfilename))
							{
								printf("failed to save %s\n", maskfilename);
								return false;
							}
							else
								printf("save %s success!\n", maskfilename);
						}
						else
						{
							ZQ_DImage<T> out(width, height, nChannels);
							T*& out_data = out.data();
							if (p == 0)
								out.copyData(bgr1);
							else
							{
								switch (opt.methodType)
								{
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_BIDRECTIONAL:
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_BIDRECTIONAL:
									if (p <= opt.speedUp / 2)
										ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels, bgr1_data, bgr2_data, u_data, v_data, weight1, out_data, opt.sampleMode, opt.blendMode);
									else
										ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels, bgr2_data, bgr1_data, u1_data, v1_data, 1 - weight1, out_data, opt.sampleMode, opt.blendMode);
									break;
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_FORWARD:
								case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_FORWARD:
								default:
									ZQ_BlendTwoImages::BlendTwoImagesByMedFilt(width, height, nChannels, bgr1_data, bgr2_data, u_data, v_data, weight1, out_data, opt.sampleMode, opt.blendMode);
									break;
								}
							}
							

							sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
							if (!ZQ_ImageIO::saveImage(out, filename))
							{
								printf("failed to save %s\n", filename);
								return false;
							}
							else
								printf("save %s success!\n", filename);
						}

					}
					out_num++;
				}			
			}

			/*********     output the final  Begin        *******/
			if (opt.hasMask)
			{
				bgra2.separate(nChannels, bgr2, mask2);
				sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
				sprintf(maskfilename, "%s\\%s%d.%s", opt.outputDir, opt.maskPrefix, opt.outBaseId + out_num, opt.suffix);
				if (!ZQ_ImageIO::saveImage(bgr2, filename))
				{
					printf("failed to save %s\n", filename);
					return false;
				}
				else
					printf("save %s success!\n", filename);
				if (!ZQ_ImageIO::saveImage(mask2, maskfilename))
				{
					printf("failed to save %s\n", maskfilename);
					return false;
				}
				else
					printf("save %s success!\n", maskfilename);

			}
			else
			{
				sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
				if (!ZQ_ImageIO::saveImage(bgr2, filename))
				{
					printf("failed to save %s\n", filename);
					return false;
				}
				else
					printf("save %s success!\n", filename);
			}
			/*********     output the final  End      *******/
			return true;
		}

		

		template<class T>
		static bool InsertFramesL1(const ZQ_InsertFramesOptions& opt)
		{
			if (opt.speedUp < 2)
			{
				printf("speed Up must be >= 2\n");
				return false;
			}
			if (opt.frameNum < 2)
			{
				//printf("frameNum must be >= 2\n");
				return false;
			}
			float scale = __min(1, __max(0.125, opt.scale));

			char filename[ZQ_InsertFramesOptions::MAX_FILENAME_LEN];
			ZQ_DImage<T> bgr1, bgr2, gray1, gray2;
			ZQ_DImage<T> fw_u, fw_v, bw_u, bw_v, warpIm2;
			T*& bgr1_data = bgr1.data();
			T*& bgr2_data = bgr2.data();
			T*& gray1_data = gray1.data();
			T*& gray2_data = gray2.data();
			T*& fw_u_data = fw_u.data();
			T*& fw_v_data = fw_v.data();
			T*& bw_u_data = bw_u.data();
			T*& bw_v_data = bw_v.data();
			T*& warpIm2_data = warpIm2.data();

			int width, height, nChannels;

			int out_num = 0;
			for (int i = 0; i < opt.frameNum - 1; i++)
			{
				if (i == 0)
				{
					sprintf(filename, "%s\\%s%d.%s", opt.inputDir, opt.prefix, opt.baseId + i, opt.suffix);
					if (!ZQ_ImageIO::loadImage(bgr1,filename,1))
					{
						printf("failed to load %d-th frame!\n", i);
						return false;
					}
					width = bgr1.width();
					height = bgr1.height();
					nChannels = bgr1.nchannels();

					_bgr_to_gray(bgr1, gray1);
					gray1.imresize(scale);
				}
				else
				{
					gray1 = gray2;
					bgr1 = bgr2;
				}

				sprintf(filename, "%s\\%s%d.%s", opt.inputDir, opt.prefix, opt.baseId + i + 1, opt.suffix);
				
				if (!ZQ_ImageIO::loadImage(bgr2, filename, 1))
				{
					printf("failed to load %d-th frame!\n", i + 1);
					return false;
				}
				if (!bgr2.matchDimension(width, height, nChannels))
				{
					printf("dimensions donot match!\n");
					return false;
				}
				_bgr_to_gray(bgr2, gray2);
				gray2.imresize(scale);

				/*************/
				ZQ_OpticalFlowOptions of_opt;
				ZQ_OpticalFlowOptions::GetDefaultOptions_HS_L1(of_opt);
				of_opt.nInnerFixedPointIterations = opt.nInnerIter;
				of_opt.alpha = opt.alpha;
				of_opt.minWidthForPyramid = opt.minWidth;
				of_opt.ratioForPyramid = opt.ratio;
				of_opt.nOuterFixedPointIterations = opt.nOuterIter;
				of_opt.nSORIterations = opt.nSORIter;
				ZQ_OpticalFlow::Coarse2Fine_HS_L1<T>(fw_u, fw_v, warpIm2, gray1, gray2, of_opt);
				fw_u.imresize(width, height);
				fw_v.imresize(width, height);
				fw_u.Multiplywith(1.0 / scale);
				fw_v.Multiplywith(1.0 / scale);
				
				if (opt.exportFlow)
				{
					IplImage* flowimg = ZQ_ImageIO::SaveFlowToColorImage(fw_u, fw_v, false, 0, 64, 0, false);
					if (flowimg)
					{
						sprintf(filename, "%s\\flow%d_%d.png", opt.outputDir, i + opt.baseId, i + 1 + opt.baseId);
						cvSaveImage(filename, flowimg);
						cvReleaseImage(&flowimg);
					}
				}


				ZQ_OpticalFlow::Coarse2Fine_HS_L1<T>(bw_u, bw_v, warpIm2, gray2, gray1, of_opt);
				bw_u.imresize(width, height);
				bw_v.imresize(width, height);
				bw_u.Multiplywith(1.0 / scale);
				bw_v.Multiplywith(1.0 / scale);
				if (opt.exportFlow)
				{
					IplImage* flowimg = ZQ_ImageIO::SaveFlowToColorImage(bw_u, bw_v, false, 0, 64, 0, false);
					if (flowimg)
					{
						sprintf(filename, "%s\\flow%d_%d.png", opt.outputDir, i + 1 + opt.baseId, i + opt.baseId);
						cvSaveImage(filename, flowimg);
						cvReleaseImage(&flowimg);
					}
				}

				/**************/
				sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
				if (!ZQ_ImageIO::saveImage(bgr1, filename))
				{
					printf("failed to save %s\n", filename);
					return false;
				}
				else
					printf("save %s success!\n", filename);

				out_num++;

				for (int p = 1; p < opt.speedUp; p++)
				{
					float weight1 = 1.0f - (float)p / opt.speedUp;
					ZQ_DImage<T> out_im;
					InsertOneFrameWithFlow_Complex(bgr1, bgr2, fw_u, fw_v, bw_u, bw_v, weight1, out_im, opt.erosionSizeForComplex, opt.finalMedfiltSizeForComplex, opt.sigmaConfidenceForComplex);

					sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
					if (!ZQ_ImageIO::saveImage(out_im, filename))
					{
						printf("failed to save %s\n", filename);
						return false;
					}
					else
						printf("save %s success!\n", filename);

					out_num++;
				}
			}

			sprintf(filename, "%s\\%s%d.%s", opt.outputDir, opt.prefix, opt.outBaseId + out_num, opt.suffix);
			if (!ZQ_ImageIO::saveImage(bgr2, filename))
			{
				printf("failed to save %s\n", filename);
				return false;
			}
			else
				printf("save %s success!\n", filename);

			return true;
		}

	private:
		template<class T>
		static bool _loadImage(ZQ_DImage<T>& im, ZQ_DImage<T>& mask, const char* filename, const char* maskfilename, bool hasmask)
		{
			if (!hasmask)
			{
				return ZQ_ImageIO::loadImage(im, filename, 1);
			}
			else
			{
				if (!ZQ_ImageIO::loadImage(im, filename, 1))
					return false;
				return ZQ_ImageIO::loadImage(mask, maskfilename, 0);
			}
		}

		template<class T>
		static void _bgr_to_gray(const ZQ_DImage<T>& bgr, ZQ_DImage<T>& gray)
		{
			int width = bgr.width();
			int height = bgr.height();
			int nChannels = bgr.nchannels();
			if (nChannels == 1)
				gray = bgr;
			else if (nChannels == 3)
			{
				gray.allocate(width, height);
				const T*& bgr_data = bgr.data();
				T*& gray_data = gray.data();
				for (int i = 0; i < width*height; i++)
				{
					float B = bgr_data[i * 3 + 0];
					float G = bgr_data[i * 3 + 1];
					float R = bgr_data[i * 3 + 2];
					gray_data[i] = R*0.299f + G*0.587f + B*0.114f;
				}
			}
			else
			{
				ZQ_DImage<T> other;
				bgr.separate(1, gray, other);
			}
		}

		template<class T>
		static void _compute_optical_flow(ZQ_DImage<T>& u, ZQ_DImage<T>& v, ZQ_DImage<T>& u1, ZQ_DImage<T>& v1, bool& has_u1v1, const ZQ_DImage<T>& gray1, const ZQ_DImage<T>& gray2, const ZQ_InsertFramesOptions& opt)
		{
			has_u1v1 = false;

			ZQ_DImage<T> gray1_(gray1);
			ZQ_DImage<T> gray2_(gray2);
			if (opt.doHistMatch)
			{
				_hist_match(gray1, gray2, gray2_);
			}

			ZQ_OpticalFlowOptions of_opt;
			of_opt.alpha = opt.alpha;
			of_opt.minWidthForPyramid = opt.minWidth;
			of_opt.ratioForPyramid = opt.ratio;
			of_opt.nOuterFixedPointIterations = opt.nOuterIter;
			of_opt.nSORIterations = opt.nSORIter;
			of_opt.nInnerFixedPointIterations = opt.nInnerIter;
			
			ZQ_DImage<T> warpIm2;
			switch (opt.methodType)
			{
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_FORWARD:
				ZQ_OpticalFlow::Coarse2Fine_HS_L1<T>(u, v, warpIm2, gray1_, gray2_, of_opt);
				break;
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L1_BIDRECTIONAL:
				ZQ_OpticalFlow::Coarse2Fine_HS_L1<T>(u, v, warpIm2, gray1_, gray2_, of_opt);
				ZQ_OpticalFlow::Coarse2Fine_HS_L1<T>(u1, v1, warpIm2, gray2_, gray1_, of_opt);
				has_u1v1 = true;
				break;
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_BIDRECTIONAL:
				ZQ_OpticalFlow::Coarse2Fine_HS_L2<T>(u, v, warpIm2, gray1_, gray2_, of_opt);
				ZQ_OpticalFlow::Coarse2Fine_HS_L2<T>(u1, v1, warpIm2, gray2_, gray1_, of_opt);
				has_u1v1 = true;
				break;
			case ZQ_InsertFramesOptions::OPTICAL_FLOW_L2_FORWARD:
			default:
				ZQ_OpticalFlow::Coarse2Fine_HS_L2<T>(u, v, warpIm2, gray1_, gray2_, of_opt);
				break;
			}
			warpIm2.clear();
		}

		template<class T>
		static void _hist_match(const ZQ_DImage<T>& gray1, const ZQ_DImage<T>& gray2, ZQ_DImage<T>& gray2_)
		{
			const T*& gray_data1 = gray1.data();
			const T*& gray_data2 = gray2.data();
			if (!gray2_.matchDimension(gray2))
				gray2_.allocate(gray2);
			T*& gray_data2_ = gray2_.data();
			int npixels1 = gray1.npixels();
			int npixels2 = gray2.npixels();
			const int nBins = 256;
			double hist1[nBins] = { 0 };
			double hist2[nBins] = { 0 };
		
			for (int i = 0; i < npixels1; i++)
			{
				int val = gray_data1[i] * 255.0;
				val = __min(255, __max(0, val));
				hist1[val] += 1;
			}
			for (int i = 0; i < nBins; i++)
				hist1[i] /= npixels1;

			for (int i = 0; i < npixels2; i++)
			{
				int val = gray_data2[i] * 255.0;
				val = __min(255, __max(0, val));
				hist2[val] += 1;
			}
			for (int i = 0; i < nBins; i++)
				hist2[i] /= npixels2;

			int trans[nBins];
			for (int i = 0; i < nBins; i++)
				trans[i] = i;
			int idx1 = 0, idx2 = 0;
			double sum1 = hist1[0], sum2 = hist2[0];
			for (;;)
			{
				if (sum1 >= sum2)
				{
					trans[idx2] = idx1;
					idx2++;
					if (idx2 == nBins)
						break;
					
					sum2 += hist2[idx2];
				}
				else
				{
					idx1++;
					if (idx1 == nBins)
					{
						for (int k = idx2; k < nBins; k++)
							trans[idx2] = nBins - 1;
						break;
					}
					sum1 += hist1[idx1];
				}
			}

			for (int i = 0; i < npixels2; i++)
			{
				int val = gray_data2[i] * 255.0;
				val = __min(255, __max(0, val));
				gray_data2_[i] = trans[val] / 255.0;
			}
		}

		template<class T>
		static bool _detect_motion_overhead(const ZQ_DImage<T>& u, const ZQ_DImage<T>& v, const float limit)
		{
			int width = u.width();
			int height = u.height();
			double avg_u = 0, avg_v = 0;
			const T*& u_data = u.data();
			const T*& v_data = v.data();
			for (int i = 0; i < width*height; i++)
			{
				avg_u += u_data[i];
				avg_v += v_data[i];
			}
			avg_u /= (width*height + 0.0001);
			avg_v /= (width*height + 0.0001);

			for (int i = 0; i < width*height; i++)
			{
				double cur_u = u_data[i] - avg_u;
				double cur_v = v_data[i] - avg_v;
				if (cur_u*cur_u + cur_v*cur_v > limit*limit)
				{
					return true;
				}
			}
			return false;
		}
		
	};
}

#endif