#ifndef _ZQ_INSERT_FRAMES_OPTIONS_H_
#define _ZQ_INSERT_FRAMES_OPTIONS_H_
#pragma once 

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace ZQ
{
	class ZQ_InsertFramesOptions
	{
	public:
		enum CONST_VAL{ MAX_FILENAME_LEN = 256 };
		enum MethodType{
			OPTICAL_FLOW_L2_FORWARD, 
			OPTICAL_FLOW_L2_BIDRECTIONAL, 
			OPTICAL_FLOW_L1_FORWARD,
			OPTICAL_FLOW_L1_BIDRECTIONAL,
			OPTICAL_FLOW_L1 
		};
		enum BackupMethodType{
			BACKUP_REPEAT,
			BACKUP_BLEND,
			BACKUP_OPTICAL_FLOW
		};
	public:
		ZQ_InsertFramesOptions(){ Reset(); }
		~ZQ_InsertFramesOptions(){}

		MethodType methodType;
		char inputDir[MAX_FILENAME_LEN];
		char outputDir[MAX_FILENAME_LEN];
		char prefix[MAX_FILENAME_LEN];
		bool hasMask;
		bool doHistMatch;
		char maskPrefix[MAX_FILENAME_LEN];
		char suffix[MAX_FILENAME_LEN];
		int baseId;
		int outBaseId;
		int frameNum;
		int speedUp;
		int blendMode;
		int sampleMode;
		float alpha;
		float minWidth;
		float ratio;
		int nOuterIter;
		int nInnerIter;
		int nSORIter;
		float scale;
		float threshRatioForMedFilt;
		bool exportFlow;
		int nCores;
		bool enableMaxMotionLimit;
		float maxMotionLimit;
		BackupMethodType backupMethod;
		float backupAlpha;

		int erosionSizeForComplex;
		int finalMedfiltSizeForComplex;
		float sigmaConfidenceForComplex;
		
		void Reset()
		{
			methodType = OPTICAL_FLOW_L2_FORWARD;
			inputDir[0] = '\0';
			outputDir[0] = '\0';
			prefix[0] = '\0';
			hasMask = false;
			doHistMatch = false;
			maskPrefix[0] = '\0';
			suffix[0] = '\0';
			baseId = 0;
			outBaseId = 0;
			frameNum = 0;
			speedUp = 2;
			blendMode = 0;
			sampleMode = 0;
			alpha = 0.06;
			minWidth = 16;
			ratio = 0.5;
			nOuterIter = 5;
			nInnerIter = 3;
			nSORIter = 30;
			scale = 0.25;
			threshRatioForMedFilt = 0.7;
			exportFlow = false;
			nCores = 1;
			enableMaxMotionLimit = false;
			maxMotionLimit = 32;
			backupMethod = BACKUP_BLEND;
			backupAlpha = 0.5;
			erosionSizeForComplex = 2;
			finalMedfiltSizeForComplex = 5;
			sigmaConfidenceForComplex = 0.3;
		}

		bool HandleArgs(const int argc, const char** argv)
		{
			for (int k = 0; k < argc; k++)
			{
				if (_strcmpi(argv[k], "MethodType") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					if (_strcmpi(argv[k], "L2_FW") == 0)
					{
						methodType = OPTICAL_FLOW_L2_FORWARD;
					}
					else if (_strcmpi(argv[k], "L2_BIDIR") == 0)
					{
						methodType = OPTICAL_FLOW_L2_BIDRECTIONAL;
					}
					else if (_strcmpi(argv[k], "L1_FW") == 0)
					{
						methodType = OPTICAL_FLOW_L1_FORWARD;
					}
					else if (_strcmpi(argv[k], "L1_BIDIR") == 0)
					{
						methodType = OPTICAL_FLOW_L1_BIDRECTIONAL;
					}
					else if (_strcmpi(argv[k], "L1") == 0)
					{
						methodType = OPTICAL_FLOW_L1;
					}
					else
					{
						printf("unknown method type:%s\n", argv[k]);
						return false;
					}
				}
				else if (_strcmpi(argv[k], "inputDir") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k-1]);
						return false;
					}
					strcpy(inputDir, argv[k]);
				}
				else if (_strcmpi(argv[k], "outputDir") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					strcpy(outputDir, argv[k]);
				}
				else if (_strcmpi(argv[k], "prefix") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					strcpy(prefix, argv[k]);
				}
				else if (_strcmpi(argv[k], "maskPrefix") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					strcpy(maskPrefix, argv[k]);
					hasMask = true;
				}
				else if (_strcmpi(argv[k], "doHistMatch") == 0)
				{
					doHistMatch = true;
				}
				else if (_strcmpi(argv[k], "suffix") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					strcpy(suffix, argv[k]);
				}
				else if (_strcmpi(argv[k], "baseId") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					baseId = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "outBaseId") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					outBaseId = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "frameNum") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					frameNum = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "speedUp") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					speedUp = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "blendMode") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					blendMode = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "sampleMode") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					sampleMode = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "alpha") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					alpha = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "minWidth") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					minWidth = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "ratio") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					ratio = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "nOuterIter") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					nOuterIter = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "nInnerIter") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					nInnerIter = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "nSORIter") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					nSORIter = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "scale") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					scale = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "threshRatioForMedFilt") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}

					threshRatioForMedFilt = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "exportFlow") == 0)
				{
					exportFlow = true;
				}
				else if (_strcmpi(argv[k], "nCores") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}

					nCores = atoi(argv[k]);
				}
				else if (_strcmpi(argv[k], "enableMaxMotionLimit") == 0)
				{
					enableMaxMotionLimit = true;
				}
				else if (_strcmpi(argv[k], "maxMotionLimit") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}

					maxMotionLimit = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "backupMethod") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					if (_strcmpi(argv[k], "repeat") == 0)
					{
						backupMethod = BACKUP_REPEAT;
					}
					else if (_strcmpi(argv[k], "blend") == 0)
					{
						backupMethod = BACKUP_BLEND;
					}
					else if (_strcmpi(argv[k], "opticalflow") == 0)
					{
						backupMethod = BACKUP_OPTICAL_FLOW;
					}
					else
					{
						printf("unknown method type:%s\n", argv[k]);
						return false;
					}
				}
				else if (_strcmpi(argv[k], "backupAlpha") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					backupAlpha = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "erosionSizeForComplex") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					erosionSizeForComplex = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "finalMedfiltSizeForComplex") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					finalMedfiltSizeForComplex = atof(argv[k]);
				}
				else if (_strcmpi(argv[k], "sigmaConfidenceForComplex") == 0)
				{
					k++;
					if (k >= argc)
					{
						printf("need value for %s\n", argv[k - 1]);
						return false;
					}
					sigmaConfidenceForComplex = atof(argv[k]);
				}
				else
				{
					printf("unknown parameter: %s\n", argv[k]);
					return false;
				}

			}
			return true;
		}
	};
}
#endif