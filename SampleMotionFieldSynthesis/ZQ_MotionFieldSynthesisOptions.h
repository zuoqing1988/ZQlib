#ifndef _ZQ_MOTION_FIELD_SYNTHESIS_OPTIONS_H_
#define _ZQ_MOTION_FIELD_SYNTHESIS_OPTIONS_H_

#include <stdio.h>
#include <string.h>

#define ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN 200

class ZQ_MotionFieldSynthesisOptions
{
public:
	enum Type{TYPE_TEXTURE_TO_FLOW, TYPE_SYNTHESIS_FIELD, TYPE_ADVECTION_STATIC, TYPE_ADVECTION_DYNAMIC};
	
	Type type;
	char texture_file[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char output_flowfile[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	bool tex_vector_field;
	char original_fold[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char original_prefix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char original_suffix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	int base_id;
	int frame_count;
	char synthesis_fold[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char synthesis_prefix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char synthesis_suffix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	int border_width;
	float reduce_factor;
	int max_level;
	int to_iter;
	int sor_iter;
	float grad_weight;
	float ctrl_weight;
	float search_probe;
	int fine_width;
	int fine_height;
	bool smoothSynthesis;
	float smoothSigma;

	float dt;
	float substeps;
	float advectionN;
	char input_density_fold[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char input_density_prefix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char input_density_suffix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char output_density_fold[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char output_density_prefix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	char output_density_suffix[ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN];
	float coarse_field_weight;
	float detail_field_weight;
	

	ZQ_MotionFieldSynthesisOptions(){Reset();}
	~ZQ_MotionFieldSynthesisOptions(){}

	void Reset()
	{
		type = TYPE_SYNTHESIS_FIELD;
		texture_file[0] = '\0';
		tex_vector_field = false;
		strcpy_s(original_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,".");
		strcpy_s(original_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,"");
		strcpy_s(original_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,"di2");
		base_id = 0;
		frame_count = 0;
		strcpy_s(synthesis_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,".");
		strcpy_s(synthesis_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,"");
		strcpy_s(synthesis_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,"di2");
		border_width = 16;
		reduce_factor = 0.5;
		max_level = 5;
		to_iter = 3;
		sor_iter = 10;
		grad_weight = 0.05;
		ctrl_weight = 0.2;
		search_probe = 0.05;
		fine_width = 128;
		fine_height = 128;
		smoothSynthesis = false;
		smoothSigma = 1.6;
		dt = 1;
		substeps = 20;
		advectionN = 5;
		strcpy_s(input_density_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, ".");
		strcpy_s(input_density_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, "");
		strcpy_s(input_density_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, "png");
		strcpy_s(output_density_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, ".");
		strcpy_s(output_density_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, "");
		strcpy_s(output_density_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, "png");
		coarse_field_weight = 0;
		detail_field_weight = 1;
	}

	bool HandleArgs(const int argc, const char** argv)
	{
		int k = 0;
		while(k < argc)
		{
			if(_strcmpi(argv[k],"type") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","type");
					return false;
				}

				if(_strcmpi(argv[k],"texture_to_flow") == 0)
				{
					type = TYPE_TEXTURE_TO_FLOW;
				}
				else if(_strcmpi(argv[k],"synthesis_field") == 0)
				{
					type = TYPE_SYNTHESIS_FIELD;
				}
				else if(_strcmpi(argv[k],"advection_static") == 0)
				{
					type = TYPE_ADVECTION_STATIC;
				}
				else if(_strcmpi(argv[k],"advection_dynamic") == 0)
				{
					type = TYPE_ADVECTION_DYNAMIC;
				}
				else
				{
					printf("invalid arg: %s\n",argv[k]);
					return false;
				}
			}
			else if(_strcmpi(argv[k],"texture_file") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","texture_file");
					return false;
				}

				strcpy_s(texture_file, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, argv[k]);
			}
			else if(_strcmpi(argv[k],"output_flowfile") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","output_flowfile");
					return false;
				}

				strcpy_s(output_flowfile, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"tex_vector_field") == 0)
			{
				tex_vector_field = true;
			}
			else if(_strcmpi(argv[k],"original_fold") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","original_fold");
					return false;
				}

				strcpy_s(original_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"original_prefix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","original_prefix");
					return false;
				}

				strcpy_s(original_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"original_suffix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","original_suffix");
					return false;
				}

				strcpy_s(original_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN, argv[k]);
			}
			else if(_strcmpi(argv[k],"base_id") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","base_id");
					return false;
				}

				base_id = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"frame_count") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","frame_count");
					return false;
				}

				frame_count = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"synthesis_fold") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","synthesis_fold");
					return false;
				}

				strcpy_s(synthesis_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"synthesis_prefix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","synthesis_prefix");
					return false;
				}

				strcpy_s(synthesis_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"synthesis_suffix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","synthesis_suffix");
					return false;
				}

				strcpy_s(synthesis_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"border_width") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","border_width");
					return false;
				}

				border_width = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"reduce_factor") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","reduce_factor");
					return false;
				}

				reduce_factor = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"max_level") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","max_level");
					return false;
				}

				max_level = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"to_iter") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","to_iter");
					return false;
				}

				to_iter = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"sor_iter") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","sor_iter");
					return false;
				}

				sor_iter = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"grad_weight") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","grad_weight");
					return false;
				}

				grad_weight = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"ctrl_weight") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","ctrl_weight");
					return false;
				}

				ctrl_weight = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"search_probe") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","search_probe");
					return false;
				}

				search_probe = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"fine_resolution") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","fine_resolution : x");
					return false;
				}
				fine_width = atoi(argv[k]);
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","fine_resolution : y");
					return false;
				}
				fine_height = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"smoothSynthesis") == 0)
			{
				smoothSynthesis = true;
			}
			else if(_strcmpi(argv[k],"smoothSigma") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","smoothSigma");
					return false;
				}

				smoothSigma = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"dt") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","dt");
					return false;
				}

				dt = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"substeps") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","substeps");
					return false;
				}

				substeps = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"advectionN") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","advectionN");
					return false;
				}

				advectionN = atoi(argv[k]);
			}
			else if(_strcmpi(argv[k],"input_density_fold") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","input_density_fold");
					return false;
				}

				strcpy_s(input_density_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"input_density_prefix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","input_density_prefix");
					return false;
				}

				strcpy_s(input_density_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"input_density_suffix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","input_density_suffix");
					return false;
				}

				strcpy_s(input_density_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"output_density_fold") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","output_density_fold");
					return false;
				}

				strcpy_s(output_density_fold, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"output_density_prefix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","output_density_prefix");
					return false;
				}

				strcpy_s(output_density_prefix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"output_density_suffix") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","output_density_suffix");
					return false;
				}

				strcpy_s(output_density_suffix, ZQ_MOTION_FIELD_SYNTHESIS_BUFLEN,argv[k]);
			}
			else if(_strcmpi(argv[k],"coarse_field_weight") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","coarse_field_weight");
					return false;
				}

				coarse_field_weight = atof(argv[k]);
			}
			else if(_strcmpi(argv[k],"detail_field_weight") == 0)
			{
				k++;
				if(k >= argc)
				{
					printf("need value for %s\n","detail_field_weight");
					return false;
				}

				detail_field_weight = atof(argv[k]);
			}
			else
			{
				printf("invalid arg: %s\n",argv[k]);
				return false;
			}

			k++;
		}
		return true;
	}

	bool CheckValid()
	{
		if(type == TYPE_SYNTHESIS_FIELD)
		{
			if(reduce_factor <= 0 || reduce_factor >= 1)
			{
				printf("invalid reduce_factor : %f\n",reduce_factor);
				return false;
			}

		}
		else if(type == TYPE_ADVECTION_STATIC)
		{
		}
		else if(type == TYPE_ADVECTION_DYNAMIC)
		{
			if(advectionN > frame_count)
				return false;
		}

		return true;
	}
};

#endif