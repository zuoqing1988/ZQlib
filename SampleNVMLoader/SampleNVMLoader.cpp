#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include "ZQ_NVMLoader.h"
using namespace ZQ;

int main(int argc, const char** argv)
{
	if (argc != 3)
	{
		printf(".exe nvm_file out_file\n");
		return -1;
	}
	const char* nvm_file = argv[1];
	const char* out_file = argv[2];

	ZQ_NVM_Container container;
	if (!container.LoadFromFile(nvm_file))
	{
		printf("failed to load nvm file %s\n", nvm_file);
		return -1;
	}

	if (container.model_list.size() == 0)
	{
		printf("no model is in nvm file %s\n", nvm_file);
		return -1;
	}

	FILE* out = fopen(out_file, "w");
	if (out == 0)
	{
		printf("failed to create file %s\n", out_file);
		return -1;
	}

	int ncam = container.model_list[0].camera_list.size();
	for (int i = 0; i < ncam; i++)
	{
		fprintf(out, "%s %f ", container.model_list[0].camera_list[i].image_name.c_str(), container.model_list[0].camera_list[i].focal);
		fprintf(out, "%f %f %f %f ", container.model_list[0].camera_list[i].qw, container.model_list[0].camera_list[i].qx, container.model_list[0].camera_list[i].qy, container.model_list[0].camera_list[i].qz);
		fprintf(out, "%f %f %f ", container.model_list[0].camera_list[i].tx, container.model_list[0].camera_list[i].ty, container.model_list[0].camera_list[i].tz);
		fprintf(out, "%f \n", container.model_list[0].camera_list[i].r);
	}

	fclose(out);
	printf("succ to save %s\n", out_file);
	return 0;
}