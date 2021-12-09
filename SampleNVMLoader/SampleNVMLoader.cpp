#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include "ZQ_NVMLoader.h"
using namespace ZQ;

int extract_camera_info(const char* nvm_file, const char* out_file);

int extract_points_info_of_camera(const char* nvm_file, const char* came_name, const char* out_file);


int main(int argc, const char** argv)
{
	if (argc != 3 && argc != 4)
	{
		printf(".exe nvm_file out_file\n");
		printf(".exe nvm_file came_name out_file\n");
		return EXIT_FAILURE;
	}
	if (argc == 3)
	{
		return extract_camera_info(argv[1], argv[2]);
	}
	else if (argc == 4)
	{
		return extract_points_info_of_camera(argv[1], argv[2], argv[3]);
	}
	return EXIT_FAILURE;
}

int extract_camera_info(const char* nvm_file, const char* out_file)
{
	ZQ_NVM_Container container;
	if (!container.LoadFromFile(nvm_file))
	{
		printf("failed to load nvm file %s\n", nvm_file);
		return EXIT_FAILURE;
	}

	if (container.model_list.size() == 0)
	{
		printf("no model is in nvm file %s\n", nvm_file);
		return EXIT_FAILURE;
	}

	FILE* out = fopen(out_file, "w");
	if (out == 0)
	{
		printf("failed to create file %s\n", out_file);
		return EXIT_FAILURE;
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
	return EXIT_SUCCESS;
}

int extract_points_info_of_camera(const char* nvm_file, const char* came_name, const char* out_file)
{
	ZQ_NVM_Container container;
	if (!container.LoadFromFile(nvm_file))
	{
		printf("failed to load nvm file %s\n", nvm_file);
		return EXIT_FAILURE;
	}

	if (!container.ExportPointsOfOneImage(came_name, out_file))
	{
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}