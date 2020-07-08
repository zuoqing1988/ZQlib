#define _CRT_SECURE_NO_WARNINGS
#include "ZQ_CPURenderer2DWorkspace.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include <opencv2\opencv.hpp>
#include <omp.h>
using namespace ZQ;

bool load_vertexs_from_txt(const char* file,std::vector<float>& all_verts)
{
	all_verts.clear();
	FILE* in = fopen(file, "r");
	if (in == 0)
		return false;
	char buffer[1000] = "";
	do {
		buffer[0] = '\0';
		fgets(buffer, 1000, in);
		if (buffer[0] == '\0')
			break;
		float x, y, z, tx, ty;
		sscanf(buffer, "%f%f%f%f%f", &x, &y, &z, &tx, &ty);
		all_verts.push_back(x);
		all_verts.push_back(y);
		all_verts.push_back(z);
		all_verts.push_back(tx);
		all_verts.push_back(ty);
	} while (true);
	fclose(in);
	return all_verts.size() >= 5;
}

bool save_vertexs_to_bin(const char* file, const std::vector<float>& all_verts)
{
	FILE* out = fopen(file, "wb");
	if (out == 0)
		return false;
	int num_float = all_verts.size();
	if (num_float > 0)
	{
		if (num_float != fwrite(&all_verts[0], sizeof(float), num_float, out))
		{
			fclose(out);
			return false;
		}
	}
	fclose(out);
	return true;
}

bool load_vertexs_from_bin(const char* file, std::vector<float>& all_verts)
{
	all_verts.clear();
	FILE* in = fopen(file, "rb");
	if (in == 0)
		return false;
	fseek(in, 0, SEEK_END);
	long file_len = ftell(in);
	fseek(in, 0, SEEK_SET);
	long num_float = file_len / sizeof(float);
	if (file_len == 0 || file_len % num_float != 0)
	{
		fclose(in);
		return false;
	}
	all_verts.resize(num_float);
	if (num_float != fread(&all_verts[0], sizeof(float), num_float, in))
	{
		fclose(in);
		return false;
	}
	fclose(in);
	return all_verts.size() >= 5;
}

bool load_triangles_from_txt(const char* file, std::vector<int>& all_triangles)
{
	all_triangles.clear();
	FILE* in = fopen(file, "r");
	if (in == 0)
		return false;
	char buffer[1000] = "";
	do {
		buffer[0] = '\0';
		fgets(buffer, 1000, in);
		if (buffer[0] == '\0')
			break;
		float x, y, z;
		sscanf(buffer, "%f%f%f", &x, &y, &z);
		all_triangles.push_back((int)x);
		all_triangles.push_back((int)y);
		all_triangles.push_back((int)z);
	} while (true);
	fclose(in);
	return all_triangles.size() >= 3;
}

bool save_triangles_to_bin(const char* file, const std::vector<int>& all_triangles)
{
	FILE* out = fopen(file, "wb");
	if (out == 0)
		return false;
	int num_int = all_triangles.size();
	if (num_int > 0)
	{
		if (num_int != fwrite(&all_triangles[0], sizeof(int), num_int, out))
		{
			fclose(out);
			return false;
		}
	}
	fclose(out);
	return true;
}


bool load_triangles_from_bin(const char* file, std::vector<int>& all_triangles)
{
	all_triangles.clear();
	FILE* in = fopen(file, "rb");
	if (in == 0)
		return false;
	fseek(in, 0, SEEK_END);
	long file_len = ftell(in);
	fseek(in, 0, SEEK_SET);
	long num_int = file_len / sizeof(int);
	if (file_len == 0 || file_len % num_int != 0)
	{
		fclose(in);
		return false;
	}
	all_triangles.resize(num_int);
	if (num_int != fread(&all_triangles[0], sizeof(int), num_int, in))
	{
		fclose(in);
		return false;
	}
	fclose(in);
	return all_triangles.size() >= 3;
}

int test()
{
	double t1 = omp_get_wtime();
	std::vector<float> all_vertexs;
	std::vector<int> all_triangles;

	if (0)
	{
		if (!load_vertexs_from_txt("vert.txt", all_vertexs) ||
			!load_triangles_from_txt("triangles.txt", all_triangles))
		{
			printf("failed to load mesh!\n");
			return EXIT_FAILURE;
		}
		//save_vertexs_to_bin("vert.bin", all_vertexs);
		//save_triangles_to_bin("triangles.bin", all_triangles);*/
	}
	else
	{
		if (!load_vertexs_from_bin("vert.bin", all_vertexs) ||
			!load_triangles_from_bin("triangles.bin", all_triangles))
		{
			printf("failed to load mesh!\n");
			return EXIT_FAILURE;
		}
	}

	int num_verts = all_vertexs.size() / 5;
	int num_triangles = all_triangles.size() / 3;

	cv::Mat ori_image = cv::imread("7.jpg");
	if (ori_image.empty())
	{
		printf("failed to load ori image!\n");
		return EXIT_FAILURE;
	}
	ZQ_DImage<float> ori_texture, ref_texture, mask_texture, assemble_texture;
	if (/*!ZQ_ImageIO::loadImage(ori_texture, "ori_texture.png", 1) ||*/
		!ZQ_ImageIO::loadImage(ref_texture, "ref_texture.png", 1) ||
		!ZQ_ImageIO::loadImage(mask_texture, "render_mask.png", 0))
	{
		printf("failed to load textures!\n");
		return EXIT_FAILURE;
	}

	//ZQ_ImageIO::Show("test", mask_texture);
	//cv::waitKey(0);
	
	double t2 = omp_get_wtime();

	cv::Mat ref_image = ori_image.clone();
	cv::Mat mask = cv::Mat(ori_image.rows, ori_image.cols, CV_MAKETYPE(8, 1));


	assemble_texture.assemble(ref_texture, mask_texture);
	ZQ_TextureSampler<float> sampler_ori, sampler_ref, sampler_mask, sampler_assemble;
	//sampler_ori.BindImage(ori_texture, false);
	//sampler_ref.BindImage(ref_texture, false);
	//sampler_mask.BindImage(mask_texture, false);
	sampler_assemble.BindImage(assemble_texture, false);

	ZQ_CPURenderer2DWorkspace render2D(ori_image.cols, ori_image.rows);
	render2D.ClearColorBuffer(0, 0, 0, 0);
	render2D.ClearDepthBuffer(10000);

	
	render2D.BindSampler(&sampler_assemble);
	//render2D.EnableTextureSampleCubic();
	render2D.EnableDepthTest();
	//render2D.DisableDepthTest();
	//render2D.EnableAlphaBlend();
	//render2D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_ONE_DST_ONE_MINUS_SRC);
	//render2D.SetAlphaBlendMode(ZQ_CPURenderer3DWorkspace::ALPHABLEND_SRC_PLUS_DST);

	double t3 = omp_get_wtime();
	render2D.RenderIndexedTriangles(&all_vertexs[0], &all_triangles[0], num_verts, num_triangles, ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_TEXCOORD2);
	double t4 = omp_get_wtime();
	
	int buffer_width2D = render2D.GetBufferWidth();
	int buffer_height2D = render2D.GetBufferHeight();
	const float*& buffer_data2D = render2D.GetColorBufferPtr();

	
	int mask_bbox_xmin = ori_image.cols;
	int mask_bbox_xmax = -1;
	int mask_bbox_ymin = ori_image.rows;
	int mask_bbox_ymax = -1;
	for (int i = 0; i < buffer_height2D; i++)
	{
		for (int j = 0; j < buffer_width2D; j++)
		{
			bool vis = buffer_data2D[(i*buffer_width2D + j) * 4 + 3] > 0.5;
			mask.ptr<unsigned char>(i)[j] = vis ? 255 : 0;
			if (vis)
			{
				mask_bbox_xmin = __min(mask_bbox_xmin, j);
				mask_bbox_xmax = __max(mask_bbox_xmax, j);
				mask_bbox_ymin = __min(mask_bbox_ymin, i);
				mask_bbox_ymax = __max(mask_bbox_ymax, i);
				ref_image.ptr<unsigned char>(i)[j * 3 + 0] = buffer_data2D[(i*buffer_width2D + j) * 4 + 0] * 255;
				ref_image.ptr<unsigned char>(i)[j * 3 + 1] = buffer_data2D[(i*buffer_width2D + j) * 4 + 1] * 255;
				ref_image.ptr<unsigned char>(i)[j * 3 + 2] = buffer_data2D[(i*buffer_width2D + j) * 4 + 2] * 255;
			}
		}
	}
	cv::Point center(0.5*(mask_bbox_xmin + mask_bbox_xmax)+0.5, 0.5*(mask_bbox_ymin+mask_bbox_ymax)+0.5);
	
	cv::Mat final_image;
	double t5 = omp_get_wtime();
	cv::seamlessClone(ref_image, ori_image, mask.clone(), center, final_image, cv::NORMAL_CLONE);
	double t6 = omp_get_wtime();
	cv::imwrite("final.png", final_image);

	printf("load: %.3f ms, prepare: %.3f ms, render: %.3f ms, fetch: %.3f ms, merge: %.3f ms\n", 
		1000*(t2 - t1), 1000*(t3 - t2), 1000*(t4 - t3), 1000*(t5 - t4), 1000*(t6 - t5));
	//cv::namedWindow("ori");
	//cv::namedWindow("mask");
	//cv::namedWindow("ref");
	//cv::namedWindow("final");
	//cv::imshow("ori", ori_image);
	//cv::imshow("mask", mask);
	//cv::imshow("ref", ref_image);
	//cv::imshow("final", final_image);
	//cv::waitKey(0);
	
	return EXIT_SUCCESS;
}

int main()
{
	int nIters = 100;
	clock_t t1 = clock();
	for (int i = 0; i < nIters; i++)
	{
		test();
	}
	clock_t t2 = clock();
	double time = 0.001*(t2 - t1);
	printf("total cost: %.3f secs, 1 iter costs %.3f ms\n", time, time / nIters * 1000);
	return EXIT_SUCCESS;
}