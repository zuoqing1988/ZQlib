#include "ZQ_CPURenderer3DWorkspace.h"
#include "ZQ_DoubleImage.h"
#include "ZQ_ImageIO.h"
#include <opencv2/opencv.hpp>

using namespace ZQ;

bool load_cfg(const char* cfg_file, float* incam_intrinsic, float* outcam_intrinsic, float* outcam_view_mat);

bool render_one_image(ZQ_CPURenderer3DWorkspace& render3d, const char* in_rgb_img_file, const char* in_depth_img_file,
	const float fx, float fy, float cx, float cy,
	const char* out_rgb_img_file, const char* out_depth_img_file);

int main(int argc, const char** argv)
{
	if (argc != 3)
	{
		printf("%s cfg_file inlist_file\n", argv[0]);
		return EXIT_FAILURE;
	}
	float incam_intrinsic[6]; //w,h,cx,cy,fx,fy
	float outcam_intrinsic[6]; //w,h,cx,cy,fx,fy
	float outcam_viewmat[16]; // matrix from rgbd cam to outcam
	const char* cfg_file = argv[1];
	const char* inlist_file = argv[2];
	if (!load_cfg(cfg_file, incam_intrinsic, outcam_intrinsic, outcam_viewmat))
	{
		return EXIT_FAILURE;
	}
	//opencv right-hand (Xright,Ydown,Zin) to left-hand(Xright,Yup,Zin)
	float mat_inL_to_inR[16] = 
	{
		1,0,0,0,
		0,-1,0,0,
		0,0,1,0,
		0,0,0,1
	};
	float mat_outR_to_outL[16] = 
	{
		1,0,0,0,
		0,-1,0,0,
		0,0,1,0,
		0,0,0,1
	};
	float mat_inL_to_outR[16];
	float* mat_inR_to_outR = outcam_viewmat;
	ZQ_MathBase::MatrixMul(mat_inR_to_outR, mat_inL_to_inR, 4, 4, 4, mat_inL_to_outR);
	float inL_to_outL[16];
	ZQ_MathBase::MatrixMul(mat_outR_to_outL, mat_inL_to_outR, 4, 4, 4, inL_to_outL);


	ZQ_CPURenderer3DWorkspace render3D(outcam_intrinsic[0], outcam_intrinsic[1], true);
	render3D.SetClip(25, 10000);
	float out_cy = outcam_intrinsic[1] - 1 - outcam_intrinsic[3];
	render3D.SetIntrinsicPara(outcam_intrinsic[2],out_cy,outcam_intrinsic[4],outcam_intrinsic[5]);
	render3D.SetViewMatrix(inL_to_outL);

#if defined(_WIN32)
	FILE* in = 0;
	if(0 != fopen_s(&in,inlist_file, "r"))
#else
	FILE* in = fopen(inlist_file, "r");
	if (in == 0)
#endif
	{
		printf("failed to open %s\n", inlist_file);
		return EXIT_FAILURE;
	}
	char buf[200];
	while (true)
	{
		buf[0] = '\0';
		fgets(buf, 200, in);
		if (buf[0] == '\0')
			break;
		int len = strlen(buf);
		if (buf[len - 1] == '\n')
			buf[--len] = '\0';
		std::string in_rgb_file(buf);
		std::string in_depth_file = in_rgb_file + ".depth";
		std::string out_rgb_file = in_rgb_file + ".out.jpg";
		std::string out_depth_file = in_rgb_file + ".out.depth";
		if (!render_one_image(render3D, in_rgb_file.c_str(), in_depth_file.c_str(), incam_intrinsic[4], incam_intrinsic[5], incam_intrinsic[2], incam_intrinsic[3],
			out_rgb_file.c_str(), out_depth_file.c_str()))
		{
			printf("failed to hand %s\n", in_rgb_file.c_str());
			continue;
		}
	}
	fclose(in);
	return EXIT_SUCCESS;
}

bool load_cfg(const char* cfg_file, float* incam_intrinsic, float* outcam_intrinsic, float* outcam_view_mat)
{
#if defined(_WIN32)
	FILE* in = 0;
	if (0 != fopen_s(&in, cfg_file, "r"))
		return false;
	fscanf_s(in, "%f%f%f%f%f%f",
		&incam_intrinsic[0], &incam_intrinsic[1], //w,h
		&incam_intrinsic[2], &incam_intrinsic[3], //cx,cy
		&incam_intrinsic[4], &incam_intrinsic[5]); //fx,cy
	fscanf_s(in, "%f%f%f%f%f%f",
		&outcam_intrinsic[0], &outcam_intrinsic[1], //w,h 
		&outcam_intrinsic[2], &outcam_intrinsic[3], //cx,cy
		&outcam_intrinsic[4], &outcam_intrinsic[5]); //fx,fy
	for (int i = 0; i < 16; i++)
	{
		fscanf_s(in, "%f", &outcam_view_mat[i]);
	}
#else
	FILE* in = fopen(cfg_file, "r");
	if (in == 0)
		return false;
	fscanf(in, "%f%f%f%f%f%f",
		&incam_intrinsic[0], &incam_intrinsic[1], //w,h
		&incam_intrinsic[2], &incam_intrinsic[3], //cx,cy
		&incam_intrinsic[4], &incam_intrinsic[5]); //fx,cy
	fscanf(in, "%f%f%f%f%f%f",
		&outcam_intrinsic[0], &outcam_intrinsic[1], //w,h 
		&outcam_intrinsic[2], &outcam_intrinsic[3], //cx,cy
		&outcam_intrinsic[4], &outcam_intrinsic[5]); //fx,fy
	for (int i = 0; i < 16; i++)
	{
		fscanf(in, "%f", &outcam_view_mat[i]);
	}
#endif
	
	fclose(in);
	return true;
}

bool render_one_image(ZQ_CPURenderer3DWorkspace& render3d, const char* in_rgb_img_file, const char* in_depth_img_file,
	const float fx, float fy, float cx, float cy,
	const char* out_rgb_img_file, const char* out_depth_img_file)
{
	cv::Mat rgb_im = cv::imread(in_rgb_img_file, 1);
	if (rgb_im.empty())
		return false;
	int im_width = rgb_im.cols;
	int im_height = rgb_im.rows;
	cv::flip(rgb_im, rgb_im, 0);//flip updown
	/*cv::namedWindow("show");
	cv::imshow("show", rgb_im);
	cv::waitKey(0);*/
	std::vector<float> in_depth_buffer(im_width*im_height, 0);
	std::vector<float> in_depth_buffer_tmp(im_width*im_height, 0);
#if defined(_WIN32)
	FILE* in = 0;
	if(0 != fopen_s(&in,in_depth_img_file,"rb"))
#else
	FILE* in = fopen(in_depth_img_file, "rb");
	if (in == 0)
#endif
	{
		return false;
	}
	if (im_width*im_height != fread(in_depth_buffer_tmp.data(), sizeof(float), im_width*im_height, in))
	{
		return false;
	}
	fclose(in);

	for (int i = 0; i < im_height; i++) // flip updown
	{
		const float* src_ptr = in_depth_buffer_tmp.data() + i * im_width;
		float* dst_ptr = in_depth_buffer.data() + (im_height - 1 - i) * im_width;
		memcpy(dst_ptr, src_ptr, sizeof(float)*im_width);
	}
	float up_cy = im_height - 1 - cy;
	int nVerts = im_width*im_height;
	float* vertices = new float[nVerts * 7];
	for (int i = 0; i < im_height; i++)
	{
		for (int j = 0; j < im_width; j++)
		{
			int idx = i*im_width + j;
			float cur_x = (float)(j - cx) / fx;
			float cur_y = (float)(i - up_cy) / fy;
			float scale = 1.0;
			vertices[idx * 7 + 0] = cur_x*in_depth_buffer[idx] * scale;
			vertices[idx * 7 + 1] = cur_y*in_depth_buffer[idx] * scale;
			vertices[idx * 7 + 2] = in_depth_buffer[idx] * scale;
			vertices[idx * 7 + 3] = rgb_im.ptr<unsigned char>(i)[j * 3 + 2];
			vertices[idx * 7 + 4] = rgb_im.ptr<unsigned char>(i)[j * 3 + 1];
			vertices[idx * 7 + 5] = rgb_im.ptr<unsigned char>(i)[j * 3 + 0];
			vertices[idx * 7 + 6] = 255;
		}
	}

	std::vector<int> indices;
	indices.reserve(nVerts * 4 * 3);
	float near_thresh = 350;
	float far_thresh = 4000.0;
	bool* valid_flag = new bool[im_width*im_height];
	for (int i = 0; i < im_width*im_height; i++)
	{
		valid_flag[i] = in_depth_buffer[i] >= near_thresh && in_depth_buffer[i] <= far_thresh;
	}

	for (int i = 0; i < im_height - 1; i++)
	{
		for (int j = 0; j < im_width - 1; j++)
		{
			int idx00 = i*im_width + j;
			int idx01 = i*im_width + j + 1;
			int idx10 = (i + 1)*im_width + j;
			int idx11 = (i + 1)*im_width + j + 1;
			if (valid_flag[idx00] && valid_flag[idx01] && valid_flag[idx10])
			{
				indices.push_back(idx00);
				indices.push_back(idx01);
				indices.push_back(idx10);
				/*indices.push_back(idx01);
				indices.push_back(idx00);
				indices.push_back(idx10);*/
			}
			if (valid_flag[idx10] && valid_flag[idx01] && valid_flag[idx11])
			{
				indices.push_back(idx10);
				indices.push_back(idx01);
				indices.push_back(idx11);
				/*indices.push_back(idx01);
				indices.push_back(idx10);
				indices.push_back(idx11);*/
			}
		}
	}

	int nTriangles = indices.size() / 3;

	render3d.ClearColorBuffer(0, 0, 0, 0);
	render3d.ClearDepthBuffer(10000);
	render3d.RenderIndexedTriangles(vertices, indices.data(), nVerts, nTriangles, ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_COLOR4);
	delete[]valid_flag;
	delete[]vertices;

	int buffer_width = render3d.GetBufferWidth();
	int buffer_height = render3d.GetBufferHeight();
	const float* color_buffer = render3d.GetColorBufferPtr();
	const float* depth_buffer = render3d.GetDepthBufferPtr();
	cv::Mat out_rgb(buffer_height, buffer_width, CV_MAKETYPE(8, 3));
	for (int i = 0; i < buffer_height; i++)
	{
		for (int j = 0; j < buffer_width; j++)
		{
			out_rgb.ptr<unsigned char>(i)[j * 3 + 2] = color_buffer[i*buffer_width * 4 + j * 4 + 0];
			out_rgb.ptr<unsigned char>(i)[j * 3 + 1] = color_buffer[i*buffer_width * 4 + j * 4 + 1];
			out_rgb.ptr<unsigned char>(i)[j * 3 + 0] = color_buffer[i*buffer_width * 4 + j * 4 + 2];
		}
	}
	
	cv::flip(out_rgb, out_rgb, 0);//flip updown
	if (!cv::imwrite(out_rgb_img_file, out_rgb))
	{
		return false;
	}
	std::vector<float> depth(buffer_width*buffer_height);
	for (int i = 0; i < buffer_height; i++) //flip updown
	{
		const float* src_ptr = depth_buffer + i*buffer_width;
		float* dst_ptr = depth.data() + (buffer_height - 1 - i)*buffer_width;
		memcpy(dst_ptr, src_ptr, sizeof(float)*buffer_width);
	}
#if defined(_WIN32)
	FILE* out = 0;
	if (0 != fopen_s(&out, out_depth_img_file, "wb"))
		return false;
#else
	FILE* out = fopen(out_depth_img_file, "wb");
	if (out == 0)
		return false;
#endif
	
	if (buffer_height*buffer_width != fwrite(depth.data(), sizeof(float), buffer_width*buffer_height, out))
	{
		fclose(out);
		return false;
	}
	fclose(out);
	return true;
}