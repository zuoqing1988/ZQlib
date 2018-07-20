#include "ZQ_ObjLoader.h"
#include "ZQ_CPURenderer3DWorkSpace.h"
#include "ZQ_Quaternion.h"
#include "ZQ_QuaternionSpline.h"
#include "ZQ_Spline.h"
#include "ZQ_MathBase.h"
#include "ZQ_ImageIO.h"
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <string>
#include <omp.h>

using namespace ZQ;

struct cam_info
{
	int width, height;
	float focal;
	float view_matrix[16];
};

bool load_cam_info(const char* file, std::vector<cam_info>& infos)
{
	std::vector<ZQ_Quaternion<double>> cam_quats;
	std::vector<int> cam_width;
	std::vector<int> cam_height;
	std::vector<double> cam_focal;
	std::vector<double> cam_tx;
	std::vector<double> cam_ty;
	std::vector<double> cam_tz;
	
	FILE* in = 0;
	if (0 != fopen_s(&in, file, "r"))
	{
		printf("failed to open file %s\n", file);
		return false;
	}
	int num = 0;
	fscanf_s(in, "%d", &num);
	infos.resize(num);
	const static int BUF_LEN = 100;
	char filename[BUF_LEN];
	for (int i = 0; i < num; i++)
	{
		int width, height;
		float focal, qw, qx, qy, qz, tx, ty, tz, k;
		fscanf_s(in, "%s%d%d%f%f%f%f%f%f%f%f%f", filename, BUF_LEN, &width, &height, &focal,
			&qw, &qx, &qy, &qz, &tx, &ty, &tz, &k);
		ZQ_Quaternion<double> quat(-qx, -qy, -qz, qw);
		quat.Normalized();
		cam_quats.push_back(quat);
		cam_width.push_back(width);
		cam_height.push_back(height);
		cam_focal.push_back(focal);
		cam_tx.push_back(tx);
		cam_ty.push_back(ty);
		cam_tz.push_back(tz);
	}
	fclose(in);

	if (num < 3)
		return false;

	ZQ_Spline spline_cam_tx, spline_cam_ty, spline_cam_tz, spline_focal;
	ZQ_QuaternionSpline qspline_cam_rot;

	std::vector<double> x, y;
	for (int i = -2; i < num + 2; i++)
	{
		x.push_back(i);
		y.push_back(cam_tx[(i + num) % num]);
	}
	spline_cam_tx.SetPoints(x, y);

	x.clear(); y.clear();
	for (int i = -2; i < num + 2; i++)
	{
		x.push_back(i);
		y.push_back(cam_ty[(i + num) % num]);
	}
	spline_cam_ty.SetPoints(x, y);

	x.clear(); y.clear();
	for (int i = -2; i < num + 2; i++)
	{
		x.push_back(i);
		y.push_back(cam_tz[(i + num) % num]);
	}
	spline_cam_tz.SetPoints(x, y);

	x.clear(); y.clear();
	for (int i = -2; i < num + 2; i++)
	{
		int id = (i + num) % num;
		x.push_back(i);
		int cur_height = cam_height[id];
		y.push_back(cam_focal[id]/cur_height*cam_height[0]);
	}
	spline_focal.SetPoints(x, y);

	std::vector<ZQ_Quaternion<double>> y_q;
	x.clear(); y.clear();
	for (int i = -2; i < num + 2; i++)
	{
		x.push_back(i);
		y_q.push_back(cam_quats[(i+num)%num]);
	}
	qspline_cam_rot.SetPoints(x, y_q, 1000);

	float scale = 25;
	float step = 1.0 / scale;
	int out_num = num * scale;
	infos.resize(out_num);
	for (int i = 0; i < out_num; i++)
	{
		double cur_x = i*step;
		ZQ_Quaternion<double> cur_q = qspline_cam_rot(cur_x);
		double t[3] = { spline_cam_tx(cur_x), spline_cam_ty(cur_x), spline_cam_tz(cur_x) };
		double cam_R[9];
		ZQ_Quaternion<double>::Quat2Rot(cur_q, cam_R);
		double cam_matrix[16] =
		{
			cam_R[0],cam_R[1],cam_R[2], t[0],
			cam_R[3],cam_R[4],cam_R[5], t[1],
			cam_R[6],cam_R[7],cam_R[8], t[2],
			0,0,0,1
		};
		double view_matrix[16];
		ZQ_MathBase::MatrixInverse(cam_matrix, 4, view_matrix);
		for (int j = 0; j < 16; j++)
			infos[i].view_matrix[j] = view_matrix[j];
		infos[i].width = cam_width[0];
		infos[i].height = cam_height[0];
		infos[i].focal = spline_focal(cur_x);
	}
	return true;
}

bool load_mesh_texture(ZQ_RawMesh& raw_mesh, ZQ_TextureSampler<float>& sampler, const char* mesh_file, const char* texture_file)
{
	if (!raw_mesh.LoadFromObjFile(mesh_file))
	{
		printf("failed to load obj %s\n", mesh_file);
		return false;
	}
	ZQ_DImage<float> texture;
	if (!ZQ_ImageIO::loadImage(texture, texture_file, 1))
	{
		printf("failed to load image %s\n", texture_file);
		return false;
	}
	texture.FlipY();
	sampler.BindImage(texture, false);
	return true;
}

int main()
{
	std::vector<cam_info> infos;
	if (!load_cam_info("info.txt", infos))
	{
		printf("failed to load cam infos\n");
		return EXIT_FAILURE;
	}
	ZQ_RawMesh raw_mesh;
	ZQ_TextureSampler<float> sampler;
	if (!load_mesh_texture(raw_mesh, sampler, "texture_mesh.obj", "texture_mesh.jpg"))
	{
		printf("failed to load mesh\n");
		return EXIT_FAILURE;
	}

	int nVert = raw_mesh.GetVertexNum();
	int nTri = raw_mesh.GetTriangleNum();
	bool has_Texture = raw_mesh.HasTexCoord();
	const float* vert_ptr = raw_mesh.GetVerticesPtr();
	const unsigned int* indices_ptr = raw_mesh.GetTriangleIndicesPtr();
	const float* tex_coord_ptr = raw_mesh.GetTexCoordPtr();

	/** render **/
	int thread_num = __max(1, omp_get_num_procs() - 1);
	int cam_num = infos.size();
	int i = 0;
	while (true)
	{
		int width = infos[i].width*0.5;
		int height = infos[i].height*0.5;
		float focal = infos[i].focal*0.5;
		std::vector<ZQ_CPURenderer3DWorkspace*> renderer(thread_num);
		float clip_far = 100;
		for (int j = 0; j < thread_num; j++)
		{
			renderer[j] = new ZQ_CPURenderer3DWorkspace(width, height, true);
			renderer[j]->SetIntrinsicPara(width / 2, height / 2, focal);
			renderer[j]->SetClip(0.001, clip_far);
			renderer[j]->SetViewMatrix(infos[i].view_matrix);
			renderer[j]->BindSampler(&sampler);
			renderer[j]->ClearDepthBuffer(clip_far+1);
		}
		std::vector<float> tmp_buffer(thread_num * 15);
		int tmp_indices[3] = { 0,1,2 };
		int vert_Channels = has_Texture ? 5 : 3;
		ZQ_CPURenderer3DWorkspace::VERTEX_FORMAT vert_fmt = has_Texture ?
			ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_TEXCOORD2 :
			ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3;

		int chunk_size = (nTri + thread_num - 1) / thread_num;
#pragma omp parallel for schedule(static, chunk_size) num_threads(thread_num)
		for (int i = 0; i < nTri; i++)
		{
			int thread_id = omp_get_thread_num();
			float* tmp_verts = &tmp_buffer[0] + thread_id*vert_Channels * 3;
			for (int j = 0; j < 3; j++)
				memcpy(tmp_verts + j*vert_Channels, vert_ptr + (i * 3 + j) * 3, sizeof(float) * 3);

			if (has_Texture)
			{
				for (int j = 0; j < 3; j++)
					memcpy(tmp_verts + j*vert_Channels + 3, tex_coord_ptr + (i * 3 + j) * 2, sizeof(float) * 2);
			}
			
			renderer[thread_id]->RenderIndexedTriangles(tmp_verts, tmp_indices, 3, 1, vert_fmt);
		}

		std::vector<const float*> color_buffer(thread_num);
		std::vector<const float*> depth_buffer(thread_num);
		for (int j = 0; j < thread_num; j++)
		{
			color_buffer[j] = renderer[j]->GetColorBufferPtr();
			depth_buffer[j] = renderer[j]->GetDepthBufferPtr();
		}
		IplImage* img = cvCreateImage(cv::Size(width, height), IPL_DEPTH_8U, 3);
		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				int offset = h*width + w;
				int id = -1;
				float cur_depth = FLT_MAX;
				for (int j = 0; j < thread_num; j++)
				{
					if (depth_buffer[j][offset] <= cur_depth)
					{
						id = j;
						cur_depth = depth_buffer[j][offset];
					}
				}
				cvSet2D(img, h, w, cvScalar(color_buffer[id][offset * 4 + 0] * 255,
					color_buffer[id][offset * 4 + 1] * 255, color_buffer[id][offset * 4 + 2] * 255));
			}
		}
		for (int j = 0; j < thread_num; j++)
			delete renderer[j];

		cvNamedWindow("show");
		cvShowImage("show", img);
		int key = cvWaitKey(10);
		cvReleaseImage(&img);
		if (key == 27)
			break;
		i = (i + 1) % cam_num;
	}

	return EXIT_SUCCESS;
}


