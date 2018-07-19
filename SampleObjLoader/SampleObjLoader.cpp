#include "ZQ_ObjLoader.h"
#include "ZQ_CPURenderer3DWorkSpace.h"
#include "ZQ_Quaternion.h"
#include "ZQ_MathBase.h"
#include "ZQ_ImageIO.h"
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <string>

using namespace ZQ;

struct cam_info
{
	int width, height;
	float focal;
	float view_matrix[16];
};

bool load_cam_info(const char* file, std::vector<cam_info>& infos)
{
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
		float qw, qx, qy, qz, tx, ty, tz, k;
		fscanf_s(in, "%s%d%d%f%f%f%f%f%f%f%f%f", filename, BUF_LEN, &infos[i].width, &infos[i].height, &infos[i].focal,
			&qw, &qx, &qy, &qz, &tx, &ty, &tz, &k);
		ZQ_Quaternion<double> view_quat(qx, qy, qz, qw);
		double cam_t[3] = { tx, ty, tz };
		view_quat.Normalized();
		double view_R[9], view_t[3];
		ZQ_Quaternion<double>::Quat2Rot(view_quat, view_R);
		ZQ_MathBase::MatrixMul(view_R, cam_t, 3, 3, 1, view_t);
		for (int j = 0; j < 3; j++)
			view_t[j] = -view_t[j];
		float view_matrix[16] =
		{
			view_R[0],view_R[1],view_R[2],view_t[0],
			view_R[3],view_R[4],view_R[5],view_t[1],
			view_R[6],view_R[7],view_R[8],view_t[2],
			0,0,0,1
		};
		memcpy(infos[i].view_matrix, view_matrix, sizeof(float) * 16);
	}
	fclose(in);
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
	int cam_num = infos.size();
	int i = 0;
	while (true)
	{
		int width = infos[i].width*0.5;
		int height = infos[i].height*0.5;
		float focal = infos[i].focal*0.5;
		ZQ_CPURenderer3DWorkspace renderer(width, height, true);
		renderer.SetIntrinsicPara(width / 2, height / 2, focal);
		renderer.SetClip(0.001, 100);
		renderer.SetViewMatrix(infos[i].view_matrix);
		//renderer.LookAt(ZQ_Vec3D(-1, -1, 1), ZQ_Vec3D(-0.5, -0.5, 1), ZQ_Vec3D(0, 1, 0));
		renderer.BindSampler(&sampler);
		float tmp_verts[15] =
		{
			-10, -10, 5, 0, 0,
			-10, 10, 5, 0, 1,
			10, 10, 5, 1, 1
		};
		int tmp_indices[3] = { 0,1,2 };
		int vert_Channels = has_Texture ? 5 : 3;
		ZQ_CPURenderer3DWorkspace::VERTEX_FORMAT vert_fmt = has_Texture ?
			ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3_TEXCOORD2 :
			ZQ_CPURenderer3DWorkspace::VERTEX_POSITION3;

		for (int i = 0; i < nTri; i++)
		{
			for (int j = 0; j < 3; j++)
				memcpy(tmp_verts + j*vert_Channels, vert_ptr + (i * 3 + j) * 3, sizeof(float) * 3);

			if (has_Texture)
			{
				for (int j = 0; j < 3; j++)
					memcpy(tmp_verts + j*vert_Channels + 3, tex_coord_ptr + (i * 3 + j) * 2, sizeof(float) * 2);
			}
			renderer.RenderIndexedTriangles(tmp_verts, tmp_indices, 3, 1, vert_fmt);
		}

		const float* color_buffer = renderer.GetColorBufferPtr();
		IplImage* img = cvCreateImage(cv::Size(width, height), IPL_DEPTH_8U, 3);
		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				cvSet2D(img, h, w, cvScalar(color_buffer[(h*width + w) * 4 + 0] * 255,
					color_buffer[(h*width + w) * 4 + 1] * 255, color_buffer[(h*width + w) * 4 + 2] * 255));
			}
		}
		cvNamedWindow("show");
		cvShowImage("show", img);
		int key = cvWaitKey(20);
		cvReleaseImage(&img);
		if (key == 27)
			break;
		i = (i + 1) % cam_num;
	}
	
	return EXIT_SUCCESS;
}


