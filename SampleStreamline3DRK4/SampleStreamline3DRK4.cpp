#include "ZQ_Advection3D.h"
#include "ZQ_DoubleImage3D.h"

using namespace ZQ;
using namespace ZQ_Advection3D;
typedef ZQ_DImage3D<float> DImage3D;

void main(int argc, char** argv)
{
	if(argc != 3 && argc != 4)
	{
		printf(" in_file out_file\n"
			" in_file out_file mask_file\n"
			);
		return ;
	}

	DImage3D flow;
	if(!flow.loadImage(argv[1]))
	{
		printf("cannot open file %s\n",argv[1]);
		return ;
	}
	else
	{
		int width = flow.width();
		int height = flow.height();
		int depth = flow.depth();
		int nChannels = flow.nchannels();
		if(nChannels != 3)
		{
			printf("channels is not 3\n");
			return ;
		}


		bool use_mask = false;
		DImage3D mask;

		if(argc == 4)
		{
			if(!mask.loadImage(argv[3]))
			{
				printf("cannot open file %s\n",argv[3]);
				return ;
			}
			if(mask.matchDimension(width,height,depth,1) == false)
			{
				printf("mask does not match dimension with flow\n");
				return ;
			}
			use_mask = true;
		}


		FILE* out = fopen(argv[2],"w");
		if(out == 0)
		{
			printf("cannot create file %s\n",argv[2]);
			return ;
		}

		fprintf(out,"format pos+vel\n");
		fprintf(out,"%d %d %d\n",width,height,depth);

		float voxel_xlen = 1;
		float voxel_ylen = 1;
		float voxel_zlen = 1;

		for(int k = 0;k < depth/8;k++)
		{
			for(int j = 0;j < height/8;j++)
			{
				for(int i = 0;i < width/8;i++)
				{
					ZQ_Vec3D start((i*8+4)*voxel_xlen,(j*8+4)*voxel_ylen,(k*8+4)*voxel_zlen);

					std::vector<ZQ_Vec3D> points1;
					std::vector<ZQ_Vec3D> points2;
					std::vector<ZQ_Vec3D> vels1;
					std::vector<ZQ_Vec3D> vels2;

					ZQ_ForwardStreamlineRK4(flow.data(),width,height,depth,voxel_xlen,voxel_ylen,voxel_zlen,0.1,0.5*voxel_xlen,100,start,points1,vels1);
					ZQ_BacktraceStreamlineRK4(flow.data(),width,height,depth,voxel_xlen,voxel_ylen,voxel_zlen,0.1,0.5*voxel_xlen,100,start,points2,vels2);
					

					int num = points1.size()+points2.size()-1;
					if(num >= 2)
					{
						fprintf(out,"%d\n",points1.size()+points2.size()-1);

						for(int c = points2.size()-1 ;c > 0; c--)
							fprintf(out,"%f %f %f %f %f %f ", points2[c].x, points2[c].y, points2[c].z, vels2[c].x, vels2[c].y, vels2[c].z);

						for(int c = 0;c < points1.size();c++)
							fprintf(out,"%f %f %f %f %f %f ", points1[c].x, points1[c].y, points1[c].z, vels1[c].x, vels1[c].y, vels1[c].z);

						fprintf(out,"\n");
					}
				}
			}

		}
		fclose(out);
	}
}