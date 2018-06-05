#include "ZQ_GridDeformation3D.h"
#include "ZQ_GridDeformation3DOptions.h"
#include "ZQ_DoubleImage3D.h"
#include <stdlib.h>

using namespace ZQ;

double m_noise(double scale)
{
	return rand()%1001/1000.0;
}

template<class T>
void test()
{
	ZQ_GridDeformation3DOptions opt;
	ZQ_GridDeformation3D<T> deform;

	int width = 8, height = 4, depth = 4;
	ZQ_DImage3D<bool> nouseful_flag(width,height,depth,1);
	ZQ_DImage3D<bool> fixed_flag(width,height,depth,1);
	bool*& nouseful_flag_data = nouseful_flag.data();
	bool*& fixed_flag_data = fixed_flag.data();
	fixed_flag_data[0] = true;
	fixed_flag_data[width-1] = true;
	fixed_flag_data[(height-1)*width] = true;
	fixed_flag_data[(height-1)*width+width-1] = true;
	fixed_flag_data[(depth-1)*height*width] = true;
	fixed_flag_data[(depth-1)*height*width+width-1] = true;
	fixed_flag_data[(depth-1)*height*width+(height-1)*width] = true;
	fixed_flag_data[(depth-1)*height*width+(height-1)*width+width-1] = true;

	ZQ_DImage3D<T> input_coord(width,height,depth,3);
	ZQ_DImage3D<T> output_coord(width,height,depth,3);
	T*& input_data = input_coord.data();
	T*& output_data = output_coord.data();

	for(int d = 0;d < depth;d++)
	{
		for(int h = 0;h < height;h++)
		{
			for(int w = 0;w < width;w++)
			{
				int off = d*height*width+h*width+w;
				input_data[off*3+0] = w+m_noise(0.2);
				input_data[off*3+1] = h+m_noise(0.2);
				input_data[off*3+2] = d+m_noise(0.2);
			}
		}
	}

	deform.BuildMatrix(width,height,depth,nouseful_flag_data,fixed_flag_data,opt);

	deform.Deformation(input_data,output_data,false);

	T sum_diff2 = 0;
	for(int i = 0;i < width*height*depth*3;i++)
	{
		T cur_diff = output_data[i] - input_data[i];
		sum_diff2 += cur_diff*cur_diff;
	}
	printf("sum_diff2 = %f\n",sum_diff2);

}


int main()
{
	test<float>();
	test<double>();
	return 0;
}