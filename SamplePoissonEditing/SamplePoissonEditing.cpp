#include "ZQ_PoissonEditingOptions.h"
#include "ZQ_PoissonEditing.h"
#include "ZQ_ImageIO.h"

using namespace ZQ;

template<class T>
void test(int argc, const char** argv)
{
	typedef ZQ_DImage<T> DImage;

	DImage mask; // the border of the mask must be 0
	DImage copy_in;
	DImage input,output;

	if(argc < 5)
	{
		printf(" .exe maskfile copyinfile inputfile outputfile [options]\n");
		return;
	}

	const char* mask_file = argv[1];
	const char* copy_in_file = argv[2];
	const char* input_file = argv[3];
	const char* output_file = argv[4];

	ZQ_PoissonEditingOptions opt;
	if(!opt.HandleParas(argc-5,argv+5))
		return ;

	// load in

	/*if(!ZQ_ImageIO::loadImage(mask,"mask1.png",0))
	return ;
	if(!ZQ_ImageIO::loadImage(copy_in,"copy_in1.png",1))
	return ;
	if(!ZQ_ImageIO::loadImage(input,"input1.png",1))
	return ;*/

	if(!ZQ_ImageIO::loadImage(mask,mask_file,0))
	{
		printf("failed to load %s\n",mask_file);
		return ;
	}
	if(!ZQ_ImageIO::loadImage(copy_in,copy_in_file,1))
	{
		printf("failed to load %s\n",copy_in_file);
		return ;
	}

	if(!ZQ_ImageIO::loadImage(input,input_file,1))
	{
		printf("failed to load %s\n",input_file);
		return ;
	}

	//


	if(!ZQ_PoissonEditing::PoissonEditing(mask,copy_in,input,output,opt))
	{
		printf("failed\n");
		return;
	}

	ZQ_ImageIO::saveImage(output,output_file);

}

int main(/*int argc, const char** argv*/)
{

	const char* m_argv[] = 
	{
		"SamplePoissonEditing.exe",
		"mask1.png",
		"copy_in1.png",
		"input1.png",
		"output1.png"
	};

	int m_argc = sizeof(m_argv)/sizeof(char*);

	test<float>(m_argc,m_argv);
	test<double>(m_argc,m_argv);
	return 0;
}