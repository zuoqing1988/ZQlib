#include "ZQ_ImageIO.h"
#include "ZQ_LazySnappingGUI.h"
using namespace std;
using namespace ZQ;

typedef float BaseType;

int main(int argc, const char** argv)
{
	if (argc < 4)
	{
		printf(".exe infile outfile outmaskfile\n");
		return 0;
	}

	const char* infile = argv[1];
	const char* forefile = argv[2];
	const char* maskfile = argv[3];

	ZQ_DImage<BaseType> in_im, tri_map;
	if (!ZQ_ImageIO::loadImage(in_im, infile, 1))
	{
		printf("failed to load %s\n", infile);
		return 0;
	}

	ZQ_LazySnappingGUI<BaseType>::GetInstance()->Run(in_im, tri_map);
	ZQ_DImage<BaseType> fore_im(in_im);
	int nChannels = in_im.nchannels();
	for (int i = 0; i < in_im.npixels(); i++)
	{
		for (int c = 0; c < nChannels; c++)
		{
			fore_im.data()[i*nChannels] *= tri_map.data()[i];
		}
	}
	ZQ_ImageIO::saveImage(tri_map, maskfile);
	ZQ_ImageIO::saveImage(fore_im, forefile);

	return 0;
}
