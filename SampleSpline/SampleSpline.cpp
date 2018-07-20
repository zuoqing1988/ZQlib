#include "ZQ_Spline.h"

using namespace ZQ;
int main(int argc, const char** argv)
{
	ZQ_Spline spline;
	std::vector<double> x;
	std::vector<double> y;
	for (int i = 0; i <= 5; i++)
	{
		x.push_back(i);
		y.push_back(i % 2);
	}
	spline.SetPoints(x, y);

	for (int i = 0; i <= 100; i++)
	{
		printf("%12.5f %12.5f\n", i*0.05, spline(i*0.05));
	}
	return EXIT_SUCCESS;
}