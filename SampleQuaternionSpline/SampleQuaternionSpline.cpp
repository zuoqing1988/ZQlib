#include "ZQ_QuaternionSpline.h"

using namespace ZQ;
using namespace std;

int main(int argc, const char** argv)
{
	ZQ_QuaternionSpline qspline;
	vector<double> ts;
	vector<ZQ_Quaternion<double>> qs;

	for (int i = 0; i < 10; i++)
	{
		ts.push_back(i);
		ZQ_Quaternion<double> q;
		q.x = rand() % 10;
		q.y = rand() % 10;
		q.z = rand() % 10;
		q.w = rand() % 10 + 0.1;
		q.Normalized();
		qs.push_back(q);
	}
	qspline.SetPoints(ts, qs, 100);
	for (double j = 0; j < 10; j += 0.1)
	{
		ZQ_Quaternion<double> q = qspline(j);
		printf("t=%8.3f, q = %.3f %.3f %.3f %.3f\n", j, q.x, q.y, q.z, q.w);
	}
	return 0;
}