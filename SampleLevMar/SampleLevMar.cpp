#include "ZQ_LevMar.h"
#include <stdio.h>

using namespace ZQ;

#define ROSD 105.0

/* Rosenbrock function, global minimum at (1, 1) */
bool _ros(const double *p, double *x, int m, int n, const void *data)
{
	register int i;

	for(i=0; i<n; ++i)
		x[i]=((1.0-p[0])*(1.0-p[0]) + ROSD*(p[1]-p[0]*p[0])*(p[1]-p[0]*p[0]));
	return true;
}

bool _jacros(const double *p, double *jac, int m, int n, const void *data)
{
	register int i, j;

	for(i=j=0; i<n; ++i){
		jac[j++]=(-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]);
		jac[j++]=(2*ROSD*(p[1]-p[0]*p[0]));
	}
	return true;
}


int main()
{
	ZQ_LevMarOptions _opts;
	ZQ_LevMarReturnInfos _infos;
	_opts.init_mu = 1e-3;
	_opts.tol_dx_square = 1e-45;
	_opts.tol_e_square = 1e-45;
	_opts.tol_max_jte = 1e-45;

	int m=2;
	int n=2;
	double p[2]={-1.2,1.0};
	double x[2] = {0};

	ZQ_LevMar::ZQ_LevMar_Der(_ros,_jacros,p,x,m,n,100000,_opts,_infos,0);

	printf("p=(%f, %f)\n",p[0],p[1]);
	printf("it=%d\n",_infos.iter_count);
	return 0;


}