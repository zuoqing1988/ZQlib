#include "ZQ_SparseLevMar.h"
#include "ZQ_SparseMatrix.h"

using namespace ZQ;

#define ROSD 105.0

/* Rosenbrock function, global minimum at (1, 1) */
template<class T>
bool _ros(const T *p, T *x, int m, int n, const void *data)
{
	for(int i=0; i<n; ++i)
		x[i]=((1.0-p[0])*(1.0-p[0]) + ROSD*(p[1]-p[0]*p[0])*(p[1]-p[0]*p[0]));
	return true;
}

template<class T>
bool _jacros(const T *p, taucs_ccs_matrix* &jac, int m, int n, const void *data)
{

	if(jac)
		ZQ_TaucsBase::ZQ_taucs_ccs_free(jac);

	if(0)
	{
		jac = new taucs_ccs_matrix();
		jac->m = n;
		jac->n = m;
		jac->flags = TAUCS_DOUBLE;
		jac->colptr = new int[m+1];
		jac->rowind = new int[4];
		jac->values.d = new double[4];
		jac->colptr[0] = 0;
		jac->colptr[1] = 2;
		jac->colptr[2] = 4;
		jac->rowind[0] = 0;
		jac->rowind[1] = 1;
		jac->rowind[2] = 0;
		jac->rowind[3] = 1;
		T* vals = (T*)(jac->values.d);
		vals[0] = (-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]);
		vals[1] = (-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]);
		vals[2] = (2*ROSD*(p[1]-p[0]*p[0]));
		vals[3] = (2*ROSD*(p[1]-p[0]*p[0]));

	}
	else
	{
		ZQ_SparseMatrix<T> mat(2,2);
		mat.SetValue(0,0,(-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]));
		mat.SetValue(1,0,(2*ROSD*(p[1]-p[0]*p[0])));
		mat.SetValue(1,0,(-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]));
		mat.SetValue(1,1,(2*ROSD*(p[1]-p[0]*p[0])));
		int flag;
		if(strcmp(typeid(T).name(),"float") == 0)
			flag = TAUCS_SINGLE;
		else if(strcmp(typeid(T).name(),"double") == 0)
			flag = TAUCS_DOUBLE;
		else
			return false;
		jac = mat.ExportCCS(flag);
	}
	

	return true;
}

template<class T>
void test()
{
	ZQ_SparseLevMarOptions _opts;
	ZQ_SparseLevMarReturnInfos _infos;
	_opts.init_mu = 1e-3;
	_opts.tol_dx_square = 1e-45;
	_opts.tol_e_square = 1e-45;
	_opts.tol_max_jte = 1e-45;
	_opts.pcg_solver_max_iter = 10;
	_opts.pcg_solver_tol = 1e-9;

	int m=2;
	int n=2;
	T p[2]={-1.2,1.0};
	T x[2] = {0};

	ZQ_SparseLevMar::ZQ_SparseLevMar_Der<T>(_ros,_jacros,p,x,m,n,100000,_opts,_infos,0);

	printf("p=(%f, %f)\n",p[0],p[1]);
	printf("it=%d\n",_infos.iter_count);
}

int main()
{
	test<float>();
	test<double>();
	return 0;

}
