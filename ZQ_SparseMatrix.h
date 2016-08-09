#ifndef _ZQ_SPARSE_MATRIX_H_
#define _ZQ_SPARSE_MATRIX_H_
#pragma once

#include <vector>
#include "ZQ_taucs.h"

namespace ZQ
{
	/***********************************************************************************************
	WARNING: 
	this class is easier to create taucs_ccs_matrix, and use less memory.
	however, it is slower than direction using the std::vector<std::map<int,double>> structures
	***********************************************************************************************/
	template<class T>
	class ZQ_SparseMatrix
	{
	private:
		struct SparseMatrixElement
		{
			int idx;
			T value;
		};

	public:
		ZQ_SparseMatrix(int rows, int cols);
		~ZQ_SparseMatrix();

		ZQ_SparseMatrix& operator=(const ZQ_SparseMatrix& other);

	private:

		int row;
		int col;
		std::vector<std::vector<SparseMatrixElement> > mat;

	public:

		int GetRow();

		int GetCol();

		int GetNNZ();

		bool AddTo(int row, int col, T value);

		bool SetValue(int row, int col, T value);

		T GetValue(int row, int col);

		void ScaleRow(int row, T scale);
		void ScaleCol(int col, T scale);

		taucs_ccs_matrix* ExportCCS(int flag = TAUCS_DOUBLE);

		ZQ_SparseMatrix* Transpose();

	private:
		bool _find(std::vector<SparseMatrixElement>& one_col, int row, T& value);

		void _addto(std::vector<SparseMatrixElement>& one_col, int row, T value); 

		void _setvalue(std::vector<SparseMatrixElement>& one_col, int row, T value); 
	};

	template<class T>
	ZQ_SparseMatrix<T>::ZQ_SparseMatrix(int rows, int cols)
	{
		this->row = rows;
		this->col = cols;

		mat.clear();

		for(int i = 0;i < this->col;i++)
		{
			std::vector<SparseMatrixElement> one_col;
			mat.push_back(one_col);
		}
	}

	template<class T>
	ZQ_SparseMatrix<T>::~ZQ_SparseMatrix()
	{
		this->row = 0;
		this->col = 0;
		mat.clear();
	}

	template<class T>
	ZQ_SparseMatrix<T>& ZQ_SparseMatrix<T>::operator =(const ZQ_SparseMatrix& other)
	{
		this->row = other.row;
		this->col = other.col;
		this->mat = other.mat;
		return *this;
	}

	template<class T>
	int ZQ_SparseMatrix<T>::GetRow()
	{
		return this->row;
	}

	template<class T>
	int ZQ_SparseMatrix<T>::GetCol()
	{
		return this->col;
	}

	template<class T>
	int ZQ_SparseMatrix<T>::GetNNZ()
	{
		int nnz = 0;
		for(int i = 0;i < this->col;i++)
			nnz += mat[i].size();

		return nnz;
	}

	template<class T>
	bool ZQ_SparseMatrix<T>::AddTo(int row, int col, T value)
	{
		if(row < 0 || row >= this->row || col < 0 || col >= this->col)
			return false;

		_addto(mat[col],row,value);

		return true;
	}

	template<class T>
	bool ZQ_SparseMatrix<T>::SetValue(int row, int col, T value)
	{
		if(row < 0 || row >= this->row || col < 0 || col >= this->col)
			return false;

		_setvalue(mat[col],row,value);

		return true;
	}

	template<class T>
	T ZQ_SparseMatrix<T>::GetValue(int row, int col)
	{
		if(row < 0 || row >= this->row || col < 0 || col >= this->col)
			return 0;

		T value = 0;
		if(!_find(mat[col],row,value))
			return 0;
		else
			return value;
	}

	template<class T>
	void ZQ_SparseMatrix<T>::ScaleRow(int row_id, T scale)
	{
		for(int i = 0;i < col;i++)
		{
			T val;
			if(_find(mat[i],row_id,val))
			{
				_setvalue(mat[i],row_id,val*scale);
			}
		}
	}

	template<class T>
	void ZQ_SparseMatrix<T>::ScaleCol(int col_id, T scale)
	{
		if(col_id >= 0 && col_id < col)
		{
			for(int i = 0;i < mat[col_id].size();i++)
			{
				mat[col_id][i].value *= scale;
			}
		}
	}

	template<class T>
	taucs_ccs_matrix* ZQ_SparseMatrix<T>::ExportCCS(int flag)
	{
		if(flag != TAUCS_DOUBLE && flag != TAUCS_SINGLE)
			return 0;

		int nRows = this->row;
		int nCols = this->col;
		int nnz = GetNNZ();

		//create taucs_ccs_matrix
		taucs_ccs_matrix* matC = new taucs_ccs_matrix;
		matC->m = nRows;
		matC->n = nCols;
		matC->rowind = 0;
		matC->colptr = new int[nCols+1];
		matC->flags = flag;

		if(flag == TAUCS_DOUBLE)
		{
			if(nnz > 0)
			{
				matC->rowind = new int[nnz];
				matC->values.d = new double[nnz];
			}
			else
			{
				matC->rowind = 0;
				matC->values.d = 0;
			}
		}
		else //flag == TAUCS_SINGLE
		{
			if(nnz > 0)
			{
				matC->rowind = new int[nnz];
				matC->values.s = new float[nnz];
			}
			else
			{
				matC->rowind = 0;
				matC->values.s = 0;
			}
		}


		// copy cols into matC
		std::vector<SparseMatrixElement>::const_iterator rit;
		int rowptrC = 0;
		if(flag == TAUCS_DOUBLE)
		{
			for (int c = 0;c < nCols;++c) 
			{
				matC->colptr[c] = rowptrC;
				for (rit = mat[c].begin(); rit != mat[c].end(); ++rit) 
				{
					matC->rowind[rowptrC] = rit->idx;
					matC->values.d[rowptrC] = rit->value;
					++rowptrC;
				}
			}
		}
		else
		{
			for (int c = 0; c < nCols; ++c) 
			{
				matC->colptr[c] = rowptrC;
				for (rit = mat[c].begin(); rit != mat[c].end();++rit) 
				{
					matC->rowind[rowptrC] = rit->idx;
					matC->values.s[rowptrC] = rit->value;
					++rowptrC;
				}
			}
		}
		matC->colptr[nCols] = nnz;
		return matC;
	}

	template<class T>
	ZQ_SparseMatrix<T>* ZQ_SparseMatrix<T>::Transpose()
	{
		ZQ_SparseMatrix* sparse = new ZQ_SparseMatrix(this->col,this->row);

		std::vector<SparseMatrixElement>::const_iterator rit;
		for(int i = 0;i < col;i++)
		{
			int cur_row = i;
			for(rit = mat[i].begin(); rit != mat[i].end(); ++rit)
			{
				int cur_col = rit->idx;
				T cur_value = rit->value;
				sparse->AddTo(cur_row,cur_col,cur_value);
			}
		}

		return sparse;
	}

	template<class T>
	bool ZQ_SparseMatrix<T>::_find(std::vector<SparseMatrixElement>& one_col, int row, T& value)
	{
		int size = one_col.size();
		if(size == 0)
		{
			value = 0;
			return false;
		}
		int low = 0;
		int high = size-1;
		int mid = size/2;
		bool find_flag = false;

		do 
		{
			if(one_col[mid].idx == row)
			{
				find_flag = true;
				value = one_col[mid].value;
				break;
			}
			else if(one_col[mid].idx < row)
			{
				low = mid+1;
				mid = (low+high)/2;
			}
			else
			{
				high = mid-1;
				mid = (low+high)/2;
			}
		} while (low <= high);

		if(find_flag)
		{
			return true;
		}
		else
		{
			value = 0;
			return false;
		}

		return true;
	}

	template<class T>
	void ZQ_SparseMatrix<T>::_addto(std::vector<SparseMatrixElement>& one_col, int row, T value)
	{
		int size = one_col.size();
		SparseMatrixElement element;
		element.idx = row;
		element.value = value;

		if(size == 0)
		{
			one_col.push_back(element);
			return ;
		}

		int low = 0;
		int high = size-1;
		int mid = (low+high)/2;
		bool find_flag = false;

		do 
		{
			if(one_col[mid].idx == row)
			{
				find_flag = true;
				one_col[mid].value += value;
				break;
			}
			else if(one_col[mid].idx < row)
			{
				low = mid+1;
				mid = (low+high)/2;
			}
			else
			{
				high = mid-1;
				mid = (low+high)/2;
			}
		} while (low <= high);

		if(find_flag)
		{
			return ;
		}
		else
		{
			if(one_col[0].idx > row)
			{
				one_col.insert(one_col.begin(),element);
			}
			else if(one_col[size-1].idx < row)
			{
				one_col.push_back(element);
			}
			else
			{
				if(one_col[high].idx < row)
				{
					do 
					{
						if(one_col[high+1].idx > row)
						{
							one_col.insert(one_col.begin()+high+1,element);
							break;
						}
						high ++;
						if(high+1 >= size)
						{
							one_col.push_back(element);
							break;
						}
					} while (true);
				}
				else
				{
					do 
					{
						if(one_col[high-1].idx < row)
						{
							one_col.insert(one_col.begin()+high,element);
							break;
						}
						high--;
						if(high < 0)
						{
							one_col.insert(one_col.begin(),element);
							break;
						}
					} while (true);
				}
			}
		}
	}

	template<class T>
	void ZQ_SparseMatrix<T>::_setvalue(std::vector<SparseMatrixElement>& one_col, int row, T value)
	{
		int size = one_col.size();
		SparseMatrixElement element;
		element.idx = row;
		element.value = value;

		if(size == 0)
		{
			one_col.push_back(element);
			return ;
		}

		int low = 0;
		int high = size-1;
		int mid = (low+high)/2;
		bool find_flag = false;

		do 
		{
			if(one_col[mid].idx == row)
			{
				find_flag = true;
				one_col[mid].value = value;
				break;
			}
			else if(one_col[mid].idx < row)
			{
				low = mid+1;
				mid = (low+high)/2;
			}
			else
			{
				high = mid-1;
				mid = (low+high)/2;
			}
		} while (low <= high);

		if(find_flag)
		{
			return ;
		}
		else
		{
			if(one_col[0].idx > row)
			{
				one_col.insert(one_col.begin(),element);
			}
			else if(one_col[size-1].idx < row)
			{
				one_col.push_back(element);
			}
			else
			{
				if(one_col[high].idx < row)
				{
					do 
					{
						if(one_col[high+1].idx > row)
						{
							one_col.insert(one_col.begin()+high+1,element);
							break;
						}
						high ++;
						if(high+1 >= size)
						{
							one_col.push_back(element);
							break;
						}
					} while (true);
				}
				else
				{
					do 
					{
						if(one_col[high-1].idx < row)
						{
							one_col.insert(one_col.begin()+high,element);
							break;
						}
						high--;
						if(high < 0)
						{
							one_col.insert(one_col.begin(),element);
							break;
						}
					} while (true);
				}
			}
		}
	}
}


#endif
