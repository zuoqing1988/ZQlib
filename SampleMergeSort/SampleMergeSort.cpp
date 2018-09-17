#include "ZQ_MergeSort.h"
#include <vector>
#include <omp.h>

using namespace ZQ;

void test_ooc(int nBlock = 7);

int main()
{
	const int N = 10;
	std::vector<int> values(N);
	std::vector<int> indices(N);
	for (int i = 0; i < N; i++)
	{
		values[i] = rand() % 10;
		indices[i] = i;
	}
	
	std::vector<int> tmp_vals;
	std::vector<int> tmp_idx;
	tmp_vals = values;
	ZQ_MergeSort::MergeSort(&tmp_vals[0], N, true);
	for (int i = 0; i < N; i++)
	{
		printf("%2d ", tmp_vals[i]);
	}
	printf("\n");

	printf("\n");
	tmp_vals = values;
	tmp_idx = indices;
	ZQ_MergeSort::MergeSort(&tmp_vals[0], &tmp_idx[0], N, true);
	for (int i = 0; i < N; i++)
	{
		printf("%2d ", tmp_vals[i]);
	}
	printf("\n");
	for (int i = 0; i < N; i++)
	{
		printf("%2d ", tmp_idx[i]);
	}
	printf("\n");

	printf("\n");
	tmp_vals = values;
	tmp_idx = indices;
	ZQ_MergeSort::MergeSortWithData(&tmp_vals[0], &tmp_idx[0], sizeof(int), N, true);
	for (int i = 0; i < N; i++)
	{
		printf("%2d ", tmp_vals[i]);
	}
	printf("\n");
	for (int i = 0; i < N; i++)
	{
		printf("%2d ", tmp_idx[i]);
	}
	printf("\n");

	/*test ooc*/
	test_ooc(5000);

	return EXIT_SUCCESS;
}

void test_ooc(int nBlock)
{
	const char* src_val_filename = "src.val";
	const char* src_dat_filename = "src.dat";
	const char* dst_val_filename = "dst.val";
	const char* dst_dat_filename = "dst.dat";
	FILE* src_val_file = 0, * src_dat_file = 0;
	if (0 != fopen_s(&src_val_file, src_val_filename, "wb"))
	{
		printf("failed to create file %s\n", src_val_filename);
		return;
	}
	if (0 != fopen_s(&src_dat_file, src_dat_filename, "wb"))
	{
		printf("failed to create file %s\n", src_dat_filename);
		fclose(src_val_file);
		return;
	}
	typedef float T;
	bool ascending_dir = false;
	int BLOCK_SIZE = 1024*1024;
	int N = BLOCK_SIZE*nBlock;
	if (nBlock <= 100)
	{
		__int64 idx = 0;
		std::vector<T> block_buffer(N);
		std::vector<__int64> data_buffer(N);
		for (int i = 0; i < N; i++)
		{
			block_buffer[i] = idx;
			data_buffer[i] = idx;
			idx++;
		}
		fwrite(&block_buffer[0], sizeof(T), N, src_val_file);
		fwrite(&data_buffer[0], sizeof(__int64), N, src_dat_file);
		fclose(src_val_file);
		fclose(src_dat_file);

		ZQ_MergeSort::MergeSortWithData_OOC<T>(src_val_filename, dst_val_filename, src_dat_filename, dst_dat_filename,
			sizeof(__int64), ascending_dir, 1024*100);

		ZQ_MergeSort::MergeSortWithData<T>(&block_buffer[0], &data_buffer[0], sizeof(__int64), N, ascending_dir);
		
		std::vector<T> result_buffer(N);
		std::vector<__int64> result_data(N);
		//check
		FILE* dst_val_file = 0;
		FILE* dst_dat_file = 0;
		fopen_s(&dst_val_file, dst_val_filename, "rb");
		fopen_s(&dst_dat_file, dst_dat_filename, "rb");
		if (N != fread(&result_buffer[0], sizeof(T), N, dst_val_file))
		{
			printf("failed to read val\n");
		}
		if (N != fread(&result_data[0], sizeof(__int64), N, dst_dat_file))
		{
			printf("failed to read dat\n");
		}
		fclose(dst_val_file);
		fclose(dst_dat_file);
		for (int i = 0; i < N; i++)
		{
			if (block_buffer[i] != result_buffer[i] || data_buffer[i] != result_data[i])
			{
				printf("error:%12d (%12lld,%12lld) != (%12lld,%12lld)\n", i, block_buffer[i], data_buffer[i], result_buffer[i], result_data[i]);
			}	
		}
	}
	else
	{

		__int64 idx = 0;
		for (int i = 0; i < nBlock; i++)
		{
			std::vector<T> block_buffer(BLOCK_SIZE);
			std::vector<__int64> data_buffer(BLOCK_SIZE);
			for (int j = 0; j < BLOCK_SIZE; j++)
			{
				block_buffer[j] = idx;
				data_buffer[j] = idx;
				idx++;
			}
			fwrite(&block_buffer[0], sizeof(T), BLOCK_SIZE, src_val_file);
			fwrite(&data_buffer[0], sizeof(__int64), BLOCK_SIZE, src_dat_file);
		}
		fclose(src_val_file);
		fclose(src_dat_file);

		double t1 = omp_get_wtime();
		ZQ_MergeSort::MergeSortWithData_OOC<T>(src_val_filename, dst_val_filename, src_dat_filename, dst_dat_filename,
			sizeof(__int64), ascending_dir, 1024*100);
		double t2 = omp_get_wtime();

		printf("N = %lld, cost: %.3f secs\n", N,t2 - t1);
		//check
		FILE* dst_val_file = 0;
		fopen_s(&dst_val_file, dst_val_filename, "rb");
		float last_val, cur_val;
		fread(&last_val, sizeof(T), 1, dst_val_file);
		for (__int64 i = 1; i < N; i++)
		{
			fread(&cur_val, sizeof(T), 1, dst_val_file);
			if (cur_val < last_val != ascending_dir)
			{
				printf("error:%lld %f %f\n", i, last_val, cur_val);
			}
		}
		fclose(dst_val_file);
	}
	
	
}