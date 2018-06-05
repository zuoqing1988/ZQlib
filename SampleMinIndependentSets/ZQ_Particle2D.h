#ifndef _ZQ_PARTICLE_2D_H_
#define _ZQ_PARTICLE_2D_H_
#pragma once

#include "ZQ_MinIndependentSets.h"
#include <vector>
#include <iostream>

using namespace ZQ;
class ZQ_Particle2D
{
public:
	float x,y;
	float radius;
	int idx;
};

/* when particles[i].idx != i , it will fail*/
bool ZQ_Particles_Graph(int num, ZQ_Particle2D* particles, ZQ_RawGraph** graph);

/* when particles[i].idx != i , it will fail*/
bool ZQ_Particles_Sets(int num, ZQ_Particle2D* particles, ZQ_RawSets** sets);

#include "ZQ_Particle2D.h"

bool ZQ_Particles_Graph(int num, ZQ_Particle2D* particles, ZQ_RawGraph** graph)
{
	if (particles == 0 || num <= 0)
		return false;
	for (int i = 0; i < num; i++)
	{
		if (particles[i].idx != i)
			return false;
	}
	float max_radius = particles[0].radius;
	for (int i = 1; i < num; i++)
	{
		if (max_radius < particles[i].radius)
		{
			max_radius = particles[i].radius;
		}
	}

	float boxmin[2], boxmax[2];
	boxmin[0] = particles[0].x;
	boxmin[1] = particles[0].y;
	boxmax[0] = particles[0].x;
	boxmax[1] = particles[0].y;

	for (int i = 1; i < num; i++)
	{
		if (boxmin[0] > particles[i].x)
			boxmin[0] = particles[i].x;
		if (boxmin[1] > particles[i].y)
			boxmin[1] = particles[i].y;
		if (boxmax[0] < particles[i].x)
			boxmax[0] = particles[i].x;
		if (boxmax[1] < particles[i].y)
			boxmax[1] = particles[i].y;
	}
	boxmin[0] -= 0.1*max_radius;
	boxmax[0] += 0.1*max_radius;
	boxmin[1] -= 0.1*max_radius;
	boxmax[1] += 0.1*max_radius;

	float dimlen = max_radius * 2;

	int xdim = (boxmax[0] - boxmin[0]) / dimlen + 1;
	int ydim = (boxmax[1] - boxmin[1]) / dimlen + 1;


	//std::vector< std::vector<int> > buckets(xdim*ydim);
	// do not use vector vector, it costs more memory

	int** buckets = (int**)malloc(sizeof(int*)*ydim*xdim);
	memset(buckets, 0, sizeof(int*)*xdim*ydim);
	int* bucket_count = new int[xdim*ydim];
	for (int i = 0; i < xdim*ydim; i++)
		bucket_count[i] = 0;

	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		bucket_count[x*ydim + y] ++;
	}
	for (int i = 0; i < xdim*ydim; i++)
	{
		if (bucket_count[i] > 0)
		{
			buckets[i] = new int[bucket_count[i]];
			bucket_count[i] = 0;
		}
	}
	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		buckets[x*ydim + y][bucket_count[x*ydim + y]] = particles[i].idx;
		bucket_count[x*ydim + y] ++;
	}

	(*graph) = new ZQ_RawGraph;
	(*graph)->vert_num = num;
	(*graph)->edge_num_of_vert = new int[num];
	(*graph)->edges = (int**)malloc(sizeof(int*)*num);

	int whole_edge_num = 0;

	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		int idx = particles[i].idx;

		std::vector<int> cur_edge;
		int bucket_idx;
		for (int nx = -1; nx <= 1; nx++)
		{
			for (int ny = -1; ny <= 1; ny++)
			{
				if (nx + x < 0 || nx + x >= xdim || ny + y < 0 || ny + y >= ydim)
					continue;
				bucket_idx = (x + nx)*ydim + (y + ny);
				for (int j = 0; j < bucket_count[bucket_idx]; j++)
				{
					int p_idx = buckets[bucket_idx][j];
					if (idx != p_idx)
					{
						if ((particles[idx].x - particles[p_idx].x)*(particles[idx].x - particles[p_idx].x)
							+ (particles[idx].y - particles[p_idx].y)*(particles[idx].y - particles[p_idx].y)
							<= (particles[idx].radius + particles[p_idx].radius)*(particles[idx].radius + particles[p_idx].radius))
						{
							cur_edge.push_back(p_idx);
						}
					}
				}
			}
		}
		whole_edge_num += cur_edge.size();
		//printf("%d,%d,whole_edge_num = %d\n",i,cur_edge.size(), whole_edge_num);
		(*graph)->edge_num_of_vert[i] = cur_edge.size();
		(*graph)->edges[i] = new int[cur_edge.size()];
		for (int j = 0; j < cur_edge.size(); j++)
			(*graph)->edges[i][j] = cur_edge[j];
		cur_edge.clear();
	}

	(*graph)->idx_of_each_row = new int[num];
	(*graph)->row_of_each_idx = new int[num];
	for (int i = 0; i < num; i++)
	{
		(*graph)->idx_of_each_row[i] = i;
		(*graph)->row_of_each_idx[i] = i;
	}

	printf("whole_edge_num = %d, average_edge = %.2f\n", whole_edge_num, (float)whole_edge_num / num);

	for (int i = 0; i < xdim*ydim; i++)
	{
		if (buckets[i])
		{
			delete[](buckets[i]);
			buckets[i] = 0;
		}
	}
	delete[]buckets;
	buckets = 0;
	return true;
}

bool ZQ_Particles_Sets(int num, ZQ_Particle2D* particles, ZQ_RawSets** sets)
{
	if (particles == 0 || num <= 0)
		return false;
	for (int i = 0; i < num; i++)
	{
		if (particles[i].idx != i)
			return false;
	}

	/************* build buckets to check the neighbor for graph******************/

	float max_radius = particles[0].radius;
	for (int i = 1; i < num; i++)
	{
		if (max_radius < particles[i].radius)
		{
			max_radius = particles[i].radius;
		}
	}

	float boxmin[2], boxmax[2];
	boxmin[0] = particles[0].x;
	boxmin[1] = particles[0].y;
	boxmax[0] = particles[0].x;
	boxmax[1] = particles[0].y;

	for (int i = 1; i < num; i++)
	{
		if (boxmin[0] > particles[i].x)
			boxmin[0] = particles[i].x;
		if (boxmin[1] > particles[i].y)
			boxmin[1] = particles[i].y;
		if (boxmax[0] < particles[i].x)
			boxmax[0] = particles[i].x;
		if (boxmax[1] < particles[i].y)
			boxmax[1] = particles[i].y;
	}
	boxmin[0] -= 0.1*max_radius;
	boxmax[0] += 0.1*max_radius;
	boxmin[1] -= 0.1*max_radius;
	boxmax[1] += 0.1*max_radius;

	float dimlen = max_radius * 2;

	int xdim = (boxmax[0] - boxmin[0]) / dimlen + 1;
	int ydim = (boxmax[1] - boxmin[1]) / dimlen + 1;


	//std::vector< std::vector<int> > buckets(xdim*ydim);
	// do not use vector vector, it costs more memory

	int** buckets = (int**)malloc(sizeof(int*)*ydim*xdim);
	memset(buckets, 0, sizeof(int*)*xdim*ydim);
	int* bucket_count = new int[xdim*ydim];
	for (int i = 0; i < xdim*ydim; i++)
		bucket_count[i] = 0;

	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		bucket_count[x*ydim + y] ++;
	}
	for (int i = 0; i < xdim*ydim; i++)
	{
		if (bucket_count[i] > 0)
		{
			buckets[i] = new int[bucket_count[i]];
			bucket_count[i] = 0;
		}
	}
	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		buckets[x*ydim + y][bucket_count[x*ydim + y]] = particles[i].idx;
		bucket_count[x*ydim + y] ++;
	}

	/*************** first iter to count how many edges each vertex has **********/
	int* edge_num_of_vert = new int[num];
	memset(edge_num_of_vert, 0, sizeof(int)*num);

	int whole_edge_num = 0;

	for (int i = 0; i < num; i++)
	{
		int x = (particles[i].x - boxmin[0]) / dimlen;
		int y = (particles[i].y - boxmin[1]) / dimlen;
		int idx = particles[i].idx;

		int bucket_idx;
		for (int nx = -1; nx <= 1; nx++)
		{
			for (int ny = -1; ny <= 1; ny++)
			{
				if (nx + x < 0 || nx + x >= xdim || ny + y < 0 || ny + y >= ydim)
					continue;
				bucket_idx = (x + nx)*ydim + (y + ny);
				for (int j = 0; j < bucket_count[bucket_idx]; j++)
				{
					int p_idx = buckets[bucket_idx][j];
					if (idx != p_idx)
					{
						if ((particles[idx].x - particles[p_idx].x)*(particles[idx].x - particles[p_idx].x)
							+ (particles[idx].y - particles[p_idx].y)*(particles[idx].y - particles[p_idx].y)
							<= (particles[idx].radius + particles[p_idx].radius)*(particles[idx].radius + particles[p_idx].radius))
						{
							edge_num_of_vert[i] ++;
						}
					}
				}
			}
		}
		whole_edge_num += edge_num_of_vert[i];
		//printf("%d,%d,whole_edge_num = %d\n",i,cur_edge.size(), whole_edge_num);
	}

	/******************************** sort and prepare for Welsh-Powell ****************************/
	int* idx = new int[num];
	int* val = new int[num];
	for (int i = 0; i < num; i++)
	{
		idx[i] = i;
		val[i] = edge_num_of_vert[i];
	}

	int len = 1;
	int K = 0;
	while (len < num)
	{
		len *= 2;
		K++;
	}
	for (int i = 0; i < K; i++)
	{
		ZQ_RawGraph::_mergeSort(num, idx, val, i);
	}

	int* idx_of_each_row = new int[num];
	int* row_of_each_idx = new int[num];
	for (int i = 0; i < num; i++)
	{
		int real_idx = idx[i];
		idx_of_each_row[i] = real_idx;
		row_of_each_idx[real_idx] = i;
	}

	/***************************** Welsh-Powell ********************************/
	std::vector< std::vector<int> > mySets;
	int num_sets = 0;
	int vert_num = num;
	int* vert_color = new int[vert_num];
	for (int i = 0; i < vert_num; i++)
		vert_color[i] = -1;

	int cur_color_num = 0;
	int start_color = 0;
	for (int i = 0; i < vert_num; i++)
	{
		int real_idx = idx_of_each_row[i];
		if (cur_color_num == 0)
		{
			std::vector<int> one_set;
			one_set.push_back(real_idx);
			mySets.push_back(one_set);
			vert_color[i] = 0;
			cur_color_num = 1;
			start_color = 0;
		}
		else
		{
			/******************** get edges for each vertex***********/
			/* why we do so? because we cannot store the whole graph
			*/
			int x = (particles[real_idx].x - boxmin[0]) / dimlen;
			int y = (particles[real_idx].y - boxmin[1]) / dimlen;

			std::vector<int> cur_edge;
			int bucket_idx;
			for (int nx = -1; nx <= 1; nx++)
			{
				for (int ny = -1; ny <= 1; ny++)
				{
					if (nx + x < 0 || nx + x >= xdim || ny + y < 0 || ny + y >= ydim)
						continue;
					bucket_idx = (x + nx)*ydim + (y + ny);
					for (int j = 0; j < bucket_count[bucket_idx]; j++)
					{
						int p_idx = buckets[bucket_idx][j];
						if (real_idx != p_idx)
						{
							if ((particles[real_idx].x - particles[p_idx].x)*(particles[real_idx].x - particles[p_idx].x)
								+ (particles[real_idx].y - particles[p_idx].y)*(particles[real_idx].y - particles[p_idx].y)
								<= (particles[real_idx].radius + particles[p_idx].radius)*(particles[real_idx].radius + particles[p_idx].radius))
							{
								cur_edge.push_back(p_idx);
							}
						}
					}
				}
			}
			/***********/
			bool* conflict = new bool[cur_color_num];
			memset(conflict, false, sizeof(bool)*cur_color_num);
			for (int j = 0; j < cur_edge.size(); j++)
			{
				int neighbor_idx = cur_edge[j];
				int neighbor_row = row_of_each_idx[neighbor_idx];
				if (neighbor_row < i)
				{
					conflict[vert_color[neighbor_row]] = true;
				}
			}
			int mycolor = cur_color_num;
			for (int j = 0; j < cur_color_num; j++)
			{
				int real_j = j + start_color + 1;
				if (real_j >= cur_color_num)
					real_j %= cur_color_num;
				if (!conflict[real_j])
				{
					mycolor = real_j;
					break;
				}
			}
			vert_color[i] = mycolor;
			start_color = mycolor;
			if (mycolor == cur_color_num)
			{
				std::vector<int> one_set;
				one_set.push_back(real_idx);
				mySets.push_back(one_set);

				cur_color_num++;
			}
			else
			{
				mySets[mycolor].push_back(real_idx);

			}
			delete[]conflict;

			cur_edge.clear();
		}
	}

	*sets = new ZQ_RawSets;
	(*sets)->set_num = cur_color_num;
	(*sets)->element_num_of_set = new int[cur_color_num];
	(*sets)->elements = (int**)malloc(sizeof(int*)*cur_color_num);
	for (int i = 0; i < cur_color_num; i++)
	{
		int elmt_num_of_set = mySets[i].size();
		(*sets)->element_num_of_set[i] = elmt_num_of_set;
		(*sets)->elements[i] = new int[elmt_num_of_set];
		for (int j = 0; j < elmt_num_of_set; j++)
			(*sets)->elements[i][j] = mySets[i][j];
	}
	delete[]vert_color;
	mySets.clear();

	return true;
}

#endif