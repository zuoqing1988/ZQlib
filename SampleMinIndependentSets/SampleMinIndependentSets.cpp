
#include <stdlib.h>
#include "ZQ_Particle2D.h"
#include <time.h>

using namespace ZQ;

void main()
{
	

	/*ZQ_Graph* graph = new ZQ_Graph;
	graph->vert_num = 5;
	graph->edge_num_of_vert = new int[5];
	graph->edge_num_of_vert[0] = 2;
	graph->edge_num_of_vert[1] = 2;
	graph->edge_num_of_vert[2] = 3;
	graph->edge_num_of_vert[3] = 3;
	graph->edge_num_of_vert[4] = 2;
	graph->edges = (int**)malloc(sizeof(int*)*5);
	graph->edges[0] = new int[2];
	graph->edges[0][0] = 1;
	graph->edges[0][1] = 3;
	graph->edges[1] = new int[2];
	graph->edges[1][0] = 0;
	graph->edges[1][1] = 2;
	graph->edges[2] = new int[3];
	graph->edges[2][0] = 1;
	graph->edges[2][1] = 3;
	graph->edges[2][2] = 4;
	graph->edges[3] = new int[3];
	graph->edges[3][0] = 0;
	graph->edges[3][1] = 2;
	graph->edges[3][2] = 4;
	graph->edges[4] = new int[2];
	graph->edges[4][0] = 2;
	graph->edges[4][1] = 3;*/
//	for(int dense = 0;dense < 1000;dense++)
	{
//	printf("dense = %d\n",dense);
	/*int N = 100;
	ZQ_Particle2D* particles = new ZQ_Particle2D[N*N];

	for(int i = 0;i < N;i++)
	{
		for(int j = 0;j < N;j++)
		{
			int idx = i*N+j;
			particles[idx].idx = idx;
			particles[idx].radius = 8;
			particles[idx].x = i;
			particles[idx].y = j;
		}
	}*/

	ZQ_Particle2D* particles = new ZQ_Particle2D[8];
	particles[0].idx = 0;
	particles[0].radius = 0.9;
	particles[0].x = 0;
	particles[0].y = 0;

	particles[1].idx = 1;
	particles[1].radius = 0.9;
	particles[1].x = 1;
	particles[1].y = 0;

	particles[2].idx = 2;
	particles[2].radius = 0.9;
	particles[2].x = 2;
	particles[2].y = 0;

	particles[3].idx = 3;
	particles[3].radius = 0.9;
	particles[3].x = 1;
	particles[3].y = 1;

	particles[4].idx = 4;
	particles[4].radius = 0.9;
	particles[4].x = 1;
	particles[4].y = 2;

	particles[5].idx = 5;
	particles[5].radius = 0.9;
	particles[5].x = 2;
	particles[5].y = 2;

	particles[6].idx = 6;
	particles[6].radius = 0.9;
	particles[6].x = 1;
	particles[6].y = 3;

	particles[7].idx = 7;
	particles[7].radius = 0.9;
	particles[7].x = 1;
	particles[7].y = 4;


	//clock_t t1 = clock();

	ZQ_RawGraph* graph = 0;
	ZQ_Particles_Graph(8,particles,&graph);
	

	graph->Sort();
	ZQ_RawSets* myset = 0;
	ZQ_MinIndependentSets::ZQ_MinIndependentSetsWelshPowell(graph,&myset);
	
	

	//clock_t t2 = clock();
	//printf("cost time:%f\n", 0.001*(t2-t1));
	myset->Print("stdout");

	

	ZQ_RawSets* myset2 = 0;
	ZQ_Particles_Sets(8,particles,&myset2);
	myset2->Print("stdout");

	delete []particles;
	delete graph;
	delete myset;
	delete myset2;
	}

}