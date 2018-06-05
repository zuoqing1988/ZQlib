#include <stdio.h>
#include "ZQ_GraphCut.h"

using namespace ZQ;

int main()
{
	typedef Graph<double,double,double> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ 9, /*estimated # of edges*/ 40); 

	g -> add_node(9); 

	const double max_val = 1e9;

	// terminals
	g->add_tweights(0,max_val,0);
	g->add_tweights(1,max_val,0);
	g->add_tweights(2,max_val,0);
	g->add_tweights(6,0,max_val);
	g->add_tweights(7,0,max_val);
	g->add_tweights(8,0,max_val);


	//edges

	g->add_edge(0,1,1,1);
	g->add_edge(1,2,2,2);
	g->add_edge(3,4,3,3);
	g->add_edge(4,5,4,4);
	g->add_edge(6,7,5,5);
	g->add_edge(7,8,6,6);
	g->add_edge(0,3,2,2);
	g->add_edge(1,4,8,8);
	g->add_edge(2,5,9,9);
	g->add_edge(3,6,10,10);
	g->add_edge(4,7,1,1);
	g->add_edge(5,8,2,2);

	int flow = g -> maxflow();

	printf("Flow = %d\n", flow);
	printf("Minimum cut:\n");
	printf("SOURCE: ");
	for(int i = 0;i < 9;i++)
	{
		if(g->what_segment(i) == GraphType::SOURCE)
			printf("%d ",i);
	}
	printf("\n");
	printf("SINK: ");
	for(int i = 0;i < 9;i++)
	{
		if(g->what_segment(i) == GraphType::SINK)
			printf("%d ",i);
	}
	printf("\n");

	delete g;

	return 0;
}
