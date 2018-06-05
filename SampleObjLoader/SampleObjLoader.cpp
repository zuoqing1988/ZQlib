#include "ZQ_ObjLoader.h"
#include <stdio.h>

using namespace ZQ;
void main()
{
	ZQ_RawMesh raw_mesh;
	raw_mesh.LoadFromObjFile("lingjian_01.obj");

	printf("%d : %d\n",raw_mesh.GetVertexNum(),raw_mesh.GetTriangleNum());
}