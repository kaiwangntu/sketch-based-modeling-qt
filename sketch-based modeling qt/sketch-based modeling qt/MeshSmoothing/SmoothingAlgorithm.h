#pragma once

#include "PreDef.h"
#include "../CGALDef.h"


class CSmoothingAlgorithm
{
public:
	CSmoothingAlgorithm(void);
	~CSmoothingAlgorithm(void);

	static void BilateralSmooth(KW_Mesh& Mesh,vector<Vertex_handle>& vecVertexToSmooth,double dSigmaC=0.0,double dKernelSize=0.0,double dSigmaS=0.0,int iNormalRingNum=0.0);

protected:

	static int GetKernelVertex(KW_Mesh& Mesh,Vertex_handle hVertex,double dKernelSize,vector<Vertex_handle>& vecKernelVertex,vector<double>& vecDistance);

	static int GetOffsets(Vertex_handle hVertex,vector<Vertex_handle>& vecKernelVertex,vector<double>& vecOffsets);


};
