#pragma once
#ifndef CWEDGEEDGE_BASED_DEFORM_H
#define  CWEDGEEDGE_BASED_DEFORM_H

#include "../GeometryAlgorithm.h"
#include "../Math/Math.h"

#define WEDGE_EDGE_CONSTRAINED_HANDLE_WEIGHT 1


class CWedgeEdgeBasedDeform
{
	friend class CMatConsDeform;

public:
	CWedgeEdgeBasedDeform(void);
	~CWedgeEdgeBasedDeform(void);

	static void WedgeEdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints,bool bTestIsoScale=false);

	//mesh vertices are selected as handle vertices directly
	static void WedgeEdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,	vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints);

	//compute wedge edge-domain mesh for rendering
	static void GetEdgeDomainMesh(KW_Mesh& PrimalMesh,map<Point_3,vector<Point_3>>& EdgeMesh);

	//build wedge edge mesh for rendering and testing
	//the bulit mesh is with cgla polyhedron_3 data structure
	static void BuildWedgeEdgeMesh(KW_Mesh& PrimalMesh,KW_Mesh& WedgeEdgeMesh);

	/////////////////////////////////////////////////////////////////////////////////
	static void Test(double dLamda,int iType,int iIterNum,KW_Mesh& PrimalMesh,
		vector<Vertex_handle>& vecHandleNb,	vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints);
	

protected:

	//get edges involved
	//anchor edges are the edges whose two end vertices both belong toe anchor vertices, they are useful for computing
	//rotation matrices
	static void GetEdgeInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle>& ROIEdges,vector<Halfedge_handle>& AnchorEdges);

	//get the ordered neighbor halfedges&vertices for each handle+roi edge
	static void GetEdgeTopo(vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>>& NeighborEdges,
		vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>>& AnchorNeighborEdges);

	//set laplacian to each edge
	static void ComputeEdgeLaplacian(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges);

	//get laplacian matrix
	static void GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,SparseMatrix& LaplacianMatrix);

	//get the constraint matrix of anchor and handle vertices
	static void GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint,
		vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix,
		SparseMatrix& HandleConstraintMatrix);

	//backup edge vectors for computing rotation in rigid deformation
	static void BackUpEdgeVectorsForRigidDeform(vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges,
		vector<Halfedge_handle>& AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges);

	//compute the right hand side of the naive laplacian equation
	static void ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	static void ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute Rotation in rigid deformation
	static void ComputeRotationForRigidDeform(int iType,vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges,
		vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges);

	//compute Scale for Laplacian deformation
	static void ComputeScaleFactor(int iType,vector<Halfedge_handle> ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges,
		vector<Halfedge_handle> AnchorEdges,vector<vector<Halfedge_handle>> AnchorNeighborEdges,bool bTestIsoScale=false);


	/////////////////////////////////////////////////////////////////////////////////
	static void TestGetROIandAnchorInEdgeMesh(KW_Mesh& PrimalMesh,
		vector<Vertex_handle>& vecHandleNb,	vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		KW_Mesh& EdgeMesh,set<int>& vecEdgeHandleNb,
		vector<Vertex_handle>& vecEdgeROIVertices,vector<Vertex_handle>& vecEdgeAnchorVertices,
		SparseMatrix& VerToEdgeMatrix);
};

#endif
