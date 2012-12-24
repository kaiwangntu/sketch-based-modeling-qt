#pragma once
#ifndef CDUAL_MESH_DEFORM_H
#define CDUAL_MESH_DEFORM_H

#include "../GeometryAlgorithm.h"
#include "../Math/Math.h"

#define  CONSTRAINED_HANDLE_WEIGHT 1

class CDualMeshDeform
{
public:
	CDualMeshDeform(void);
	~CDualMeshDeform(void);

	//dual mesh deformation
	static void DualMeshRigidDeformTest(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecPrimalHandlePoint,vector<Vertex_handle>& vecPrimalHandleNb,
		vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
		vector<Point_3>& vecPrimalDeformCurvePoint3d,
		KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices);


	static void DualMeshRigidDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecPrimalHandlePoint,vector<Vertex_handle>& vecPrimalHandleNb,
		vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
		vector<Point_3>& vecPrimalDeformCurvePoint3d,
		KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices);


	static void FlexibleDualMeshDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecPrimalHandlePoint,vector<Vertex_handle>& vecPrimalHandleNb,
		vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
		vector<Point_3>& vecPrimalDeformCurvePoint3d,
		KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices);

	//flexible dual mesh deformation,using similar data structures to edge-based method
	static void FlexibleDualMeshDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints,bool bTestIsoScale=false);

	static void GetDualMeshInfo(KW_Mesh& PrimalMesh,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,vector<DualMeshVertexStruct>& DualMesh,
		int* pDualMeshVertexNum);

	static void BuildDualMesh(KW_Mesh& PrimalMesh,vector<Vertex_handle>& vecPrimalHandleNb,
		vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
		KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices);

	//compute dual mesh for rendering (test)
	static void GetDualMeshTest(KW_Mesh& PrimalMesh,map<Point_3,vector<Point_3>>& DualMesh);

	//iteratively solve the laplacian problem
	static void IterativeDualLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecPrimalHandlePoint,vector<Vertex_handle>& vecPrimalHandleNb,
		vector<Vertex_handle>& PrimalROIVertices,vector<Vertex_handle>& vecPrimalAnchorVertices,
		vector<Point_3>& vecPrimalDeformCurvePoint3d,
		KW_Mesh& DualMesh,vector<Vertex_handle>& vecDualHandle,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices);



protected:

	//get dual laplacian matrix
	static void ComputeDualLaplacianMatrix(int iType,KW_Mesh& Mesh,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<vector<double> >& vecvecLaplacianMatrix);

	//get the matrix from primal to dual
	static void GetPrimalToDualMatrix(vector<Vertex_handle> vecPrimalHandleNb,vector<Vertex_handle> PrimalROIVertices,
		vector<Vertex_handle> vecPrimalAnchorVertices,vector<Vertex_handle> vecDualHandle,vector<Vertex_handle> DualROIVertices,
		vector<Vertex_handle> vecDualAnchorVertices,vector<vector<double> >& PrimalToDualMatrix);
	//given the handle neighbor vertices and the new curve positions, compute the new positions for the handle neighbors
	static void ComputeHandleNbNewPos(KW_Mesh& Mesh,vector<HandlePointStruct>& vecHandlePoint,
		vector<Vertex_handle>& vecHandleNb,vector<Point_3>& vecDeformCurvePoint3d,
		vector<Point_3>& vecHandleNbNewPos);

	//compute the right hand side of the naive dual equation
	//It contains 1: the delta value of laplacian of dual Handle&ROI,2:coordinates value of primal anchor
	//3: the deformed coordinates value of the primal handle vertices
	static void ComputeNaiveDualRightHandSide(int iType,vector<Vertex_handle> vecDualHandleNb,
		vector<Vertex_handle> DualROIVertices,vector<Point_3> vecPrimalDeformCurvePoint3d,
		vector<Vertex_handle> vecPrimalAnchorVertices,
		vector<vector<double> >& LaplacianRightHandSide,vector<vector<double> >& AnchorRightHandSide,
		vector<vector<double> >& HandleRightHandSide);

	//compute the right hand side of the rigid equation
	//It contains 1: the delta value of rigid of Dual Handle&ROI,2:coordinates value of primal anchor
	//3: the deformed coordinates value of the primal handle vertices
	static void ComputeDualRigidRightHandSide(int iType,vector<Vertex_handle>& vecDualHandleNb,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices,
		vector<Point_3> vecPrimalDeformCurvePoint3d,vector<Vertex_handle> vecPrimalAnchorVertices,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,
		vector<vector<double> >& HandleRightHandSide);


	//compute the right hand side of the rigid equation
	//It contains 1: the delta value of rigid of Dual Handle&ROI,2:coordinates value of primal anchor
	//3: the deformed coordinates value of the primal handle vertices
	static void ComputeFlexibleDualRightHandSide(double dLamda,int iType,vector<Vertex_handle>& vecDualHandleNb,
		vector<Vertex_handle>& DualROIVertices,vector<Vertex_handle>& vecDualAnchorVertices,
		vector<Point_3> vecPrimalDeformCurvePoint3d,vector<Vertex_handle> vecPrimalAnchorVertices,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,
		vector<vector<double> >& HandleRightHandSide);


	//update dual mesh geoemtry after primal mesh's been deformed
	static void UpdateDualMeshGeometry(vector<Vertex_handle> vecPrimalHandleNb,vector<Vertex_handle> PrimalROIVertices,
		vector<Vertex_handle> vecPrimalAnchorVertices,vector<Vertex_handle> vecDualHandle,vector<Vertex_handle> DualROIVertices,
		vector<Vertex_handle> vecDualAnchorVertices,vector<vector<double> > PrimalToDualMatrix);

	//given the new handle neighbor veertex positions,compute the new positions for dual handle vertices.
	static void ComputeDualHandleNewPos(vector<Point_3> vecHandleNbNewPos,
		vector<Vertex_handle>& vecDualHandle,vector<Point_3>& vecNewDualHandlePos);

	//compute primal mesh from dual mesh
	static void ConvertDualMeshToPrimal(vector<Vertex_handle>& vecPrimalHandleNb,vector<Vertex_handle>& PrimalROIVertices,
		vector<Vertex_handle>& vecPrimalAnchorVertices,vector<Point_3> vecPrimalHandleNbNewPos,
		vector<Vertex_handle>& vecDualHandle,vector<Vertex_handle>& DualROIVertices,
		vector<Vertex_handle>& vecDualAnchorVertices);
	//get the constraint matrix dual anchor and handle vertices
	static void GetDualConstraintMatrix(vector<Vertex_handle> vecDualHandle,vector<Vertex_handle> DualROIVertices,
		vector<Vertex_handle> vecDualAnchorVertices,vector<vector<double> >& AnchorConstraintMatrix,
		vector<vector<double> >& HandleConstraintMatrix);

	//iterately update right hand side of dual laplacian(update the normal direction and laplacian magnitude)
	static void IterativeUpdateDualLaplacianRightHandSide(int iType,vector<Vertex_handle> vecDualHandleNb,
		vector<Vertex_handle> DualROIVertices,vector<Point_3> PrimalAnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

/////////////////////////////////////////////////////////////////////////////////////////////////////
	//get facets involved
	static void GetFacetInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle>& ROIFacets);

	//get the ordered neighbor facets for each handle+roi facet
	static void GetFacetTopo(vector<Facet_handle> ROIFacets,vector<vector<Facet_handle>>& NeighborFacets);

	//set laplacian to each facet
	static void ComputeFacetLaplacian(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets);

	//get half more layer anchor,who are the vertices of the neighbor triangles of roi facets and not
	//included in the current anchors
	static void GetHalfMoreLayerAnchor(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets,
		vector<Vertex_handle>& OneMoreLayerAnchor);

	//get laplacian matrix
	static void GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,vector<Facet_handle> ROIFacets,
		vector<vector<Facet_handle>> NeighborFacets,SparseMatrix& LaplacianMatrix);

	//backup edge vectors for computing rotation in rigid deformation
	static void BackUpEdgeVectorsForRigidDeform(vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets);

	//compute the right hand side of the naive laplacian equation
	static void ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<Facet_handle> ROIFacets,vector<vector<Facet_handle>> NeighborFacets,
		vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	static void ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<Facet_handle> ROIFacets,
		vector<vector<Facet_handle>> NeighborFacets,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute Rotation in rigid deformation
	static void ComputeRotationForRigidDeform(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets);

	//compute Scale for Laplacian deformation
	static void ComputeScaleFactor(int iType,vector<Facet_handle>& ROIFacets,vector<vector<Facet_handle>> NeighborFacets,bool bTestIsoScale=false);

};
#endif
