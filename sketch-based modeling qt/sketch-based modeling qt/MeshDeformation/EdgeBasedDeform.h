#pragma once
#ifndef CEDGE_BASED_DEFORM_H
#define  CEDGE_BASED_DEFORM_H

#include "../GeometryAlgorithm.h"
#include "../Math/Math.h"

#define CONSTRAINED_HANDLE_WEIGHT 1


class CEdgeBasedDeform
{
public:
	CEdgeBasedDeform(void);
	~CEdgeBasedDeform(void);

	static void EdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints);

	static void IterativeEdgeBasedDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints);

	//get edges involved
	static void GetEdgeInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle>& ROIEdges);

protected:
	//get one more layer anchor
	static void GetOneMoreLayerAnchor(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<Vertex_handle>& OneMoreLayerAnchor);

	//get laplacian matrix
	static void GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,vector<vector<double> >& vecvecLaplacianMatrix);

	static void GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,SparseMatrix& LaplacianMatrix);

	//get the constraint matrix of anchor and handle vertices
	static void GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint,
		vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<vector<double> >& AnchorConstraintMatrix,
		vector<vector<double> >& HandleConstraintMatrix);

	static void GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint,
		vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix,
		SparseMatrix& HandleConstraintMatrix);

	//compute the right hand side of the naive laplacian equation
	static void ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//get the ordered neighbor halfedges&vertices for each handle+roi edge
	static void GetEdgeTopo(vector<Halfedge_handle> ROIEdges,
							vector<vector<Halfedge_handle>>& NeighborEdges);

	//set laplacian to each edge
	static void ComputeEdgeLaplacian(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges);

	//iterately update right hand side of laplacian(update the normal direction and laplacian magnitude)
	static void IterativeUpdateLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,
		vector<vector<double> >& LaplacianRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//backup edge vectors for computing ratation in rigid deformation
	static void BackUpEdgeVectorsForRigidDeform(vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges);

	//compute Rotation in rigid deformation
	static void ComputeRotationForRigidDeform(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges);

	//compute Scale for Laplacian deformation
	static void ComputeScaleFactor(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges);

	static void ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

};

#endif
