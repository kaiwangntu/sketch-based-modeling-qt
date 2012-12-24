#pragma once

#include "../GeometryAlgorithm.h"


class CRSRCellDeform
{
public:
	CRSRCellDeform(void);
	~CRSRCellDeform(void);

	//flexible deformation,the transformation matrix of Laplacian is RSR
	static void RSRCellDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);


protected:

	//compute the right hand side of the RSR deform
	static void ComputeRSRRightHandSide(double dLamda,int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Point_3>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute Scale matrix,be careful,here all the vertices should be involved!
	static void ComputeScaleFactor(int iType,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices);

	//compute Rotation in rigid deformation
	static void ComputeRSRMatrixForDeform(KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices);

};
