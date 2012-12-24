#pragma once
#ifndef CMATCONS_DEFORM_H
#define  CMATCONS_DEFORM_H

#include "../GeometryAlgorithm.h"
#include <QString>

class CMatConsDeform
{
public:
	CMatConsDeform(void);
	~CMatConsDeform(void);

	//set uniform materials for vertices
	static void SetUniformMaterial(double dUniMat,KW_Mesh& Mesh,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices);

	//set materials for vertices according to harmonic function
	static void SetHarmonicMaterial(int iType,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices);

	//set the material for vertices involved in the deformation
	static void SetMatColor(vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices);
	
	//set material for the whole mesh
	static void SetMatColor(KW_Mesh& Mesh);

	static void SetMatAndColor(vector<Vertex_handle>& vecVer,double dMaterial);

	//material-constrained vertex-based flexible deformation 
	static void MatConsVerFlexibleDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale);

	//material-constrained edge-based flexible deformation 
	static void MatConsEdgeFlexibleDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale);

	//learn material from template models
	static void LearnMatFromSrc(int iType,KW_Mesh& Mesh,vector<QString> vecTmpName);

	//export and import material
	static void ExportMat(KW_Mesh& Mesh,QString MatFileName);
	static void ImportMat(KW_Mesh& Mesh,QString MatFileName);


protected:

	//compute the right hand side of harmonic
	static void ComputeHarmonicRightHandSide(int iType,vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<vector<double> >& LaplacianRightHandSide,vector<vector<double> >& AnchorRightHandSide,
		vector<vector<double> >& HandleRightHandSide);

	static void ComputeMatConsVerFlexibleRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Point_3>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	static void SetEdgeMaterial(vector<Halfedge_handle>& ROIEdges);

	static void ComputeMatConsEdgeFlexibleRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
		vector<vector<Halfedge_handle>> NeighborEdges,
		vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//learn rotation using source model and its deformed shape
	static void LearnRotation(KW_Mesh& SrcMesh,KW_Mesh& DefModel);
	//learn scale using source model and its deformed shape
	static void LearnScale(KW_Mesh& SrcMesh,KW_Mesh& DefModel);
	//compute material from learning result
	static void ComputeMat(KW_Mesh& SrcMesh,KW_Mesh& DefModel,vector<double>& vecLearnedMat);
	

};
#endif