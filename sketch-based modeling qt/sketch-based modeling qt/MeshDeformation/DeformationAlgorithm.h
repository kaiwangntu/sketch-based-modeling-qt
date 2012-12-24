#pragma once
#ifndef CDEFORMATION_ALGORITHM_H
#define  CDEFORMATION_ALGORITHM_H

#include "../GeometryAlgorithm.h"
#include "../Math/Math.h"



#define  CONSTRAINED_HANDLE_WEIGHT 1

class CDeformationAlgorithm
{
	friend class CDualMeshDeform;
	friend class CRSRCellDeform;
	friend class CWedgeEdgeBasedDeform;
	friend class CMatConsDeform;

public:
	CDeformationAlgorithm(void);
	~CDeformationAlgorithm(void);

	//naive laplacian deformation without roration&scale considered
	static void NaiveLaplacianDeform(int iType,KW_Mesh& Mesh,vector<HandlePointStruct>& vecHandlePoint,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//rotated laplacian deformation with roration considered
	static void RotatedLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//rigid deformation 
	static void RigidDeform(int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//flexible deformation 
	static void FlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale);
	//flexible deformation, mesh vertices are selected as handle vertices directly
	static void FlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);


	//flexible deformation,the transformation matrix of Laplacian is RSR
	static void FlexibleRSRDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//interpolates the new handle positions,deform iteratively 
	static void IterativeFlexibleDeform(double dLamda,int iType,int iInterpoNum,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//iteratively solve the laplacian problem
	static void IterativeLaplacianDeform(int iType,int iIterNum,KW_Mesh& Mesh,vector<HandlePointStruct>& vecHandlePoint,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//linear interpolate between lambda==0 and lambda==1
	static void FlexibleLinearInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//interpolate Laplacian vector between lambda==0 and lambda==1
	static void FlexibleLaplacianInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	//interpolate Lambda between lambda==0 and lambda==1
	static void FlexibleLambdaInterpolation(int iInterpoNum,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

	static void BackUpMeshGeometry(KW_Mesh Mesh,vector<Point_3>& CurrentPos);
	static void RestoreMeshGeometry(KW_Mesh& Mesh,vector<Point_3> OldPos);

	static void BackUpMeshLaplacian(int iWeightType,vector<Vertex_handle>& AllVertices,vector<Vector_3>& CurrentLaplacian);
	static void RestoreMeshLaplacian(int iWeightType,vector<Vertex_handle>& AllVertices,vector<Vector_3> OldLaplacian);

	///////////////////////////////////////////////
	//flexible mesh deformation through propagating local transformations
	static void FlexibleTransPropDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale);

	///////////////////////////////////////////////
	//flexible deformation, using affine+svd to compute rotation
	static void FlexibleImpDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,bool bTestIsoScale);


	////////////////////////////////////////////////
	//flexible deformation, mesh vertices are selected as handle vertices directly
	static void TestFlexibleDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh,
		set<int>& setEdgeHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d);

protected:
	//get laplacian matrix which contains 3 parts:1-handle vertices 2-ROI 3-anchor vertices
	static void ComputeLaplacianMatrix(int iType,KW_Mesh& Mesh,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<vector<double> >& vecvecLaplacianMatrix);

	static void ComputeLaplacianMatrix(int iType,KW_Mesh& Mesh,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		SparseMatrix& LaplacianMatrix);

	//get the constraint matrix of anchor and handle vertices
	static void GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint,
		vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<vector<double> >& AnchorConstraintMatrix,
		vector<vector<double> >& HandleConstraintMatrix);

	static void GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint,
		vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix,
		SparseMatrix& HandleConstraintMatrix);

	//for no handle point case, only handle vertices exist
	static void GetConstraintsMatrixToNaiveLaplacian(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix,
		SparseMatrix& HandleConstraintMatrix);


	//get the constraint matrix of anchor and handle vertices, mesh vertices are selected as handle vertices directly
	static void GetConstraintsMatrixToNaiveLaplacian(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,vector<vector<double> >& AnchorConstraintMatrix,
		vector<vector<double> >& HandleConstraintMatrix);


	//compute the right hand side of the naive laplacian equation
	//It contains 1: the delta value of uniform laplacian of Handle&ROI,2:coordinates value of anchor
	//3: the deformed coordinates value of the handle vertices
	static void ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Vertex_handle> vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute the right hand side of the rotated laplacian equation
	//It contains 1: the delta value of uniform rigid of Handle&ROI,2:coordinates value of anchor
	//3: the deformed coordinates value of the handle vertices
	static void ComputeRotatedLaplacianRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//backup edge vectors for computing ratation in rigid deformation
	static void BackUpEdgeVectorsForRigidDeform(KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices);
	//compute Rotation in rigid deformation
	static void ComputeRotationForRigidDeform(int iType,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices);

	//compute Second Rotation in rigid deformation
	static void ComputeSecondRotationForRigidDeform(int iType,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices);


	//compute Scale for Laplacian deformation
	static void ComputeScaleFactor(int iType,KW_Mesh& Mesh,
		vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,
		vector<Vertex_handle>& vecAnchorVertices,bool bTestIsoScale=false);

	//compute the right hand side of the rigid equation
	//It contains 1: the delta value of rigid of Handle&ROI,2:coordinates value of anchor
	//3: the deformed coordinates value of the handle vertices
	static void ComputeRigidRightHandSide(int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute the right hand side of the flexible deform
	//It contains 1: the delta value of  lamda*rigid+(1-lamda)*RotatedLaplacian of Handle&ROI,
	//2:coordinates value of anchor 3: the deformed coordinates value of the handle vertices
	static void ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Point_3>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute the right hand side of the flexible RSR deform
	//It contains 1: the delta value of  lamda*rigid+(1-lamda)*RotatedLaplacian of Handle&ROI,
	//2:coordinates value of anchor 3: the deformed coordinates value of the handle vertices
	static void ComputeFlexibleRSRRightHandSide(double dLamda,int iType,vector<Vertex_handle>& vecHandleNb,
		vector<Vertex_handle>& ROIVertices,vector<Point_3>& vecAnchorVertices,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& RigidRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);


	static void GetInterpolationResult(vector<Point_3> vecDeformCurvePoint3d,
		vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNb);

	//iterately update right hand side of laplacian(update the normal direction and laplacian magnitude)
	static void IterativeUpdateLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb,
		vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
		vector<Point_3> vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	///////////////////////////////////////////////
	//related subfunctions for flexible deformation, using affine+svd to compute rotation
	
	static void FlexibleImpFirstStep(int iType,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL,
		vector<Vertex_handle>& vecAnchorVertices,vector<Point_3>& vecDeformCurvePoint3d,SparseMatrix& LaplacianMatrix,
		SparseMatrix& AnchorConstraintMatrix,SparseMatrix& HandleConstraintMatrix,vector<vector<double> >& AnchorRightHandSide,
		vector<vector<double> >& HandleRightHandSide,vector<GeneralMatrix>& vecCoeffMat);
	
	static void FlexibleImpSecondStep(int iType,double dLambda,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL,
		SparseMatrix& LaplacianMatrix,SparseMatrix& AnchorConstraintMatrix,SparseMatrix& HandleConstraintMatrix,vector<vector<double>>& AnchorRightHandSide,
		vector<vector<double>>& HandleRightHandSide,vector<GeneralMatrix>& vecCoeffMat);

	//compute the matrix for converting coordinates to vectors at each vertex
	static void GetCo2VecMatrix(vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL,vector<GeneralMatrix>& vecCo2VecMat);
	//compute the vector matrix for handle+roi
	static void GetVecMatrix(vector<Vertex_handle>& vecHandleROI,vector<GeneralMatrix>& vecVectorMat);
	//test if vector matrix is right
	static void VerifyVecMatrix(vector<GeneralMatrix>& vecCo2VecMat,vector<Vertex_handle>& vecALL,vector<GeneralMatrix>& vecVectorMat);
	//do svd of vector matrix and get coefficient matrix
	static void FactorizeVecMatrix(vector<GeneralMatrix>& vecVectorMat,vector<GeneralMatrix>& vecCoeffMat);
	//get second part of system matrix
	static void GetSubSysMat(int iType,vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecALL,
		vector<GeneralMatrix>& vecCoeffMat,SparseMatrix& SubSysMat);
	//compute righthand side of first system
	static void GetFirstRHS(vector<Vertex_handle>& vecHandleROI,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& vecDeformCurvePoint3d,vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide);

	//compute the deformation gradient of the second step
	static void GetDGMat(vector<GeneralMatrix>& vecCoeffMat,vector<GeneralMatrix>& vecVectorMat,vector<GeneralMatrix>& vecDGMat);
	//extract rotation and uniform scale from deformation gradient
	static void ExtractRotScal(vector<GeneralMatrix>& vecDGMat,vector<Vertex_handle>& vecHandleROI);
	//compute righthand side of second system
	static void GetSecondRHS(int iType,double dLamda,vector<Vertex_handle>& vecHandleROI,vector<vector<double> >& LaplacianRightHandSide);




	///////////////////////////////////////////////////////////////////////////////////
	static void TESTGetConstraintsMatrixToNaiveLaplacian(set<int> setHandleIndex,vector<Vertex_handle> ROIVertices,
		vector<Vertex_handle> vecAnchorVertices,SparseMatrix& AnchorConstraintMatrix,
		SparseMatrix& HandleConstraintMatrix);



};

#endif
