#pragma once

#ifndef CCURVEDEFORM_H
#define CCURVEDEFORM_H

#include "CGALDef.h"
#include "Math/Math.h"


class CCurveDeform
{
public:
	CCurveDeform(void);
	~CCurveDeform(void);

	//naive laplacian deformation without roration&scale considered
	//vecCurvePoint:closed curve  to deform
	//vecHandleIndex: indices of handle points
	//vecDeformCurvePoint: new positions of handle points
	//vecROIRange: ROI points number next to each handle point
	//plane type 0: parallel to xoy 1: parallel to xoz 2: parallel to yoz 3:non-planar
	static void OpenCurveNaiveLaplacianDeform(vector<Point_3>& vecCurvePoint,
		vector<int> vecHandleIndex,vector<Point_3> vecDeformCurvePoint,int iROIRange,
		int iPlaneType=3);

	static void CCurveDeform::ClosedCurveNaiveLaplacianDeform(vector<Point_3>& vecCurvePoint, vector<int> vecHandleIndex,
		vector<Point_3> vecDeformCurvePoint,int iPlaneType/* =3 */);


protected:

	static void ComputeLaplacianMatrix(int iTotalPointsNum,SparseMatrix& LaplacianMatrix);
	//matrix order:left anchor+left roi+handle+right roi+right anchor
	static void ComputeOpenLaplacianMatrix(int iROIRange,SparseMatrix& LaplacianMatrix);


	static void GetConstraintsMatrix(int iTotalPointsNum,
		vector<int> vecHandleIndex,SparseMatrix& HandleConstraintMatrix);
	static void GetOpenConstraintsMatrix(int iTotalPointsNum,SparseMatrix& HandleConstraintMatrix);

	static void ComputeNaiveLaplacianRightHandSide(vector<Point_3> vecCurvePoint,
		vector<int> vecHandleIndex,vector<Point_3> vecDeformCurvePoint,int iPlaneType,
		vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& HandleRightHandSide);
	static void ComputeOpenNaiveLaplacianRightHandSide(vector<Point_3> vecCurvePoint,
		int iHandleIndex,Point_3 DeformCurvePoint,
		vector<int> vecROIIndex,vector<int> vecAnchorIndex,
		vector<int> vecTotalIndex,int iPlaneType,
		vector<vector<double> >& LaplacianRightHandSide,
		vector<vector<double> >& HandleRightHandSide);

};

#endif
