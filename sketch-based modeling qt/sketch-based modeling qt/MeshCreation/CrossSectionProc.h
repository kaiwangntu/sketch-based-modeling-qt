#pragma once
#ifndef CROSS_SECTION_PROC_H
#define CROSS_SECTION_PROC_H

//#include "../PreDef.h"
#include "MeshCreation_Struct_Def.h"

class CCrossSectionProc
{
public:
	CCrossSectionProc(void);
	~CCrossSectionProc(void);

	//set the orientations of the curve network on one plane
	//outermost: ccw,then, cw,...
	static void SetCNOrientation(CurveNetwork& CnToSet);

	//get the intersection points between the curves on each plane
	//and other orthogonal planes other curves lie on
	//static void GetCNIntersectPoints(vector<CurveNetwork>& vecCurveNetwork);

	//get the intersection points between curves
	//and current reference planes
	static void GetCurvePlaneIntersectPoints(vector<CurveNetwork> vecCurveNetwork,Plane_3 RefPlane[3],
		vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//copy curve network from the last parallel plane
	static void CopyCNFromLastParaPlane(int iPlaneType,Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork,
		vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//intersect with other curves on the plane or not
	static bool CheckIntersectOthers(CurveNetwork CnToCheck,Polygon_2 NewProfile2D,int iExclProfileInd=-1);

	//given the curve index in the selection mode, get the curve network info of the selected curve
	static bool GetCNFromSelectIndex(int iSelInd,vector<CurveNetwork> vecCurveNetwork,int& iWhichCN,int& iWhichCS);

	//delete the selected cross section (sketched)
	static void DeleteSpecifiedCS(Plane_3 RefPlane[3],int& iWhichCN,int& iWhichCS,vector<CurveNetwork>& vecCurveNetwork, 
		vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//delete the last cross section (sketched or computed)
	static void DeleteLastCS(Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
		vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//compute the intersection points for the specified curve&other orthogonal plane
	//and curves on other planes&the plane the specified curve lie on, deform the curves
	static bool FitSpecifiedCN(int iSpecifiedCN,Plane_3 RefPlane[3],vector<CurveNetwork>& vecCurveNetwork, 
		vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//for the x/y/z coordinates of a cn to be exactly the same with the plane parameter
	static void ForceCNCoordinates(CurveNetwork& CnToSet);

	//add the computed curve network (cross sections) to the overall curve network
	static void AddComputedCNToTotalCN(vector<vector<Point_3>> vecComputedCS,vector<CurveNetwork>& vecCurveNetwork,
		Plane_3 RefPlane[3],int iPlaneType,vector<Point_3>& vecCurvePlaneIntersectPoint,vector<int>& vecCurvePlaneIntersectType);

	//functions related to local modify a computed CN/selected CN containing partial CS
	//judge which CS to modify,return the index of CS in CN
	static int GetCSToModify(vector<vector<Point_3>> vecCS,vector<Point_3> InputCurve);
	static void ReverseModifyPointsID(int iCSLen,vector<int>& vecModifiedPointInd);
	//update the PartProfile3D field
	static void SavePartialCSInfo(CurveNetwork& CnToSave,int iCSInd,vector<int> PartialPointInd);

	//read cross section data directly from file to vector<CurveNetwork>
	//nothing saved in Ctr2SufManager
	static bool ImportCrossSections(const char* fnames,vector<CurveNetwork>& vecCurveNetwork);
	//save cross section data directly from vector<CurveNetwork> to file
	//nothing saved in Ctr2SufManager
	static bool ExportCrossSections(const char* fnames,vector<CurveNetwork> vecCurveNetwork);

};

#endif
