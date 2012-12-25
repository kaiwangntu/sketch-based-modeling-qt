#pragma once
#ifndef CTUNNELING_H
#define CTUNNELING_H

#include "../GeometryAlgorithm.h"

class CTunnelling
{
public:
	CTunnelling(void);
	~CTunnelling(void);

	//clear the tunneling circles
	void clear();

	//get the tunneling circles
	vector<vector<Vertex_handle>> GetTunVer();

	//extract skeleton from ExtrusionSilhPoints
	//ExtrusionSilhPoints: extrusion silhouette curve; iSampleNum: number of desired result points
	//SkeletonCurve: output skeleton points,return size of skeleton curve
	int ExtractSkelFromSilhCurve(vector<Point_3> ExtrusionSilhPoints,int iSampleNum,vector<Point_3>& SkeletonCurve);

	//get the tunnel direction curve
	vector<Point_3> GetTunnelDirectCurve();
	//set the tunnel direction curve
	void SetTunnelDirectCurve(vector<Point_3> vecInput);

	//input the tunneling circles
	void InputTunVer(vector<Vertex_handle> vecTunVertex);

	//calculate the reference plane, its rotation axis and its boundary
	void SetRefPlane(Plane_3& BestFittingPlane,Point3D* PlaneBoundaryPoints,Point_3& RotateXAxisStartPoint,
		Point_3& RotateXAxisEndPoint);
	
	//correspond the two tunnel circles
	void CorrespondTunCircles();
	//dig the hole
	void Tunnel(KW_Mesh& Mesh,vector<Point_3>& vecTestPoint);

private:
	//two circles of tunneling vertices
	vector<vector<Vertex_handle>>  vecvecTunVertex;
	//the curve indicating the direction of the tunnel
	vector<Point_3> TunnelDirectCurve;

	Point_3 CenterA,CenterB;//the center of the two tunnel circles

	//compute the shape of the intermediate circle(how many point and the base position of each point)
	//the agent curves are then calculated by translating the circles
	//arclengthA,B,C: piece-wise arclength of tunnel cirve A,B, and agent curve C
	void ComputeAgentCurveShape(vector<Point_3>& vecAgentPoint,Point_3& CenterC,vector<double>& arclengthA,vector<double>& arclengthB,
		vector<double>& arclengthC);

	void ComputeAgentCurves(vector<Point_3> vecAgentPoint,Point_3 CenterC,vector<Point_3> TunnelDirectCurve,
		vector<vector<Point_3>>& vecvecAgentCurveA,vector<vector<Point_3>>& vecvecAgentCurveB,
		vector<Point_3>& vecMiddleAgentCurve);

	//get the two set of halfedges along two tunneling curves
	void GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurveA,vector<Halfedge_handle>& hhClosedCurveB);
	
	//delete the facets inside the closed curve, bFlip: flip the halfedges or not
	void DeleteFacetsInClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve,bool bFlip=false);
	int GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,vector<Facet_handle> ExemptionTri);
	void TempFillHole(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve,Point_3 Center,bool bFlip=false);

	//connect tunnel curve A and agent C,hFirstCSeg: first border halfedge segment pointing from C[0] to C[1]
	void ConnectTunAAgent(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurveA,vector<Point_3> vecAgentPoint,
		vector<double>& arclengthA,vector<double>& arclengthC,Halfedge_handle& hFirstCSegA,vector<Point_3>& vecTestPoint);
	//connect tunnel curve B and agent C,hFirstCSeg: first halfedge segment pointing from C[0] to C[1]
	void ConnectTunBAgent(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurveB,vector<Point_3> vecAgentPoint,
		vector<double>& arclengthB,vector<double>& arclengthC,Halfedge_handle& hFirstCSegB,vector<Point_3>& vecTestPoint);

	//expand the half tunnel A to the middle agent curve
	void ExpandHalfTunA(KW_Mesh& Mesh,vector<vector<Point_3>>& vecvecAgentCurveA,Halfedge_handle& hFirstCSegA,
		vector<Point_3>& vecTestPoint);
	//expand the half tunnel B to the middle agent curve
	void ExpandHalfTunB(KW_Mesh& Mesh,vector<vector<Point_3>>& vecvecAgentCurveB,Halfedge_handle& hFirstCSegB,
		vector<Point_3>& vecTestPoint);

};

#endif