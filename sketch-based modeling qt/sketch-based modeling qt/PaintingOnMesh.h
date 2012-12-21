#pragma once

#include "GeometryAlgorithm.h"


class CPaintingOnMesh
{
public:
	CPaintingOnMesh(void);
	~CPaintingOnMesh(void);

	//return: HandleCurvePoint3d.size() 
	int PaintingOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
		GLdouble* modelview,vector<Vertex_handle>& hCurveVertex3d);

	//return: HandleCurvePoint3d.size() 
	int PaintingOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
		GLdouble* modelview,vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNbVertex);

	//used for extrusion
	//return: HandleCurvePoint3d.size()
	int PaintingClosedStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
		GLdouble* modelview,vector<Vertex_handle>& hCurveVertex3d,bool bRemesh=true);

	//"painting" the curve on frontal mesh onto the rear
	//i.e. extrude the curve on frontal mesh along the given direction 
	int PaintingFrontalCurveOnRearMesh(KW_Mesh& Mesh,vector<Vertex_handle> hFrontalCurveVertex3d,Vector_3 GivenDirection,
		vector<Vertex_handle>& hRearCurveVertex3d,vector<Point_3>& testpoint,bool bRemesh=true);

	int PaintingClosedStrokeOnFrontalAndRearMesh(KW_Mesh& Mesh,vector<Point_3> UserCurvePoint,
		GLdouble* modelview,vector<Vertex_handle>& hCurveVertex3d);

	int PaintingOnBFPlane(Plane_3 BestFittingPlane,GLdouble* modelview,GLdouble* projection,GLint* viewport,
		const vector<QPoint> DeformCurvePoint,vector<Point_3>& DeformCurvePoint3d);

	int PaintingPointOnSphere(Sphere_3 sphere,GLdouble* modelview,GLdouble* projection,GLint* viewport,
		QPoint InputPoint,vector<Point_3>& IntersectPoints);

	bool PaintingPointOnFrontalMesh(KW_Mesh& Mesh,Point_3& UserPoint,Facet_handle& UserPointFacet,GLdouble* modelview);
	bool PaintingScrPointOnFrontalMesh(KW_Mesh& Mesh,QPoint UserScrPoint,Point_3& UserPoint,Facet_handle& UserPointFacet,
		GLdouble* modelview,GLdouble* projection,GLint* viewport);

	//circle to select the vertices on the front of the mesh
	int CircleFrontVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve,
		GLdouble* modelview,GLdouble* projection,GLint* viewport,
		vector<Vertex_handle>& vecCirVer);

	//circle to select the vertices on both the front and back of the mesh
	int CircleAllVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve,
		GLdouble* modelview,GLdouble* projection,GLint* viewport,
		vector<Vertex_handle>& vecCirVer);

private:
	//move the point on face to the middle of its two adjacent points on edge
	int SmoothHandleCurvePoint3d(vector<Point_3>& HandleCurvePoint3d,bool bAverage=false);

	vector<Halfedge_handle> hhPrevs;

	//make each handle point a linear combination of the mesh vertices 
	int GetLinearCombineInfo(vector<HandlePointStruct>& vecHandlePoint,vector<Vertex_handle>& vecHandleNbVertex);

	//remesh after painting an open stroke on frontal mesh    return:hCurveVertex3d.size()
	//bool bFacetVertexInclude=true: new vertices include both points on facets and edges
	//bool bFacetVertexInclude=false: new vertices include only points on edges
	int RemeshOpenStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3>& HandleCurvePoint3d,
		vector<Vertex_handle>& hCurveVertex3d,bool bFacetVertexInclude=true);

	//remesh after painting an closed stroke on frontal mesh    return:hCurveVertex3d.size()
	//bool bFacetVertexInclude=true: new vertices include both points on facets and edges
	//bool bFacetVertexInclude=false: new vertices include only points on edges
	int RemeshClosedStrokeOnFrontalMesh(KW_Mesh& Mesh,vector<Point_3>& HandleCurvePoint3d,
		vector<Vertex_handle>& hCurveVertex3d,bool bFacetVertexInclude=true);

	//in case that two points in the same triangle,delete the latter one
	int DeleteExtraPointsOnFacet(vector<Point_3>& HandleCurvePoint3d,bool bStrokeClosed);

	void CheckMesh(KW_Mesh Mesh);

	//for debug
	void CheckLinearCombineInfo(vector<HandlePointStruct> vecHandlePoint,vector<Vertex_handle> vecHandleNbVertex);

};
