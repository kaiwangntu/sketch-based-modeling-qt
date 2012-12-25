#pragma once
//#include "../OBJHandle.h"
//#include "../PaintingOnMesh.h"
#include "../GeometryAlgorithm.h"
#include <Qpoint>

class SketchDoc;

class CMeshCutting
{
public:
	CMeshCutting(void);
	~CMeshCutting(void);

	void Init(SketchDoc* pDataIn);

	//input one point each time
	void InputCurvePoint2D(QPoint Point2D);
	//input the whole curve, used for getting input from "Editing Mode"
	void SetCurvePoint2D(vector<QPoint> Input) {this->CurvePoint2D=Input;}

	void SetDrawingCurveType(int iType);
	int GetDrawingCurveType();

	void Conver2DCurveTo3D(KW_Mesh& Mesh);

	vector<Vertex_handle> GetCuttingClosedCurveVertex3d();
	void SetCuttingClosedCurveVertex3d(vector<Vertex_handle> hCuttingClosedCurveVertex3dIn);

	bool CutMesh(KW_Mesh& Mesh);

	//void SetPlaneSpin(double dStep);
	//double GetPlaneSpin();
	//void RotateTunnelingPlaneX();
	//void AdjustPlaneBoundary(int iIncrease);


	void Render(bool bSmoothView,GLenum mode);


private:

	SketchDoc* pDoc;

	vector<QPoint> CurvePoint2D;//user drawn 2D curve(screen coordinate)
	int iDrawingCurveType;//0 for default,1 for cutting curve,2 for selecting curve(which part to cut)

	Plane_3 BestFittingPlane;//best fitting plane for tunneling
	Point3D PlaneBoundaryPoints[4];
	double Plane_spin;
	Point_3 RotateXAxisStartPoint,RotateXAxisEndPoint;
	double dAccumulatedAngleX;//angle,not radian

	Point_3 SidePoint;//the mesh part which is at the same side of the SidePlane is to be cut off
	Plane_3 SidePlane;//plane for judging which part to cut

	vector<Vertex_handle>	hCuttingClosedCurveVertex3d;

	//get halfedge handle connecting handle vertices
	int GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurve);

	//delete facet on one side of the closed curve
	int DeleteFacets(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve);

	//return: number of such kind of triangles
	int GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,
		vector<Facet_handle> ExemptionTri);

	//fill the hole incident to hhClosedCurve
	int FillHole(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve);

	//dig a hole on the mesh
	//CTunnelling Tunel;


	//test points
	vector<Point_3> vecTestPoint;

	void RenderTestPoint();
	void RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport);
	void RenderCuttingCurve3D();
	void RenderTunelCurve3D();
	void RenderTunnelDirectCurve();
	void RenderRefPlane(bool bSmoothView);

};
