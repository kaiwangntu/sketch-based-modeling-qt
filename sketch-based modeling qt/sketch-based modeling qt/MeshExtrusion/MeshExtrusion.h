#pragma once
//#include "../OBJHandle.h"
//#include "../PaintingOnMesh.h"
//#include "../GeometryAlgorithm.h"
#include "Tunnelling.h"
#include <Qpoint>

class SketchDoc;


class CMeshExtrusion
{
public:
	CMeshExtrusion(void);
	~CMeshExtrusion(void);

	void Init(SketchDoc* pDataIn);

	//input one point each time
	void InputCurvePoint2D(QPoint Point2D);
	//input the whole curve, used for getting input from "Editing Mode"
	void SetCurvePoint2D(vector<QPoint> Input) {this->CurvePoint2D=Input;}

	void SetDrawingCurveType(int iType);
	int GetDrawingCurveType();

	void Conver2DCurveTo3D(KW_Mesh& Mesh);

	//set selected plane/curve
	void SetSelectedItem();
	//manipulate selected plane (rotate)/curve(translate)
	void ManipSelItem(int g_iStepX, int g_iStepY);

	bool ExtrudeMesh(KW_Mesh& Mesh,vector<vector<Point_3> >& testvecvecNewEdgeVertexPos);

	void GetExtrudeSilhPlane(Plane_3& BestFittingPlane,
		Point3D* PlaneBoundaryPoints,GLdouble* modelview,GLdouble* projection,GLint* viewport);

	//the rotate axis is the diameter of the closed curve projection on its best-fitting plane
	//(i.e. parallel to the closed curve)
	void RotateExtrudeSilhPlaneX();

	//the rotate axis is the segment passing through the closed curve projection on its best-fitting plane
	//and its centroid projection(i.e. perpendicular to the closed curve)
	void RotateExtrudeSilhPlaneY();

	void AdjustPlaneBoundary(int iIncrease);

	void Render(bool bSmoothView,GLenum mode);

private:
	SketchDoc* pDoc;

	vector<QPoint> CurvePoint2D;//user drawn 2D curve(screen coordinate)
	int iDrawingCurveType;//0 for default,1 for closed curve,2 for silhouette curve
	Plane_3 BestFittingPlane;//best fitting plane for deformation
	Point3D PlaneBoundaryPoints[4];
	double Plane_spin;

	//dig a hole on the mesh
	CTunnelling Tunel;

	vector<Point_3> vecTestPoints;
	void RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport);
	void RenderClosedCurve3D();
	void RenderSilhCurve3D();
	void RenderRefPlane(bool bSmoothView,GLenum mode);
	void RenderTestPoints();

	vector<Vertex_handle> hExtrusionClosedCurveVertex3d;
	vector<Point_3> ExtrusionSilhPoints;//3d points indicate the silhouette of extrusion curve

	Point_3 RotateXAxisStartPoint,RotateXAxisEndPoint;
	Point_3 RotateYAxisStartPoint,RotateYAxisEndPoint;
	vector<Point_3> ClosedCurveProj;//projection points of the closed curve on its best-fitting plane
	double dAccumulatedAngleX;//angle,not radian
	vector<vector<Vertex_handle> > vecvecExtrudedVertex;

	//get halfedge handle connecting handle vertices
	int GetClosedStrokeHH(vector<Halfedge_handle>& hhClosedCurve);
	
	//delete facet inside the closed curve
	int DeleteFacetsInClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle> hhClosedCurve);

	//return: number of such kind of triangles
	int GetFacetsWithBorderEdge(KW_Mesh& Mesh,vector<Facet_handle>& fhWithBorderEdge,
								vector<Facet_handle> ExemptionTri);

	int ExtrudeClosedCurve(KW_Mesh& Mesh,vector<Halfedge_handle>& hhClosedCurve,
		Plane_3 BestFittingPlane,vector<vector<Point_3> >& testvecvecNewEdgeVertexPos,
		vector<Facet_handle>& fhExtrudedFacet);

	//extrude a new layer
	int ExtrudeNewLayer(KW_Mesh& Mesh,vector<Point_3> NewEdgeVertexPos,
						vector<Halfedge_handle>& hhNewToCenter);//new Halfedge_handle point to new center

	int GetExtrusionPointsPos(Plane_3 BestFittingPlane,vector<vector<Point_3> >& vecvecNewEdgeVertexPos);

	//get extruded vertices to allow further edit
	int GetExtrudedVertices(vector<vector<Vertex_handle> >& OutvecvecExtrudedVertex);

	//get vertices to smooth after extrusion
	int GetVerticesToSmooth(vector<Vertex_handle>& vecVertexToSmooth);
};
