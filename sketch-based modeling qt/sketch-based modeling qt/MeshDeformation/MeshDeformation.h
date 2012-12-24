#pragma once
#include "../GeometryAlgorithm.h"
#include <QPoint>
class SketchDoc;

#define DEFAULT_REFSPHERE_RADIUS 0.3

class CMeshDeformation
{
public:
	CMeshDeformation(void);
	~CMeshDeformation(void);

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


	vector<Vertex_handle> GetHandleNbVertex();
	vector<Point_3> GetDeformCurvePoints();
	void SetDeformCurvePoints(vector<Point_3> vecDeformCurvePoint3dIn);

	bool GetHandleStrokeType();
	void SetHandleStrokeType(bool bStrokeType);

//	map<int,vector<int>> GetDesiredNeighborVertex();
	vector<Vertex_handle> GetROIVertices();

	//provide for 2D Edit
	int GetCurrentDesiredPointsPos(vector<HPCPoint> & DesiredPointsPosition);//return num of DesiredPoints
	void SetModifiedPointsPos(KW_Mesh& Mesh,vector<HPCPoint> DesiredPointsNewPosition);
	//provide for 3D Edit directly
	bool SetModifiedPointsPos(KW_Mesh& Mesh,vector<Point_3>& testpoints);

	void SetSelectionRange(double iRadius,KW_Mesh& Mesh);
	double GetSelectionRange();

	//find roi by painting strokes
	void PaintROIVertices(KW_Mesh& Mesh,GLdouble* modelview,GLdouble* projection,GLint* viewport);
	
	void SetFlexibleDeformPara(int iIter,double dLambda);
	void GetFlexibleDeformPara(int& iIter,double& dLambda);

	double GetMaterial() {return this->dMaterial;}
	void SetMaterial(double dDataIn) {this->dMaterial=dDataIn;}

	//RefPoints help to make the BFplane perpendicular to the screen
	void GetDeformationPlane(vector<Point_3> RefPoints,Plane_3& BestFittingPlane,Point3D * PlaneBoundaryPoints);
	void RotateDeformationPlane();
	void TranslateDeformCurvePoint3dOnBFPlane(double dTransX,double dTransY);

	void AdjustPlaneBoundary(int iIncrease);

	//calculate dual mesh
	void ComputeDualMesh(KW_Mesh& PrimalMesh);

	//calculate edge mesh
	void ComputeEdgeMesh(KW_Mesh& PrimalMesh);

	//linear interpolation for deformation
	void DeformInterpolation(KW_Mesh& Mesh);

	//auto set uniform material
	void SetAutoUniMat();
	//auto compute material according to harmonic
	void SetAutoHarMat();
	//learn material from template models
	void LearnMat();
	//export material to file
	void ExportMat();
	//import material from file
	void ImportMat();

	//local refinement related functions
	//local refine after deformation
	void DeformationLocalRefine(KW_Mesh OldMesh,KW_Mesh& NewMesh,vector<Point_3> OldHandlePos,vector<Point_3> NewHandlePos,
		double dSquaredDistanceThreshold,vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices,
		vector<Point_3>& testCentroidPoint,vector<Point_3>& testmovedCentroidPoint,vector<Facet_handle>& testfhRefineTri);
	//mark all vertices: 0: irrelevant vertices; 1: handle; 2: roi; 3:static; 4: new roi; 5: new static
	void SetVerMark(KW_Mesh& NewMesh,vector<Vertex_handle>& vecHandleNb,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices);
	//since more vertices are added, the roi and static vertices need to be adjusted
	void ResetRoiStaticVer(KW_Mesh& NewMesh,vector<Vertex_handle>& ROIVertices,vector<Vertex_handle>& vecAnchorVertices);
	//find triangles needed to be locally refined because of centroid movement
	int GetRefineTri(KW_Mesh OldMesh,KW_Mesh& NewMesh,vector<Point_3> OldHandlePos,vector<Point_3> NewHandlePos,
		double dSquaredDistanceThreshold,vector<Point_3>& testCentroidPoint,vector<Point_3>& testmovedCentroidPoint,vector<Facet_handle>& fhRefineTri);
	//update vertex positions after dividing
	void LocalRefineUpdateVertexPos(vector<Vertex_handle> vecNewEdgeVertex,vector<Vertex_handle> vecOriVertex);


	//render this plane or not
	//plane0: normal plane1:tangential plane2:binormal
	bool bRenderRefPlane[3];
	//render the sphere or not
	bool bRenderSphere;

	//render handle+roi+static vertices or not
	bool bRenderHandleNb;
	bool bRenderROI;
	bool bRenderAnchor;


	void Render(bool bSmoothView,GLenum mode,bool bShowDualMesh);

private:

	SketchDoc* pDoc;

	vector<QPoint> CurvePoint2D;//user drawn 2D curve(screen coordinate)
	int iDrawingCurveType;//0 for default,1 for selecting handle curve,2 for drawing deformed handle curve
	//3 for drawing projection(shadow) of deformed handle curve
	Plane_3 BestFittingPlane;//best fitting plane for deformation
	Plane_3 RefTangentialPlane;//orthogonal plane for drawing projections(shadows)
	Plane_3 RefBiNormalPlane;//binormal plane
	Point3D PlaneBoundaryPoints[4];
	Point3D RefTangentialPlaneBoundaryPoints[4];
	Point3D RefBiNormalPlaneBoundaryPoints[4];
	double Plane_spin;

	//reference sphere for a single handle vertex
	Sphere_3 RefSphere;

	double dFlexibleDeformLambda;
	int iFlexibleDeformIterNum;

	//material for material-constrained deformations
	double dMaterial;

//	vector<DualMeshVertexStruct> DualMesh;
	KW_Mesh DualMesh;
	vector<Vertex_handle> vecDualHandle,vecDualROI,vecDualAnchor;

	map<Point_3,vector<Point_3>> DualMeshTest;

	void RenderCurvePoint2D(GLdouble* modelview,GLdouble* projection,GLint* viewport);
	void RenderHandleCurve(GLenum mode);
	void RenderResultHandleCurve();
	void RenderDeformCurve(GLenum mode);
	void RenderDeformCurveProj();
	void RenderRefPlane(bool bSmoothView,GLenum mode);
	void RenderRefTangentialPlane(bool bSmoothView,GLenum mode);
	void RenderRefBiNormalPlane(bool bSmoothView);
	void RenderRefSphere(bool bSmoothView);
	void RenderHandleNb();
	void RenderROI();
	void RenderAnchor();
	void RenderDualMesh();

	vector<Point_3> vecTestPoint;
	void RenderTestPoint();

	KW_Mesh WedgeEdgeMesh;
	map<Point_3,vector<Point_3>> EdgeMesh;
	void RenderEdgeMesh();


	vector<HandlePointStruct> vecHandlePoint;
	vector<Vertex_handle> vecHandleNbVertex;
	
	vector<Point_3> vecDeformCurvePoint3d;
	vector<Point_3> vecDeformCurveProjPoint3d;//projection of deformed curve
	bool bHandleStrokeType;//whether to deform an open handle curve(true) or closed(false)

	//combine deform curve and its shadow
	void CombineDeformCurveAndProj();

	//1 if the distace between the neighbor vertex and FDTVInCurrentTriangle
	//is below this distance,it should be moved according to the FDTVs
	//2 if the distance between the neighbor and its FDTVInCurrentTriangle
	//is below the threshold,the weight equals to 0
	double dSquaredDistanceThreshold;

	//find the neibor vertices of the FinalDesiredTriangleVertex
	//then to move them accordingly
//	map<int,vector<int>> DesiredNeighborVertex;
	vector<Vertex_handle> ROIVertices;
	vector<Vertex_handle> AnchorVertices;

	//find roi according to the distance
	void FindROIVertices(KW_Mesh& Mesh);

	//find roi by drawing a closed stroke
	void CircleROIVertices(KW_Mesh& Mesh,vector<QPoint> vecBoundingCurve,
		GLdouble* modelview,GLdouble* projection,GLint* viewport);

	//get anchor vertices
	int GetAnchorVertices();

	void ReOrderDeformCurve();

	//void RBFGetCoefficient(vector<Point_3> OldHandlePos,vector<Point_3> NewHandlePos,vector<double>* Ci,
	//					double& Si);
	//double RBFGetMultiQuadricSi(vector<Point_3> OldHandlePos);

	////after the desired point is moved,its neighbors also should be moved.
	//void RBFGetNeighborNewPos(KW_Mesh Mesh,map<int,Point_3>  NewDesiredPointPos,vector<double>* Ci,double Si,
	//						vector<Point_3> OldHandlePos,vector<Point_3> &NeighborNewPos);

	//void CheckRBF(KW_Mesh Mesh,vector<double>* Ci,double Si,
	//	vector<Point_3> OldHandlePos,vector<Point_3>& NewHandlePos);

};
