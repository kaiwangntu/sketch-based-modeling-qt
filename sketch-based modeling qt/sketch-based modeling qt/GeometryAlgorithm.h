#pragma once

#include "PreDef.h"
#include "CGALDef.h"
#include "CarveCSGDef.h"
#include "Math/Tree.h"
#include <qpoint.h>




//type define
typedef std::pair <int,int> Int_Int_Pair;

//faster than CGAL,so preserved
typedef struct _Point3D
{
	double x;
	double y;
	double z;
}Point3D,*pPoint3D;
//high precision CPoint
typedef struct _HPCPoint
{
	double x;
	double y;
}HPCPoint,*pHPCPoint;

typedef struct _UserCurveStruct 
{
	Point_3 UCP;//User Curve Point on Znear
	Point_3 IP;//intersection point between ray and mesh triangles
//	int iTri;//index of the mesh triangle IP lies on
	Facet_handle fTri;//Facet of the mesh triangle IP lies on
	int iGroup;//group num of UCP
	int iReserve;//reserved
}UserCurveStruct,*pUserCurveStruct;

//structure for mesh deformation
typedef struct _HandlePointStruct
{
	Point_3 PointPos;
	//indices of three vertices of the facet which handle point lies on
	//the vertices are stored in vector<Vertex_Handle> vecHandleNbVertex;
	std::vector<int> vecVertexIndex;
	//Para[0] for iVertexIndex[0],Para[1] for iVertexIndex[1],(1-Para[0]-Para[1]) for iVertexIndex[2]
	std::vector<double> vecPara;
}HandlePointStruct,*pHandlePointStruct;

//structure for mesh deformation
typedef struct _DualMeshVertexStruct
{
	Point_3 PointPos;
	//index of primal facet it lies on
	Facet_handle PrimalFacet;
	//indices of vertices of the primal facet which dual mesh point lies on
	//the vertices are stored in vector<Vertex_Handle> vecHandleNbVertex+ROI+Anchor;
	std::vector<int> vecPrimalVertexIndex;
	//indices of neighbor dual mesh points
	std::vector<int> vecDualNeighborIndex;
	//weight for each Dual neighbor
	std::vector<double> vecDualNeighborWeight;
	//sum of above
	double dSumWeight;
	//type: 1:handle,2:ROI,3:anchor
	int iType;
}DualMeshVertexStruct,*pDualMeshVertexStruct;

//Build a dual mesh 
template <class HDS>
class Build_DualMesh : public CGAL::Modifier_base<HDS> {
public:
	Build_DualMesh(KW_Mesh& MeshIn) {PrimalMesh=MeshIn;}
	void operator()( HDS& hds) {
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
		B.begin_surface(PrimalMesh.size_of_facets(),PrimalMesh.size_of_vertices());
		//input dual mesh vertices(each w.r.t a facet on primal mesh)
		int iIndex=0;
		for ( Facet_iterator i=PrimalMesh.facets_begin();i!=PrimalMesh.facets_end();i++)
		{
			vector<Point_3> VertexPosOnFacet;
			Halfedge_around_facet_circulator Hafc = i->facet_begin();
			do 
			{
				VertexPosOnFacet.push_back(Hafc->vertex()->point());
			} while(++Hafc != i->facet_begin());
			Point_3 CentroidPoint=CGAL::centroid(VertexPosOnFacet.begin(),VertexPosOnFacet.end());
			Vertex_handle DualMeshVertex=B.add_vertex(CentroidPoint);
			DualMeshVertex->SetFacetOnPrimalMeshIndex(iIndex);
			i->SetDualMeshVertexIndex(iIndex);
			iIndex++;
		}

		//input dual mesh facets(each w.r.t a vertex on primal mesh)
		for (Vertex_iterator i=PrimalMesh.vertices_begin();i!=PrimalMesh.vertices_end();i++)
		{
			//Halfedge_around_vertex_circulator is clockwise!Be careful!
			vector<int> vecVertexIndexOnDualFacet;
			Halfedge_around_vertex_circulator Havc=i->vertex_begin();
			do 
			{
				vecVertexIndexOnDualFacet.push_back(Havc->facet()->GetDualMeshVertexIndex());
				Havc++;
			} while(Havc!=i->vertex_begin());
			reverse(vecVertexIndexOnDualFacet.begin(),vecVertexIndexOnDualFacet.end());

			B.begin_facet();
			for (unsigned int j=0;j<vecVertexIndexOnDualFacet.size();j++)
			{
				B.add_vertex_to_facet(vecVertexIndexOnDualFacet.at(j));
			}
			B.end_facet();

		}
		B.end_surface();
	}
private:
	KW_Mesh PrimalMesh;
};

class GeometryAlgorithm
{
public:

	//uniformly sample points on a circle
	static void SampleCircle(Point_3 CircleCenter,double dRadius,int iSamplePointNum,std::vector<Point_3>& SamplePoints);

	//get an arbitrary point in a 2d polygon
	static bool GetArbiPointInPolygon(Polygon_2 PolygonIn,Point_2& ResultPoint);

	//Judge if the point is in triangle
	//bool IfPointInTriangle2d(HPCPoint* Triangle, CPoint point);
	bool IfPointInTriangle2d(HPCPoint* Triangle, QPoint point);
	
	//get the intersection point between a circle and a segment
	//this function is still imperfect
	bool GetCircleAndSegIntersection(Circle_2 circle,Segment_2 seg,Point_2& IntersectionPoint);

	static double GetDerivation(std::vector<double> vecInputNumber);

	//convert a planar polygon to that on the XOY plane
	//currently can handle only xoy,yoz and xoz or parallel cases
	static void PlanarPolygonToXOY(std::vector<Point_3> InputPolygon,Polygon_2& OutputPolygon, int iPlaneType);

	//similar to PlanarPolygonToXOY,ensure the CCW of InputPolygon and OutputPolygon
	static void PlanarPolygonToXOYCCW(std::vector<Point_3>& InputPolygon,Polygon_2& OutputPolygon, int iPlaneType);

	//inverse to PlanarPolygonToXOY
	//currently can handle only xoy,yoz and xoz or parallel cases
	//SamplePoint: a sample point on the destination plane
	static void XOYPolygonTo3DPlanar(Polygon_2 InputPolygon,std::vector<Point_3>& OutputPolygon,int iPlaneType,Point_3 SamplePoint);

	//judge whether a planar polygon is clockwise or not
	//currently can handle only xoy,yoz and xoz or parallel cases
	static bool IsPlanarPolygonCCW(std::vector<Point_3> InputPolygon, int iPlaneType);
	
	//judge whether two planar polygons on the same plane intersect 
	//currently can handle only xoy,yoz and xoz or parallel cases
	static bool IsPlanarPolygonsIntersect(std::vector<Point_3> InputPolygon0,std::vector<Point_3> InputPolygon1, int iPlaneType);
	//2d version
	static bool IsPlanarPolygonsIntersect2D(Polygon_2 InputPolygon0,Polygon_2 InputPolygon1);

	//judge whether two planar polygons are contained in each other
	//0: 0 is contained in 1; 1: 1 is contained in 0; 2:intersect; 3: no intersection
	//currently can handle only xoy,yoz and xoz or parallel cases
	static int PlanarPolygonsContained(std::vector<Point_3> InputPolygon0,std::vector<Point_3> InputPolygon1, int iPlaneType);
	//2d version
	static int PlanarPolygonsContained2D(Polygon_2 InputPolygon0,Polygon_2 InputPolygon1);
	

	/*
	判断两条线段是否相交(有交点)
	*/
	bool IsLineSegmentCross(HPCPoint pFirst1, HPCPoint pFirst2, HPCPoint pSecond1, HPCPoint pSecond2);

	static int GetLineSphereIntersection(Line_3 line,Sphere_3 sphere,std::vector<Point_3>& IntersectPoints);

	double GetAngleBetweenTwoVectors2d(Vector_2 vecFrom,Vector_2 vecTo);

	//with orientation considered,the result is between 0&360
	static double GetAngleBetweenTwoVectors2d2PI(Vector_2 vecFrom,Vector_2 vecTo);

	//with orientation considered,the result is between 0&360
	static double GetAngleBetweenTwoVectors3d(Vector_3 vecFrom,Vector_3 vecTo,Vector_3& vecCrossResult);

	//without orientation considered,the result is between 0&180
	static double GetAngleBetweenTwoVectors3d(Vector_3 vecFrom,Vector_3 vecTo,bool bReturnRadius=false);

	//compute the center of a facet of mesh
	static Point_3 GetFacetCenter(Facet_handle facet);

	//get the weight of weighted laplacian for the current edge(between vertex0 and vertex1)
	static double GetWeightForWeightedLaplacian(Vertex_handle Vertex0,Vertex_handle Vertex1,int iWeightType);

	static double GetWeightForWeightedLaplacian(Point_3 Vertex0,Point_3 Vertex1,Point_3 Vertex1Prev,Point_3 Vertex1Next,int iWeightType);

	void ComputeRotatedPlane(Point_3 StartPointOnPlane,Point_3 EndPointOnPlane,double Plane_spin,Plane_3 & plane);

	void ComputeRotatedCurve(Point_3 RotateAxisStartPoint,Point_3 RotateAxisEndPoint,
							double Point_spin,std::vector<Point_3>& Curve3d);

	void ComputeRotatedPoint(Point_3 RotateAxisStartPoint,Point_3 RotateAxisEndPoint,
		double Point_spin,Point_3& Point3d);

	//compute the inverese matrix of a given matrix
	//a is the in/out matrix,n is the step
	void GetInverseMatrix(double  *a,int  n);

	//compute the position of a point after rotation&translation
	void ComputeTransformedPointPos(Point3D* point,double * ModelViewMatrix);

	//filter,resample and smooth a 2d curve
	//int ProcessCurverPoint(std::vector<CPoint>& CurvePoint,double dSpaceThreshold=25,int iDesiredPointNum=0);
	int ProcessCurverPoint(std::vector<QPoint>& CurvePoint,double dSpaceThreshold=25,int iDesiredPointNum=0);
	//delete the point which is within certain distance of its upstream point
	//int FilterCurvePoint(std::vector<CPoint>& CurvePoint,double dSpaceThreshold);
	int FilterCurvePoint(std::vector<QPoint>& CurvePoint,double dSpaceThreshold);
	//resample the curve to the desired point number
	//int ResampleCurvePoint(std::vector<CPoint>& CurvePoint,int iDesiredPointNum);
	int ResampleCurvePoint(std::vector<QPoint>& CurvePoint,int iDesiredPointNum);
	//smooth the curve
	//int SmoothCurvePoint(std::vector<CPoint>& CurvePoint);
	int SmoothCurvePoint(std::vector<QPoint>& CurvePoint);

	//resample the 3d curve to the desired point number
	static int ResampleCurvePoint3D(std::vector<Point_3>& CurvePoint,int iDesiredPointNum);
	//local modification of a 3D curve
	//ModifiedPointInd:which points are modified
	static int LocalModifyCurve3D(std::vector<Point_3>& OriginalCurve,std::vector<Point_3> ModifyCurve,std::vector<int>& ModifiedPointInd,double dBlendPara);

	//judge if a 2D curve is closed or not
	//static bool JudgeCurveOpen2D(std::vector<CPoint> CurvePoint);
	static bool JudgeCurveOpen2D(std::vector<QPoint> CurvePoint);
	//convert an open 2D curve to a closed symmetric one
	//static bool MakeSymmetricCurve2D(std::vector<CPoint>& CurvePoint);
	static bool MakeSymmetricCurve2D(std::vector<QPoint>& CurvePoint);


	//make the curve a closed stroke
	//int MakeClosedStroke2d(std::vector<CPoint>& CurvePoint);
	int MakeClosedStroke2d(std::vector<QPoint>& CurvePoint);

	bool ReversePointsOrder3d(Point_3 OriginCurveStartPoint,std::vector<Point_3>& DeformCurve);
	bool ReversePointsOrder2d(HPCPoint OriginCurveStartPoint,std::vector<HPCPoint>& DeformCurve);


	//local refine after deformation
	static void ExtrusionLocalRefine(KW_Mesh& NewMesh,std::vector<Facet_handle>& fhRefineTri);

	static void LaplacianSmooth(int iIterNum,double dLambda,std::vector<Vertex_handle>& vecVertexToSmooth);
	static void LaplacianSmooth(int iIterNum,double dLambda,KW_Mesh& Mesh);

	static void TaubinLambdaMuSmooth(int iIterNum,double dLambda,double dMu,std::vector<Vertex_handle>& vecVertexToSmooth);
	static void TaubinLambdaMuSmooth(int iIterNum,double dLambda,double dMu,KW_Mesh& Mesh);

	//compute the plane for shown
	bool GetPlaneBoundaryPoints(Point_3 StartPointProj,Point_3 EndPointProj,
		Plane_3 plane,Point3D* PlaneBoundaryPoints);

	void AdjustPlaneBoundary(int iIncrease,Point3D* PlaneBoundaryPoints);
	void AdjustPlaneBoundary(int iIncrease,Point_3* PlaneBoundaryPoints);

	static void GetNearestPointOnCurve(Point_3 RefPoint,std::vector<Point_3> Curve,bool bRefPointOnCurve,int iRefPointInd,
										double& dMinSquaredDist,int& iNearestInd);

	static void GetFurthestPointOnCurve(Point_3 RefPoint,std::vector<Point_3> Curve,bool bRefPointOnCurve,int iRefPointInd,
										double& dMaxSquaredDist,int& iFurthestInd);

	//get the intersection points between a closed polyline and a plane
	static int GetClosedCurvePlaneIntersection(std::vector<Point_3> Curve,Plane_3 plane,
										std::vector<std::vector<int> >& SegPoinsInd,
										std::vector<Point_3>& InterSectPoints);

	//vecPoint0 & vecPoint1: two groups of points with equal numbers
	//GroupResult: each element contains two indices, each of them represent the index of the
	//nearst point on each curve
	//vecMidPoint: the midpoint of each pair of nearest points
	static void GroupNearestPoints(std::vector<Point_3> vecPoint0,std::vector<Point_3> vecPoint1,
		std::vector<Int_Int_Pair>& GroupResult,std::vector<Point_3>& vecMidPoint);

	//get the intersection curves between a polyhedron_3 and a plane
	static int GetMeshPlaneIntersection(KW_Mesh mesh,Plane_3 plane,std::vector<std::vector<Point_3>>& IntersectCurves);
	
	//judge if an open curve intersect with a polyhedron_3
	static bool JudgeMeshOpenCurveIntersec(KW_Mesh mesh,std::vector<Point_3> OpenCurve);


	//compute the laplacian coordinates of the whole mesh
	static void ComputeCGALMeshUniformLaplacian(KW_Mesh& mesh);
	static void ComputeCGALMeshWeightedLaplacian(KW_Mesh& mesh,int iWeightType);//2:tan 3:cot

	//compute the laplacian coordinates of input vertices
	static void ComputeCGALMeshUniformLaplacian(std::vector<Vertex_handle>& Vertices);
	static void ComputeCGALMeshWeightedLaplacian(std::vector<Vertex_handle>& Vertices,int iWeightType);//2:tan 3:cot

	//compute the laplacian coordinates of dual mesh vertices
	static void ComputeCGALDualMeshUniformLaplacian(std::vector<Vertex_handle>& Vertices);
	static void ComputeCGALDualMeshWeightedLaplacian(std::vector<Vertex_handle>& Vertices,int iWeightType);

	//compute the old edge vectors around a vertex
	static void ComputeOldEdgeVectors(std::vector<Vertex_handle>& Vertices);
	static void ComputeOldEdgeVectors(KW_Mesh& mesh);

	//judge if vertex i and j are neighbors,return the index of j among all the neighbors of i(start from 1)
	static int JudgeIfNeighbors(Vertex_handle i,Vertex_handle j);
	//judge if Dual vertex i and Dual vertex with index j are neighbors
	static bool JudgeIfNeighbors(DualMeshVertexStruct i,int j);

	static void ComputeMeshMeanCurvature(KW_Mesh& mesh);

	static void ComputeMeshGaussianCurvature(KW_Mesh& mesh);

	//set color for each vertex
	static void SetUniformMeshColor(KW_Mesh& mesh,std::vector<double> vecColor);

	static bool MultiplyMatrixAandMatrixB(std::vector<std::vector<double>> MatrixA,std::vector<std::vector<double>> MatrixB,
		std::vector<std::vector<double>>& Result);

	//set the index of each vertex in sequential order
	static void SetOrderForVer(std::vector<Vertex_handle>& Vertices);

	//A*X=B
	static void SolveLinearEquation(std::vector<std::vector<double>> A,std::vector<double>B,std::vector<double>& X);
	
	static void DivideFacets(KW_Mesh& NewMesh,std::vector<Facet_handle>& fhRefineTri,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);

protected:

	//divide facet with 6 edges into 4 triangles
	static void DivideFacet6Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);
	//divide facet with 5 edges into 4 triangles
	static void DivideFacet5Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);
	//divide facet with 4 edges into 4 triangles
	static void DivideFacet4Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);
	//divide facet with 3 edges into 4 triangles
	static void DivideFacet3Eto4Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);
	//divide left facets with more than 4 edges into 4 Triangles after fhRefineTris have been divided
	static bool DivideFacetsto4Tri(KW_Mesh& NewMesh,std::vector<Vertex_handle>& vecNewEdgeVertex,
		std::vector<Vertex_handle>& vecOriVertex);
	//divide facet with 4 edges into 2 triangles
	static void DivideFacet4Eto2Tri(KW_Mesh& NewMesh,Facet_handle& fhRefineTri);

	static void SetCurvatureColor(KW_Mesh& mesh,int iCurType);//0 for mean curvature,1 for gaussian curvature

	static void NormalizeMeshMeanCurvature(KW_Mesh& mesh);

public:
	GeometryAlgorithm(void);
	~GeometryAlgorithm(void);
};














