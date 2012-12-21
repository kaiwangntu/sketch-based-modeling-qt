#pragma once
#ifndef MESH_CREATION_STRUCT_DEF
#define MESH_CREATION_STRUCT_DEF
//#include "../GeometryAlgorithm.h"
#include "../PreDef.h"
#include "../CGALDef.h"

//curve network for mesh creation
typedef struct _CurveNetwork
{
	//curves on current plane
	std::vector<std::vector<Point_3> > Profile3D;
	//curves to render,if the whole cs is sketched,the element is empty;
	//key: which CS is partial  value: which points on the CS belong to the partial parts
	//since partial parts may contain many segments, so use vector<vector<int>>
	std::map<int,std::vector<std::vector<int>>> PartProfile3D;
	//corresponding 2D polygon
	std::vector<Polygon_2> Profile2D;
	//plane on which the profiles lie
	Plane_3 plane;
	//0:profile on xoy plane,1:profile on xoz plane,2:profile on yoz plane
	int ProfilePlaneType;

	////intersection point between curves and orthogonal planes
	//std::vector<Point_3> CurvePlaneIntersect;
	////index of the curve each intersection point lies on
	//std::vector<int> IntersectCurveIndex;
	////indices of two neighbor points of each intersection point
	//std::vector<std::vector<int>> NeighborInd;
	////intersect with which curve network(which plane)?
	//std::vector<int> IntersectCNInd;

	//type corresponds to each curve,-1 stands for the outside of a ring/circle
	//other number,stands for the inside of a ring,and the number if the index of
	//its parent curve(outside of the ring)
	std::vector<int> CurveInOut;
}*pCurveNetwork,CurveNetwork;

typedef struct _Resorted_Face
{
	//boundary face or not. if boundary face,don't record the following info
	bool bBoundaryFace;
	//id of plane it belongs to(==id of curve network)
	int iFacePlaneID;
	//four vertices
	vector<Point_3> vecFaceVertex;
	//id of its two subspaces
	vector<int> vecSubspaceId;
	//orientations in its two subspaces
	vector<bool> vecOrient;
	//height vectors in its two subspace
	vector<Vector_3> vecHeightVect;
}*pResortedFace,ResortedFace;


typedef struct _Polygon_with_holes_3
{
	//outer boundary of a hole/circle
	vector<Point_3> outer_boundary;
	//indicate which outer edges do not belong to the original input CN
	vector<int> AssistOuterEdge;
	//inner of holes (if exist)
	vector<vector<Point_3>> inner_hole;
	//indicate which inner edges of each hole do not belong to the original input CN
	//on each face, the inner polygon should not contain part of the face bounding edge,so delete this item
	//vector<vector<int>> AssistInnerEdge;
}*pPolygon_with_holes_3,Polygon_with_holes_3;

typedef std::list<Polygon_with_holes_3>   Pwh_list_3;
typedef std::vector<Polygon_with_holes_3>   Pwh_vector_3;


//polygons formed by curve network and bounding edges on each face
typedef struct _PolygonOnFace
{
	//intersections of each CS in CN with the bounding face
	//each one corresponds to a CS in CN: each CS may form many circles/holes(after subtracting its inner circles), so use Pwh_list_2 instead of Polygon_with_holes_2
	//this is also the reason why not merge all Pwh_list_2 of all CS -- each Pwh_list_2 corresponds to each CS in CN
	vector<Pwh_list_2> vecPwhList2D;

	//corresponding 3D polygons with holes(if exist) on faces
	vector<Pwh_list_3> vecPwhList3D;

}*pPolygonOnFace,PolygonOnFace;

////polyhedrons related to each face formed PolygonOnFace
////it corresponds to PolygonOnFace closely
//typedef struct _PolyhedronFromPOF
//{
//	//the id of subspace on the positive side of the face
//	int iPosSpaceId;
//	//the polyhedrons positive subspace 
//	vector<vector<GmpPolyhedron>> PosPolyhedrons;
//	//the id of subspace on the negative side of the face
//	int iNegSpaceId;
//	//the polyhedrons negtive subspace 
//	vector<vector<GmpPolyhedron>> NegPolyhedrons;
//}*pPolyhedronFromPOF,PolyhedronFromPOF;

//polyhedrons formed from Polygon_with_holes_3s on each face
//lose correspondence with PolygonOnFace here!!! only collect the total result
typedef struct _PolyhedronFromPOF
{
	//id of the face
	int iFaceID;
	//Polygon_with_holes_3s on the face
	Pwh_vector_3 vecPwh3D;
	//corresponding 2D info of vecPwh3D
	//Pwh_vector_2 vecPwh2D;
	//polyhedrons formed from Pwh3D
//	vector<GmpPolyhedron> vecPwhCylinder;
	vector<KW_Mesh> vecPwhCylinder;
}*pPolyhedronFromPOF,PolyhedronFromPOF;


#endif