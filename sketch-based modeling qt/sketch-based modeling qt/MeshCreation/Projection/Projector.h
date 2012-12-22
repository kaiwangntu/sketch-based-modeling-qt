#ifndef _PROJECTOR_H
#define _PROJECTOR_H

#include "../config.h"
#include "../Math/mymath.h"
#include "../Util/VerEdgePlaneOp.h"
#include "../SubspaceContour/sscontour.h"

//for each projected edge, its corresponding contour edges
struct correspnCtrEdge
{
	int facei;		//the face index in the current subspace
	int edgei;		//the edge index in the contours edge on this face
	bool samedirec;	//if the direction of the contour edge is the same as the projected edge on Medial Axis.
};

//projected edges on seam or sheet of MA
struct EdgeRegItem
{
	int posInMeshEdge;	//the position of the edge in mesh edges
	vector<correspnCtrEdge> crspCtrEdges;
};
typedef vector<EdgeRegItem> EdgeReg;	//edge registration information, for seams and sheets.

////seam registration
//struct seamRegItem_Ver
//{
//	float param;	//parameter of the projected vertex on this seam relative to the two endpoints
//	int posInMeshVer;	//the projected vertex is candidate for the future mesh, the position of the vertex in the mesh vertex
//};

//struct seamRegItem_Edge
//{
//	//int vers[ 2 ];	//the two vertices of the edge
//	int posInMeshEdge;	//the position of the edge in mesh edges
//    vector<correspnCtrEdge> crspCtrEdges;
//};
//
////sheet registration
//struct sheetRegItem_Edge
//{
//	//int vers[ 2 ];
//	int posInMeshEdge;	//the position of the edge in mesh edges
//	vector<correspnCtrEdge> crspCtrEdges;
//};

class Projector
{
public:
	Projector(){}
	~Projector(){}

	//project the vertices and edges on faces of subspace onto MA
	static void projectCtr(floatvector& meshVer,
		intvector& meshEdge,
	//	intvector& meshFace,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector* seamVerReg,		//for each seam, the projected vertices
		EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
		intvector* sheetVerReg,		//for each sheet, the projected vetices on it
		EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it

		//all the vertices and edges in the subspace
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//ma
		int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
		float* subparam,int* subver2wver,int* subedge2wedge,int majptnum,float* majpt,int maseamnum,
		int* maseam,MapArraySR& doubleface2sheet,int* seamonsheetnum,int** seamonsheet,float* sheettab,int* ver2jpt);

	//debug
	static void writeProjection_NoSplit(
		int spacei,
		//mesh
		floatvector& meshVer,
		intvector& meshEdge,

		//jptreg seamreg sheetreg
		int majptnum,	int maseamnum,	int masheetnum,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector* seamVerReg,		//for each seam, the projected vertices
		EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
		intvector* sheetVerReg,		//for each sheet, the projected vetices on it
		EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it
		//dcontour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec);
	static void writeProjection_NoSplit_bak(
		int spacei,
		//mesh
		floatvector& meshVer,
		intvector& meshEdge,

		//jptreg seamreg sheetreg
		int majptnum,	int maseamnum,	int masheetnum,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector* seamVerReg,		//for each seam, the projected vertices
		EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
		intvector* sheetVerReg,		//for each sheet, the projected vetices on it
		EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it
		//dcontour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec);
	
};

#endif