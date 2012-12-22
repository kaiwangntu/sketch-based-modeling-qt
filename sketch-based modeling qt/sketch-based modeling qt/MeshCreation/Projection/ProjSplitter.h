#ifndef _PROJSPLITTER_H
#define _PROJSPLITTER_H

#include "../config.h"
#include "../Util/VerEdgePlaneOp.h"
#include "../Projection/Projector.h"
#include "../SubspaceContour/sscontour.h"

#include "../Math/mymath.h"

//new seam registration
struct newSeamReg
{
	int vernum;
	bool* subEdgeIsIn;		//n	subedges
	int* verPosInMeshVer;	//n+1 vertices
	EdgeRegItem* edgelist;	//for each, {posinMeshEdge, corresponding contour edges}
};

struct subEdge
{
	int posInMeshVer[ 2 ];
	int posInMeshEdge;
	int posInCtrEdge;
	//int startEdge;	//with which edge from the other face, this new subedge results from
};
typedef vector<subEdge> oneEdge_vec;

class ProjSplitter{
public:
	ProjSplitter(){}
	~ProjSplitter(){}

	//split the projected edges on ma sheet
private:
	static void splitProjEdgeSeam_One(
		//input and output
		floatvector& meshVer, intvector& meshEdge,
		//output
		newSeamReg& nseamReg,
		//all the vertices and edges in the subspace
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//registration information
		int* jptreg,
		intvector& seamVerReg,		//for each seam, the projected vertices
		EdgeReg& seamEdgeReg,		//for each seam, the projected edges on it

		int majptnum,int maseamnum,int masheetnum,
		float* majpt, int jpt2[ 2 ] ,
		int seami
		);
	static void splitProjEdgeSeam(
		//input and output
		floatvector& meshVer, intvector& meshEdge,
		//output
		newSeamReg*& nseamReg,
		//all the vertices and edges in the subspace
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector*& seamVerReg,		//for each seam, the projected vertices
		EdgeReg*& seamEdgeReg,		//for each seam, the projected edges on it
	//	intvector* sheetVerReg,		//for each sheet, the projected vetices on it
	//	EdgeReg* sheetEdgeReg,	
		int majptnum,int maseamnum,int masheetnum,
		int* maseam, float* majpt  );
	static void splitCtrEdge(
		intvector& subedgelist1,
		floatvector& paramlist1,
		int facei,
		int sheeti,
		//mesh
		floatvector& meshVer, 
		//contour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//edges registered for facei and facej
		oneEdge_vec& edgeFacei,	//only subedges on one edge are passed in
		int pos1
		);
	static void splitProjEdgeSheet_One(
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//contour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		int facei, int facej,
		EdgeReg& sheetReg,
		//output
		EdgeReg& nsheetReg,	//only one vector in all the vectors
		int sheeti
		);
	static void SplitProjEdgeSheet(
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//contour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//input
		EdgeReg* sheetEdgeReg,
		//output
		EdgeReg* nsheetReg,
		//ma
		int subfacenum,
		MapArraySR& doubleface2sheet
		//ma
		/*int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
		float* subparam,int* subver2wver,int* subedge2wedg,int majptnum,float* majpt,int maseamnum,
		int* maseam,MapArraySR doubleface2sheet,int* seamonsheetnum,int** seamonsheet,int* ver2jpt*/
		);
	static void refreshAfterSegIntersect(
		int intertype,
		intvector& subedgelist1, intvector& subedgelist2,
		floatvector& paramlist1, floatvector& paramlist2,
		float newpt[ 3 ],
		int sheeti,
		int facei,
		int facej,
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//contour
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//edges registered for facei and facej
		oneEdge_vec& edgeFacei,	//only subedges on one edge are passed in
		oneEdge_vec& edgeFacej,
		EdgeReg& nsheetReg,
		intvector& startedge,
		int edgei,	//the index of the edge on facei in process
		int edgej,	//the index of the edge on facej in process
		int& pos1,
		int& pos2);
	static bool isRepeat(
		intvector& gatheredge,
		int veris[ 2 ],
		int cmplen,
		int& repeati,
		bool& samedirec
		);
public:
	static int edgeIntersect(
		float vers[ 12 ],	//position of the four vertices
		int veris[ 4 ],		//position of the four end points in meshVer
		intvector& subedgelist1,	//end points of all the subedges of edge1 after intersection
		floatvector& paramlist1,	//the parameter of the second point relative to original edge
		intvector& subedgelist2,	//similar
		floatvector& paramlist2,
		bool newptexist,	//if the intersection point between the two segments exist or not
		float newpt[ 3 ]	//if exist, the position is saved in it
		);
	static void SplitProjectedEdge(
		floatvector& meshVer, intvector& meshEdge,
		newSeamReg* nseamReg,
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		EdgeReg* nsheetReg,
		//ma
		int subfacenum,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector*& seamVerReg,		//for each seam, the projected vertices
		EdgeReg*& seamEdgeReg,		//for each seam, the projected edges on it
		intvector*& sheetVerReg,		//for each sheet, the projected vetices on it
		EdgeReg*& sheetEdgeReg,	
		int majptnum,int maseamnum,int masheetnum,
		int* maseam, float* majpt,
		MapArraySR& doubleface2sheet);

	static void WriteInfoAfterSplit( int spacei,
		//mesh
		floatvector& meshVer,
		intvector& meshEdge,
		//jptreg seamreg sheetreg
		int majptnum,	int maseamnum,	int masheetnum,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		newSeamReg* nseamReg,
		EdgeReg* sheetEdgeReg,	//new sheet registration information actually
	vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec);
};

#endif