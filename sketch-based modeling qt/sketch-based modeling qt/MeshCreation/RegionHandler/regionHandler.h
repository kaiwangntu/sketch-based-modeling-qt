#ifndef _REGIONHANDLER_H
#define _REGIONHANDLER_H

#include "../config.h"
#include "../SubspaceContour/sscontour.h"
#include "../Projection/ProjSplitter.h"
#include "../Projection/Projector.h"
#include "../math/mymath.h"

struct  sheetRegion
{

	//each boundary has all the edge index in it
	vector<intvector> boundaries;	
	//from the previous segmentor to current, is a boundary cluster
	//intvector segmentor;

	//in each region, it includes the boundary index which enclose the region
	vector<intvector> regions;

	//index of the outmost region
	int outregion;

	//neighbors of each region, region index list
	vector<intset> regionneighbrs;	//one vector for one region.

	//material
	intvector mat;	//upside material and downside material 2 for one region
};
//struct sheetRegion
//{
//	vector<intvector> 
//};
class regionHandler{
public:
	regionHandler(){};
	~regionHandler(){};

	//////////////////////////////////////////////////////////////////////////
	/*static intvector tempshtVposInMV_ivec;
	static intvector tempshtEposInME_ivec;*/
	//////////////////////////////////////////////////////////////////////////

	//in regionhandler.cpp
	//amplify the mesh, with all the junction points added
	static void amplifyMesh(
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//vertex registration
		int*& ajptReg,	//new junction point registration with all of them added into Meshver
		int majptnum,		float*& majpt,
		//edge registration
		newSeamReg*& nseamReg, //new seam registration info
		int maseamnum,		int*& maseam		);
	static void setEdgeMat(
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		EdgeRegItem& subedgereg,		bool dir[ 2 ],		int facei,		int facej,		int mat[ 4 ]		);
	static void getVerEdgeOnOneSheet(
		intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
		intvector& shtMat_ivec,		//each of them has a list of material in it, four number corresponds to one edge 
		//left mat, right mat above, left mat, right mat, below
		int& seamedgenum,				//the number of edges on seam in the edge list
		floatvector& meshVer,		//mesh
		intvector& meshEdge,		newSeamReg* nseamReg,	//seam reg
		EdgeReg& nsheetReg,		//sheet reg
		int*& seamonsheet,		int seamonsheetnum,		int*& faceside,
		int facei	,			//facei < facej
		int facej,
		//vector<SSPCTRVERVEC> sspctrver_vec,
		vector<SSPCTREDGEVEC> sspctredge_vec
		);
	static bool findClosedRegionMatConfig(
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//vertex registration
		int*& ajptReg,	//new junction point registration with all of them added into Meshver
		int majptnum,		float*& majpt,
		//edge registration
		newSeamReg*& nseamReg, //new seam registration info
		EdgeReg*& nsheetReg,		int maseamnum,		int*& maseam,		int mafacenum,
		MapArraySR& doubleface2sheet,		int masheetnum,		int*& seamonsheetnum,
		int**& seamonsheet,		int*& faceside,		float*& sheettab, //6 for each, point and normal
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
		intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
		intvector*& shtMat_arr_ivec,		//each of them has a list of material in it, four number corresponds to one edge 
		//left mat, right mat above, left mat, right mat, below
		int*& seamedgenum_iarr,				//the number of edges on seam in the edge list
		sheetRegion*& region_vec
		);

	//in regionmatsetter.cpp
	static void setMatOnOneSheet(intvector& shtEcmpos_ivec,intvector& shtMat_ivec,	sheetRegion& region	);
	static void insertNewSeamEdgeInfo(vector<intvector>& seamedgeinfo,int facei,int facej,int mati,int matj);
	static void broadAcrossSeam(
		intvector*& shtEposInME_arr_ivec,
		intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
		int*& seamedgenum_iarr,	sheetRegion*& region_vec,
		//intset& seamedgeset,	
		int sheetnum,	newSeamReg*& nseamReg,
		MapArraySR& doubleface2sheet,	int*& seamonsheetnum,	int facenum	);

	//in closedRegionFinder.cpp
	static void findClosedBoundaryCluster(
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
		float zdir[ 3 ],	float xdir[ 3 ],		sheetRegion& region,	intvector& boundseg		);
	static float distVer2Edge(
		float ver[ 3 ],		floatvector& shtVpos_fvec,		int edgevi[ 2 ],
		//type - 0, normal case, minimal distance to edge is real distance to line edge lies on
		//tyep - 1, the minimal distance to one vertex of the edge
		int& type,		int& veri);
	static bool isInBoundary(
		float ver[ 3 ],
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		intvector& oneboundary,	float zdir[ 3 ]) ;
	static bool getRegionFromBoundaryCluster(
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
	//	intvector& shtMat_ivec,	
	float zdir[ 3 ],		float xdir[ 3 ],
		sheetRegion& region,		intvector& boundseg  );
	static bool setOutMostRegion(
		sheetRegion& region,
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,
		int seamedgenum,		int seamonsheetnum,		int*& seamonsheet,		int*& maseam,
		float*& majpt,		float zdir[ 3 ]		);
	static void setRegionNeighbr(			intvector& shtEcmpos_ivec,			sheetRegion& region		);
	static bool findClosedRegions(
			floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
			intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
			//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
			//	intvector& shtMat_ivec,
			float zdir[ 3 ],			float xdir[ 3 ],
			sheetRegion& region,			int seamedgenum,			int seamonsheetnum,
			int*& seamonsheet,			int*& maseam,			float*& majpt  );

	//for debug
	static void writeVerEdgeOnOneSheet_db(
		intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,
		FILE* fout );
	static void writeVerEdgeOnSheet_db(
		intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
		intvector*& shtEcmpos_arr_ivec,
		int sheetnum,
		int spaci
		);

	static void writeVerEdgeRegionOneSht_db(
		intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		sheetRegion& region,							 
		int seamedgenum,						//the number of edges on seam in the edge list
		FILE* fout 
		);
	static void writeVerEdgeRegion_db(
		int spaci,
		int sheetnum,
		intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
		intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
		int*& seamedgenum_iarr,						//the number of edges on seam in the edge list
		sheetRegion*& region_vec
		);
	static void writeMat_db(
		int spaci,
		int sheetnum,
		intvector*& shtMat_arr_ivec			  );

	//for debug
	static void writeRegion_db(
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
		//	intvector& shtMat_ivec,
		float zdir[ 3 ],
		float xdir[ 3 ],
		sheetRegion& region,
		intvector& boundseg 
		);
	static void writeVerEdgeDirOneSheet(
		floatvector& shtver,
		intvector& shtedge,
		float dir[ 3 ],
		int spaci,
		int sheeti
		);
};
#endif