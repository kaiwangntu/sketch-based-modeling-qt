#ifndef _FACEGENERATOR_H_
#define _FACEGENERATOR_H_

#include "../config.h"
#include "../Triangulation/Triangulation.h"
#include "../RegionHandler/regionHandler.h"

class FaceGenerator
{
public:
	FaceGenerator(){}
	~FaceGenerator(){}

	static void get2DVer(
		floatvector& shtVpos_fvec,
		float xdir[ 3 ],
		float zdir[ 3 ],
		floatvector& vpos2D
		);
	static void getCycle(
		sheetRegion& region, 
		//intvector& shtVposInMV_ivec,  floatvector& shtVpos_fvec,  
		intvector& shtEcmpos_ivec,
		int regioni,
		//float zdir[ 3 ],  float xdir[ 3 ],
		//floatvector& ver2D,  
		//intvector& vposinmesh, 
		vector<intvector>& cycles  );
	
	static void	GFaceOnSheets(
		intvector& meshFace,
		intvector*& shtVposInMV_arr_ivec,
		floatvector*& shtVpos_arr_fvec,	intvector*& shtEcmpos_arr_ivec,
		sheetRegion*& region_vec,
		float*& sheettab,		int facenum,		int*& maseamnum,
		MapArraySR& doubleface2sheet);
	static void addCtrVerOneFace(
		floatvector& meshVer,
		SSPCTRVERVEC& ctrver,
		//SSPCTREDGEVEC& ctredge,
		int*& cvposinmv,
		//int*& ver2jpt,
		int*& jpt2ver,	//from junction to subspace vertex index
		int* njptreg,		//map from the junction point to the position of it in meshver		
		int spacedgenum, int maseamnum,		newSeamReg*& nseamReg,
		float*& ssver,		int*& ssedge,		
		//int subspacedgenum,
		//int ssfaceedgenum,		
		//int*& ssfaceedge,
		int*& edge2wedge,		intvector*& sverreg,	vector<intvector>*& sedgereg,
		int spaci);
	static void GFaceNormalSeams(
		intvector& meshFace,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		newSeamReg*& nseamReg,	int maseamnum,		int subedgenum,
		int**& cvposinmesh,		int spaci,		intset*& sfacectrei,
		int*& ssface,		//the face index in the whole subspace
		int*& sfacespaci,	//for each facei, save two subspaces neighboring with it
		vector<intvector>*& sfaceregface,
		int*& faceside);
	static void GFaceSheet(
		intvector& meshEdge,
		intvector& meshFace,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		int**& cvposinmesh,	
		EdgeReg*& nsheetReg,
		int facenum,
		MapArraySR& doubleface2sheet,
		int* seamonsheetnum,
		int spaci,
		int*& ssface,		//the face index in the whole subspace
		int*& sfacespaci,	//for each facei, save two subspaces neighboring with it
		vector<intvector>*& sfaceregface,
		intset*& sfacectrei,
		int*& faceside);

	//for debug
	//write out ver2d and cycles
	static void writeVerCycle(
		floatvector& ver2d, 
		vector<intvector>& cycles, int sheeti, int regioni);
	//write out the triangle list
	static void writeSubMeshOut(
		floatvector& meshVer,
		intvector& meshFace,
		int spaci
		);
};

#endif


