#ifndef _TRIANGULATION_H
#define _TRIANGULATION_H

#include "../config.h"
#include "../Util/VerEdgePlaneOp.h"
#include "../math/mymath.h"

class Triangulation{
public:
	Triangulation(){}
	~Triangulation(){}

	//
	static bool checkCCW(
		floatvector& ver2d,
		vector<intvector>& cycles,
		int veri[ 6 ]
		);
static bool checkCCW( floatvector& ver2d,  int vi[ 3 ]);
static bool checkInBetween(
		floatvector& ver2d,
		vector<intvector>& cycles,
		int cycle1, int v1, int cycle2, int v2);
static bool checkEdgeInter(
	floatvector& ver2d,
	int rveri[ 3 ],	//the three vertices of the triangle
	int evi[ 2 ]
	);
static bool checkTri(
	floatvector& ver2d,
	vector<intvector>& cycles,
	int veri[ 6 ]
	);
static void triangulate( floatvector& ver2d,
	vector<intvector>& cycles,
	intvector& tri_vec	//resulting triangles
	);

//debug
static void writeVerCycle(
				   floatvector& ver2d, 
				   vector<intvector>& cycles, int sheeti, int regioni);
////write out the triangle list
//static void writeSubMeshOut(
//							floatvector& meshVer,
//							intvector& meshFace,
//							int spaci
//							);

};

#endif