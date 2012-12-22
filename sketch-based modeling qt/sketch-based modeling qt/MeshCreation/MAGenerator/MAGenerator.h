#ifndef _MAGENERATOR_H_
#define _MAGENERATOR_H_

#include "../config.h"
#include "../Util/MapArraySR.h"
#include "../Util/SetOperation.h"
#include "../Util/VerEdgePlaneOp.h"

struct MAJunctionPoint{
	float pos[ 3 ];
	intvector governor;	//four is a group
    intvector seam_pos;	//in which seams and which vertex of the seam, two is a group
} ;

struct MASeam{
	int jpts[ 2 ];		//the two junction points of the seam
	float dir[ 3 ];
	intvector governor;	//governor of the seam
	int cutface[ 2 ];	//which faces cut the seam at the two junction points
};

class MAGenerator
{
public:
	MAGenerator();
	~MAGenerator();

	//subspace information
	static void inline getSubspaceInfo(int planenum, float* planeparam, int ssvernum, 
		float* ssver,	int ssedgenum,int* ssedge,int ssfacenum, int* ssfaceedgenum, int** ssface,
		int* ssface_planeindex, int ssspacenum,  int* ssspacefacenum,	int** ssspace, int** ssspace_planeside, 
		int subspacei,int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
		int**& subface,float*& subparam,int*& subver2wver,  int*& subedge2wedge );
	static void writeSubspaceInfo(const char* fname,
		int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
		int**& subface,float*& subparam,
		int*& subver2wver,  int*& subedge2wedge);

	//topology information
	static void computeSheetParam(float*& subparam, int facei, int facej, float*& sheettab);
	static bool computeSheetNorm(float*& subparam, int facei, int facej, float*& sheettab, int posinsheet);
	static void gatherTopology(int& subvernum, float*& subver,int& subedgenum, int*& subedge,
		int& subfacenum,int*& subfaceedgenum,	int**& subface,float*& subparam,
		MapArraySR& doubleface2sheet, float* sheettab,	MapArraySR& tripleface2ver, MapArraySR& doubleface2edge,
		int**& ver2face,int*& ver2facenum,int*& edge2face, int**& face2ver, int* face2vernum );
	static void writeTopology(const char* fname,int subvernum, int subedgenum, int subfacenum,
		MapArraySR& doubleface2sheet, float* sheettab,MapArraySR& tripleface2ver, MapArraySR& doubleface2edge,
		int**& ver2face,int*& ver2facenum,int*& edge2face, int**& face2ver, int* face2vernum );

	//compute the first junction point of current subspace
	static void findFirstJpt(const int& subvernum, float*& subver, int*& subedge,
		int& subfacenum,	float*& subparam,
		int**& ver2face, int* ver2facenum,
		MapArraySR& doubleface2sheet,  float* sheettab,MapArraySR& doubleface2edge,
		intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
		vector<MASeam>& seamtab,MapArraySR& tripleface2seam,
		int*& ver2jpt);
	static void writeSubspaceInfoWithFirstJpt(const char* fname,
		int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
		int**& subface,float*& subparam,
		int*& subver2wver,  int*& subedge2wedge,
		vector<MAJunctionPoint>& jpttab);

	//trace algorithm, from the first junction point to trace out all the seams and junction points
	static void getSeam(float* subparam, MapArraySR& doubleface2sheet,  float* sheettab, int* triset, 
		float dir[ 3 ] );
	static int isOldSeam(int juncpti, float seamdir[ 3 ], vector<MAJunctionPoint>& jpttab,vector<MASeam>& seamtab);
	static void inline computeNewJpt(int subfacenum,
		MapArraySR& doubleface2sheet,  float* sheettab,
		int activejpti, float seamdir[ 3 ], int*& triset,
		intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
		vector<MASeam>& seamtab,MapArraySR& tripleface2seam);
	static void inline traceSeamDir(int subfacenum,
		MapArraySR& doubleface2sheet,  float* sheettab,
		int activejpti, float seamdir[ 3 ], int*& triset,
		intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
		vector<MASeam>& seamtab,MapArraySR& tripleface2seam);
	static void traceMA(float* subver,float* subparam, int subfacenum,
		MapArraySR& tripleface2ver, MapArraySR& doubleface2sheet,  float* sheettab,
		intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
		vector<MASeam>& seamtab,MapArraySR& tripleface2seam,
		int*& ver2jpt);


	//process the active junciton points	
	void static generateMA(int planenum, float* planeparam,int ssvernum, float* ssver,int ssedgenum,
		int* ssedge,int ssfacenum, int* ssfaceedgenum, int** ssface,int* ssface_planeindex, 
		int ssspacenum,  int* ssspacefacenum,	int** ssspace, int** ssspace_planeside, int subspacei,
		//subspace info
		int& subvernum, float*& subver, int& subedgenum,int*& subedge,int& subfacenum,
		int*& subfaceedgenum, int**& subface, float*& subparam, int*& subver2wver,int*& subedge2wedge,
		//ma info
		int& majptnum, float*& majpt, int& maseamnum, int*& maseam, 
		MapArraySR& doubleface2sheet, int*& seamonsheetnum, int**& seamonsheet,  float*& sheettab,
		int*& ver2jpt );

	void static writeMA(
		const char* fname,
		//subspace info
		int& subvernum, float*& subver, int& subedgenum,int*& subedge,int& subfacenum,
		int*& subfaceedgenum, int**& subface, 
		//ma info
		int& majptnum, float*& majpt, int& maseamnum, int*& maseam, 
		MapArraySR& doubleface2sheet, int*& seamonsheetnum, int**& seamonsheet, 
		int*& ver2jpt);
	void static writeJptSeamTab(vector<MAJunctionPoint>& jpttab,
		vector<MASeam>& seamtab,
		int count);

};


#endif