#ifndef  _SSCONTOUR_H_
#define  _SSCONTOUR_H_

#include "../config.h"
#include "../Util/MapArraySR.h"
#include "../Util/VerEdgePlaneOp.h"
#include "../Util/SetOperation.h"
//#include "../Ctr2SufManager/Ctr2SufManager.h"

const int VER_JPT = 1;	//the contour vertex will be projected to a junction point
const int VER_SEAM = 2;	
const int VER_SHEET = 3;
const int EDGE_SEAM = 2;
const int EDGE_SHEET = 3;
struct SUBSPACECTRVER
{
	float pos[ 3 ];
	int type;	//which kind of element of the medial axis this contour vertex will project to.
	int val;	//the value of the element
};
struct SUBSPACECTREDGE
{
	int veris[ 2 ];	//the two vertices of the edge
	int mat[ 2 ];	//material on the left side and the right side
	int type;	//which kind of element of the medial axis this edge is going to project to
	int val;	//the value of the element.
	int ancestor;	//the original edge index in the plane, useful when stitching.
};
typedef vector<SUBSPACECTRVER> SSPCTRVERVEC;
typedef vector<SUBSPACECTREDGE> SSPCTREDGEVEC;
class SSContour{
public:
	static void sortSheetSeams(
		int* maseam,int subfacenum,
		MapArraySR& doubleface2sheet,int*& seamonsheetnum,int**& seamonsheet
		);
	static void gatherMA2Face(
		int spacei, int facei,
		int subfacenum,int* maseam,MapArraySR& doubleface2sheet,int* seamonsheetnum,int** seamonsheet,
		//result
		int*& fjpts,int& fjptnum,
		int*& fseams, int& fseamnum,
		int*& fsheetis, int& fsheetinum);
	static void projectionOfJpts(
		int* fjpt, int* fjptmark, int fjptnum,
		float fparam[ 4 ], float* majpt,float* subver,
		//result
		float* projpt			 );
	static void getVerProp(
		SUBSPACECTRVER& ctrver,
		int* fjpt, int* fjptmark, float* projpt, int fjptnum,
		int* fseam, int fseamnum,
		int* fsheeti, int fsheetinum,
		int subedgenum,
		float*majpt, int* maseam, int maseamnum, int** maseamonsheet, int* maseamonsheetnum
		);
	static void markVer(
		SSPCTRVERVEC& sspctrver_vec,
		int* fjpt, int* fjptmark, float* projpt, int fjptnum,
		int* fseam, int fseamnum,
		int* fsheeti, int fsheetinum,
		int subvernum,int subedgenum,
		float*majpt, int* maseam, int maseamnum, int** maseamonsheet, int* maseamonsheetnum,
		 int* ver2jpt, int* ver2wver, int* edge2wedge);
	static bool getEdgeProp(
		SSPCTRVERVEC& sspctrver_vec,
		SUBSPACECTREDGE& ctredge,
		int* fjpt,
		int* fseam,
		int* fsheet,
		int* jpt2seamnum,
		int** jpt2seam,
		int* jpt2sheetnum,
		int** jpt2sheet,
		int* seam2sheetnum,
		int** seam2sheet );
	static void markEdge(SSPCTREDGEVEC& sspctredge_vec, SSPCTRVERVEC& sspctrver_vec,
		int subedgenum,int maseamnum,int* edge2wedge,int* fjpt,int* fseam,int* fsheet,int* jpt2seamnum,
		int** jpt2seam,int* jpt2sheetnum,int** jpt2sheet,int* seam2sheetnum,int** seam2sheet,intvector& needsplitedge);
	static void processSplitEdge(
		intvector& needsplitedge,
		SSPCTRVERVEC& sspctrver_vec,
		SSPCTREDGEVEC& sspctredge_vec,
		int* fjpt,float* projpt, int fjptnum,
		int* fseam, int fseamnum,
		int* fsheet,
		int subedgenum, 
		int* maseam, int maseamnum,
		int* jpt2seamnum,
		int** jpt2seam,
		int* jpt2sheetnum,
		int** jpt2sheet,
		int* seam2sheetnum ,
		int** seam2sheet,
		float param[ 4 ]);
	static void divideOneFaceContour(
		int spacei,
		int facei,
		//contour
		SSPCTRVERVEC& sspctrver_vec,
		SSPCTREDGEVEC& sspctredge_vec,
		//ma
		int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
		float* subparam,int* subver2wver,int* subedge2wedge,int majptnum,float* majpt,int maseamnum,
		int* maseam,MapArraySR& doubleface2sheet,int* maseamonsheetnum,int** maseamonsheet,int* ver2jpt
		);

	static void writeGatheredCtr_DB(vector<SSPCTRVERVEC>& sspctrver_vec,vector<SSPCTREDGEVEC>& sspctredge_vec, int spacei,
		int* ssspacefacenum);
	static void writeTopJptSeamSheet_DB(float* majpt,int* maseam,
		int* fjpt, int* fjptmark, int fjptnum,int* fseam,int fseamnum,	int* fsheet, int fsheetnum,
		int** jpt2seam, int* jpt2seamnum,int** jpt2sheet, int* jpt2sheetnum,int** seam2sheet, int * seam2sheetnum,int spacei, int facei);
	SSContour(){}

	~SSContour(){}
};

#endif