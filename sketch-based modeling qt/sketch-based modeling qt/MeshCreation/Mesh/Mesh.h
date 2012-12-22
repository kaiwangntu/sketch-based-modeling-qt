#ifndef _MESH_H
#define  _MESH_H

#include "../config.h"
#include "../PLYHandler/PLYWriter.h"
//#include <FL/fl_ask.H>
//#include "./util/mymath.h"
#include "../Math/mymath.h"
#include "../util/HashMap.h"
#include <vector>
//#include <FL/gl.h>
#include <GL/glu.h>
//#include "Common/glut.h"

//const float cols[18][3] =	{{255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 255, 255}, 	{0, 0, 255}, {255, 0, 255}, 
//{128, 	255, 0}, {255, 128, 0}, {0, 128, 255}, {128, 0, 255}, {255, 128, 128}, {128, 128, 255}, 
//{243, 117, 76}, {127, 145, 110}, {128, 23, 82}, {79, 184, 196}, {102,2, 102}, {200, 200, 200}};
const float cols[18][3] =	{ {79, 184, 196},{255, 0, 0}, {0, 255, 0}, {0, 255, 255}, 	{0, 0, 255}, {255, 0, 255}, 
{128, 	255, 0}, {255, 128, 0}, {0, 128, 255}, {128, 0, 255}, {255, 128, 128}, {128, 128, 255}, 
{243, 117, 76}, {127, 145, 110}, {128, 23, 82}, {249, 249, 19},//gold
{102,2, 102}, {240, 240, 240}};

class Mesh
{
public:
	//data
	//contour
	float center[3];
	float unitlen;
	int ctrplanenum;			//how many planes
	float* ctrplaneparam;		//plane parameter
	float** ctrver;			//vertices positions for different planes
	int* ctrvernum;			//vertex number on each plane
	int** ctredge;				//edge of each plane
	int* ctredgenum;			//edge number on each plane*/
	int* ctrtrinum;				//for every plane, how many triangles there are
	int** ctrtriconfig;			//for each plane, the triangulation configuration

	//mesh
	int sufvernum;
	int suffacenum;
	float* sufver;
	int* sufface;
	float* suffacenorm;
	int* sufmat;
	int sufctredgenum;
	int* sufctredge;
	vector<int> matlist;	
	unsigned int curmat;

	//render
	//render color
	float fcols[18][3];
	//render mode
	bool smoothshading;
	bool wireframe;
	//render option
	bool showContour;
	bool showMesh;
	bool showAll;	//true - show all materials false - show the specified one
	bool flipOut;	//true - inside out, false - as it is
	bool showOutline;	//true - show edge  false - not show edge

	//opengl parameters
	int width;
	int height;
	//	int nearplane;
	//	int farplane;
	float nearplane;
	float farplane;

public:
	vector<intvector> verneighbors;
	vector<int> debugpts;	/*-----------*/
	int curdebugpt;
	void movedebugpt(){
		curdebugpt = (curdebugpt+1)%sufvernum;
		debugpts.clear();
		debugpts.push_back( curdebugpt );
		for( unsigned int i = 0; i < verneighbors[curdebugpt].size(); i ++)
		{
			debugpts.push_back( verneighbors[curdebugpt][i]);
		}
	}
	//	vector<int> dbnmedgelist;
	void writeMeshJu(const char* fname);

	Mesh();
	Mesh( floatvector mver, intvector mface, intvector ctrmedge, 
		const float center2[ 3 ], const float unitlen2, const float PROCSIZE );

	//set opengl param
	void setGLParam(int width, int height, int nearplane, int farplane);

	//read files
	bool readMesh(const char* fname);
	void writeMeshSuf(const char* fname);
	void writeMeshPly(const char* fname);
	bool readContour(const char* fname);			//read contour from file

	//splitting/refinement
	/*------debug vector , to save them and show them ------*/
	/*vector<int>dbedgelist;
	vector<int>dbnmedgelist;
	vector<int>dbedge2facelist;
	void renderDBStuff();*/
	/*--------------*/
	//	vector<int> edgelist;
	/*vector<intvector> edge2facelist;
	vector<int> ctredgelist;*/
	//	vector<int>	nmedgelist;
	/*vector<int> normedgelist;
	vector<float> edgelenlist;
	vector<int> edgetypelist;*/
	/*--------------*/
	void gatherEdgeInfo(vector<int>& edgelist,vector<intvector>& edge2facelist,
		HashMap& ver2edgehash,vector<int>& ctredgelist,vector<int>&	nmedgelist,
		vector<int>& normedgelist,vector<float>& edgelenlist);
	//old one, no vermark
	void gatherVerInfo(vector<float>& verlist,vector<float>&verattrlist, vector<int>&edgelist, vector<float>&edgelenlist,
		vector<int>&nmedgelist, vector<int>&normedgelist, vector<int>&ctredgelist );
	//new one, with vermark
	void gatherVerInfo(vector<float>& verlist,vector<float>&verattrlist, vector<int>&vermark, vector<int>&edgelist, vector<float>&edgelenlist,
		vector<int>&nmedgelist, vector<int>&normedgelist, vector<int>&ctredgelist );
	void gatherFaceInfo(vector<int>& facelist);
	void splitOneNMEdge(int segnum, int i, vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
		vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist, HashMap& ver2edgehash);
	void splitNMEdge(vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
		vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist,		HashMap& ver2edgehash);
	void splitOneNormEdge(int segnum, int i, vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
		vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist, HashMap& ver2edgehash);
	//split non contour edge with two contour vertices
	void splitNCtrEdgeTwoCtrVer(vector<float>& verlist,vector<float>& verattrlist,vector<int>&vermark, vector<int>& edgelist,vector<intvector>& edge2facelist,
		vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist,		HashMap& ver2edgehash, const int oldedgenum);
	void swapEdge(vector<float>&verlist,vector<int>& edgelist,vector<intvector>& edge2facelist,vector<int>&normedgelist, vector<int>&facelist,HashMap& ver2edgehash);
	void getEdgeTypeList(vector<int>&normedgelist, vector<int>&nmedgelist, vector<int>&ctredgelist,vector<int>&edgetypelist);
	bool splitTriangle(vector<float>&verlist, vector<float>&verattrilist, HashMap& ver2edgehash2,
		vector<int>&edgelist, vector<int>&normedgelist, vector<intvector>&edge2facelist,vector<int>&facelist, float alpha);
	void LiepaRefine(float alpha);

	//smoothing/fair
	bool interpolate;	//true interpolate false - approximate
	void toggleInterpolate(bool val){ interpolate = val; }
	/*-----------------*/
	//vector<int>vermark;
	/*-----------------*/

	void LaplacianSmooth(float ratio, int times);

	void gatherInfoForFair(vector<int>& vermark, vector<intvector>& verneighbr,HashMap& ver2edgehash, vector<int>&edgelist);
	void inline computeWeightList(vector<int>&edgelist, vector<float>& wlist);
	void inline computeDiffer(vector<float>& oldval, vector<float>& differ, vector<float>& wlist, vector<intvector>&verneighbr, HashMap& ver2edgehash);
	void inline computeDiffer(float* oldval, float* differ, vector<float>& wlist,vector<intvector>&verneighbr, HashMap& ver2edgehash);
	void inline computeDiffer(float* oldval, float* differ, vector<float>& wlist, vector<int>&vermark,	vector<intvector>&verneighbr, HashMap& ver2edgehash);
	void inline refreshByDiffer(float* oldval, float* newval, float* differ, float ratio);
	void inline computeRefreshWeightList(vector<intvector>&verneighbr, vector<float>& w2list);
	void inline SDUmbraFair(float* oldver, float* newver, float* oldfirstdiffer, float* newfirstdiffer, 
		float* seconddiffer, vector<int>&vermark, vector<intvector>& verneighbr,HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist);
	void SDUmbraFair(int times);//scale - dependent umbrella operator fair algorithm

	//	void gatherInfoForJuFair();
	void inline JUFairCenter(float ratio, float* oldver, float* newver, float* oldfirstdiffer, float* newfirstdiffer, 
		float* seconddiffer, vector<int>&vermark, vector<intvector>& verneighbr,HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist, float* norms, float* mags);
	void inline computeNorm(float* oldval, float* norms, float* mags);
	void JUFair(float ratio, int times);		//Ju's fair algorithm

	//the laplacian at each vertex is the averaged laplacian of all its neighbors'
	void AverageSmooth(float ratio, int times);
	//taubin lambda mu smooth
	void TaubinSmooth(float fLambda,float fMu,int times);
	//smoothing algorithm of FiberMesh,written by kw
	void inline FBFairCenter(float ratio, float* oldver, float* newver, float* oldfirstdiffer, float* newfirstdiffer, 
		float* seconddiffer, vector<int>&vermark, vector<intvector>& verneighbr,HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist, float* norms, float* mags);
	void FBFair(float ratio, int times);		

	// liepa refinement <-> sdumbrafair
	void splitsmooth(float alpha0, float alphan, int times, int stimes);
	//liepat refinement <-> jufair
	void splitsmooth2(float alpha0, float alphan, int times, int stimes, float ratio);
	//liepa refinement <-> FiberMesh smooth,written by kw
	void splitsmooth3(float alpha0, float alphan, int times, int stimes, float ratio);

	//load standard mesh and compare
	float* stdverlist;
	int* stdfacelist;
	int stdvernum;
	int stdfacenum;
	void loadStd( const char* fname);
	void compareDiff( const char* fname);
	bool showStdMesh;
	void toggleShowStdMesh( bool val ){ showStdMesh = val;}
	void renderStdMesh( );

	//render
	void renderContour();
	void renderMesh();
	void renderMeshAll();
	void renderMeshCurComponent();
	void render();	
	//select
	void renderSelectableShapes();
	vector<int> selectVerList;		//for the selected vertices
	void selectVert(int verindex){selectVerList.push_back(verindex);}
	void clearLastSelectVert(){		if( selectVerList.size() > 0 )	selectVerList.pop_back(); }
	void clearAllSelectVert(){selectVerList.clear();}
	void renderSelectedVer();

	//render option
	void toggleWireframe(bool wireframe);
	void toggleShading();
	void toggelShowContour();
	void toggleShowMesh();
	void toggleShowAll(bool showAll);
	void toggleNext();
	void toggleFlipOut();
	void toggleShowOutline(int showoutline );
	~Mesh();
};

#endif