#ifndef _SPACEPARTITIONER_H
#define _SPACEPARTITIONER_H

#include "../config.h"
#include "../Util/VerEdgePlaneOp.h"

class SpacePartitioner
{
public:
	SpacePartitioner();
	~SpacePartitioner();

    void inline getVerMark( float planeparam[ 4 ],
		floatvector& ssver, int*& vermark);
	void inline getEdgeMark( float planeparam[ 4 ],
		floatvector& ssver, int*& vermark,
		intvector& ssedge, int*& edgenewver, int*& edgenewedge);
	void inline getFaceMark(int*& vermark,int*& edgenewver, int*& edgenewedge,intvector& ssedge,
		vector<intvector>&ssface, intvector& ssface_planeindex,
		int*& facenewedge, int*&facenewface );
	void processSubspace(int planei,intvector& ssedge,vector<intvector>& ssface, intvector& ssface_planeindex,vector<intvector>&ssspace, vector<intvector>& ssspace_planeside,
		int*& vermark,		int*& facenewedge, int*& facenewface	);
	void insertOnePlane(float planeparam[ 4 ], int planei,
		floatvector& ssver, intvector& ssedge,
		vector<intvector>&ssface, intvector& ssface_planeindex, 
		vector<intvector>& ssspace, vector<intvector>& ssspace_planeside);
	void partition(const int planenum, float*& param,const float boundingbox[ 6 ], const float enlarge[ 3 ],
		floatvector& ssver, intvector& ssedge, vector<intvector>&ssface, intvector& ssface_planeindex, 
		vector<intvector>& ssspace, vector<intvector>& ssspace_planeside);	

	//for cutting planes sharing with common line case
	 inline void reorganizeCMNL
		(
		int& comnedge,
		vector<intvector>&ssface, intvector& ssface_planeindex, 
		vector<intvector>& ssspace, vector<intvector>& ssspace_planeside
		);

	inline void mapOneSubspaceCMNL
		(  int spacei, intvector& ssedge,int& comnedge,
		vector<intvector>&ssface, vector<intvector>& ssspace,
		//reset the edges in the face, index starts from 0 and condensed
		vector<intvector>& nssface,  intvector& oedgelist,
		intvector& nssedge, intvector&overlist, //reset vertex in edgelist
		int& ncomnedge
		);
	inline void markVerCMNL
		(
		float param[ 4 ],	
		floatvector& ssver, intvector& overlist,
		intvector& vermark);

	inline void splitEdgeCMNL
		(
		float param[ 4 ],		//parameter of the plane

		floatvector& ssver,		//vertices positions
		intvector& ssedge,			//the original edge list
		intvector& oedgelist,		//the edges in current subspace
		intvector& nssedge,		//the edges composed of new vertices index
		intvector& overlist,		//vertices that are in current subspace
		intvector& vermark,			//vertex mark of the vertices

		intvector& edgetype,		//mark the edge type of the old edge list
		intvector& edgeval			//mark the edge value of the old edge list
		);

	inline void splitFaceCMNL
		(
		//int planei,
		int spacei, vector<intvector>& ssspace, 
		vector<intvector>& ssface,	//all the faces
		intvector& ssface_planeindex,
		vector<intvector>& nssface,
		intvector& ssedge,
		intvector& oedgelist, 
		intvector& edgemark, intvector& edgeval,

		intvector& facemark, intvector& faceval
		);

	void splitSubspaceCMNL
		(
		int planei,
		int spacei,
		bool above,		//true - push the half one above the plane at the end, false- the one below at the end
		vector<intvector>& ssspace,  
		vector<intvector>& ssspace_planeside,
		vector<intvector>& ssface,
		intvector& ssface_planeindex,
		intvector& facemark, intvector& faceval
		);
	void intersectPlaneSubspaceCMNL
		(
		int planei,
		float param[ 4 ],		//the parameter of the plane
		bool above,				//true intersect with the subspace above previous cutting plane, false - below
		floatvector& ssver, intvector& ssedge,  int& comnedge,
		vector<intvector>&ssface, intvector& ssface_planeindex, 
		vector<intvector>& ssspace, vector<intvector>& ssspace_planeside
		);


	void partition_ComnLine
		(
		const int planenum, float*& param,
		const float boundingbox[ 6 ], const float enlarge[ 3 ],
		floatvector& ssver, intvector& ssedge,  int& comnedge,
		vector<intvector>&ssface, intvector& ssface_planeindex, 
		vector<intvector>& ssspace, vector<intvector>& ssspace_planeside,
		float comndir[ 3 ], float comnpt[ 3 ]  //common line of the cutting planes
		);

	void wfilePartition(const char* fname,int ssvernum,float* ssver,int ssedgenum,int* ssedge,int ssfacenum,
	int* ssfaceedgenum,	int** ssface,int* ssface_planeindex,int ssspacenum,int* ssspacefacenum,	int** ssspace,	int** ssspace_planeside);

	void writemapOneSubspaceCMNL(int spacei, 
		intvector& ssedge,  int comnedge, 
		vector<intvector>& ssface,  vector<intvector>& ssspace,
		vector<intvector>& nssface, 		intvector& oedgelist,	
		intvector& nssedge, intvector& overlist,	
		int ncomnedge);	
	void writeOVerOEdgeList(intvector& overlist, intvector& oedgeslist);
	void writeOneIntvector(intvector& vec,  FILE* fout );
};

#endif