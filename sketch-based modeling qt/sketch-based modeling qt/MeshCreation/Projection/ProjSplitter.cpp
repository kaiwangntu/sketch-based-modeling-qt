#include "../Projection/ProjSplitter.h"

//////////////////////////////////////////////////////////////////////////
//int tempsheetnum;
//////////////////////////////////////////////////////////////////////////

//split all the edges lying on one seam
//split the mesh and contour edges if necessary
//return new seam edge registration
void ProjSplitter::splitProjEdgeSeam_One(
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
									float* majpt, int jpt2[ 2 ],
									int seami )
{
	int regvernum = seamVerReg.size();

	//step1. sort all the regisered vertices and finally add the two junction points on this seam
	//compute parameters of all the vertices 
	float* param = new float[ regvernum ];
	int maxdirec = getMajorDirec( majpt + jpt2[0]*3, majpt + jpt2[ 1 ]*3 );
	float maxlen = majpt[ 3*jpt2[ 1 ] + maxdirec ] - majpt[ 3*jpt2[ 0 ] + maxdirec ];
	for( int i = 0; i < regvernum; i ++ )
	{
		param[ i ] = (meshVer[ 3*seamVerReg[ i ] + maxdirec ] - majpt[ 3*jpt2[ 0 ] + maxdirec ])/maxlen;
	}
	for(unsigned int i = 1; i <= seamVerReg.size(); i ++ )
	{
		nseamReg.verPosInMeshVer[ i ] = seamVerReg[ i - 1 ];
	}
	//sort the vertex out by parameters of them, select sort
	for( int i = 0; i < regvernum; i ++ )
	{
		int mini = i;
		//find the minimum of the rest data
		for( int j = i+1; j < regvernum; j ++)
		{
			if( param [ j ] < param[ mini ] )
				mini = j;
		}
		if( mini == i )
			continue;

		//change the parameter and change the vertex position of them
		float t = param[ mini ];
		param[ mini ] = param[ i ];
		param[ i ] = t;
		int tver = nseamReg.verPosInMeshVer[ mini + 1 ];
		nseamReg.verPosInMeshVer[ mini + 1 ] = nseamReg.verPosInMeshVer[ i + 1 ];
		nseamReg.verPosInMeshVer[ i + 1 ] = tver;
	}
	//append the vertex at first junction point, and vertex at the last junction point, if not exist, -1
//	nseamReg.verPosInMeshVer[ 0 ] = jptreg[ jpt2[ 0 ]];
//	nseamReg.verPosInMeshVer[ regvernum + 1] = jptreg[ jpt2[ 1 ]];
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"all the"<<nseamReg.vernum<<" vertices on this seam;";
	for( int i = 0; i < regvernum + 2; i ++ )
	{
		cout<<nseamReg.verPosInMeshVer[i ]<<"  ";
	}
	cout<<endl;
	cout<<"all the"<<seamEdgeReg.size()<<" seams on this seam, position in meshedge" ;
	for( int i = 0; i < seamEdgeReg.size(); i ++ )
	{
		cout<<seamEdgeReg[ i ].posInMeshEdge<<" ";
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//step2. initialize the newseamregistration information
	for( int i = 0; i < regvernum + 1; i ++ )
	{
		nseamReg.subEdgeIsIn[ i ] = 0;	//all of them have not been marked yet, assume not exist at the beginning.
		nseamReg.edgelist->posInMeshEdge = -1;	//set it to be -1, not known yet, set later
	}

	//step3. go through all the registered edges on the seams, and split the contour edges if necessary
	int medgei;		//edge index in mesh edge
	int mveris[ 2 ];	//vertices indices in the mesh vertex
	int sveris[ 2 ];
	for(unsigned int ei = 0; ei < seamEdgeReg.size(); ei ++ )
	{
		medgei = seamEdgeReg[ ei ].posInMeshEdge;
		mveris[ 0 ] = meshEdge[ 2*medgei ];
		mveris[ 1 ] = meshEdge[ 2*medgei + 1 ];
		//find the two vertices in the sorted vertex list
		sveris[ 0 ] = sveris[ 1 ] = -1;
		for( int i = 0; i < regvernum + 2; i ++ )
		{
			if(( sveris[ 0 ] == -1) && (nseamReg.verPosInMeshVer[ i ] == mveris[ 0 ]) )
				sveris[ 0 ] = i;
			if(( sveris[ 1 ] == -1) && (nseamReg.verPosInMeshVer[ i ] == mveris[ 1 ]) )
				sveris[ 1 ] = i;
			if( (sveris[ 0 ] != -1) && ( sveris[ 1 ] != -1 ) )
				break;
		}

		//reverse the edge if necessary
		if( sveris[ 0 ] > sveris[ 1 ])
		{
			int t = sveris[ 0 ];
			sveris[ 0 ] = sveris[ 1 ];
			sveris[ 1 ] = t;

			//change the direction of all the corresponding edges
			for(unsigned int i = 0; i < seamEdgeReg[ ei ].crspCtrEdges.size(); i ++ )
			{
				seamEdgeReg[ ei ].crspCtrEdges[ i ].samedirec = !(seamEdgeReg[ ei ].crspCtrEdges[ i ].samedirec);
			}
		}

		//mark the subedge if it is in.
		for( int i = sveris[ 0 ]; i < sveris[ 1 ]; i ++ )
		{
			nseamReg.subEdgeIsIn[ i ] = 1;	//all of them have not been marked yet, assume not exist at the beginning.
		}

		//push all the crspCtrEdges into the nseamReg's crspCtrEdges
		for(unsigned int i = 0; i < seamEdgeReg[ ei ].crspCtrEdges.size(); i ++ )
		{
			nseamReg.edgelist[ sveris[ 0 ]].crspCtrEdges.push_back( seamEdgeReg[ ei ].crspCtrEdges[ i ]);
		}
		
		//split the edge if necessarry
		if( (sveris[ 0 ] + 1) == sveris[ 1 ]) //no need to split the corresponding vertices
		{			
			continue;
		}
		//need splitting
        //refresh: contour vertex, contour edge, and corresponding edges
		//refresh corresponding edge in new seam registration
		int k = 0;
		for( int i = sveris[ 0 ] + 1; i < sveris[ 1 ]; i ++ )
		{
			for(unsigned int j = 0;j < seamEdgeReg[ ei ].crspCtrEdges.size(); j ++ )
			{
				correspnCtrEdge tmp;
				memcpy(&tmp, &(seamEdgeReg[ ei ].crspCtrEdges[ j ]), sizeof(EdgeRegItem));
			//	tmp.facei = seamEdgeReg[ ei ].crspCtrEdges[ j ].facei;
				tmp.edgei = sspctredge_vec[ tmp.facei ].size() - 1 + k;
			//	tmp.samedirec = seamEdgeReg[ ei ].crspCtrEdges[ j ].samedirec;
				nseamReg.edgelist[ i ].crspCtrEdges.push_back( tmp );
			}
			k ++;
		}

		//split all the contour edges in corresponding edges
		int tvernum = sveris[ 1 ] - sveris[ 0 ] + 1;
		float paramdiff = param[ sveris[ 1 ] - 1] - param[ sveris[ 0 ] - 1] ;
		float param0 = param[ sveris[ 0 ] - 1 ];
		
		for(unsigned int i = 0; i < seamEdgeReg[ ei ].crspCtrEdges.size(); i ++ )
		{
			int facei = seamEdgeReg[ ei ].crspCtrEdges[ i ].facei;
			int edgei = seamEdgeReg[ ei ].crspCtrEdges[ i ].edgei;
			int direc = seamEdgeReg[ ei ].crspCtrEdges[ i ].samedirec;

			int* pedge = sspctredge_vec[ facei ][ edgei ].veris;

		//	int& vertype = sspctrver_vec[ facei ][ pedge[ 0 ]].type;
		//	int& verval = sspctrver_vec[ facei ][ pedge[ 0 ]].val;
		//	int& edgetype = sspctredge_vec[ facei ][ edgei ].type;
		//	int& edgeval = sspctredge_vec[ facei ][ edgei ].val;
		//	int* edgemat = sspctredge_vec[ facei ][ edgei ].mat;
		//	int& edgeancestor = sspctredge_vec[ facei ][ edgei ].ancestor;
			
			float* pt1;
			float* pt2;
			pt1 = sspctrver_vec[ facei ][ pedge[ 0 ]].pos;
			pt2 = sspctrver_vec[ facei ][ pedge[ 1 ]].pos;
			int oldvernum = sspctrver_vec[ facei ].size();
			//refresh contour vertex
			for( int j = 0; j < tvernum - 2; j ++)
			{
				SUBSPACECTRVER nver;
				MyMath::getPtOnSeg(pt1, pt2, (param[ j + sveris[ 0 ]] - param0 )/paramdiff, nver.pos );
				nver.type = VER_SEAM;
				nver.val = seami;
				//nver.type = vertype;
				//nver.val = verval;
				sspctrver_vec[ facei ].push_back( nver );
			}

			//refresh contour edge
			//add new ones, change the old one
			if( direc )	//v0 oldvernum, oldvernum+1,...v1
			{
				//new edges
				for( int j = 0; j < tvernum - 3; j ++ )
				{
					SUBSPACECTREDGE tmp;
					memcpy( &tmp, &sspctredge_vec[ facei ][ edgei ], sizeof( SUBSPACECTREDGE ));
					tmp.veris[ 0 ] = oldvernum + j;
					tmp.veris[ 1 ] = oldvernum + j + 1;
					/*tmp.type = edgetype;
					tmp.val = edgeval;
					tmp.mat[ 0 ] = edgemat[ 0 ]; tmp.mat[ 1 ] = edgemat[ 1 ];
					tmp.ancestor = edgeancestor;*/
					sspctredge_vec[ facei ].push_back( tmp );
				}
				//the last new edge
				SUBSPACECTREDGE tmp;
				memcpy( &tmp, &sspctredge_vec[ facei ][ edgei ], sizeof( SUBSPACECTREDGE ));
				tmp.veris[ 0 ] = oldvernum + tvernum - 3;
				sspctredge_vec[ facei ].push_back( tmp );
				//the old edge
				sspctredge_vec[ facei ][ edgei ].veris[ 1 ] = oldvernum;
			}
			else		//v1 oldvernum, oldvernum+1,...v0
			{
				//new edges
				for( int j = 0; j < tvernum - 3; j ++ )
				{
					SUBSPACECTREDGE tmp;
					memcpy( &tmp, &sspctredge_vec[ facei ][ edgei ], sizeof( SUBSPACECTREDGE ));
					tmp.veris[ 1 ] = oldvernum + j;	//since direc is set to be false
					tmp.veris[ 0 ] = oldvernum + j + 1;
					/*tmp.type = edgetype;
					tmp.val = edgeval;
					tmp.mat[ 0 ] = edgemat[ 0 ]; tmp.mat[ 1 ] = edgemat[ 1 ];
					tmp.ancestor = edgeancestor;*/
					sspctredge_vec[ facei ].push_back( tmp );
				}
				//the last new edge
				SUBSPACECTREDGE tmp;
				memcpy( &tmp, &sspctredge_vec[ facei ][ edgei ], sizeof( SUBSPACECTREDGE ));
				tmp.veris[ 1 ] = oldvernum + tvernum - 3;
				tmp.veris[ 0 ] = pedge[ 0 ];
				sspctredge_vec[ facei ].push_back( tmp );

				//the old edge
				sspctredge_vec[ facei ][ edgei ].veris[ 0 ] = oldvernum;
			}	
			pedge = NULL;
		//	edgemat = NULL;
			pt1 = NULL;
			pt2 = NULL;
		}
	}

	//step4. refresh mesh edges and set the edge index for nseamreg
	int subedgenum = nseamReg.vernum - 1;
	int oldseamedgenum = seamEdgeReg.size();
    for( int i = 0; i < oldseamedgenum; i ++ )	//replace the old by the new ones
	{
		meshEdge[ seamEdgeReg[ i ].posInMeshEdge * 2 ] = nseamReg.verPosInMeshVer[ i ];
		meshEdge[ seamEdgeReg[ i ].posInMeshEdge* 2 + 1 ] = nseamReg.verPosInMeshVer[ i + 1 ];
		nseamReg.edgelist[ i ].posInMeshEdge = seamEdgeReg[ i ].posInMeshEdge;
	}
	//add new ones
	//////////////////////////////////////////////////////////////////////////
	//cout<<"oldseamedgenum:"<<oldseamedgenum<<"subedgenum:"<<subedgenum<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = oldseamedgenum; i < subedgenum; i++ )
	{
		meshEdge.push_back( nseamReg.verPosInMeshVer[ i ] );
		meshEdge.push_back( nseamReg.verPosInMeshVer[ i + 1 ] );
		nseamReg.edgelist[ i ].posInMeshEdge = meshEdge.size()/2 - 1;
	}

	//clear the temp vars
	delete []param;
}


//split the edges on all the seams.
//call the above function to process them one by one
void ProjSplitter::splitProjEdgeSeam(
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
								//	intvector*& sheetVerReg,		//for each sheet, the projected vetices on it
								//	EdgeReg*& sheetEdgeReg,	
									int majptnum,int maseamnum,int masheetnum,
									int* maseam, float* majpt )
{
	for( int i = 0; i < maseamnum; i++ )
	{
		//if no subedge on this seam, set nseamreg
		int vernum = seamVerReg[ i ].size() + 2;
		nseamReg[ i ].vernum = vernum;
		nseamReg[ i ].subEdgeIsIn = new bool[ vernum - 1 ];
		nseamReg[ i ].verPosInMeshVer = new int[ vernum ];
		nseamReg[ i ].edgelist = new EdgeRegItem[ vernum - 1 ];
		nseamReg[ i ].verPosInMeshVer[ 0 ] = jptReg[ maseam[ 2*i ]];
		nseamReg[ i ].verPosInMeshVer[ vernum - 1 ] = jptReg[ maseam[ 2*i + 1 ]];
		//if( (seamEdgeReg[ i ].size() == 0) && ( seamVerReg[ i ].size() == 0) )	//no edge on this seam
		//{
		//	nseamReg[ i ].subEdgeIsIn[ 0 ] = false;
		//	nseamReg[ i ].edgelist[ 0 ].posInMeshEdge = -1;
		//	continue;
		//}
		//if( seamEdgeReg[ i ].size() == 0 )	//no edge on this seam
		//{
		//	nseamReg[ i ].subEdgeIsIn[ 0 ] = false;
		//	nseamReg[ i ].edgelist[ 0 ].posInMeshEdge = -1;
		//	continue;
		//}
		/*if( seamVerReg[ i ].size() == 0 )
		{			
			nseamReg[ i ].subEdgeIsIn[ 0 ] = false;
			nseamReg[ i ].verPosInMeshVer[ 0 ] = nseamReg[ i ].verPosInMeshVer[ 1 ] = -1;
			nseamReg[ i ].edgelist[ 0 ].posInMeshEdge = -1;
			continue;
		}*/
		// subedge exists on this seam, call function to process it.
		splitProjEdgeSeam_One(
			meshVer, meshEdge, nseamReg[ i ],sspctrver_vec, sspctredge_vec,
			jptReg,		seamVerReg[ i ], seamEdgeReg[ i ],
			majptnum, maseamnum,  masheetnum, majpt, maseam + 2*i, i);
		//clear the old seam registration
		seamVerReg[ i ].clear();
		for(unsigned int j = 0 ; j < seamEdgeReg[ i ].size(); j ++ )
		{
			seamEdgeReg[ i ][ j ].crspCtrEdges.clear() ;
		}
		seamEdgeReg[ i ].clear();
	}
	delete []seamVerReg;
	seamVerReg = NULL;
	delete []seamEdgeReg;
	seamEdgeReg = NULL;
}

const int NO_INTERPT = 0;	//no intersection point exists
const int NORMAL_INTERPT = 1;	//the two segments intersect, and at some middle places of the two
const int OTHER_INTERPT = 2;	//all other cases
//true, intersection exists
//false, intersection not exist

//intersect two edges. 
//the result are in the last 6 vars
//include subedges, parameters, and new intersection points
int ProjSplitter::edgeIntersect(
	float vers[ 12 ],	//position of the four vertices
	int veris[ 4 ],		//position of the four end points in meshVer
	intvector& subedgelist1,	//end points of all the subedges of edge1 after intersection
	floatvector& paramlist1,	//the parameter of the second point relative to original edge
	intvector& subedgelist2,	//similar
	floatvector& paramlist2,
	bool newptexist,	//if the intersection point between the two segments exist or not
	float newpt[ 3 ]	//if exist, the position is saved in it
	)
{
	//case1. the two edges share one common vertex
	//case1.1 not the same direction
	//case1.2 the same direction
	//case2. the two edges do not share common vertex
	//case2.1 they lie on the same line
	//case2.2 they do not lie on the same line
	//case2.2.1 v3 lies on v1 v2
	//case2.2.2 v4 lies on v1 v2
	//case2.2.3 v1 lies on v3v4
	//case2.2.4 v2 lies on v3v4

	//bounding box test first
	float box1[ 6 ], box2[ 6 ];
	for( int i = 0; i < 3; i ++ )
	{
		if( vers[ i ] < vers[ i + 3 ])
		{
			box1[ i ] = vers[ i ];
			box1[ i + 3 ] = vers[ i + 3 ];
		}
		else
		{
			box1[ i ] = vers[ i + 3 ];
			box1[ i + 3 ] = vers[ i ];
		}
		if( vers[ i + 6 ] < vers[ i + 9 ])
		{
			box2[ i ] = vers[ i + 6];
			box2[ i + 3 ] = vers[ i + 9 ];
		}
		else
		{
			box2[ i + 3 ] = vers[ i + 6 ];
			box2[ i ] = vers[ i + 9 ];
		}

		if( box1[ i + 3 ] + TOLERANCE_SAME_VER*2 < box2[ i ] )
			return NO_INTERPT;
		if( box2[ i + 3 ] + TOLERANCE_SAME_VER*2 < box1[ i ] )
			return NO_INTERPT;
	}	

	bool shareComnVer = true;
	//case1. the two edges share one common vertex
	int sameverpos[ 2 ];
	if(veris[ 0 ] == veris[ 2 ])
	{
		sameverpos[ 0 ] = 0;
		sameverpos[ 1 ] = 2;
	}
	else if( veris[ 0 ] == veris[ 3 ])
	{
		sameverpos[ 0 ] = 0;
		sameverpos[ 1 ] = 3;
	}
	else if( veris[ 1 ] == veris[ 2 ])
	{
		sameverpos[ 0 ] = 1;
		sameverpos[ 1 ] = 2;
	}
	else if( veris[ 1 ] == veris[ 3 ] )
	{
		sameverpos[ 0 ] = 1;
		sameverpos[ 1 ] = 3;
	}
	else	//no same vertices exist
		shareComnVer = false;
	if( shareComnVer )
	{
		int otherpos[ 2 ] = { 1 - sameverpos[ 0 ], 5 - sameverpos[ 1 ] };
		float dir[ 6 ];
		MyMath::getVec(vers + 3* sameverpos[ 0 ], vers + 3* otherpos[ 0 ], dir );
		MyMath::getVec(vers + 3* sameverpos[ 1 ], vers + 3* otherpos[ 1 ], dir + 3);
		//case1.1 not the same direction
		if( !isSameDir( dir, dir + 3 ))
			return NO_INTERPT;

		//case1.2 the same direction
		//compute the param of v3 on v1v2
		float param = getParamOnSeg(vers + 3*sameverpos[ 0 ], vers + 3*otherpos[ 0 ], vers + 3*otherpos[ 1 ]);
		//////////////////////////////////////////////////////////////////////////
		/*cout<<"verpos:"<<sameverpos[ 0 ] << "  " <<otherpos[ 0 ]
		<<sameverpos[ 1 ]<<otherpos[ 1 ]
		<<endl;
		cout<<"vers:"<<endl
			<<vers [3*sameverpos[ 0 ]]<<","<<vers [3*sameverpos[ 0 ] + 1]<<","<<vers [3*sameverpos[ 0 ] + 2]<<endl
			<<vers [3*otherpos[ 0 ]]<<","<<vers [3*otherpos[ 0 ] + 1]<<","<<vers [3*otherpos[ 0 ] + 2]<<endl
			<<vers [3*otherpos[ 1 ]]<<","<<vers [3*otherpos[ 1 ] + 1]<<","<<vers [3*otherpos[ 1 ] + 2]<<endl;
			cout<<"param:"<<param;*/
		//////////////////////////////////////////////////////////////////////////
		if( param < 1 && param > 0.999999 ) param = 0.999999;	//to avoid overlap
		if( param > 1 && param < 1.000001 ) param = 1.000001;
		if( param < 0.000001 ) param = 0.000001;

		//////////////////////////////////////////////////////////////////////////
		//cout<<"final param:"<<param<<endl;
		//////////////////////////////////////////////////////////////////////////
		
		// no new vertex
		newptexist = false;

		//param is smaller than 1
		if( param < 1 )	//v1v2 is longer
		{
			
			//divide the first edge
			subedgelist1.push_back( veris[ 0 ]);
			subedgelist1.push_back( veris[ otherpos[ 1 ] ] );
			subedgelist1.push_back( veris[ 1 ]);
			if( sameverpos[ 0 ] == 0 )
				paramlist1.push_back( param );
			else 
				paramlist1.push_back( 1- param );

			//the second edge no deviding
			subedgelist2.push_back( veris[ 2 ]);
			subedgelist2.push_back( veris[ 3 ]);

			/////////////////////////////////////////////////////////////////////////////
			//cout<<"first edge contains the second one:"<<endl;
			//cout<<"edge1: "<<veris[ 0 ]<<"\t"<<veris[ otherpos[ 1 ]]<<"\t"<<veris[ 1 ]<<endl;
			//cout<<"edge2: "<<veris[ 2 ] <<"\t"<<veris[ 3 ] <<endl;
			////////////////////////////////////////////////////////////////////////////
		}
		//param is larger than 1
		else
		{
			//first edge no dividing, divide the second edge
			subedgelist1.push_back( veris[ 0 ]);
			subedgelist1.push_back( veris[ 1 ]);
			subedgelist2.push_back( veris[ 2 ]);
			subedgelist2.push_back( veris[otherpos[ 0 ]]);
			subedgelist2.push_back( veris[ 3 ]);
            if( sameverpos[ 1 ] == 2 )
				paramlist2.push_back( 1/param );
			else
				paramlist2.push_back( 1 - 1/param);

			/////////////////////////////////////////////////////////////////////////////
			//cout<<"second edge contains the first one:"<<endl;
			//cout<<"edge1: "<<veris[ 0 ]<<"\t"<<veris[ 1] <<endl;
			//cout<<"edge2: "<<veris[ 2 ]<<"\t"<<veris[ otherpos[ 0 ]]<<veris[ 3 ] <<endl;
			////cout<<"edge 1"<<veris[ 0 ]<<"\t"<<veris[ otherpos[ 1 ]]<<"\t"<<veris[ 1 ]<<endl;
		//	cout<<"edge 2"<<veris[ 2 ] <<"\t"<<veris[ 3 ] <<endl;
			//////////////////////////////////////////////////////////////////////////
		}
		return OTHER_INTERPT;	//intersect without new intersection point
	}	

	bool onSameLine = false;
	bool v3Onv1v2 = false;
	bool v4Onv1v2 = false;
	//case2. the two edges do not share common vertex
	float dir[ 3 ];
	getDirOfTrian( vers, vers+3, vers+6, dir );
	float v1v2 [ 3 ];
	MyMath::getVec( vers, vers+3, v1v2 );
	float v1v2len = MyMath::vectorlen( v1v2 );
	if( MyMath::vectorlen( dir )/v1v2len < TOLERANCE_SAME_VER )
	{
		v3Onv1v2 = true;
	}
	getDirOfTrian( vers, vers+3, vers+9, dir );
	if( MyMath::vectorlen(dir)/v1v2len < TOLERANCE_SAME_VER )
	{
		v4Onv1v2 = true;
	}

	//case2.1 they lie on the same line
	if( v3Onv1v2 && v4Onv1v2 )
	{
		//compute the parameters of the four points
		float verparam[ 4 ] = {0,1,0,0};
		verparam[ 2 ] = getParamOnSeg( vers, vers+3,  vers+6 );
		verparam[ 3 ] = getParamOnSeg( vers, vers+3, vers+9 );
		//sort the param while changing the index 
		int ind[ 4 ] = {0, 1, 2, 3};
		//selecting sort algorithm
		for( int i = 0 ;i  < 4; i ++ )
		{
			int mini = i;
			for( int j = i ; j < 4; j ++ )
			{
				if( verparam[ j ] < verparam[ mini  ])
				{
					mini = j;
				}
			}
			if( mini == i )
				continue;
			int tind = ind[ mini ];
			ind[ mini ] = ind[ i ];
			ind[ i ] = tind;
			float tparam = verparam[ mini ];
			verparam[ mini ] = verparam[ i ];
			verparam[ i ] = tparam;
		}
		//split the two edges
		newptexist = false;
		//subedgelist1
		int endptpos1[ 2 ];
		int endptpos2[ 2 ];
		for( int i = 0; i < 4; i++)
		{
			if( ind[ i ] == 0 )
			{
				endptpos1[ 0 ] = i;
			}
			else if( ind[ i ] == 1 )
			{
				endptpos1[ 1 ] = i;
			}
			else if( ind[ i ] == 2) 
			{
				endptpos2[ 0 ] = i;
			}
			else if( ind[ i ] == 3 )
			{
				endptpos2[ 1 ] = i;
			}
		}
        for( int i = endptpos1[ 0 ]; i <= endptpos1[ 1 ]; i ++)
		{
			subedgelist1.push_back( veris[ ind[ i ]]);
		}
		for( int i = endptpos1[ 0 ] + 1; i < endptpos1[ 1 ]; i ++ )
		{
			paramlist1.push_back( (verparam[ i ]-verparam[ endptpos1[ 0 ]]) );
		}
		//subedgelist2
		float paramdiff = verparam[ endptpos2[ 1 ]] - verparam[ endptpos2[ 0 ]];
		int delta = 1;
		if( endptpos2[ 0 ] > endptpos2[ 1])
			delta = -1;
		for( int i = endptpos2[ 0 ]; i <= endptpos2[ 1 ]; i += delta)
		{
			subedgelist2.push_back( veris [ ind[ i ] ]);
		}
		for( int i = endptpos2[ 0 ] + delta; i != endptpos2[ 1 ]; i += delta )
			paramlist2.push_back( (verparam[ i ] - verparam[ endptpos2[ 0 ]])/paramdiff );
		return OTHER_INTERPT;
	}

	//case2.2 they do not lie on the same line
	//case2.2.1 v3 lies on v1 v2
	if( v3Onv1v2 )
	{
		//if v1v2 lies on different side of v3v4
		int mdirec = getMajorDirec( vers, vers+3);
		float tp =  (vers[ 6+mdirec ] - vers[ mdirec ])/( vers[ 3 + mdirec ] - vers[ mdirec ]);
		if( tp > 1 || tp < 0 )
			return NO_INTERPT;

		//subedgelist1
		subedgelist1.push_back( veris[ 0 ] );
		subedgelist1.push_back( veris[ 2 ]);
		subedgelist1.push_back( veris[ 1 ]);
		paramlist1.push_back( getParamOnSeg( vers, vers + 3, vers + 6 ));
		subedgelist2.push_back( veris[ 2 ]);
		subedgelist2.push_back( veris[ 3 ]);
		newptexist = false;
		return OTHER_INTERPT;
	}
	//case2.2.2 v4 lies on v1 v2
	if( v4Onv1v2 )
	{
		int mdirec = getMajorDirec( vers, vers+3);
		float tp =(vers[ 9+mdirec ] - vers[ mdirec ])/( vers[ 3 + mdirec ] - vers[ mdirec ]);

		if(tp > 1 || tp < 0 )
			return NO_INTERPT;

		//subedgelist1	
		subedgelist1.push_back( veris[ 0 ]);
		subedgelist1.push_back( veris[ 3 ]);
		subedgelist1.push_back( veris[ 1 ]);
		paramlist1.push_back( getParamOnSeg( vers, vers+3, vers + 9));

		//subedgelist2
		subedgelist2.push_back( veris[ 2 ]);
		subedgelist2.push_back( veris[ 3 ]);

		newptexist = false;
		return OTHER_INTERPT;
	}
	//case2.2.3 v1 lies on v3v4
	getDirOfTrian( vers+6, vers+9, vers, dir);
	float v3v4len = MyMath::vectorlen( vers + 6, vers + 9 );
	if( MyMath::vectorlen( dir )/v3v4len < TOLERANCE_SAME_VER )
	{
		//v1 v2 must lie on different sides of v3 v4
		int mdirec = getMajorDirec( vers + 6, vers+ 9 );
		float tp = (vers[ mdirec ] - vers[ mdirec + 6])/( vers[ 9 + mdirec ] - vers[ mdirec +6 ]);
		if ( tp > 1 || tp < 0 )
			return NO_INTERPT;

		//subedgelist1
		subedgelist1.push_back( veris[ 0 ]);
		subedgelist1.push_back( veris[ 1 ]);

		//subedgelist2
		subedgelist2.push_back( veris[ 2 ]);
		subedgelist2.push_back( veris[ 0 ]);
		subedgelist2.push_back( veris[ 3 ]);
		
		paramlist2.push_back( getParamOnSeg( vers + 6, vers + 9, vers ));

		newptexist = false;
		return OTHER_INTERPT;
	}

	//case2.2.4 v2 lies on v3v4
	getDirOfTrian( vers + 6, vers + 9, vers + 3, dir );
	if( MyMath::vectorlen( dir )/v3v4len < TOLERANCE_SAME_VER )
	{
		//v1 v2 must lie on different sides of v3 v4
		int mdirec = getMajorDirec( vers + 6, vers+9);
		float tp = (vers[ mdirec + 3] - vers[ mdirec + 6])/( vers[ 9 + mdirec ] - vers[ mdirec +6 ]);
		if( tp  > 1 || tp < 0 )
			return NO_INTERPT;

		//subedgelist1
		subedgelist1.push_back( veris[ 0 ]);
		subedgelist1.push_back( veris[ 1]);

		//subedgelist2
		subedgelist2.push_back( veris[ 2 ]);
		subedgelist2.push_back( veris[ 1 ]);
		subedgelist2.push_back( veris[ 3 ]);

		paramlist2.push_back( getParamOnSeg( vers + 6, vers + 9, vers + 3 ));

		newptexist = false;

		return OTHER_INTERPT;		
	}

	//case3 no commmon vertex, not on the same line
	//case3.1 end point of one edge should lie on different sides of the other edge
	float dir1[ 3 ];
	float dir2[ 3 ];
	getDirOfTrian( vers, vers+3, vers + 6, dir1);
	getDirOfTrian( vers, vers+3, vers + 9, dir2);
	if( MyMath::dotProduct( dir1, dir2 ) > 0 )
		return NO_INTERPT;
	int mdirec2 = getMajorDirec( dir1 );
	float param2 = dir1[ mdirec2 ]/( dir1[ mdirec2] - dir2[ mdirec2 ]);

	getDirOfTrian( vers+6, vers + 9, vers, dir1 );
	getDirOfTrian( vers+6, vers + 9, vers+ 3, dir2 );
	if ( MyMath::dotProduct( dir1, dir2 ) > 0 )
		return NO_INTERPT;

	//case3.2 intersects with each other, and with a new intersection point
	//compute intersection point
	newptexist = true;
	int mdirec = getMajorDirec( dir1 );
	float param =  dir1[ mdirec ]/( dir1[ mdirec] - dir2[ mdirec ]);
	MyMath::getPtOnSeg( vers, vers + 3, param, newpt );
//	float dist[ 2 ];
//	dist[ 0 ] = MyMath::vectorlen( dir1 );
//	dist[ 1 ] = MyMath::vectorlen( dir2 );
//	MyMath::getPtOnSeg( vers, vers+3, dist[ 0 ]/(dist[ 0 ] + dist[ 1]), newpt );

	//subedgelist1
	subedgelist1.push_back( veris[ 0 ]);
	subedgelist1.push_back( -1 );	//don't know the index yet
	subedgelist1.push_back( veris[ 1]);
	paramlist1.push_back( param );

	//subedgelist2
	subedgelist2.push_back( veris[ 2 ]);
	subedgelist2.push_back( -1 );
	subedgelist2.push_back( veris[ 3 ]);
	paramlist2.push_back( param2 );
    return NORMAL_INTERPT;
}



void ProjSplitter::splitCtrEdge(
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
	)
{
	SUBSPACECTRVER tver;
	tver.type = VER_SHEET;
	tver.val = sheeti;
	//	SUBSPACECTREDGE tedge;
	//	tedge.type = EDGE_SHEET;
	int tverlen;
	int tedgelen;
	int tnvernum;	//number of new contour vertices
	int tnedgenum;	//number of new contour edges
	int oldedgeiInCtr = edgeFacei[ pos1 ].posInCtrEdge;	//position of the edge in contour edge
	int* oldveriInCtr = sspctredge_vec[ facei ][ oldedgeiInCtr ].veris;
	float endpts[ 6 ];
	int ptnum;
	//split the first contour edge
	ptnum = subedgelist1.size();
	if( ptnum == 2 )	//no need to split
		return;
			
	tverlen = sspctrver_vec[ facei ].size();
	tnvernum = paramlist1.size();
	tnedgenum = paramlist1.size();
	sspctrver_vec[ facei ].resize( tverlen + tnvernum, tver );
	for( int i = 0; i < 3; i ++)
	{
	//	endpts[ i ] = meshVer[ subedgelist1[ 0 ] * 3 + i ];
	//	endpts[ i + 3 ] = meshVer[ subedgelist1[ ptnum - 1 ]*3 + i ];
		endpts[ i ] = sspctrver_vec[ facei ][ oldveriInCtr[ 0 ] ].pos[ i ];
		endpts[ i + 3 ] = sspctrver_vec[ facei ][ oldveriInCtr[ 1 ] ].pos[ i ];
	}
	oldveriInCtr = NULL;

	//refresh the contour vertex		
	for( int i = 0; i < tnvernum; i ++ )
	{
		//set the vertex position for the new vertex
		MyMath::getPtOnSeg( endpts, endpts + 3, paramlist1[ i ],
			sspctrver_vec[ facei ][ tverlen + i ].pos );
	}

	//refresh the contour edge
	tedgelen = sspctredge_vec[ facei ].size();
	
	SUBSPACECTREDGE tedge;
	memcpy(&tedge, &(sspctredge_vec[facei][ oldedgeiInCtr ]), sizeof( SUBSPACECTREDGE ));
	//////////////////////////////////////////////////////////////////////////
	//cout<<"old edge ancestor"<<sspctredge_vec[facei][ oldedgeiInCtr ].ancestor<<endl;
	//cout<<"the new ones:"<<tedge.ancestor<<endl;
	//////////////////////////////////////////////////////////////////////////

	sspctredge_vec[ facei ].resize( tedgelen + tnedgenum, tedge);		
	//new edge with two new vertices
	for( int i = 0; i < tnedgenum-1; i ++ )
	{
		sspctredge_vec[ facei ][ tedgelen + i ].veris[ 0 ] = tverlen + i;
		sspctredge_vec[ facei ][ tedgelen + i ].veris[ 1 ] = tverlen + i + 1;
		//////////////////////////////////////////////////////////////////////////
	//	cout<<"the actural added edges:"<<sspctredge_vec[ facei ][ tedgelen + i ].ancestor<<" ";
		//////////////////////////////////////////////////////////////////////////
	}
	//////////////////////////////////////////////////////////////////////////
	//cout<<endl;
	//////////////////////////////////////////////////////////////////////////

	//the last new edge
	sspctredge_vec[ facei ][ tedgelen + tnedgenum - 1].veris[ 0 ] = tverlen + tnedgenum - 1;
	sspctredge_vec[ facei ][ tedgelen + tnedgenum - 1].veris[ 1 ] = 
		sspctredge_vec[ facei ][ oldedgeiInCtr ].veris[ 1 ];
	//the old edge
	sspctredge_vec[ facei ][ oldedgeiInCtr ].veris[ 1 ] = tverlen;
}
//find in gatheredge, if repeated, return true
//if not repeated return false
bool ProjSplitter::isRepeat(
		 intvector& gatheredge,
		 int veris[ 2 ],
		 int cmplen,
		 int& repeati,
		 bool& samedirec
			 )
{
	for( int i = 0; i < cmplen; i++ )
	{
		if( ( gatheredge [ 4*i ] == veris[ 0 ]) && ( gatheredge [ 4*i + 1] == veris[ 1 ] ) )
		{
			repeati = i;
			samedirec = true;
			return true;
		}
		if(  ( gatheredge [ 4*i ] == veris[ 1 ]) && ( gatheredge [ 4*i + 1] == veris[ 0 ] ) )
		{
			repeati = i;
			samedirec = false;
			return true;
		}
	}

	return false;
}
//after computing intersection between two edges
//refresh (1) mesh, (2) contour (3) edgefacei edgefacej nsheetreg startedge(4) pos2
//return true, if the first edge is contained by the second edge, and it is get rid of
//next round, a new subedge from facei is going to intersect with all the subedges on facej
//return false, other case
void ProjSplitter::refreshAfterSegIntersect(
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
	int& pos2)
{
	//step1. according to the intersection result, split the contour edges
	//and save the subedge position in contour

	//step2. refresh other vars	
	//--case1. new intersection point exists
	//add new edges into mesh, and change the old ones
	//refresh edgefacei, edgefacej and return

	//--case2. overlap or share common vertex
	//availedge: the old two indices of edges in meshEdge
	//gatheredge: all the gathered subedges after intersection	
	// 5 is a group
	const int meshVerPos1 = 0;
	const int meshVerPos2 = 1;
	const int subEdgeType = 2;
	const int ctrEdgePos = 3;
//	const int meshEdgePos = 4;
	//subEdgeType
	const int SUBEDGE_OVERLAP = 0;
	const int SUBEDGE_FACEI = 1;
	const int SUBEDGE_FACEJ = 2;
	//memtab: the memorized same subedges in the two subedge list
	//memdir: if the overlapped edge has the same direction as it is in the first face

	int oldctredgelen1 = sspctredge_vec[ facei ].size();
	int oldctredgelen2 = sspctredge_vec[ facej ].size();

	//step1. according to the intersection result, split the contour edges
	//and save the subedge position in contour
	splitCtrEdge( subedgelist1, paramlist1, facei, sheeti, meshVer,
		sspctrver_vec, sspctredge_vec, edgeFacei, pos1);
	splitCtrEdge( subedgelist2, paramlist2, facej, sheeti, meshVer,
		sspctrver_vec, sspctredge_vec, edgeFacej, pos2);

	
	//step2. refresh other vars
	//--case1. new intersection point exists
	//add new edges into mesh, and change the old ones
	//refresh edgefacei, edgefacej and return
	int availMeshEdgePos[ 2 ] = {edgeFacei[ pos1 ].posInMeshEdge,edgeFacej[ pos2 ].posInMeshEdge};
	if( intertype == NORMAL_INTERPT )
	{
		//add new vertex into meshVer
		meshVer.push_back( newpt[ 0 ]);
		meshVer.push_back( newpt[ 1 ]);
		meshVer.push_back( newpt[ 2 ]);

		//add new edges into meshEdge
		int meshVerLen = meshVer.size()/3;		//leng of mesh vertex after pushing in new one
		int oldMeshEdgeLen = meshEdge.size() /2 ;	//length before pushing new edge
		//new edge
		meshEdge.push_back( meshVerLen - 1);
		meshEdge.push_back( meshEdge[ 2*availMeshEdgePos[ 0 ] + 1 ]);
		meshEdge.push_back( meshVerLen - 1);
		meshEdge.push_back( meshEdge[ 2*availMeshEdgePos[ 1 ] + 1 ]);
		//old edge
		meshEdge[ 2*availMeshEdgePos[ 0 ] + 1 ] = meshVerLen - 1;
		meshEdge[ 2*availMeshEdgePos[ 1 ] + 1 ] = meshVerLen - 1;

		//set edgeFacei, edgeFacej and startedge
		int oldsubedgenum[ 2] = { edgeFacei.size(), edgeFacej.size() };
		//subedgelist for facei
		edgeFacei.resize( oldsubedgenum[ 0 ] + 1);
		//set the new subedge
		edgeFacei[ oldsubedgenum[ 0 ]].posInMeshVer[ 0 ] = meshEdge[ 2*oldMeshEdgeLen ];
		edgeFacei[ oldsubedgenum[ 0 ]].posInMeshVer[ 1 ] = meshEdge[ 2*oldMeshEdgeLen + 1 ];
		edgeFacei[ oldsubedgenum[ 0 ]].posInMeshEdge = oldMeshEdgeLen;
		edgeFacei[ oldsubedgenum[ 0 ]].posInCtrEdge = sspctredge_vec[ facei ].size() - 1;
		//set the old subedge
		edgeFacei[ pos1 ].posInMeshVer[ 1 ] = meshVerLen - 1;	//the new vertex
		startedge.push_back( edgej + 1 );	
				//for this new subedge, only has to process starting from edgej + 1 in edgefacej

		//subedgelist for facej
		//insert the new edge
		oneEdge_vec::iterator iter = edgeFacej.begin();
		iter = iter + pos2 + 1;
		subEdge tsubedge;
		tsubedge.posInMeshVer[ 0 ] = meshEdge[ 2*oldMeshEdgeLen + 2];
		tsubedge.posInMeshVer[ 1 ] = meshEdge[ 2*oldMeshEdgeLen + 3];
		tsubedge.posInCtrEdge = sspctredge_vec[ facej ].size() - 1;
		tsubedge.posInMeshEdge = oldMeshEdgeLen + 1;
		edgeFacej.insert( iter, tsubedge);
		//change the old one
		edgeFacej[ pos2 ].posInMeshVer[ 1 ] = meshVerLen - 1;

		pos2 += 2;	//next time, jump the added new subedge to intersect
		return;		
	}

	//--case2. overlap or share common vertex
	//availedge: the old two indices of edges in meshEdge
	//gatheredge: all the gathered subedges after intersection	
	// 4 is a group
	//const int meshVerPos1 = 0;
	//const int meshVerPos2 = 1;
	//const int subEdgeType = 2;
	//const int whichFace = 3;

	////subEdgeType
	//const int SUBEDGE_OVERLAP = 0;
	//const int SUBEDGE_FACEI = 1;
	//const int SUBEDGE_FACEJ = 2;
	//memtab: the memorized same subedges in the two subedge list
	//memdir: if the overlapped edge has the same direction as it is in the first face
	int* ctreimesheilist1;	//#contour edge #mesh edge, two is a group for one subedge
	int* ctreimesheilist2;	//#contour edge #mesh edge, two is a group for one subedge
	int nedgelen1 = subedgelist1.size() - 1;	//subedge number corresponding facei
	int nedgelen2 = subedgelist2.size() - 1;
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"the two subedgelists are:"<<endl;
	for( int i = 0; i < subedgelist1.size(); i ++ )
	{
		cout<<subedgelist1[ i ]<<" ";
	}
	cout<<endl;
	for( int i = 0; i < subedgelist2.size(); i ++ )
	{
		cout<<subedgelist2[ i ]<<" ";
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//cout<<"nedgelen1: "<<nedgelen1<<"nedgelen2:"<<nedgelen2<<endl;
	//////////////////////////////////////////////////////////////////////////
	ctreimesheilist1 = new int[ nedgelen1*2 ];
	ctreimesheilist2 = new int[ nedgelen2*2 ];	
	ctreimesheilist1[ 0 ] = edgeFacei[ pos1 ].posInCtrEdge;
	ctreimesheilist2[ 0 ] = edgeFacej[ pos2 ].posInCtrEdge;
	int nctredgelen1 = sspctredge_vec[ facei ].size();	//contour edge number after adding new ones
	int nctredgelen2 = sspctredge_vec[ facej ].size();
	int j = nctredgelen1 - nedgelen1 + 1;	//old edge number in sspctredge_vec[facei]
	//////////////////////////////////////////////////////////////////////////
	//cout<<"nctredgelen1:"<<nctredgelen1<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 2; i < nedgelen1*2; i+=2 )
	{
		//set the contour edge index
		ctreimesheilist1[ i ] = j;
		//////////////////////////////////////////////////////////////////////////
		//cout<<"the set corresponding contour edge index:"<<j<<endl;
		//////////////////////////////////////////////////////////////////////////
		j++;
	}
	
	j = nctredgelen2 - nedgelen2 + 1;	//old edge number in sspctredge_Vec[facej ]
	//////////////////////////////////////////////////////////////////////////
	//cout<<"nctredgelen2: "<<nctredgelen2<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 2; i < nedgelen2*2; i+= 2 )
	{
		ctreimesheilist2[ i ] = j;
		//////////////////////////////////////////////////////////////////////////
		//cout<<"the set correspondoing contour edge ind:"<<j<<endl;
		//////////////////////////////////////////////////////////////////////////
		j++;
	}

	////////////////////////////////////////////////////////////////////////////
	//cout<<"intersection type: no new intersection point, share common vertex on one contains the other"<<endl;
	////////////////////////////////////////////////////////////////////////////
	intvector gatheredge;
	//the repeated contour edge on facej, because the one on facei is kept in gatheredge, no need to save
	int repeatedctredge;	
	bool samedir;	//the second subedge is in the same direction or not
	//add subedges in the two list into meshEdge
	//the new one
//	int tlen1 = subedgelist1.size() - 1;
	////////////////////////////////////////////////////////////////////////////
	//cout<<"for facei, the subedges on it:"<<endl;
	////////////////////////////////////////////////////////////////////////////
	for(int i = 0; i < nedgelen1; i ++ )
	{
		//remeber all the subedges information
		gatheredge.push_back( subedgelist1[ i ]);
		gatheredge.push_back( subedgelist1[ i + 1] );		//which type
		////////////////////////////////////////////////////////////////////////////
		//cout<<subedgelist1[ i ]<<"\t"<<subedgelist1[ i + 1 ]<<endl;
		////////////////////////////////////////////////////////////////////////////
		gatheredge.push_back( SUBEDGE_FACEI );
		gatheredge.push_back( i );		//the ith subedge in subedgelist1
	//	gatheredge.push_back( tctredgepos );		//pos in contour edge
	//	tctredgepos ++;								
//		gatheredge.push_back( -1 );		//pos in mesh edge, haven't decided yet
		//medgelen ++;

	//	meshEdge.push_back( subedgelist1[ i ]);
	//	meshEdge.push_back( subedgelist1[ i + 1] );		
	}
		
	int subedgelen1 = gatheredge.size()/4;	//compare with all these subedges to check if repeated
	
	//push all the subedges in subedgelist2 into meshedge
	bool needjudge = true;
	int repeati;
//	int tlen2 = subedgelist2.size() - 1;
	//////////////////////////////////////////////////////////////////////////
	//cout<<"for edge on the second face:"<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < nedgelen2; i++)
	{
		if( !needjudge )	//no longer need judge.
		{
			//add gatheredge
			gatheredge.push_back( subedgelist2[ i ]);
			gatheredge.push_back( subedgelist2[ i + 1]);
			//////////////////////////////////////////////////////////////////////////
			//cout<<subedgelist2[ i ]<<"\t"<<subedgelist2[ i + 1 ]<<endl;
			//////////////////////////////////////////////////////////////////////////
			gatheredge.push_back( SUBEDGE_FACEJ);
			gatheredge.push_back( i );	//the ith subedge in subedgelist2
			continue;
		}	
		//check if repeated or not		
		int subveris[ 2 ] = { subedgelist2[ i ], subedgelist2[ i + 1]};
		if( isRepeat(gatheredge, subveris, subedgelen1, repeati, samedir ) )	//the first time repeated is set to be true
		{
			needjudge = false;	//no longer necessary to judge repeatition
			
			////////////////////////////////////////////////////////////////////////////
			//cout<<"find repeated edge!"<<endl;
			//cout<<subedgelist2[ i ]<<"\t"<<subedgelist2[ i + 1 ]<<endl;
			////////////////////////////////////////////////////////////////////////////

			//change gatheredge
			gatheredge[ repeati * 4 + subEdgeType ] = SUBEDGE_OVERLAP;
			repeatedctredge = i;	//the ith subedgelist in subedgelist2.
		}
		else
		{
			gatheredge.push_back( subveris[ 0 ]);
			gatheredge.push_back( subveris[ 1 ]);
			//////////////////////////////////////////////////////////////////////////
			//cout<<subedgelist2[ i ]<<"\t"<<subedgelist2[ i + 1 ]<<endl;
			//////////////////////////////////////////////////////////////////////////
			gatheredge.push_back( SUBEDGE_FACEJ);
			gatheredge.push_back( i );
		}
	}

	//push all the edges in gatheredge into meshedge	
	int subedgelen = gatheredge.size()/4;
	int* meposlist = new int[ subedgelen ];
	int nmepos = meshEdge.size() / 2;
	meposlist[ 0 ] = availMeshEdgePos[ 0 ];
	meposlist[ 1 ] = availMeshEdgePos[ 1];
	for( int i = 2; i < subedgelen; i ++ )
	{
		meposlist[ i ] = nmepos;
		nmepos++;
	}
	if( subedgelen > 2 )
		meshEdge.resize( nmepos* 2);
//	int repeatpos[ 2 ];
	bool isfirst[ 2 ] = {true, true};
	for( int i = 0; i < subedgelen; i ++ )
	{
		nmepos = meposlist[ i ];
		int subedgetp = gatheredge[ 4*i + subEdgeType ];
	
		if( subedgetp  == SUBEDGE_OVERLAP )
		{
			int repeatpos1 =  gatheredge[ 4*i + ctrEdgePos ];
		//	repeatpos[ 0 ] = gatheredge[ 5*i + ctrEdgePos ];
		//	repeatpos[ 1 ] = repeatedctredge;

			//add the repeated edge into nsheetreg
			int nregepos = nsheetReg.size(); 
			nsheetReg.resize( nregepos + 1 );
            nsheetReg[ nregepos ].posInMeshEdge = nmepos;
			nsheetReg[ nregepos ].crspCtrEdges.resize( 2 );
			nsheetReg[ nregepos ].crspCtrEdges[ 0 ].facei = facei;
			nsheetReg[ nregepos ].crspCtrEdges[ 0 ].edgei = 
				ctreimesheilist1[repeatpos1 * 2 ];
			nsheetReg[ nregepos ].crspCtrEdges[ 0 ].samedirec = true;
			nsheetReg[ nregepos ].crspCtrEdges[ 1 ].facei = facej;
			nsheetReg[ nregepos ].crspCtrEdges[ 1 ].edgei = 
				ctreimesheilist2[repeatedctredge * 2 ] ;
			nsheetReg[ nregepos ].crspCtrEdges[ 1 ].samedirec = samedir;

			ctreimesheilist1[ repeatpos1* 2 ] = -1;	//mark it as not valid
			ctreimesheilist2[ repeatedctredge * 2 ] = -1; //mark it as not valid
		//	ctreimesheilist1[ repeatpos[ 0 ]*2 + 1 ] = nmepos;
		//	ctreimesheilist2[ repeatedctredge*2 + 1 ] = nmepos;

		}
		else if( subedgetp == SUBEDGE_FACEI )
		{
		//	ctreimesheilist1[ gatheredge[ 5*i + ctrEdgePos ]*2 + 1 ] = nmepos;
			
			//replace the old subedge of edgeFacei at position pos1
			if( isfirst[ 0 ] )
			{
				isfirst[ 0 ] = false;
                edgeFacei[ pos1 ].posInMeshVer[ 0 ] = gatheredge[ 4*i + meshVerPos1 ];
				edgeFacei[ pos1 ].posInMeshVer[ 1 ] = gatheredge[ 4*i + meshVerPos2 ];
				edgeFacei[ pos1 ].posInCtrEdge = ctreimesheilist1[ gatheredge[ 4*i + ctrEdgePos ]*2 ];
				edgeFacei[ pos1 ].posInMeshEdge = nmepos;

                ctreimesheilist1[  gatheredge[ 4*i + ctrEdgePos ] * 2 ] = -1;	//set it not valid
//				continue;
			}
			else
				//set the corresponding mesh edge for this subedge
				ctreimesheilist1[  gatheredge[ 4*i + ctrEdgePos ] * 2 + 1 ] = nmepos;

		}
		else	//subedge_facej
		{
			if( isfirst[ 1 ] )
			{
				isfirst[ 1 ] = false;
				edgeFacej[ pos2 ].posInMeshVer[ 0 ] = gatheredge[ 4*i + meshVerPos1 ];
				edgeFacej[ pos2 ].posInMeshVer[ 1 ] = gatheredge[ 4*i + meshVerPos2 ];
				edgeFacej[ pos2 ].posInCtrEdge = ctreimesheilist2[ gatheredge[ 4*i + ctrEdgePos ]*2 ];
				edgeFacej[ pos2 ].posInMeshEdge = nmepos;

				ctreimesheilist2[  gatheredge[ 4*i + ctrEdgePos ] * 2 ] = -1;	//set it not valid
				//continue;
			}
			else
				//set the corresponding mesh edge for this subedge
				ctreimesheilist2[  gatheredge[ 4*i + ctrEdgePos ] * 2 + 1 ] = nmepos;
		}

		meshEdge[ 2*nmepos ] = gatheredge[ 4*i + meshVerPos1 ];
		meshEdge[ 2*nmepos + 1] = gatheredge[ 4*i + meshVerPos2 ];
	//	nmepos++;

		//push the new subedge into meshedge
	//	meshEdge.push_back( gatheredge[ 4*i + meshVerPos1 ] );
	//	meshEdge.push_back( gatheredge[ 4*i + meshVerPos2 ] );		
	}

//	bool result = false;
	//refresh edgeFacei and edgeFacej
	//not set to be false, the old one is not replaced by a new one
	//the only case, is the subedge is contained in the other subedge in facej
	oneEdge_vec::iterator iter;
	intvector::iterator iter2;
	if( isfirst[ 0 ])	
	{
	//	result = true;	//the subedge is got rid of, when return, a new subedge from facei is
						//going to intersect with all subedges in facej
		iter = edgeFacei.begin();
		iter += pos1;
		edgeFacei.erase( iter );
		iter2 = startedge.begin();
		iter2 += pos1;
		startedge.erase( iter2 );

		pos1 --;	//because pos1 will be added by 1, so move it one edge backward.
	}
	else	//the old one is replaced, some new one may be added in
	{
		for( int i = 0; i < nedgelen1; i++ )
		{
			int ctrei = ctreimesheilist1 [ 2*i ];
			if( ctrei == -1 ) continue;
			int mei = ctreimesheilist1[ 2* i + 1 ];
			subEdge tse;
			tse.posInMeshVer[ 0 ] = meshEdge[ 2*mei ];
			tse.posInMeshVer[ 1 ] = meshEdge[ 2*mei + 1 ];
			tse.posInMeshEdge = mei;
			tse.posInCtrEdge = ctrei;
			edgeFacei.push_back( tse );
			//the new subedge only has to check intersection from the edgej + 1 in facej
			startedge.push_back( edgej + 1 );	
		}
	}
	
	//the second edge is contained by the first one
	if( isfirst[ 1 ])	
	{
		iter = edgeFacej.begin();
		iter += pos2;
		edgeFacej.erase( iter );
	}
	else
	{
		//the old one has already been replaced, add new ones
		pos2 += 1;
		iter = edgeFacej.begin();
		iter = iter + pos2 ;
		for( int i = 0; i < nedgelen2; i++  )
		{
			int ctrei = ctreimesheilist2[ 2*i ];
			if( ctrei == -1 )continue;
			int mei = ctreimesheilist2[ 2* i + 1 ];
			subEdge tse;
			tse.posInMeshVer[ 0 ] = meshEdge[ 2*mei ];
			tse.posInMeshVer[ 1 ] = meshEdge[ 2*mei + 1 ];
			tse.posInMeshEdge = mei;
			tse.posInCtrEdge = ctrei;
			edgeFacej.insert( iter, tse );
		//	edgeFacej.push_back( tse );
			pos2++;
		}
	}
	gatheredge.clear();
	delete []ctreimesheilist1;
	delete []ctreimesheilist2;
	delete []meposlist;
}

//split the projected edges on one sheet
void ProjSplitter::splitProjEdgeSheet_One(
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
								   )
{
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"vertex in mesh:"<<endl;
	for( int i = 0; i < meshVer.size(); i += 3 )
	{
		cout<<meshVer[ i ]<<"\t"<<meshVer[ i + 1]<<"\t"<<meshVer[ i + 2] <<endl;
	}
	cout<<endl;

	cout<<"edge in mesh:"<<endl;
	for( int i= 0; i< meshEdge.size(); i += 2 )
	{
		cout<<meshEdge[ i ]<<"\t"<<meshEdge[ i + 1 ]<<endl;
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////
	//step1. sort out the three tabs.
	//1. on both of them, add directly into nsheetreg, 
	//2. only on sheeti	(smaller)
	//3. only on sheetj	(bigger )
	vector<oneEdge_vec> edgeFacei;
	vector<oneEdge_vec> edgeFacej;
	for(unsigned int i = 0; i < sheetReg.size(); i ++)
	{
		//overlapped projected edge
		if( sheetReg[ i ].crspCtrEdges.size() == 2 )
		{
			EdgeRegItem tmp;
			tmp.posInMeshEdge = sheetReg[ i ].posInMeshEdge;
			for(unsigned int j = 0; j < sheetReg[ i ].crspCtrEdges.size(); j ++ )
			{
				correspnCtrEdge tcspe;
				memcpy(&tcspe, &(sheetReg[ i ].crspCtrEdges[ j ]), sizeof( correspnCtrEdge ));
				tmp.crspCtrEdges.push_back( tcspe );
			}
			nsheetReg.push_back( tmp );
			tmp.crspCtrEdges.clear();
			continue;
		}
		//the projected edge is only from facei
        if( sheetReg[ i ].crspCtrEdges[ 0 ].facei == facei )
		{
			int nedgepos = edgeFacei.size();
			edgeFacei.resize( nedgepos + 1 );
			edgeFacei[ nedgepos ].resize( 1 );
			int tedgei = sheetReg[ i ].posInMeshEdge ;
			edgeFacei[ nedgepos ][ 0 ].posInMeshVer[ 0 ] = meshEdge[ tedgei * 2 ];
			edgeFacei[ nedgepos ][ 0 ].posInMeshVer[ 1 ] = meshEdge[ tedgei * 2 + 1 ];
			edgeFacei[ nedgepos ][ 0 ].posInMeshEdge = tedgei;
			edgeFacei[ nedgepos ][ 0 ].posInCtrEdge = sheetReg[ i ].crspCtrEdges[ 0 ].edgei;
			
			continue;
		}
		//it is only from facej
		int nedgepos = edgeFacej.size();
		edgeFacej.resize( nedgepos + 1 );
		edgeFacej[ nedgepos ].resize( 1 );
		int tedgei = sheetReg[ i ].posInMeshEdge ;
		edgeFacej[ nedgepos ][ 0 ].posInMeshVer[ 0 ] = meshEdge[ tedgei * 2 ];
		edgeFacej[ nedgepos ][ 0 ].posInMeshVer[ 1 ] = meshEdge[ tedgei * 2 + 1 ];
		edgeFacej[ nedgepos ][ 0 ].posInMeshEdge = tedgei;
		edgeFacej[ nedgepos ][ 0 ].posInCtrEdge = sheetReg[ i ].crspCtrEdges[ 0 ].edgei;
	}

	//step2. if any of the edgeFacei or edgeFacej is empty, add them into the new sheet reg, and return
	if( edgeFacei.size() == 0 )
	{
		if( edgeFacej.size() == 0 )	//no need to proecess, both of them are empty
			return;
		int nstartPos = nsheetReg.size();
		nsheetReg.resize( nstartPos + edgeFacej.size() );
		//add all the subedges in edgefacej into the nsheetreg
		for(unsigned int i = 0; i < edgeFacej.size(); i ++ )
		{
			nsheetReg[ nstartPos + i ].posInMeshEdge = edgeFacej[ i ][ 0 ].posInMeshEdge;
			nsheetReg[ nstartPos + i ].crspCtrEdges.resize( 1 );
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].facei = facej;
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].edgei = edgeFacej[ i ][ 0 ].posInCtrEdge;
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].samedirec = true;

			edgeFacej[ i ].clear();
		}

		//clear temp vars
		edgeFacej.clear();
		return;
	}

	if( edgeFacej.size() == 0 )
	{
		int nstartPos = nsheetReg.size();
		nsheetReg.resize( nstartPos + edgeFacei.size() );
		for(unsigned int i = 0; i < edgeFacei.size(); i ++)
		{
			nsheetReg[ nstartPos + i ].posInMeshEdge = edgeFacei[ i ][ 0 ].posInMeshEdge;
			nsheetReg[ nstartPos + i ].crspCtrEdges.resize( 1 );
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].facei = facei;
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].edgei = edgeFacei[ i ][ 0 ].posInCtrEdge;
			nsheetReg[ nstartPos + i ].crspCtrEdges[ 0 ].samedirec = true;

			edgeFacei[ i ].clear();
		}
		edgeFacei.clear();
		return;
	}

	//if coming here, there exist edges both from the two faces
	//step3. go through each subedge of facei , and compute intersection with subedge in edgeFacej
	vector<intvector> startedge;
	startedge.resize( edgeFacei.size() );
	for(unsigned int i = 0; i < edgeFacei.size(); i ++ )
	{
		startedge[ i ].resize( 1 );
		startedge[ i ][ 0 ] = 0;
	}
	int edgenumi, edgenumj;
	edgenumi = edgeFacei.size();
	edgenumj = edgeFacej.size();

	bool newround = false;
	intvector::iterator iter_int;
	oneEdge_vec::iterator iter_edge;
	//////////////////////////////////////////////////////////////////////////
	/*if( tempsheetnum == 21 )
	{
		cout<<"edge number for facei:"<<edgenumi<<"\t for facej :" <<edgenumj<<endl;
		cout<<"after refresh:"<<endl;
		cout<<"facei"<<facei<<":mv1,mv2,me,ctre\n";
		for(int t1 = 0; t1 < edgenumi; t1++)
		{
		for( int t2 = 0; t2 < edgeFacei[ t1 ].size(); t2 ++ )
		{
		cout<<edgeFacei[ t1 ][ t2 ].posInMeshVer[ 0 ]<<","<<
		edgeFacei[ t1 ][ t2 ].posInMeshVer[ 1 ]<<","<<
		edgeFacei[t1][t2].posInMeshEdge<<","
		<<edgeFacei[ t1 ][ t2 ].posInCtrEdge<<"\t";
		int tvi1[ 2] = {edgeFacei[ t1 ][ t2 ].posInMeshVer[ 0 ],edgeFacei[ t1 ][ t2 ].posInMeshVer[ 1 ]};
		cout<<"{"<<meshVer[ 3*tvi1[ 0 ] ]<<","<<meshVer[ 3*tvi1[ 0 ] + 1]<<","<<meshVer[ 3*tvi1[ 0 ] +2 ]<<"}  "
			<<"{"<<meshVer[ 3*tvi1[ 1 ] ]<<","<<meshVer[ 3*tvi1[ 1 ] + 1]<<","<<meshVer[ 3*tvi1[ 1 ] + 2]<<"}\n";
		}
		cout<<endl;
		}
		cout<<"facej"<<facej<<"\n";
		for( int t1 = 0; t1 < edgenumj; t1 ++ )
		{
		for( int t2 = 0; t2 < edgeFacej[ t1 ].size(); t2 ++ )
		{
		cout<<edgeFacej[ t1 ][ t2 ].posInMeshVer[ 0 ]<<","<<
		edgeFacej[ t1 ][ t2 ].posInMeshVer[ 1 ]<<","<<
		edgeFacej[t1][t2].posInMeshEdge<<","
		<<edgeFacej[ t1 ][ t2 ].posInCtrEdge<<"\t";
		int tvi1[ 2] = {edgeFacej[ t1 ][ t2 ].posInMeshVer[ 0 ],edgeFacej[ t1 ][ t2 ].posInMeshVer[ 1 ]};
		cout<<"{"<<meshVer[ 3*tvi1[ 0 ] ]<<","<<meshVer[ 3*tvi1[ 0 ] + 1 ]<<","<<meshVer[ 3*tvi1[ 0 ] +2]<<"}  "
			<<"{"<<meshVer[ 3*tvi1[ 1 ] ]<<","<<meshVer[ 3*tvi1[ 1 ] + 1]<<","<<meshVer[ 3*tvi1[ 1 ] +2]<<"}\n";
		}
		cout<<endl;
		}
	}*/
	
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < edgenumi; i ++ )
	{
		int pos1 = 0;
		unsigned int len1 = edgeFacei[ i ].size();
		while((unsigned int) pos1 < len1 )	//for each subedge in the edgei in edgeFacei, compute intersection with all subedges from another facej
		{
			//////////////////////////////////////////////////////////////////////////
		/*	cout<<"-- for facei subedge with vers: "<<edgeFacei[ i ][ pos1 ].posInMeshVer[ 0 ]<<"\t"
				<<edgeFacei[ i ][ pos1 ].posInMeshVer[ 1 ] 
				<<"starting edge:"<<startedge[ i ][ pos1 ]		
				<<endl;*/

			//////////////////////////////////////////////////////////////////////////
			for( int j = startedge[ i ][ pos1 ]; j < edgenumj; j ++)
			{
			//	//////////////////////////////////////////////////////////////////////////
			//	cout<<"go through each edge in facej, curretn index :" <<j;
			//	//////////////////////////////////////////////////////////////////////////
				int pos2 = 0;
				int len2 = edgeFacej[ j ].size();
				while( pos2 < len2 )
				{
					//////////////////////////////////////////////////////////////////////////
					//cout<<"curretn subedge:"<<pos2<<endl;
					//////////////////////////////////////////////////////////////////////////
					//case1. the two edges actually overlapped
					if( (( edgeFacei[ i ][ pos1 ].posInMeshVer[ 0 ] == edgeFacej[ j ][ pos2 ].posInMeshVer[ 0 ])&&
						( edgeFacei[ i ][ pos1 ].posInMeshVer[ 1 ] == edgeFacej[ j ][ pos2 ].posInMeshVer[ 1 ]) )
						||
						(( edgeFacei[ i ][ pos1 ].posInMeshVer[ 1 ] == edgeFacej[ j ][ pos2 ].posInMeshVer[ 0 ])&&
						( edgeFacei[ i ][ pos1 ].posInMeshVer[ 0 ] == edgeFacej[ j ][ pos2 ].posInMeshVer[ 1 ]) )
						)
					{
						//add it into nsheetreg
						int tpos = nsheetReg.size();
						nsheetReg.resize( tpos + 1 );
						nsheetReg[ tpos ].posInMeshEdge = edgeFacei[ i ][ pos1 ].posInMeshEdge;
						nsheetReg[ tpos ].crspCtrEdges.resize( 2 );
						nsheetReg[ tpos ].crspCtrEdges[ 0 ].facei = facei;
						nsheetReg[ tpos ].crspCtrEdges[ 0 ].edgei = edgeFacei[ i ][ pos1 ].posInCtrEdge;
						nsheetReg[ tpos ].crspCtrEdges[ 0 ].samedirec = true;
						nsheetReg[ tpos ].crspCtrEdges[ 1 ].facei = facej;
						nsheetReg[ tpos ].crspCtrEdges[ 1 ].edgei = edgeFacej[ j ][ pos2 ].posInCtrEdge;
						if( edgeFacei[ i ][ pos1 ].posInMeshVer[ 0 ] == edgeFacej[ j ][ pos2 ].posInMeshVer[ 0 ] )
							nsheetReg[ tpos ].crspCtrEdges[ 1 ].samedirec = true;
						else
							nsheetReg[ tpos ].crspCtrEdges[ 1 ].samedirec = false;
						//remove this subedge from both of them and start a new subedge round
						iter_edge = edgeFacei[ i ].begin() + pos1;
                        edgeFacei[ i ].erase( iter_edge );
						iter_int = startedge[ i ].begin() + pos1;
						startedge[ i ].erase( iter_int );
						iter_edge = edgeFacej[ j ].begin() + pos2;
                        edgeFacej[ j ].erase( iter_edge );
						newround = true;
						//////////////////////////////////////////////////////////////////////////
						//cout<<"the edge is the same as one of the subedge on another face!"<<endl;
						//////////////////////////////////////////////////////////////////////////
						break;
					}
					
					//normal case: compute intersection between the two subedges
					//call the edgeintersect to compute the intersection points
					intvector subedgelist1;
					intvector subedgelist2;
					floatvector paramlist1;
					floatvector paramlist2;
					float newpt[ 3 ];
					bool newptexist = false;
					float vers[ 12 ];
					int veris[ 4 ];
					veris[ 0 ] = edgeFacei[ i ][ pos1 ].posInMeshVer[ 0 ];
					veris[ 1 ] = edgeFacei[ i ][ pos1 ].posInMeshVer[ 1 ];
					veris[ 2 ] = edgeFacej[ j ][ pos2 ].posInMeshVer[ 0 ];
					veris[ 3 ] = edgeFacej[ j ][ pos2 ].posInMeshVer[ 1 ];
					//////////////////////////////////////////////////////////////////////////
					////output the informaiton i want to know when it crashes. :)
					/*if( tempsheetnum == 21)
					cout<<"two edges:"<<veris[ 0 ]<<"  "
						<<veris[ 1 ]<<"  "
						<<veris[ 2 ]<<"  "
						<<veris[ 3 ]<<endl;*/
				//	cout<<"facei, edge index, subedge index"<<endl;
				//	cout<<"first:"<<facei<<","<<i<<","<<pos1<<endl;
				//	cout<<"secon:"<<facej<<","<<j<<","<<pos2<<endl;
				////	cout<<"current sheet:"<<sheeti<<"\t facei:"<<facei<<"\t facej:"<<facej<<endl;
				////	cout<<"edge "<<i<<" on facei\t"<<"edge "<<j<<" on facej\n";
				////	cout<<"subedge "<<pos1<<" of edgei"<<"\tsubedge"<<pos2<<" on edgej"<<endl;
					////////////////////////////////////////////////////////////////////////
					for( int tk = 0; tk < 4; tk++ )
					{
						for( int tk2 = 0; tk2 < 3; tk2 ++ )
						{
							vers[ 3*tk + tk2 ] = meshVer[ 3*veris[ tk ] + tk2 ];
						}
					}
					//////////////////////////////////////////////////////////////////////////
					//if( tempsheetnum == 21)
					//{
					//	cout<<"two edges:"<<endl;
					//	for( int i = 0; i< 12; i ++ )
					//	{
					//		cout<<vers[ i ];
					//		if( ((i + 1)%3) != 0)
					//			cout<<",";
					//		else
					//			cout<<" ";
					//		if( (i + 1)%6 == 0 )
					//			cout<<endl;
					//	}
					//	//cout<<endl;

					//}
					//////////////////////////////////////////////////////////////////////////
					int intertype = edgeIntersect( vers, veris, subedgelist1, paramlist1, 
						subedgelist2, paramlist2, newptexist, newpt);

					//no intersection point at all
					if( intertype == NO_INTERPT )
					{
						pos2 ++;		//go to the next subedge in edgefacej
						subedgelist1.clear();
						subedgelist2.clear();
						paramlist1.clear();
						paramlist2.clear();
						//////////////////////////////////////////////////////////////////////////
						/*if( tempsheetnum == 21)
						{
							cout<<"no intersection at all!"<<endl;
						}*/
						//////////////////////////////////////////////////////////////////////////
						continue;
					}
					else
					{			
						//////////////////////////////////////////////////////////////////////////
						/*if( tempsheetnum == 21)
						{
							cout<<"intersection type: ";
						if( intertype == NORMAL_INTERPT )
							cout<<"new intersection point exists! \n";
						else
							cout<<"overlap ! \n";
						for( int t1 = 0; t1 < subedgelist1.size(); t1 ++ )
						{
							cout<<subedgelist1[ t1 ] <<" ";
						}
						cout<<endl;
						for( int t2 = 0; t2 < subedgelist2.size(); t2 ++)
						{
							cout<<subedgelist2[ t2 ]<<" ";
						}
						cout<<endl;
						}*/
						//////////////////////////////////////////////////////////////////////////
						refreshAfterSegIntersect( intertype, subedgelist1,
							subedgelist2,paramlist1,paramlist2, newpt, sheeti, facei, facej,
							meshVer, meshEdge, sspctrver_vec, sspctredge_vec,
							edgeFacei[ i ], edgeFacej[ j ], nsheetReg, startedge[ i ], i, j, 
							pos1, pos2);
						subedgelist1.clear();
						subedgelist2.clear();
						paramlist1.clear();
						paramlist2.clear();

						//////////////////////////////////////////////////////////////////////////
						/*cout<<"vertex in mesh:"<<endl;
						for( int i = 0; i < meshVer.size(); i += 3 )
						{
							cout<<meshVer[ i ]<<"\t"<<meshVer[ i + 1]<<"\t"<<meshVer[ i + 2] <<endl;
						}
						cout<<endl;

						cout<<"edge in mesh:"<<endl;
						for( int i= 0; i< meshEdge.size(); i += 2 )
						{
							cout<<meshEdge[ i ]<<"\t"<<meshEdge[ i + 1 ]<<endl;
						}
						cout<<endl*/;
						//////////////////////////////////////////////////////////////////////////

						//////////////////////////////////////////////////////////////////////////
						/*cout<<"after refresh:"<<endl;
						cout<<"facei"<<facei<<":mv1,mv2,me,ctre\n";
						for(int t1 = 0; t1 < edgenumi; t1++)
						{
							for( int t2 = 0; t2 < edgeFacei[ t1 ].size(); t2 ++ )
							{
								cout<<edgeFacei[ t1 ][ t2 ].posInMeshVer[ 0 ]<<","<<
									edgeFacei[ t1 ][ t2 ].posInMeshVer[ 1 ]<<","<<
									edgeFacei[t1][t2].posInMeshEdge<<","
									<<edgeFacei[ t1 ][ t2 ].posInCtrEdge<<"\t";
							}
							cout<<endl;
						}
						cout<<"facej"<<facej<<"\n";
						for( int t1 = 0; t1 < edgenumj; t1 ++ )
						{
							for( int t2 = 0; t2 < edgeFacej[ t1 ].size(); t2 ++ )
							{
								cout<<edgeFacej[ t1 ][ t2 ].posInMeshVer[ 0 ]<<","<<
									edgeFacej[ t1 ][ t2 ].posInMeshVer[ 1 ]<<","<<
									edgeFacej[t1][t2].posInMeshEdge<<","
									<<edgeFacej[ t1 ][ t2 ].posInCtrEdge<<"\t";
							}
							cout<<endl;
						}*/

						//////////////////////////////////////////////////////////////////////////
					}
					if( len1 > edgeFacei[ i ].size() )	//current subedge no long exists for facei
					{
						//////////////////////////////////////////////////////////////////////////
						//cout<<"len1 > edgefacei i size, current subedge no longer exists !"<<endl;
						//////////////////////////////////////////////////////////////////////////
						break;
					}
					len2 = edgeFacej[ j ].size();
				}
				//the two if can't change sequence, because the second one contains the first one.
				//well, actually i think i can get rid of the first if. :-)
				if( newround )	//start a new subedge in facei
				{
					newround = false;
					break;
				}		
				if( len1 > edgeFacei[ i ].size() )	//current subedge no longer exists for facei
				{
					////////////////////////////////////////////////////////////////////////////
					//cout<<"len1 > edgefacei i size, current subedge no longer exists !"<<endl;
					////////////////////////////////////////////////////////////////////////////
					break;
				}
		
				////////////////////////////////////////////////////////////////////////////
				//cout<<"hey, end of the for loop! go to the next edge!"<<endl;
				////////////////////////////////////////////////////////////////////////////
			}
            pos1 ++;
			len1 = edgeFacei[ i ].size();
		}
	}

	//step4. add all the subedges in the two vectors into nsheetreg
	int nstartPos = nsheetReg.size();
//	nsheetReg.resize( nstartPos + edgeFacei.size() + edgeFacej.size() );
	//add all the subedges in edgefacej into the nsheetreg
	for(unsigned int i = 0; i < edgeFacei.size(); i ++)
	{
		if( edgeFacei[ i ].size() == 0 )
			continue;
		nsheetReg.resize( nstartPos + edgeFacei[ i ].size() );
		for(unsigned int j = 0; j < edgeFacei[ i ].size(); j ++ )
		{
			nsheetReg[ nstartPos ].posInMeshEdge = edgeFacei[ i ][ j ].posInMeshEdge;
			nsheetReg[ nstartPos ].crspCtrEdges.resize( 1 );
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].facei = facei;
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].edgei = edgeFacei[ i ][ j ].posInCtrEdge;
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].samedirec = true;
			nstartPos ++;
		}
		edgeFacei[ i ].clear();
		startedge[ i ].clear();
		//nstartPos += edgeFacei[ i ].size();
	}
	//nstartPos += edgeFacei.size();
	edgeFacei.clear();
	startedge.clear();

	for(unsigned int i = 0; i < edgeFacej.size(); i ++ )
	{
		if( edgeFacej[ i ].size() == 0 )
			continue;
		nsheetReg.resize( nstartPos + edgeFacej[ i ].size() );
		for(unsigned int j = 0; j < edgeFacej[ i ].size(); j ++ )
		{
			nsheetReg[ nstartPos ].posInMeshEdge = edgeFacej[ i ][ j ].posInMeshEdge;
			nsheetReg[ nstartPos ].crspCtrEdges.resize( 1 );
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].facei = facej;
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].edgei = edgeFacej[ i ][ j ].posInCtrEdge;
			nsheetReg[ nstartPos ].crspCtrEdges[ 0 ].samedirec = true;
			nstartPos ++;
		}
		edgeFacej[ i ].clear();
	}

//	edgeFacei.clear();
//	startedge.clear();
	edgeFacej.clear();
}

//split all the projected edges on sheets
void ProjSplitter::SplitProjEdgeSheet
	(
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
	)
{
	int keys[ 2 ];
	int sheeti;
	//go through each sheet, and split it
	for( int i = 0; i < subfacenum - 1; i ++)
	{
		keys[ 0 ] = i;
		for( int j = i+1; j < subfacenum; j ++ )
		{
			keys[ 1 ] = j;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );
			//both of them are empty or not
            if( sheetEdgeReg[ sheeti ].size() == 0 )
				continue;
			//////////////////////////////////////////////////////////////////////////
			//cout<<"keys:"<<i<<","<<j<<"sheet:"<<sheeti<<endl;
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//tempsheetnum = sheeti;
			//////////////////////////////////////////////////////////////////////////

			splitProjEdgeSheet_One(meshVer, meshEdge, sspctrver_vec, sspctredge_vec,
				i, j, sheetEdgeReg[ sheeti ], nsheetReg[ sheeti ], sheeti);
			for(unsigned int k = 0; k < sheetEdgeReg[ sheeti ].size(); k++)
			{
				sheetEdgeReg[ sheeti ][ k ].crspCtrEdges.clear();
			}
			sheetEdgeReg[ sheeti ].clear();
		}
	}
	delete []sheetEdgeReg;
}
void ProjSplitter::SplitProjectedEdge(
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
	MapArraySR& doubleface2sheet)
{
	splitProjEdgeSeam(
		meshVer, meshEdge,nseamReg,
		 sspctrver_vec, sspctredge_vec,jptReg,	seamVerReg,	
		 seamEdgeReg, 
		majptnum,maseamnum, masheetnum,maseam,  majpt  );
	SplitProjEdgeSheet(
		meshVer, meshEdge,sspctrver_vec,sspctredge_vec,
		sheetEdgeReg,nsheetReg,subfacenum,
		doubleface2sheet);	
}
void ProjSplitter::WriteInfoAfterSplit( int spacei,
						 //mesh
						 floatvector& meshVer,
						 intvector& meshEdge,
						 //jptreg seamreg sheetreg
						 int majptnum,	int maseamnum,	int masheetnum,
						 int* jptReg,				//for each junction point, corresponding vertex in meshver
						newSeamReg* nseamReg,
						EdgeReg* sheetEdgeReg,	//new sheet registration information actually
						 vector<SSPCTRVERVEC>& sspctrver_vec,
						 vector<SSPCTREDGEVEC>& sspctredge_vec)
{
	char fname[ 1024 ] = "mmdebug/onesubspace/projsplit";
	char numstr[ 3 ];
	itoa( spacei + 1, numstr, 10);
	strcat( fname, numstr);
	strcat( fname, ".txt");
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open the file "<<fname<<" to write!"<<endl;
		return;
	}

	//mesh
	fprintf( fout, "{{{{%f,%f,%f}", meshVer[ 0 ],meshVer[1], meshVer[ 2]);
	for(unsigned int i = 1; i < meshVer.size()/3; i ++)
		fprintf( fout, ",{%f,%f,%f}", meshVer[ 3*i ], meshVer[ 3*i + 1], meshVer[ 3*i + 2]);
	fprintf( fout, "},{{%d,%d}", meshEdge[ 0 ] + 1, meshEdge[ 1 ] + 1);
	for(unsigned int i = 1; i < meshEdge.size()/2 ; i ++)
		fprintf( fout, ",{%d,%d}", meshEdge[ 2*i ] + 1, meshEdge[ 2*i +1 ] + 1);

	//dcontour
	fprintf( fout, "}},{");
	int facenum = sspctredge_vec.size();
	for( int i = 0; i < facenum; i ++ )
	{
		if( i != 0 )
			fprintf(fout, ",");
		//vertex
		if( sspctrver_vec[ i ].size() == 0 )
			fprintf( fout, "{{");
		else
		{
			fprintf( fout, "{{{%f,%f,%f}", sspctrver_vec[ i ][ 0 ].pos[ 0 ], 
				sspctrver_vec[ i ][ 0 ].pos[1], sspctrver_vec[ i ][ 0 ].pos[ 2 ]);
			for(unsigned int j = 1; j < sspctrver_vec[ i ].size(); j ++ )
			{
				fprintf( fout, ",{%f,%f,%f}", sspctrver_vec[ i ][ j ].pos[ 0 ], 
					sspctrver_vec[ i ][ j ].pos[1], sspctrver_vec[ i ][ j ].pos[ 2 ]);
			}
		}

		//edge
		if( sspctredge_vec[ i ].size() == 0 )
			fprintf( fout, "},{");
		else
		{
			fprintf( fout, "},{{%d,%d}", sspctredge_vec[ i ][ 0 ].veris[ 0 ] + 1,
				sspctredge_vec[ i ][ 0 ].veris[ 1 ] + 1);
			for(unsigned int j = 1; j < sspctredge_vec[ i ].size(); j ++ )
			{
				fprintf( fout, ",{%d,%d}", sspctredge_vec[ i ][ j ].veris[ 0 ] + 1,
					sspctredge_vec[ i ][ j ].veris[ 1 ] + 1);
			}
		}
		fprintf( fout, "}}");
	}

	//jptreg
	fprintf( fout, "},{%d", jptReg[ 0 ] + 1);
	for( int i = 1; i < majptnum; i ++)
		fprintf( fout, ",%d", jptReg[ i ] + 1);

	////seamverreg
	//for( int i = 0; i < maseamnum; i ++ )
	//{
	//	if( seamVerReg[ i ].size() == 0 )
	//	{
	//		if( i == 0 )
	//			fprintf( fout, "},{{");
	//		else
	//			fprintf( fout,"},{");
	//		continue;
	//	}

	//	if( i == 0 )
	//		fprintf( fout, "},{{%d", seamVerReg[ i ][ 0 ] + 1);
	//	else
	//		fprintf( fout, "},{%d", seamVerReg[ i ][ 0 ] + 1);
	//	for( int j = 1;j < seamVerReg[ i ].size(); j ++ )
	//	{	
	//		fprintf( fout, ",%d", seamVerReg[ i ][ j ] + 1);
	//	}
	//}	

	//seamedgereg
	//add a temp var to put new one into it
	EdgeReg* seamEdgeReg = new EdgeReg[ maseamnum ];
	for( int i= 0; i < maseamnum; i ++)
	{
		seamEdgeReg[ i ].resize( nseamReg[ i ].vernum - 1 );
		for( int j = 0; j < nseamReg[ i ].vernum - 1;  j ++)
		{
			seamEdgeReg[ i ][j ].posInMeshEdge = nseamReg[ i ].edgelist[ j ].posInMeshEdge;
			for(unsigned int k = 0; k < nseamReg[ i ].edgelist[ j ].crspCtrEdges.size(); k ++ )
			{
				seamEdgeReg[ i ][ j ].crspCtrEdges.push_back( 
					nseamReg[ i ].edgelist[ j ].crspCtrEdges[ k ]);
			}
		}
	}


	int dir = 1;
	for( int i = 0; i < maseamnum; i ++ )
	{
		if( seamEdgeReg[ i ].size() == 0 )
		{
			if( i == 0 )
				fprintf( fout, "},{{");
			else
				fprintf( fout, "},{");
			continue;
		}
		for(unsigned int j = 0; j < seamEdgeReg[ i ].size(); j ++ )
		{
			if( j == 0 )
			{
				if( i == 0 )
				{
					fprintf( fout, "},{{{%d,{", seamEdgeReg[ i ][ 0 ].posInMeshEdge+1);
				}
				else
					fprintf( fout, "},{{%d,{", seamEdgeReg[ i ][ 0 ].posInMeshEdge + 1 );
			}
			else
			{
				fprintf( fout, "}},{%d,{", seamEdgeReg[ i ][ j ].posInMeshEdge + 1 );
			}
			for(unsigned int k = 0; k < seamEdgeReg[ i ][ j ].crspCtrEdges.size(); k ++)
			{
				dir = 0;
				if( seamEdgeReg[ i ][ j ].crspCtrEdges[ k ].samedirec )
					dir = 1;
				if(k != 0 )
					fprintf( fout, ",");
				fprintf( fout, "{%d,%d,%d}",
					seamEdgeReg[ i ][ j ].crspCtrEdges[ k ].facei + 1,
					seamEdgeReg[ i ][ j ].crspCtrEdges[ k ].edgei + 1,
					dir);
			}			
		}
		fprintf( fout, "}}");
	}

	//clear the temp val
	for( int i = 0; i < maseamnum; i ++ )
	{
		for(unsigned int j = 0; j < seamEdgeReg[ i ].size(); j ++ )
            seamEdgeReg[ i ][ j ].crspCtrEdges.clear();
		seamEdgeReg[ i ].clear();
	}
	delete []seamEdgeReg;

	////sheetverreg
	//for( int i = 0; i < masheetnum; i ++ )
	//{
	//	if( sheetVerReg[ i ].size() == 0 )
	//	{
	//		if( i == 0 )
	//			fprintf(fout, "}},{{");
	//		else
	//			fprintf( fout, "},{");
	//		continue;
	//	}

	//	for( int j = 0; j < sheetVerReg[ i ].size(); j ++ )
	//	{
	//		if( j == 0 )
	//		{
	//			if ( i == 0 )
	//				fprintf( fout, "}},{{%d",sheetVerReg[ i ][ j ] + 1);
	//			else
	//				fprintf( fout, "},{%d", sheetVerReg[ i ][ j ] + 1 );
	//			continue;
	//		}
	//		fprintf( fout, ",%d", sheetVerReg[ i ][ j ] + 1);
	//	}
	//}

	//sheetedgereg
	for( int i = 0; i < masheetnum; i ++ )
	{
		if( sheetEdgeReg[ i ].size() == 0)
		{
			if ( i ==  0 )
				fprintf(fout, "}},{{");
			else
				fprintf( fout, "},{");
			continue;
		}
		for(unsigned int j = 0; j < sheetEdgeReg[ i ].size(); j ++ )
		{
			if( j == 0 )	//the first edge on this sheet
			{
				if( i == 0 )		//first sheet
					fprintf( fout, "}},{{");
				else
					fprintf( fout, "},{");
				////edge position
				//if( i == 0 )
				//	fprintf( fout, "}},{{{%d,{", sheetEdgeReg[ i ][ j ].posInMeshEdge + 1 );
				//else
				//	fprintf( fout, "},{{%d,{", sheetEdgeReg[ i ][ j ].posInMeshEdge);
				////
				//continue;		
			}
			//output all the edges on this sheet
			if( j != 0 )	//not the first edge
				fprintf( fout, "}},");
			fprintf( fout, "{%d,{", sheetEdgeReg[ i ][ j ].posInMeshEdge + 1 );
			for(unsigned int k = 0; k < sheetEdgeReg[ i ][ j ].crspCtrEdges.size(); k ++ )
			{
				if( k != 0 )
					fprintf(fout, ",");
				dir = 0;
				if( sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].samedirec )
					dir = 1;
				fprintf( fout, "{%d,%d,%d}", 
					sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].facei + 1,
					sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].edgei + 1,
					dir);
			}
		}
		fprintf( fout, "}}");
	}

	fprintf( fout, "}}}");
	fclose( fout );
}