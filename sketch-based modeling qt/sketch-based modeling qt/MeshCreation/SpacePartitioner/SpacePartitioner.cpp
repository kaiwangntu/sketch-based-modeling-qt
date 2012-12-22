#include "../SpacePartitioner/SpacePartitioner.h"
void inline SpacePartitioner::getVerMark( float planeparam[ 4 ],
										 floatvector& ssver, int*& vermark)
{
	//go through all the vertices and mark them all
	int vernum = ssver.size()/ 3;
	float* vers = new float[ vernum * 3 ];
	for( int i = 0; i < 3 * vernum ;i ++)
	{
		vers[ i ] = ssver[ i ];
	}
	//kw: judge whether the vertex lies on the plane, or the positive side, or the negtive side
	for( int i = 0; i < vernum ;i ++)
	{
		float val = MyMath::dotProduct( planeparam, vers + 3*i );
		if( MyMath::isEqualInToler(val, planeparam[ 3 ] , POINT_ON_PLANE_TOLERANCE)) 
		{
			vermark[ i ] = 0;
		}
		else if( val < planeparam[ 3 ] )
			vermark[ i ] = -1;
		else
			vermark[ i ] = 1;		
	}

	delete []vers;
}

void inline SpacePartitioner::getEdgeMark( float planeparam[ 4 ], 
										  floatvector& ssver, int*& vermark,
											intvector& ssedge, int*& edgenewver, int*& edgenewedge)
{
	float ver1[ 3 ];
	float ver2[ 3 ];
	float interpt[ 3 ];
	int vernum = ssver.size()/3;
	int oldedgenum = ssedge.size()/2;
	int edgenum = oldedgenum;

	//go through each edge, split it when needed, and add the intersection point and the new edge into edgenew**
	int vind[ 2 ];
	int j = 0;	//the index of edgenewver, to avoid multiplication.
	for( int i = 0 ;i < oldedgenum ; i ++)
	{
		j += 2;	//index of the edgenewver and the vertices of the edge in ssedge

		vind[ 0 ] = ssedge[ j - 2 ]; vind[  1 ]= ssedge[ j - 1 ];
		int mul = vermark[ vind[ 0 ]] * vermark[ vind[ 1 ]];

		//no intersection point
		if( mul == 1)
			continue;

		//intersection point exists
		if( mul == -1 )
		{
			for( int k = 0; k < 3; k ++ )
			{
				ver1[ k ] = ssver[ 3 * vind[ 0 ] + k];
				ver2[ k ] = ssver[ 3 * vind[ 1 ] + k];
			}
			//compute intersection point
			interPt_PlaneEdge( planeparam, ver1, ver2, interpt);

			//split the edge
			ssver.push_back( interpt[ 0 ]);
			ssver.push_back( interpt[ 1 ]);
			ssver.push_back( interpt[ 2 ]);
			ssedge.push_back( vernum );
			if( vermark[ vind[ 0 ]] == -1 )
			{
				ssedge.push_back( vind[ 0 ]);
				ssedge[ j - 2 ] = vernum;
			}
			else
			{
				ssedge.push_back( vind[ 1 ] );
				ssedge[ j - 1 ] = vernum;
			}
			//set the two mark arrays
			edgenewver[ j - 2 ] = vernum;
			edgenewedge[ i ] = edgenum;
            
			vernum ++;
			edgenum ++;
			continue;
		}
        
		//at least one of them is 0
		if( vermark[ vind [ 0 ]] == 0)
		{
			edgenewver[ j - 2 ] = vind[ 0 ];
			if( vermark[ vind[ 1 ] ] == 0 )
				edgenewver[ j - 1 ] = vind[ 1 ];
		}
		else	//vermark[vind[1]] must be 0 and vind[ 0] is not 0
		{
			edgenewver[ j - 2 ] = vind[ 1 ];
		}
	}
}

void inline SpacePartitioner::getFaceMark(int*& vermark,int*& edgenewver, int*& edgenewedge,
										  intvector& ssedge,vector<intvector>&ssface, intvector& ssface_planeindex,
								int*& facenewedge, int*&facenewface )
{
	int edgenum = ssedge.size()/2;
	int oldfacenum= ssface.size();
	int facenum = oldfacenum;
	
    intvector singletouchedge;		//ont vertex of the edge is on the plane
	intvector doubletouchedge;	//both of the vertices of the edge are on the plane
	intvector splitedge;			//the edge intersects with the plane
	
    //go through all the faces, split it when needed
	for( int i = 0; i <  oldfacenum; i ++)
	{
		//gather all the touch edges, and gather all the intersection edges
		int tedgelen = ssface[ i ].size();

		for( int j = 0;  j < tedgelen ; j ++) 
		{
			int edgei = ssface[ i ][ j ];
			if( edgenewver[ edgei * 2 ] != -1 )
			{
				if( edgenewver[  edgei * 2 + 1 ] != -1)
					doubletouchedge.push_back( edgei);
				else
				{
					if( edgenewedge[ edgei ] != -1)
					{
						splitedge.push_back( edgei );
					}
					else
						singletouchedge.push_back( edgei );
				}					
			}
		}

//#ifdef debug		//run in my debug mode
//		cout<<"single touch edge number:"<<singletouchedge.size()<<endl
//			<<"double touch edge number:"<<doubletouchedge.size()<<endl
//			<<"splitedge size:"<<splitedge.size()<<endl;
//#endif 
		//case1: there exists one double touch edge
		if( doubletouchedge.size() == 1)
		{
			facenewedge[ i ] = doubletouchedge[ 0 ];
		}
		//case2: there exists one edge splitting the face
		else if( !splitedge.empty() || singletouchedge.size() == 4)
		{
			intvector newedgevers;	//new vertices of the new edge
			for(unsigned int j = 0; j < splitedge.size(); j ++)
			{
				newedgevers.push_back(edgenewver[ 2*splitedge[ j ] ]);
			}
            for(unsigned int j = 0; j < singletouchedge.size(); j ++)
			{
				newedgevers.push_back( edgenewver[ 2 * singletouchedge[ j ]]);
			}
			int newedgevernum = newedgevers.size();
#ifdef debug
	//		cout<<"number of new vertices for the new edge:"<<newedgevernum<<endl;
	//		for( int j = 0; j < newedgevernum; j ++)
	//			cout<<newedgevers[ j ]<<" ";
	//		cout<<"----------------------"<<endl;
#endif
			int v1, v2;
			v1 = newedgevers[ 0 ];
			for(unsigned int j = 1;j < newedgevers.size(); j++)
			{
				v2 = newedgevers[ j ];
				if( v2 != v1)
					break;
			}
			
			//add new edge and split the old face
			ssedge.push_back( v1 );
			ssedge.push_back( v2 );
			intvector oldfaceedges;
			intvector newfaceedges;
			oldfaceedges.push_back( edgenum );
			newfaceedges.push_back( edgenum );
			facenewedge[ i ] = edgenum;
			edgenum++;
			
			for( int j = 0; j < tedgelen; j ++ )
			{
				int edgei = ssface [ i ][ j ];
				//if split edge
				if( edgenewedge[ edgei ] != -1 )
				{
					oldfaceedges.push_back( edgei );
					newfaceedges.push_back( edgenewedge[ edgei ]);
					continue;
				}
                				
				int edgevers[ 2 ] = {ssedge[ edgei * 2 ], ssedge[ edgei * 2 + 1 ]};
				int tsum = vermark[ edgevers[ 0 ]] + vermark[edgevers[ 1 ]];
				if( (tsum == 2) || (tsum == 1) )	//1+1 0+1
				{
					oldfaceedges.push_back( edgei );
				}
				else if ( (tsum == -2) || (tsum == -1))	//-1 + -1 0 + -1
				{
					newfaceedges.push_back( edgei );
				}
			}
			ssface_planeindex.push_back( ssface_planeindex[ i ] );
			ssface.push_back( newfaceedges );
			ssface[ i ].clear();
			ssface[ i ] = oldfaceedges;

			//set face mark
			facenewface[ i ] = facenum;
			facenum++;

			//clear the temporary vectors
			newedgevers.clear();
			oldfaceedges.clear();
			newfaceedges.clear();			
		}
		singletouchedge.clear();
		doubletouchedge.clear();
		splitedge.clear();
	}
}

void SpacePartitioner::processSubspace(
									   int planei,
									   intvector& ssedge, vector<intvector>& ssface, intvector& ssface_planeindex,
									     vector<intvector>&ssspace, vector<intvector>& ssspace_planeside,
										 int*& vermark,
									   int*& facenewedge, int*& facenewface  )
{	
	int facenum = ssface.size();
	int spacenum = ssspace.size();

	//go through each subspace, split it when needed
	intvector newfaceedges;

	for( int subspacei = 0; subspacei < spacenum;  subspacei ++)
	{
		int tspacefacenum = ssspace[ subspacei ].size();

		for( int i = 0; i < tspacefacenum; i ++)
		{	
			int tcurfacenewedge = facenewedge[ ssspace[ subspacei ] [ i ]] ;
			if(tcurfacenewedge == -1)	
				continue;
			newfaceedges.push_back( tcurfacenewedge );
		}
		if( newfaceedges.size() == 0 )
			continue;

		//new face
		ssface.push_back( newfaceedges);
		ssface_planeindex.push_back( planei );

		//split the old subspace
		intvector oldsspacefaces;
		intvector newsspacefaces;
		intvector oldspacefacesides;
		intvector newspacefacesides;
		oldsspacefaces.push_back( facenum );		//index starts from 0, that's why before adding facenum
		newsspacefaces.push_back( facenum );
		facenum ++;											//one new face is added
		oldspacefacesides.push_back( 1 );		//the part of current subspace that is above current plane
		newspacefacesides.push_back( 0 );		//below the plane
		for( int i = 0; i < tspacefacenum; i ++)
		{
			int tfacei = ssspace[ subspacei ][ i ];
			int tnfacei = facenewface[ tfacei ];
			int tfaceside = ssspace_planeside[ subspacei ][ i ];
			if( tnfacei != -1 ) // the face is split into two
			{
				oldsspacefaces.push_back( tfacei );
				newsspacefaces.push_back( tnfacei );
				oldspacefacesides.push_back( tfaceside );
				newspacefacesides.push_back( tfaceside );
			}
			else	//the face is not split into two: case1, maybe touch some vertices, case2, no vertex is on the plane
			{
				//go through all the vertex on the face, stop until some vermark is 1 or -1
				int tedgenum = ssface[ tfacei ].size();
				int tvers[ 2 ];
#ifdef debug
		//		bool isset = false;
#endif
				for( int j = 0; j < tedgenum ;j ++)
				{
					int tedgei = ssface[ tfacei ][ j ];
					tvers[0] = ssedge[ 2*tedgei ];
					tvers[ 1 ] = ssedge[ 2*tedgei + 1];
					int tsum = vermark[ tvers[ 0 ]] +vermark[ tvers[ 1 ] ] ;
					if( tsum < 0 )	//the face should be in the newspacefaces
					{
						newsspacefaces.push_back( tfacei );
						newspacefacesides.push_back( tfaceside );
#ifdef debug
		//				isset = true;
#endif
						break;
					}
					else if( tsum > 0 )	//the face should be in the oldfspacefaces
					{
						oldsspacefaces.push_back( tfacei );
						oldspacefacesides.push_back( tfaceside );
#ifdef debug
		//				isset = true;
#endif
						break;
					}
				}
#ifdef debug
			//	if( !isset)	cout<<"face "<<tfacei <<" was not put into either old subspace or new subspace!"<<endl;
#endif
			}
		}
		ssspace.push_back( newsspacefaces );
		ssspace_planeside.push_back( newspacefacesides );
		ssspace[ subspacei ].clear();
		ssspace[ subspacei ] = oldsspacefaces;
		ssspace_planeside[ subspacei ].clear();
		ssspace_planeside[ subspacei ] = oldspacefacesides;

		newfaceedges.clear();
		oldspacefacesides.clear();
		oldsspacefaces.clear();
		newspacefacesides.clear();
		newsspacefaces.clear();
	}    
}
/**
* Function that take the parameter of the planes and cut the whole space into subspaces
* @param param the parameters of the planes, ax+by+cz=d, abcd represents one plane
* @param planenum the number of all the planes
* @param ver the resulting vertices of the subspaces, x, y , z, three number is one point
* @param vernum the number of the vertices
* @param edge the resulting edges of the subspaces, v1, v2, index of the two vertices in the ver
* @param edgenum the number of the edges in the subspace
* @param face the resulting faces, edge1,.... edgen, index of the edges in edge
* @param faceedgenum, how many edges are in the corresponding face
* @param facenum the faces number in all the subspaces
* @param faceside, which side of the plane this subspace is in
* @subspacenum the number of the subspaces
*
*/
void SpacePartitioner::insertOnePlane(float planeparam[ 4 ], int planei,
									  floatvector& ssver, intvector& ssedge,
									  vector<intvector>&ssface, intvector& ssface_planeindex, 
									  vector<intvector>& ssspace, vector<intvector>& ssspace_planeside)
{
	int vernum = ssver.size()/3;
	int edgenum = ssedge.size()/2;
	int facenum = ssface.size();
	int spacenum = ssspace.size();
	
	//step1. mark all the vertices
	int* vermark = new int[ vernum ];
	getVerMark( planeparam, ssver, vermark);

	//step2. mark all the edges, split it when needed
	//EVENT: NEW VERTEX, NEW EDGE
	//ssver, ssedge
	//mark of them
	//intersection point index for each edge
	//case 1: one vertex + one vertex -, then intersection point index, -1
	//case 2: both of them are + or -, then -1 -1
	//case 3: one of them is on plane, then touch vertex index, -1
	//case 4: both of them are on the plane, then touch vertex indices
	int* edgenewver = new int[ 2*edgenum ];
	//new edge index if the edge is split into two
	//case1: no intersection point or at least one vertex of the edge is on the plane, -1
	//case 2: intersection point exists, the negative new edge index is in it
	int* edgenewedge = new int[ edgenum ];
	int j = 0;
	for( int i = 0 ;i < edgenum ; i ++)
	{
		edgenewver[ j++ ] = -1;
		edgenewver[ j++ ] = -1;
		edgenewedge [ i ] = -1;
	}
	getEdgeMark( planeparam, ssver, vermark,ssedge,edgenewver,edgenewedge);

	//step3. mark all the faces, split it when needed
	//EVENT: NEW EDGE, NEW FACE
	//ssedge, ssface ssface_planeindex
	//mark of them
	int* facenewedge = new int[ facenum ];
	int* facenewface = new int[ facenum ];
	for( int i = 0; i < facenum ; i++)
	{
		facenewedge[ i ] = -1;
		facenewface[ i ] = -1;			
	}
	getFaceMark(vermark,edgenewver, edgenewedge,ssedge,ssface,ssface_planeindex,facenewedge, facenewface);

	//step4. go through all the subspaces, split it when needed
	//EVENT: NEW FACE, NEW SUBSPACE
	//ssface, ssspace, ssspace_planeside
	processSubspace(planei, ssedge, ssface, ssface_planeindex,ssspace, ssspace_planeside,vermark,facenewedge, facenewface );

	delete []vermark;
	delete []edgenewedge;
	delete []edgenewver;
	delete []facenewface;
	delete []facenewedge;
}


/**
* Function partition the space with the parameters of the planes
* @param param: parameters of all the planes  ax+by+cz=d, abcd represents one plane
* @param planenum: total planenumber
* @param boundingbox the bounding box of the space,to cut
* @param enlarge, the times to enlarge the bounding box
*/
void SpacePartitioner::partition(const int planenum, float*& param,const float boundingbox[ 6 ], const float enlarge[ 3 ],floatvector& ssver, intvector& ssedge, vector<intvector>&ssface,
					  intvector& ssface_planeindex, vector<intvector>& ssspace, vector<intvector>& ssspace_planeside)
{
	//int oldsize = planenum * 4; 
	//move them to the resize( ) part
	//step0 normalize all the planes parameters
	int oldsize = planenum * 4; 

	/*for( int i = 0; i < planenum; i ++)
	{
		float veclen = MyMath::vectorLen( param[ i * 4 ], param[ i*4 + 1 ], param[ i*4+ 2 ]);
		for(int j = 0; j < 4; j ++)
			param[ 4*i + j] = param[ 4*i + j]/veclen;
	}*/

	//step1, compute the boundingbox
	float nbx[ 6 ];
	float tmin, tmax;
	for( int i = 0; i < 3; i ++)
	{
		tmin = boundingbox[ i ];
		tmax = boundingbox[ i + 3 ];
		if( tmin == tmax)//kw: all curves on one plane
		{
			//kw: can set to -1000 or smaller to get a fatter shape
			//meanwhile, larger bounding box avoid partition error in progressive suface reconstruction
			tmin = tmin  - 10;//10
			//kw: can set to 1000 or larger to get a fatter shape
			tmax = tmax + 10;//10
		}
		nbx[ i ] = (( 1 + enlarge[ i ])*tmin + ( 1 - enlarge[ i])*tmax)/2;
		nbx[ i + 3 ] = (( 1 - enlarge[ i ])*tmin + ( 1 + enlarge[ i])*tmax)/2;
	}

	//step2. add 6 planes into the param list
	for( int i = 0; i < 24;i ++)
		param[ oldsize + i ] = 0;
	param[ oldsize ] = -1;
	param[ oldsize + 3 ] = -nbx[ 0 ];

	param[ oldsize + 5 ] = 1;
	param[ oldsize + 7 ] = nbx[ 4 ];
	
	param[ oldsize + 8 ] = 1;
	param[ oldsize + 11 ] = nbx[ 3 ];
	
	param[ oldsize + 13] = -1;
	param[ oldsize + 15 ] = -nbx[ 1 ];
	
	param[ oldsize + 18] = 1;
	param[ oldsize + 19] = nbx[ 5 ];
	
	param[ oldsize + 22] = -1;
	param[ oldsize + 23 ] = -nbx[ 2 ];

	//step3. initialize the first space
//	floatvector tssver;
//	intvector tssedge;
//	vector<intvector>tssface;
//  intvector tssface_planeindex;
//vector<intvector> tssspace;
//vector<intvector> tssspace_planeside;

	//ver
	//kw: virtual vertices composed of those of the new bounding boxes
	ssver.resize( 24 );
	ssver[ 0 ] = nbx[ 0 ]; ssver[ 1 ] = nbx[ 1 ]; ssver[ 2 ] = nbx[ 2 ];
	ssver[ 3 ] = nbx[ 3 ]; ssver[ 4 ] = nbx[ 1 ]; ssver[ 5 ] = nbx[ 2 ];
	ssver[ 6 ] = nbx[ 3 ]; ssver[ 7 ] = nbx[ 4 ]; ssver[ 8 ] = nbx[ 2 ];
	ssver[ 9 ] = nbx[ 0 ]; ssver[ 10 ] = nbx[ 4 ]; ssver[ 11 ] = nbx[ 2 ];
	ssver[ 12 ] = nbx[ 0 ]; ssver[ 13 ] = nbx[ 1 ]; ssver[ 14 ] = nbx[ 5 ];
	ssver[ 15 ] = nbx[ 3 ]; ssver[ 16 ] = nbx[ 1 ]; ssver[ 17 ] = nbx[ 5 ];
	ssver[ 18 ] = nbx[ 3 ]; ssver[ 19 ] = nbx[ 4 ]; ssver[ 20 ] = nbx[ 5 ];
	ssver[ 21 ] = nbx[ 0 ]; ssver[ 22 ] = nbx[ 4 ]; ssver[ 23 ] = nbx[ 5 ];

	//edge
	//kw: virtual edges composed of those of the new bounding boxes
	ssedge.resize( 24 );
	ssedge[ 0 ] = 0; ssedge[ 1 ] = 1;
	ssedge[ 2 ] = 1; ssedge[ 3 ] = 2;
	ssedge[ 4 ] = 2; ssedge[ 5 ] = 3;
	ssedge[ 6 ] = 3; ssedge[ 7 ] = 0;
	ssedge[ 8 ] = 4; ssedge[ 9 ] = 5;
	ssedge[ 10 ] = 5; ssedge[ 11 ] = 6;
	ssedge[ 12 ] = 6; ssedge[ 13 ] = 7;
	ssedge[ 14 ] = 7; ssedge[ 15 ] = 4;
	ssedge[ 16 ] = 0; ssedge[ 17 ] = 4;
	ssedge[ 18 ] = 1; ssedge[ 19 ] = 5;
	ssedge[ 20 ] = 2; ssedge[ 21 ] = 6;
	ssedge[ 22 ] = 3; ssedge[ 23 ] = 7;

	//face
	//kw: 6 faces of the bounding box, each row consists of 
	//the indices of edges of the bounding box
	ssface.resize( 6 );
	ssface_planeindex.resize( 6 );
	for( int i = 0;i  < 6; i ++)
	{
		ssface_planeindex[ i ] = -(i+1);	//which plane the face is on
		ssface[ i ].resize( 4 );
	}
	ssface[ 0 ][ 0 ] = 0;	ssface[ 0 ][ 1 ] = 1;	ssface[ 0 ][ 2 ] = 2;	ssface[ 0 ][ 3 ] = 3;
	ssface[ 1 ][ 0 ] = 4;	ssface[ 1 ][ 1 ] = 5;	ssface[ 1 ][ 2 ] = 6;	ssface[ 1 ][ 3 ] = 7;
	ssface[ 2 ][ 0 ] = 0;	ssface[ 2 ][ 1 ] = 4;	ssface[ 2 ][ 2 ] = 8;	ssface[ 2 ][ 3 ] = 9;
	ssface[ 3 ][ 0 ] = 1;	ssface[ 3 ][ 1 ] = 5;	ssface[ 3 ][ 2 ] = 9;	ssface[ 3 ][ 3 ] = 10;
	ssface[ 4 ][ 0 ] = 2;	ssface[ 4 ][ 1 ] = 6;	ssface[ 4 ][ 2 ] = 10;	ssface[ 4 ][ 3 ] = 11;
	ssface[ 5 ][ 0 ] = 3;	ssface[ 5 ][ 1 ] = 7;	ssface[ 5 ][ 2 ] = 8;	ssface[ 5 ][ 3 ] = 11;

	//subpace
	ssspace.resize( 1 );
	ssspace[ 0 ].resize( 6 );
	ssspace_planeside.resize( 1 );
	ssspace_planeside[ 0 ].resize( 6 );
	for( int i = 0; i < 6; i ++)
	{
		ssspace[ 0 ][ i ] = i;
		ssspace_planeside[ 0 ][ i ] = 0;	//negative side of the planes
	}

	//insert one plane by one
	for( int i = 0; i < planenum; i ++)
//	for( int i = 0 ; i< 1; i ++)
	{
		insertOnePlane(param + 4*i, i, ssver, ssedge, ssface, ssface_planeindex, ssspace, ssspace_planeside);
	}
//	for( int i = 0; i < planenum * 4 ; i++)
//		cout<<param[ i ]<<" ";
	cout<<endl;
}




void SpacePartitioner::wfilePartition(const char* fname,int ssvernum,float* ssver,int ssedgenum,int* ssedge,int ssfacenum,
									  int* ssfaceedgenum,	int** ssface,int* ssface_planeindex,int ssspacenum,int* ssspacefacenum,	int** ssspace,	int** ssspace_planeside)
{
	//{Length[resultVer], Length[resultEdge], Length[resultFace], Length[resultSubspace], resultVer,resultEdge, resultFace, resultSubspace} >> "partitionResult.txt";
	FILE*fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open the file : "<<fname<<"to write!"<<endl;
	}
	
	fprintf( fout, "{");
	//vertex number, edgenumber, facenumber, subspacenumber
	fprintf( fout,"%d,%d,%d,%d,{", ssvernum,ssedgenum, ssfacenum,  ssspacenum );
	//ver
	fprintf( fout, "{%f,%f,%f}", ssver[ 0 ], ssver[ 1 ], ssver[ 2 ]);
	for( int i = 1; i < ssvernum; i ++)
	{
		fprintf( fout, ",{%f,%f,%f}", ssver[3*i ], ssver[ 3*i + 1 ], ssver[ 3*i + 2 ]);
	}
	//edge
	fprintf( fout, "},{{%d,%d}", ssedge[ 0 ] + 1, ssedge[ 1 ] + 1 );
	for( int i = 1; i < ssedgenum; i ++)
	{
		fprintf( fout, ",{%d,%d}", ssedge[ 2*i ] +1, ssedge[ i * 2 + 1 ] + 1);
	}
	//face
	fprintf( fout, "},{{{%d", ssface[ 0 ][ 0 ] + 1);
	for( int i = 1; i < ssfaceedgenum[ 0 ];  i ++)
	{
		fprintf(fout, ",%d", ssface[ 0 ][ i ] + 1);
	}
	int faceside = ssface_planeindex[ 0 ];
	if( faceside >= 0 )
		faceside += 1;
	fprintf( fout, "},{%d}}", faceside);
	for( int i = 1; i < ssfacenum; i ++)
	{
		fprintf( fout, ",{{%d", ssface[ i ][ 0 ] + 1);
		for( int j = 1; j < ssfaceedgenum[ i ]; j ++)
		{
			fprintf( fout, ",%d", ssface[ i ][ j ] + 1);
		}
		faceside = ssface_planeindex[ i ];
		if( faceside >= 0 )
			faceside += 1;
		fprintf( fout,  "},{%d}}", faceside );
	}
	//subspace
	fprintf( fout, "},{{{%d", ssspace[ 0 ][ 0 ]+ 1);
	for( int i = 1; i < ssspacefacenum[ 0 ]; i ++)
	{
		fprintf( fout, ",%d", ssspace[0][ i ] + 1);
	}
	fprintf( fout, "},{%d", ssspace_planeside[ 0 ][ 0 ]);
	for( int i = 1; i < ssspacefacenum[ 0 ]; i ++)
	{
		fprintf( fout, ",%d", ssspace_planeside[ 0 ][ i ]);
	}
	fprintf( fout, "}}");
	for( int i = 1; i < ssspacenum; i ++)
	{
		fprintf( fout, ",{{%d", ssspace[ i ][ 0 ] + 1 );
		for( int j = 1; j < ssspacefacenum[ i ]; j ++)
			fprintf( fout, ",%d", ssspace[ i ][ j ] + 1);
		fprintf( fout, "},{%d", ssspace_planeside[ i ][ 0 ]);
		for( int j = 1; j < ssspacefacenum[ i ]; j ++)
			fprintf(fout, ",%d", ssspace_planeside[ i ][ j ]);
		fprintf(fout, "}}");
	}
	fprintf( fout, "}}");
	fclose( fout );
}

SpacePartitioner::SpacePartitioner()
{

}

SpacePartitioner::~SpacePartitioner()
{

}

