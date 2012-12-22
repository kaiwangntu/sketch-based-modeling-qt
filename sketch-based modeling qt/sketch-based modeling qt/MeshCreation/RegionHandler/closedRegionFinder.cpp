#include "../RegionHandler/regionHandler.h"

//each cluster is is a set of neighboring boundaries
void regionHandler::findClosedBoundaryCluster(
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
				//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
	float zdir[ 3 ],
	float xdir[ 3 ],
	sheetRegion& region,
	intvector& boundseg
		)
{
	int oldedgenum = shtEcmpos_ivec.size()/5;
	shtEcmpos_ivec.resize( oldedgenum * 10, -1 );

	vector<intvector> vincidedge_vec_vec;
//	vector<floatvector> edgeangl_vec_vec;
	int nedgenum = oldedgenum * 2;
	int vnum = shtVpos_fvec.size() / 3;
	vincidedge_vec_vec.resize( vnum);
  //  edgeangl_vec_vec.resize( vnum );

	//step1. add twin edge and set twin edge for each edge
	int ind1 = 0;
	int ind2 = oldedgenum;
	for( int i = oldedgenum*5, i2 = 0; i < nedgenum*5; i +=5, i2 += 5)
	{
		//add twin edge into edgelist
		shtEcmpos_ivec[ i ] = shtEcmpos_ivec[ i2 + 1 ];
		shtEcmpos_ivec[ i+ 1 ] = shtEcmpos_ivec[ i2 ];

		//set twin edge
		shtEcmpos_ivec[ i2 + 2 ] = ind2;
		shtEcmpos_ivec[ i + 2 ] = ind1;
		
		//add incident edge into vertex incident edge list
		vincidedge_vec_vec[ shtEcmpos_ivec[ i2 + 1 ]].push_back( ind2 );
		vincidedge_vec_vec[ shtEcmpos_ivec[ i2 ]].push_back( ind1 );

		ind1 ++;
		ind2 ++;
	}

	//step2. for each vertex, sort the incident edges
	for( int i = 0; i < vnum; i ++ )
	{
		//compute the angles of all the incident edges
		//edgeangl_vec_vec[ i ].resize( vincidedge_vec_vec[ i ].size() );	

		//all the edges incident with the vertex
		int edgenum = vincidedge_vec_vec[ i ].size();
		int* edgei = new int[ edgenum ];
	//	float* edgevec = new float[ edgenum*3 ];
		float* edgeval = new float[ edgenum ];
		for( int j = 0; j < edgenum; j ++ )
		{
			//the edge indices		
			edgei[ j ] = vincidedge_vec_vec[ i ][ j ];
			int veri[ 2 ] = { shtEcmpos_ivec[ edgei[ j ] * 5 ], 
				shtEcmpos_ivec[ edgei[ j ]*5 + 1 ]};

			//edge vector
			float edgevec[ 3 ];
			for( int k = 0;k < 3; k ++ )
			{
				edgevec[ k ] = 
					shtVpos_fvec[ 3*veri[ 1 ] + k ] - shtVpos_fvec[ 3*veri[ 0 ] + k];
			}

			//dotproduct between the vector and xaxis
			edgeval[ j ] = getAngleVecX( edgevec, xdir, zdir );
		}
		
		//sort the edgeval together with the edge index. select sort
		for( int k1 = 0; k1 < edgenum; k1 ++ )
		{
			//find the min val
			int mini = k1;
			for( int k2 = k1 + 1; k2 < edgenum; k2 ++ )
			{
				if ( edgeval[ k2 ] < edgeval[ mini ])
				{
					mini = k2;
				}
			}

			if( mini == k1 )
			{
				continue;
			}
			
			float tval = edgeval[ mini ];
			edgeval[ mini ] = edgeval[ k1 ];
			edgeval[ k1 ] = tval;

			int tedgei = edgei[ mini ];
			edgei[ mini ] = edgei[ k1 ];
			edgei[ k1 ] = tedgei;
		}

		//for each edge, set the next edge for it
		for( int k1 = 0; k1 < edgenum; k1 ++ )
		{
			int ei = edgei[ k1 ];
			int twei = shtEcmpos_ivec[ 5*ei + 2];	//twin edge
			if ( k1 == 0 )
                shtEcmpos_ivec[ 5*twei + 3 ] = edgei[ edgenum - 1 ];	//set next edge
			else
				shtEcmpos_ivec[ 5*twei + 3 ] = edgei[ k1 -1 ];	//set next edge
		}

		//clear all the temp vars
		delete []edgei;
		delete []edgeval;

		//clear incident vector
		vincidedge_vec_vec[ i ].clear();
	}
	vincidedge_vec_vec.clear();
	//step3. along the next edge, find the closed boundary, and also cluster
		//the boundaries according to their neighborhood relationship
//	intvector boundseg;
	int* edgemark = new int[ nedgenum ];
	for( int i = 0; i < nedgenum; i ++ )	//not visited, 0
		edgemark[ i ] = 0;

	stack<int> estack;
	estack.push( 0 );
	int curbound = -1;
	int firstei;
	int twei;	//twin edge
	int nxei;	//next edge
	bool exist;
    while(true)
	{
		//find new boundary cluster
		while( true )
		{
			exist = false;
			while( !estack.empty() )
			{
				firstei = nxei = estack.top();
				estack.pop();
				if( edgemark[ firstei ] == 0 )
				{
					exist = true;
					break;	
				}
			}
			if( !exist )	//boundary cluster is over
			{
				boundseg.push_back( curbound );
				break;
			}

			//start a brand new boundary
			curbound ++;		

			//from current edge to find one closed boundary
			region.boundaries.resize( curbound + 1);
			region.boundaries[ curbound ].push_back( nxei );
			edgemark[ nxei ] = 1;	//added into boundary
			shtEcmpos_ivec[ 5*nxei  + 4] = curbound;
			twei = shtEcmpos_ivec[ 5*nxei + 2 ];
			if( edgemark[ twei ] == 0 )
			{
				//edgemark[ twei ] = 1;
				estack.push( twei );
			}
			nxei = shtEcmpos_ivec[ 5*nxei + 3 ];			
			while( nxei != firstei )
			{
				edgemark[ nxei ] = 1;
				region.boundaries[ curbound ].push_back( nxei );
				shtEcmpos_ivec[ 5*nxei + 4] = curbound;
				//curedge = nxei;
				twei = shtEcmpos_ivec[ 5*nxei + 2 ];
				if( edgemark[ twei ] == 0 )
				{
					//edgemark[ twei ] = 1;
					estack.push( twei );
				}
				nxei = shtEcmpos_ivec[ 5*nxei + 3 ];
			}
		}

		//find if there exist any edge not processed
		exist = false;
		for( int i = 0; i < nedgenum; i ++ )
		{
			if( edgemark[ i ] == 0 )
			{
				exist = true;
				estack.push( i );
				break;
			}
		}
		if( !exist ) //all edges have been processed
			break;
	}

	delete []edgemark;
}

//return the minimal distance from vertex to edge
//the minimal type is saved in type
//and the type is vertex, save the vertex index in veri
float regionHandler::distVer2Edge(
	 float ver[ 3 ],
	floatvector& shtVpos_fvec,
	 int edgevi[ 2 ],
	 //type - 0, normal case, minimal distance to edge is real distance to line edge lies on
	 //tyep - 1, the minimal distance to one vertex of the edge
	 int& type,
	//type == 1, veri saved the vertex index
	 int& veri)
{
	float normvec[ 3 ];
	float vec1[ 3 ];
	float vec2[ 3 ];
	for( int i = 0; i < 3; i ++ )
	{
		normvec[ i ] = 
			shtVpos_fvec[ 3*edgevi[ 1 ] + i ] - shtVpos_fvec[ 3*edgevi[ 0 ] + i ];
		vec1[ i ] = shtVpos_fvec[ 3*edgevi[ 0 ] + i] - ver[ i ];
		vec2[ i ] = shtVpos_fvec[ 3*edgevi[ 1 ] + i] - ver[ i ];
	}
	float dist1 = MyMath::dotProduct( vec1, normvec );
	float dist2 = MyMath::dotProduct( vec2, normvec );
	
	if( MyMath::getSign( dist1 ) * MyMath::getSign( dist2 ) == -1 )	//different sign.
	{
		type = 0;
		float dist = 0;
		float normveclen = 0;
		for( int i = 0; i < 3; i ++ )
		{
			dist = dist + vec1[ i ]*vec1[ i ];
			normveclen = normveclen + normvec[ i ] * normvec[ i ];
		}
		
		dist = dist - pow((float)( dist1),2)/normveclen;
		dist = sqrt( dist );
		return dist;
	}

	//the minimal distance is to one of the vertex
	type = 1;
	if( abs( dist1 ) < abs( dist2 ))
	{
		veri = edgevi[ 0 ];
		return MyMath::vectorlen( vec1 );
	}
	veri = edgevi[ 1 ];
	return MyMath::vectorlen( vec2 );
}

//decide if the vertex is in the region or not
bool regionHandler::isInBoundary(
	float ver[ 3 ],
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
	intvector& oneboundary,
	float zdir[ 3 ]) 
{
	float mindis, curdis;
	int mintype, curtype;
	int minei, curei;
	int minvi, curvi;

	//compute the minimal distance to vertex or edge
	for(unsigned int i = 0; i < oneboundary.size(); i ++)
	{
		curei = oneboundary[ i ];
		int edgevi[ 2 ] ={shtEcmpos_ivec[ 5*curei ], shtEcmpos_ivec[ 5*curei + 1 ]};
		curdis = distVer2Edge( ver, shtVpos_fvec, edgevi, curtype, curvi );

		if( i == 0 )
		{
			mindis = curdis;
			mintype = curtype;
			minei = i;
			minvi = curvi;
			continue;
		}
		
		//not the first edge
		if( curdis < mindis  )
		{
			mindis = curdis;
			mintype = curtype;
			minei = i;
			minvi = curvi;
		}
	}
	
	//if type == 0, minimal distance to the edge
	if( mintype == 0 )
	{
		//if the vertex lies to the left of the edge or not
		float dir[ 3 ];
		float vec[ 3 ];
		minei = oneboundary[ minei ] ;	//from index in boundary to the real edge index
		int edgevi[ 2 ] ={shtEcmpos_ivec[ 5*minei ], shtEcmpos_ivec[ 5*minei + 1 ]};
		
		for( int i = 0; i< 3; i ++ )
		{
			dir[ i ] = shtVpos_fvec[ 3*edgevi[ 1 ] + i ] - shtVpos_fvec[ 3*edgevi[ 0 ] + i];
			vec[ i ] = ver[ i ] - shtVpos_fvec[ 3*edgevi[ 0 ] + i ];
		}

		//left or not?
		float normdir[ 3 ];
		MyMath::crossProductNotNorm( dir, vec, normdir );
		if( MyMath::dotProduct( normdir, zdir ) > 0 )
		{
			//////////////////////////////////////////////////////////////////////////
			//cout<<"the nearest is one edge: return TRUE!!! "<<endl;
			////output the vertex and the nearest edge
			//cout<<"position of the checking vertex:"<<ver[ 0 ]<<","<<ver[ 1 ]<<","<<ver[2]<<endl;
			//cout<<"position of the vertices of the edge:"<<endl;
			//cout<<shtVpos_fvec[ 3*edgevi[ 0] + 0 ] <<","
			//	<<shtVpos_fvec[ 3*edgevi[ 0 ] + 1 ] <<","<<shtVpos_fvec[ 3*edgevi[ 0 ] + 2] <<"|"
			//	<<shtVpos_fvec[ 3*edgevi[ 1 ] + 0 ] <<","
			//	<<shtVpos_fvec[ 3*edgevi[ 1 ] + 1 ] <<","
			//	<<shtVpos_fvec[ 3*edgevi[ 1 ] + 2 ] <<endl;
			//////////////////////////////////////////////////////////////////////////
			return true;
		}
		//////////////////////////////////////////////////////////////////////////
		//cout<<"the nearest is one edge: return FALSE!!! "<<endl;
		//////////////////////////////////////////////////////////////////////////
		return false;        
	}

	//else, type == 1, minimal distance is to one vertex
	//decide if the vertex lies between two neighboring edges
	int tei[ 2 ];
	int edgelen = oneboundary.size();
    if( shtEcmpos_ivec[ oneboundary[minei] * 5 ] == minvi )
	{
		tei[ 0 ] = oneboundary[ (minei - 1 + edgelen) % edgelen ];
		tei[ 1 ] = oneboundary[ minei ];
	}
	else
	{
		tei[ 0 ] = oneboundary[ minei ];
		tei[ 1 ] = oneboundary[ (minei + 1) % edgelen ];
	}

	//compoute the two angles
	//angle from the second edge to the minvi-ver, and angle from second edge to the first edge

	//use the second edge as x axis
	float xdir[ 3 ];

	int evi[ 2 ] = {shtEcmpos_ivec[ 5*tei[ 1 ]], shtEcmpos_ivec[5*tei[ 1 ] + 1]};
	float thirdv = shtEcmpos_ivec[ 5*tei[ 0 ]];
	float vec[ 3 ];
	float vec1[ 3 ];
	float vec2[ 3 ];
	for( int i = 0; i < 3; i ++ )
	{
		vec[ i ] = shtVpos_fvec[ 3*evi[ 1 ] + i] - shtVpos_fvec[ 3*evi[ 0 ] + i ];
		vec1[ i ] = ver[ i ] - shtVpos_fvec[ 3*evi[ 0 ] + i ];
		vec2[ i ] = shtVpos_fvec[ 3*thirdv + i ] - shtVpos_fvec[ 3*evi[ 0 ] + i ];		
	}

	MyMath::normalize(vec, xdir);
	
	//first angle

	float ang1 = getAngleVecX( vec1, xdir , zdir );
	float ang2 = getAngleVecX( vec2, xdir, zdir);
	////////////////////////////////////////////////////////////////////////////
	//cout<<"zdir:"<<zdir[ 0 ]<<","<<zdir[ 1 ]<<","<<zdir[2]<<endl;
	//cout<<"angl:"<<ang1<<"\tang2:"<<ang2<<endl;
	//	//////////////////////////////////////////////////////////////////////////    
	if( ang1 < ang2 )
	{
		//////////////////////////////////////////////////////////////////////////
		/*cout<<"the nearest is one vertex: return TRUE!!! "<<endl;
		cout<<"position of the checking vertex:"<<ver[ 0 ]<<","<<ver[ 1 ]<<","<<ver[2]<<endl;
		cout<<"posiition of the three vertices:"<<endl;
		cout<<shtVpos_fvec[ 3*thirdv] <<","
			<<shtVpos_fvec[ 3*thirdv + 1 ]<<", "
			<<shtVpos_fvec[ 3*thirdv + 2 ]<<endl;
		cout<<shtVpos_fvec[ 3*evi[ 0 ] + 0] <<" , "
				<<shtVpos_fvec[ 3*evi[ 0 ] + 1] <<" , "
				<<shtVpos_fvec[ 3*evi[ 0 ] + 2] <<endl;
		cout<<shtVpos_fvec[ 3*evi[ 1 ] + 0] <<","
				<<shtVpos_fvec[ 3*evi[ 1 ] + 1] <<","
				<<shtVpos_fvec[ 3*evi[ 1 ] + 2] <<endl;*/

		/*cout<<"position of the vertices of the edge:"<<endl;
		cout<<shtVpos_fvec[ 3*edgevi[ 0] + 0 ] <<","
			<<shtVpos_fvec[ 3*edgevi[ 0 ] + 1 ] <<","<<shtVpos_fvec[ 3*edgevi[ 0 ] + 2] <<"|"
			<<shtVpos_fvec[ 3*edgevi[ 1 ] + 0 ] <<","
			<<shtVpos_fvec[ 3*edgevi[ 1 ] + 1 ] <<","
			<<shtVpos_fvec[ 3*edgevi[ 1 ] + 2 ] <<endl;*/
		//////////////////////////////////////////////////////////////////////////
		return true;
	}
	//////////////////////////////////////////////////////////////////////////
	//cout<<"the nearest is one vertex: return FALSE!!! "<<endl;
	//////////////////////////////////////////////////////////////////////////
	return false;   
}

//find the boundary enclosing one region
//if fail, return false
bool regionHandler::getRegionFromBoundaryCluster(
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
	//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
//	intvector& shtMat_ivec,
	float zdir[ 3 ],
	float xdir[ 3 ],
	sheetRegion& region,
	intvector& boundseg  )
{
	//////////////////////////////////////////////////////////////////////////
	//write the boundary out
//	writeRegion_db(
//		shtVpos_fvec,	shtEcmpos_ivec, zdir,
//		 xdir,	 region,boundseg		);
	////////////////////////////////////////////////////////////////////////////

	//initialization
	region.regions.resize( boundseg[ 0 ] + 1 );
	//region.regions[ 0 ].resize( boundseg[ 0 ] + 1);
	for( int i = 0; i <= boundseg[ 0 ]; i ++ )
		region.regions[ i ].push_back( i ) ;	//ith boundary

	//go through each boundary cluster
	float tei;
	float tvi;
	float tver[ 3 ];
	float tbouni[ 2 ];
    for(unsigned int i = 0; i < boundseg.size() - 1; i ++ )	
	{
		tbouni[ 0 ] = boundseg[ i ] + 1;
		tbouni[ 1 ] = boundseg[ i + 1 ];
		//////////////////////////////////////////////////////////////////////////
		//cout<<"boudnary segment:"<<tbouni[ 0 ] <<" "<<tbouni[ 1 ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		//step1. find in which region, the current boundary cluster is in
		int regioni = -1;
		tei = region.boundaries[ tbouni[ 0 ]][ 0 ];
        tvi = shtEcmpos_ivec[ 5*tei ];
		for( int j= 0; j < 3; j ++ )
			tver[ j ] = shtVpos_fvec[ 3*tvi + j ];
		for(unsigned int j = 0; j < region.regions.size(); j ++ )
		{
			//////////////////////////////////////////////////////////////////////////
			//cout<<"check the exisitng region: "<<j<<"check the boundaries in it!"<<endl;
			//////////////////////////////////////////////////////////////////////////
			bool suc = true;
			for(unsigned int k = 0; k < region.regions[ j ].size(); k++ )
			{				
				//////////////////////////////////////////////////////////////////////////
				//cout<<"boundary :"<<k<<"\t";
				//////////////////////////////////////////////////////////////////////////
				suc = isInBoundary( tver, shtVpos_fvec, shtEcmpos_ivec,
					region.boundaries[ region.regions[ j ][ k ] ], zdir	);
				if( !suc )
					break;
			}
			//////////////////////////////////////////////////////////////////////////
			//cout<<endl;
			//////////////////////////////////////////////////////////////////////////
			if( suc )
			{
				regioni = j;
				break;
			}
		}

		if( regioni == -1 )
		{
			cout<<"ERROR! Unable to find a region to put current boundary cluster!"<<endl;
			return false;	//not able to find one region
		}
		//////////////////////////////////////////////////////////////////////////
		//cout<<"--find the region it is in! "<<regioni<<endl;
		//////////////////////////////////////////////////////////////////////////

		//step2. for all the boundaries in the regioni, find the boundary
		//in current boundary cluster it is in
		int boundnum = region.regions[ regioni ].size();
		int* boundi = new int[ boundnum ];
		//////////////////////////////////////////////////////////////////////////
		//cout<<"in the found region, there are "<<boundnum<<" boundaries in it!"<<endl;
		//cout<<"to checked boundaries are:";
		//for( int j = 0; j < boundnum; j ++ )
		//{
		//	cout<<region.regions[regioni][ j ]<<",";
		//}
		//cout<<endl;
		////////////////////////////////////////////////////////////////////////////
		for( int j = 0; j < boundnum; j ++ )
		{
			boundi[ j ] = -1;
			int curbound = region.regions[ regioni ][ j ];
			tei = region.boundaries[ curbound ][  0 ];
			tvi = shtEcmpos_ivec[ 5*tei ];
			////////////////////////////////////////////////////////////////////////////
			//cout<<"checking vertex: from boundary"<<curbound<<"edge:"
			//	<<tei <<" edge"<<tvi<<" vertex"<<endl;
			////////////////////////////////////////////////////////////////////////////
			for(int k = 0; k < 3; k ++ )
				tver[ k ] = shtVpos_fvec[ 3*tvi + k ];
	
			//////////////////////////////////////////////////////////////////////////
			//cout<<"the position of it is:"<<tver[ 0 ] <<","<<tver[ 1]<<","<<tver[ 2 ]<<endl;
			//////////////////////////////////////////////////////////////////////////
			//find which boundary in the boundary cluster it is in
			for( int k = tbouni[ 0 ]; k <= tbouni[ 1 ]; k ++ )
			{
				//////////////////////////////////////////////////////////////////////////
				//cout<<"check boundary "<<k<<" NOW!!!!"<<endl;
				//////////////////////////////////////////////////////////////////////////
				if( isInBoundary( tver, shtVpos_fvec, shtEcmpos_ivec, 
					region.boundaries[ k ], zdir ))
				{
					//////////////////////////////////////////////////////////////////////////
					//cout<<"i am going to set the buondary for it! it it   "<<k<<endl;
					//////////////////////////////////////////////////////////////////////////
					boundi[ j ] = k;
					break;
				}
			}
			if( boundi[ j ] == -1)
			{
				return false;	//unable to find one boundary it is in
			}
		}

		//step3. put the new boundaries in the boundary cluster into regions
		//sort the boundaries in regioni into increasing order according to the 
		//corresponding boundary index in the new boundary cluster
		for( int j = 0; j < boundnum; j ++ )
		{
			int mini =  j ;
			for( int k = j + 1; k < boundnum; k ++ )
			{
				if( boundi[ k ] < boundi[ mini ] )
					mini = k;
			}
			if( mini == j )continue;

			int ti = boundi[ mini ];
			boundi[ mini ] = boundi[ j ];
			boundi[ j ] = ti;

			ti = region.regions[ regioni ][ j ];
			region.regions[ regioni ][ j ] = region.regions[ regioni ][ mini ];
			region.regions[ regioni ][ mini ] = ti;
		}

		//put the first cluster into the old region, and erase all the other ones,
		//and add other ones with corresponding boundaries as new regions
        intvector tseg;
	//	tseg.push_back( -1 );
		for( int j = 0; j < boundnum - 1; j++ )
		{
			if( boundi[ j ] != boundi[ j + 1 ])
			{
				tseg.push_back( j );
			}
		}
		tseg.push_back( boundnum - 1);
		int tsegsize = tseg.size();

		int oldregionsize = region.regions.size();
		//////////////////////////////////////////////////////////////////////////
		/*cout<<"the size of the regions:"
			<<"oldregionsize:"<<oldregionsize
			<<"newsize"<<oldregionsize + tbouni[ 1 ] - tbouni[ 0 ] <<endl;
		cout<<"boundi[0]"<<boundi[ 0 ]
		<<"the two boundary limits:"<<tbouni[ 0 ]<<" "<<tbouni[ 1 ]<<endl;*/

		//////////////////////////////////////////////////////////////////////////
		region.regions.resize( oldregionsize + tbouni[ 1 ] - tbouni[ 0 ] );
		int j1 = 0;
		int j2 = 1;
		for( int j = tbouni[ 0 ] ; j  <= tbouni[ 1 ]; j++ )
		{
			if( j == boundi[ 0 ]) continue;
			region.regions[ oldregionsize + j1 ].push_back( j );
			if( j2 < tsegsize && boundi[ tseg[ j2 ]] == j )
			{
				for( int k = tseg[ j2 -1 ] + 1; k <= tseg[ j2 ]; k++ )
				{
					region.regions[ oldregionsize + j1].push_back( 
						region.regions[ regioni ][ k ]	);
				}
				j2 ++;
			}
			j1 ++;
		}

		
		/*for( int j = 1; j < tsegsize; j ++ )
		{
			int starti = tseg[ j - 1] + 1;
			int endi = tseg[ j ];
			int bi = boundi[ tseg[ starti ] ] - tbouni[ 0 ];
			for( int k = starti; k <= endi; k ++ )
			{
				region.regions[ bi ].push_back( region.regions[ regioni ][ k ]);
			}
		}*/

		/*intvector::iterator iter = region.regions[ regioni ].begin() + tseg[ tsegsize - 1 ] + 1;
		if( iter != region.regions[ regioni ].end()) 
		{
			region.regions[regioni].erase( iter, region.regions[ regioni ].end() );
		}*/
		if( tsegsize != 1 )
		{
			intvector::iterator iter = region.regions[ regioni ].begin() + tseg[ 0 ] + 1;
			region.regions[regioni].erase( iter, region.regions[ regioni ].end() );
		}
		region.regions[ regioni ].push_back( boundi[ 0 ]);

		//clear temp var
		delete []boundi;
		tseg.clear();
	}

	return true;
}

bool regionHandler::setOutMostRegion(
	sheetRegion& region,
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,
	int seamedgenum,
	int seamonsheetnum,
	int*& seamonsheet,
	int*& maseam,
	float*& majpt,
	float zdir[ 3 ]
					  )
{
	int sheetedgenum = shtEcmpos_ivec.size()/10;
	int regionnum = region.regions.size();
	if( regionnum == 2 )
	{
		//compute the center of the sheet
		float cent[ 3 ] = {0,0,0};
		for( int i = 0; i < seamonsheetnum; i ++ )
		{
			int seami = seamonsheet[ i ];
			int* jpt = maseam + 2*seami;
			for( int j = 0; j < 3; j ++ )
			{
				cent[ j ] += (majpt[ jpt[ 0 ] * 3 + j ]/(seamonsheetnum*2));
				cent[ j ] += (majpt[ jpt[ 1 ] * 3 + j ]/(seamonsheetnum*2));
			}
		}

		int tbi = region.regions[ 0 ][ 0 ];
		int tei = region.boundaries[ tbi ][ 0 ];
		int tvi[ 2 ] = { shtEcmpos_ivec[ 5*tei], shtEcmpos_ivec[ 5*tei + 1 ]};
		float vec1[ 3 ];
		float vec2[ 3 ];
		for( int i = 0; i < 3; i ++ )
		{
			vec1[ i ] = shtVpos_fvec[ 3*tvi[ 1 ] + i ] - shtVpos_fvec[ 3*tvi[ 0 ] + i ];
			vec2[ i ] = cent[ i ] - shtVpos_fvec[ 3*tvi[ 0 ] + i ];
		}
        
		float normdir[ 3 ];
		MyMath::crossProductNotNorm( vec1, vec2, normdir );
		if( MyMath::dotProduct( normdir, zdir ) > 0 )
			region.outregion = 1;
		else
			region.outregion = 0;
		return true;
	}

	bool suc = true;
	for(int i = 0; i < regionnum; i ++ )
	{
		if( region.regions[ i ].size() != 1 )	//impossible
			continue;
		int tbi = region.regions[ i ][ 0 ];
		int benum = region.boundaries[ tbi ].size();
		suc = true;
		for( int j = 0; j < benum; j ++ )
		{
			int tei = region.boundaries[ tbi ][ j ];
			if( tei >= seamedgenum && tei < sheetedgenum )
			{
				suc = false;
				break;
			}
			if( tei >= sheetedgenum + seamedgenum )
			{
				suc = false;
				break;
			}
		}
		if( suc )
		{
			region.outregion = i;
			break;
		}
	}

	if( suc )
		return true;
	return false;
}

void regionHandler::setRegionNeighbr(
	intvector& shtEcmpos_ivec,
	sheetRegion& region
					  )
{
	int bnum = region.boundaries.size();
	int* bound2reg = new int[ bnum ];
	int rnum = region.regions.size();
	for( int i = 0; i < rnum; i ++ )
	{
		for(unsigned int j = 0; j < region.regions[ i ].size(); j ++ )
		{
			bound2reg[ region.regions[ i ][ j ]] = i;
		}
	}

	region.regionneighbrs.resize( rnum );
	//go through each edge, and set the region neighbors
	int edgenum = shtEcmpos_ivec.size()/10;
	for( int i = 0; i < edgenum; i ++ )
	{
        int twei = shtEcmpos_ivec[ 5*i + 2 ];
		int bi[ 2 ] ={ shtEcmpos_ivec[ 5*i + 4 ], shtEcmpos_ivec[ 5*twei + 4]};
		int regi[ 2 ] ={ bound2reg[ bi[ 0 ] ], bound2reg[ bi[ 1 ] ] };
		region.regionneighbrs[ regi[ 0 ]].insert( regi[ 1 ] );
		region.regionneighbrs[ regi[ 1 ]].insert( regi[ 0 ] );
		shtEcmpos_ivec[ 5*i + 4 ] = regi[ 0 ];
		shtEcmpos_ivec[ 5*twei + 4 ] = regi[ 1 ];
	}

	delete []bound2reg;
}

//if fail, return false
bool regionHandler::findClosedRegions(
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
	//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
//	intvector& shtMat_ivec,
	float zdir[ 3 ],
	float xdir[ 3 ],
	sheetRegion& region,
	int seamedgenum,
	int seamonsheetnum,
	int*& seamonsheet,
	int*& maseam,
	float*& majpt  )
{
	//step1. find boundary cluster
	intvector boundseg;
	findClosedBoundaryCluster(
		shtVpos_fvec,shtEcmpos_ivec, zdir, xdir, region, boundseg );

	//right one is below
	//step2. put boundaries into regions
	if( !getRegionFromBoundaryCluster(
		shtVpos_fvec,shtEcmpos_ivec, 
		zdir, xdir, region, boundseg ) )
	{
		cout<<"Not able to find boundaries!"<<endl;
		return false;
	}
	boundseg.clear();

	//step3. find the outer most region
	if( !setOutMostRegion(region,
		shtVpos_fvec,shtEcmpos_ivec,
		seamedgenum,seamonsheetnum,
		seamonsheet, maseam,majpt,zdir))
	{
		//////////////////////////////////////////////////////////////////////////
		//write out the configured region at current time
		FILE* fout = fopen("aaaregion.txt","w");
		intvector tempshtVposInMV_ivec ;
        intvector tempshtEposInME_ivec;
		tempshtVposInMV_ivec.resize( shtVpos_fvec.size()/3 );
		tempshtEposInME_ivec.resize( shtEcmpos_ivec.size()/10);
		writeVerEdgeRegionOneSht_db(
			 tempshtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
			 tempshtEposInME_ivec ,		//each of them is the corresopnding the edge index in mesh edges
			 shtVpos_fvec,		//each of them has the position of the vertices on one sheet
			 shtEcmpos_ivec,		//each of them has a list of edges in it.
			 region,							 
			 seamedgenum,						//the number of edges on seam in the edge list
			fout 
			);
		fclose( fout );
		//////////////////////////////////////////////////////////////////////////
		cout<<"Not able to set out most region!"<<endl;
		return false;
	}

	//step4. set neighboring regions for each region
	setRegionNeighbr(shtEcmpos_ivec, region);
	return true;

	//bool suc = true;
	//suc = getRegionFromBoundaryCluster(
	//	shtVpos_fvec,shtEcmpos_ivec, 
	//	zdir, xdir, region, boundseg );
	//boundseg.clear();

	////step3. find the outer most region
	//if( !setOutMostRegion(region,
	//	shtVpos_fvec,shtEcmpos_ivec,
	//	seamedgenum,seamonsheetnum,
	//	seamonsheet, maseam,majpt,zdir))
	//	suc = false;

	////step4. set neighboring regions for each region
	//setRegionNeighbr(shtEcmpos_ivec, region);
	//return suc;
}


 //void findClosedRegion()
 //{
	////step1. find closed boundary
	////step2 put boundaries together to form regions
	////step3 find the out most region

	////step1. find closed boundary
	//
	////step2. put boundaries together to form regions
	////step3. find the out most region
 //