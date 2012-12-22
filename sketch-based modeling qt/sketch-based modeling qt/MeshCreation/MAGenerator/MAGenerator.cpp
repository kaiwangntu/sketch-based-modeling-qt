#include "../MAGenerator/MAGenerator.h"

/**
* Function: get vertex, edge, face and planeparam of current subspace
*/
void inline MAGenerator::getSubspaceInfo(int planenum, float* planeparam, int ssvernum, 
										 float* ssver,	int ssedgenum,int* ssedge,int ssfacenum, int* ssfaceedgenum, int** ssface,
										 int* ssface_planeindex, int ssspacenum,  int* ssspacefacenum,	int** ssspace, int** ssspace_planeside, 
										 int subspacei,int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
										 int**& subface,float*& subparam,
										 int*& subver2wver,  int*& subedge2wedge)
{
	int* ssver2subver = new int[ ssvernum ];
	int* ssveradded = new int[ ssvernum ];
	for( int i = 0; i < ssvernum ;i ++)
		ssver2subver[ i ] = -1;
	subvernum = 0;
    
	int* ssedge2subedge = new int[ ssedgenum ];
	int* ssedgeadded = new int[ ssedgenum ];
	for( int i = 0; i < ssedgenum ;i ++)
		ssedge2subedge[ i ] = -1;
	subedgenum = 0;

	//go through all the face of current subspace
	for( int i = 0; i< ssspacefacenum[ subspacei ]; i ++)
	{
		int tfacei = ssspace[ subspacei ][ i ];
		int tedgenum = ssfaceedgenum[ tfacei ];
		for( int j = 0; j < tedgenum ; j ++)
		{
			int tedgei = ssface[ tfacei ][ j ];
			if( ssedge2subedge[ tedgei ] == -1 )	//not added yet!
			{
				ssedgeadded[ subedgenum ] = tedgei;
				ssedge2subedge[ tedgei ] = subedgenum;
				subedgenum ++;
			}
		}
	}

	//go through all the subspace edges
	for( int i = 0; i < subedgenum ;i ++)
	{
		int tedgei = ssedgeadded[ i ];
		for(int j = 0 ; j < 2; j ++)
		{
			int tver = ssedge[ tedgei * 2 + j ];
			if( ssver2subver[ tver ] == -1 )	//not added yet!
			{
				ssveradded[ subvernum ] = tver;
				ssver2subver[ tver ] = subvernum;
				subvernum ++;
			}
		}
	}

	subver = new float[ subvernum * 3 ];
	subver2wver = new int[ subvernum ];
	subedge = new int[ subedgenum * 2 ];
	subedge2wedge = new int[ subedgenum  ];
	for( int i = 0; i < subvernum ;i ++)
	{
		memcpy( subver + 3 * i , ssver + 3 * ssveradded[ i ], sizeof( float ) * 3 );
		subver2wver[ i ] = ssveradded[ i ];
	}
	for( int i = 0; i < subedgenum ; i ++)
	{
		int tedge = ssedgeadded[ i ];
		for( int j = 0; j < 2; j ++)
		{
			int tver = ssedge[ tedge * 2 + j ];
			subedge[ 2 * i + j ] = ssver2subver[ tver ];
		}
		subedge2wedge[ i ] = ssedgeadded[ i ];
	}

	subfacenum = ssspacefacenum[ subspacei ];
	subfaceedgenum = new int[ subfacenum ];
	subface = new int*[ subfacenum ];
	for( int i = 0; i < subfacenum; i ++)
	{
		int tfacei = ssspace[ subspacei ][ i ];
		subfaceedgenum[ i ] = ssfaceedgenum[ tfacei ];
		subface[ i ] = new int[ subfaceedgenum[ i ]];
		for( int j = 0; j < subfaceedgenum[ i ]; j ++)
		{
			subface[ i ][ j ] = ssedge2subedge[ ssface[ tfacei ][ j ]];
		}
	}
	
	delete []ssver2subver;
	delete []ssveradded;
	delete []ssedge2subedge;
	delete []ssedgeadded;

	subparam = new float[ subfacenum * 4 ];
	int j = 0;
	int ti = 0;
	for( int i = 0; i < subfacenum ; i ++)
	{
		int tfacei = ssspace[ subspacei ][ i ];
		int tfaceplane = ssface_planeindex[ tfacei ];
		if( tfaceplane < 0 )
		{
			ti = (planenum + 6 + tfaceplane) * 4;
		}
		else
			ti = tfaceplane * 4;
		memcpy( subparam + j, planeparam + ti , sizeof( float )*4 );
		int tfaceside = ssspace_planeside[ subspacei ][ i ];
		if( tfaceside == 0 )	//negative side, change the plane parameters
		{
			for( int k = 0; k < 4; k ++)
				subparam[ j + k ] = - subparam[ j + k ];
		}
		j += 4;
	}
}

/**
* Function: write the ver.edge.face.and planeparam out!
*/
void MAGenerator::writeSubspaceInfo(const char* fname,
									int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
									int**& subface,float*& subparam,
									int*& subver2wver,  int*& subedge2wedge)
{
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file "<<fname<<" to write!";
		return;
	}

	//write ver
	fprintf( fout, "{{{%f,%f,%f}", subver[ 0 ], subver[ 1 ], subver[ 2 ]);
	for( int i = 1; i < subvernum ;i ++)
	{
		fprintf( fout, ",{%f,%f,%f}", subver[ 3*i ], subver[ 3*i + 1 ], subver[ 3*i + 2 ]);
	}

	//write edge
	fprintf( fout, "},{{%d,%d}", subedge[ 0 ]+1, subedge[ 1 ]+1);
	for( int i = 1; i < subedgenum; i ++)
	{
		fprintf( fout, ",{%d,%d}", subedge[ i * 2 ]+1, subedge[ i * 2 + 1 ]+1);
	}

	//write face
	fprintf( fout, "},{{%d", subface[ 0 ][ 0 ]+1 );
	for( int i = 1; i < subfaceedgenum[ 0 ]; i ++)
	{
		fprintf( fout, ",%d", subface[ 0 ][ i ]+1);
	}
	fprintf( fout, "}");
	for( int i = 1; i < subfacenum; i ++)
	{
		fprintf( fout, ",{%d", subface[ i ][ 0 ]+1);
		for( int j = 1; j < subfaceedgenum[ i ]; j ++)
		{
			fprintf( fout, ",%d", subface[ i ][ j ]+1);
		}
		fprintf(fout, "}");
	}

	//write planeparam
	fprintf( fout, "},{{%f,%f,%f,%f}", subparam[ 0 ], subparam[ 1 ], subparam[ 2 ], subparam[ 3 ]);
	for ( int i = 1; i < subfacenum; i ++)
	{
		fprintf( fout, ",{%f,%f,%f,%f}", subparam[ 4*i ], subparam[ 4*i + 1 ], subparam[ 4*i +  2 ], subparam[ 4*i +  3 ]);
	}

	//wirte vermap
	fprintf( fout, "},{%d", subver2wver[ 0 ]+1);
	for( int i = 1; i < subvernum ; i ++)
	{
		fprintf( fout, ",%d", subver2wver[ i ]+1);
	}
	//write edgemap	
	fprintf( fout, "},{%d", subedge2wedge[ 0 ]+1);
	for( int i = 1; i < subedgenum; i ++)
	{
		fprintf(fout, ",%d", subedge2wedge[ i ]+1);
	}
	fprintf(fout, "}}");

	fclose( fout );
}

/**
* set the normalized normal of the sheet, always pointing to the face with bigger index
* return true, if parallel otherwise, false
*/
bool MAGenerator::computeSheetNorm(float*& subparam, int facei, int facej, float*& sheettab, int posinsheet)
{
	float dotval = MyMath::dotProduct( subparam + facei * 4, subparam + facej * 4);
	
	if( facei > facej )	//facei is smaller index
	{
		int t = facei;
		facei = facej;
		facej = t;
	}

	//parallel
	if( MyMath::isEqualInToler( dotval, -1, PLANE_PARAM_PARALLEL_TOLERANCE ) )
	{
		//set direction of the sheet
		memcpy( sheettab + posinsheet * 6 + 3, subparam + facei * 4, sizeof( float) * 3 );
		return true;
	}

	//set normal of the sheet
	float dir[ 3 ];
	MyMath::getPtOnSeg( subparam + facei * 4, subparam + facej * 4, 0.5, dir );
	float dir2[ 3 ];
	MyMath::crossProductNotNorm( subparam + facei * 4, subparam + facej * 4, dir2);
	MyMath::crossProduct( dir, dir2, sheettab + posinsheet * 6 + 3 );
	return false;
}
/**
* Function: gather topology informaiton of current subspace
* @param  
* @param
*/
void MAGenerator::gatherTopology(int& subvernum, float*& subver,
								 int& subedgenum, int*& subedge,
								 int& subfacenum,int*& subfaceedgenum,
								 int**& subface,float*& subparam,
								 MapArraySR& doubleface2sheet, float* sheettab,
								 MapArraySR& tripleface2ver, MapArraySR& doubleface2edge,
								 int**& ver2face,int*& ver2facenum,
								 int*& edge2face, int**& face2ver, int* face2vernum )
{
	//edges to faces map
	for( int i = 0; i < subedgenum * 2; i ++)
	{
		edge2face[ i ] = -1;
	}
	for( int i = 0; i < subfacenum; i ++)
	{
		for( int j = 0; j < subfaceedgenum[ i ]; j ++ )	//for each edge
		{
			int tedgei = subface[ i ][ j ];
			if( edge2face[ tedgei * 2 ] == -1 )
			{
				edge2face[ tedgei * 2 ] = i;
			}
			else
				edge2face[ tedgei * 2 + 1 ] = i;
		}
	}

	//faces to vertex map & vertex to face map
	vector<intvector> tver2face;
	tver2face.resize( subvernum );

	for( int i = 0; i < subfacenum; i ++)
	{
		face2vernum[ i ] = subfaceedgenum[ i ];
		face2ver[ i ] = new int[ face2vernum[ i ]];
		int ind = 0;
		for( int j = 0; j < subfaceedgenum[ i ]; j ++)
		{
			for( int k = 0; k < 2; k ++)
			{
				int tveri = subedge[ 2*subface[ i ][ j ] + k];
				bool isnew = true;
				for( int ik = 0; ik < ind; ik++)
				{
					if( face2ver[ i ][ ik ] == tveri )
					{
						isnew = false;
						break;
					}
				}
				if( isnew )
				{
					face2ver[ i ][ ind ] = tveri;
					ind ++;
					tver2face[ tveri ].push_back( i );
				}
			}
		}
	}

	for( int i = 0; i < subvernum; i ++)
	{
		ver2facenum[ i ] = tver2face[ i ].size();
		ver2face[ i ] = new int[ ver2facenum[ i ] ];
		for( int j = 0; j < ver2facenum[ i ]; j ++)
		{
			ver2face[ i ][ j ] = tver2face[ i ][ j ];
		}
		tver2face[ i ].clear();
	}
	tver2face.clear();

	//tripleface to ver map
	int keys[ 3 ];
	for( int i = 0; i < subvernum; i ++)
	{
		int tfacenum = ver2facenum[ i ];
		for( int j1 = 0; j1 < tfacenum; j1++ )
		{
			for( int j2 = j1 + 1 ; j2 < tfacenum; j2 ++)
			{
				for( int j3 = j2 + 1; j3 < tfacenum; j3++ )
				{
					keys[ 0 ] = ver2face[i][j1];
					keys[ 1 ] = ver2face[i][j2];
					keys[ 2 ] = ver2face[i][j3];
					tripleface2ver.insertKeyVal( keys, 3, false, i );
				}
			}
		}
	}
	
	//double face to edge map
    for( int i = 0; i < subedgenum; i ++)
	{
		keys[ 0 ] = edge2face[ 2*i ];
		keys[ 1 ] = edge2face[ 2*i + 1 ];
		doubleface2edge.insertKeyVal( keys, 2, false, i );
	}
	
	//double face to sheet map
	int posinsheet = 0;
	for( int i = 0; i < subfacenum; i ++)
	{
		for( int j = i + 1; j < subfacenum; j ++)
		{
			keys[ 0 ] = i;
			keys[ 1 ] = j;
			doubleface2sheet.insertKeyVal( keys, 2, true, posinsheet );
			//compute the parameters of current sheet
			//set normal of the sheet
			bool isparal = computeSheetNorm( subparam, i, j, sheettab,posinsheet);
			//set one point on this sheet
			if( isparal )
			{
				int tvers[ 2 ];
				tvers[ 0 ] = face2ver[ i ][ 0 ];
				tvers[ 1 ] = face2ver[ j ][ 0 ];
				MyMath::getPtOnSeg( subver + tvers[ 0 ]*3, subver + tvers[ 1 ]*3,  0.5, sheettab + posinsheet * 6 );
				posinsheet ++;
				continue;
			}
			
			//not parallel
            //case1, the two faces share at least one vertex
			int tveri;
			bool suc = findOneCommon( face2ver[ i ], face2ver[ j ], face2vernum[ i ], face2vernum[ j ], tveri );
			if( suc )	//find one common vertex on both of the faces
			{
				memcpy( sheettab + posinsheet * 6, subver + 3 * tveri, sizeof(float)*3 );
				posinsheet++;
				continue;
			}
			//case2, no common vertex
			for(int k1 = 0; k1 < subfaceedgenum[ i ]; k1 ++)
			{
				int tedgei = subface[ i ][ k1 ];
				bool isparal = isParalLinePlane( subver + subedge[ 2*tedgei ] * 3, 
					subver + subedge[ 2*tedgei + 1 ] * 3, subparam + 4 * j, true);
				if( isparal ) continue;
				//compute the intersection point
				interPt_PlaneEdge( subparam + 4 * j,  subver + subedge[ 2*tedgei] * 3, 
					subver + subedge[ 2*tedgei + 1 ]*3, sheettab + posinsheet * 6 );
				break;
			}
            posinsheet++;			
		}
	}
}
void MAGenerator::writeTopology(const char* fname,
								int subvernum, int subedgenum, int subfacenum,
								MapArraySR& doubleface2sheet, float* sheettab,
								MapArraySR& tripleface2ver, MapArraySR& doubleface2edge,
								int**& ver2face,int*& ver2facenum,
								int*& edge2face, int**& face2ver, int* face2vernum )
{
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file :"<<fname<<" to write!"<<endl;
		return;
	}

	//double face 2 sheet
	bool first = true;
	int keys[ 3 ];
	int val;
	for( int i = 0; i < subfacenum; i ++)
	{
		for( int j = i + 1; j < subfacenum; j ++)
		{
			keys[ 0 ] = i;
			keys[ 1 ] = j;
			doubleface2sheet.getKeyVal( keys, 2, true, val );
			if( first )
			{
				first = false;
				fprintf( fout, "{{{%d,%d,%d}", keys[ 0 ] + 1, keys[ 1 ] + 1,  val+1 );
			}
			else
			{
				fprintf(fout, ",{%d,%d,%d}", keys[ 0 ] + 1, keys[ 1 ] + 1, val + 1);
			}
		}
	}
	fprintf( fout, "}");

	//sheettab
	fprintf( fout, ",{{{%f", sheettab[ 0 ]);
	for( int i = 1; i < 3; i ++)
		fprintf(fout, ",%f", sheettab[ i ]);
	fprintf( fout, "},{%f", sheettab[ 3 ]);
	for( int i = 4; i < 6; i ++)
		fprintf( fout, ",%f", sheettab[ i ]);
	
	int sheetsize = subfacenum * (subfacenum - 1)/2;
	for( int i = 1; i < sheetsize; i ++)
	{
		fprintf( fout, "}},{{%f", sheettab[ i * 6 ]);
		for( int j = 1; j < 3; j ++)
			fprintf( fout, ",%f", sheettab[ i * 6 + j ]);
		fprintf( fout, "},{%f", sheettab[ i * 6 + 3 ]);
		for( int j = 4; j< 6; j++)
		{
			fprintf( fout, ",%f", sheettab[ i * 6 + j ]);
		}
	}

	//tripleface2ver
	fprintf( fout, "}}},{");
	first = true;
	for( int i = 0; i < subfacenum; i ++ )
	{
		for( int j = i + 1; j < subfacenum; j++)
		{
			for( int k = j + 1; k < subfacenum; k ++)
			{
				int tveri = 0;
				keys[ 0 ] = i;
				keys[ 1 ] = j;
				keys[ 2 ] = k;
				if( tripleface2ver.getKeyVal( keys, 3, true, tveri ) )
				{
					if( first)
					{
						first = false;
						fprintf(fout, "{%d,%d,%d,%d}", i + 1, j + 1, k + 1, tveri + 1);
					}
					else
					{
						fprintf( fout, ",{%d,%d,%d,%d}", i + 1, j + 1, k + 1, tveri + 1);
					}
				}
			}
		}
	}

	//doubleface2edge
	fprintf( fout, "},{");
	first = true;
	for( int i = 0; i < subfacenum; i++)
	{
		for( int j = i + 1; j < subfacenum; j ++)
		{
			int tedgei = 0;
			keys[ 0 ] = i;
			keys[ 1 ] = j;
			if( doubleface2edge.getKeyVal( keys, 2, true, tedgei ))
			{
				if( first )
				{
					first = false;
					fprintf( fout, "{%d,%d,%d}", i + 1, j + 1, tedgei + 1);
				}
				else
				{
					fprintf( fout, ",{%d,%d,%d}", i + 1, j + 1, tedgei + 1);
				}
			}
		}
	}

	//ver2face
	fprintf(fout, "},{{%d", ver2face[ 0 ][ 0 ]+1);
	for( int i = 1; i < ver2facenum[0 ]; i ++)
	{
		fprintf( fout, ",%d", ver2face[ 0 ][ i ] + 1 );
	}
	for( int i = 1; i < subvernum;i ++)
	{
		fprintf( fout, "},{%d", ver2face[ i ][ 0 ] + 1 );
		for( int j = 1; j < ver2facenum[ i ]; j ++)
			fprintf( fout, ",%d", ver2face[ i ][ j ] + 1);
	}

	//edge2face
	fprintf( fout, "}},{{%d,%d}", edge2face[ 0 ] + 1, edge2face[ 1 ]+1);
	for( int i = 1; i < subedgenum; i ++)
		fprintf(fout, ",{%d,%d}", edge2face[ i * 2 ]+ 1, edge2face[ i * 2 + 1 ] + 1);

	//face2ver
	fprintf( fout, "},{{%d", face2ver[ 0 ][ 0 ] + 1 );
	for( int i = 1; i < face2vernum[ 0 ]; i ++)
		fprintf( fout, ",%d", face2ver[ 0 ][ i ] + 1);
	for( int i = 1; i < subfacenum; i ++)
	{
		fprintf( fout, "},{%d", face2ver[ i ][ 0 ] + 1);
		for( int j = 1; j < face2vernum[ i ]; j ++)
		{
			fprintf( fout, ",%d", face2ver[ i ][ j ] + 1);
		}
	}
	fprintf( fout, "}}}");
	fclose( fout );
}


/**
* find the first junction point
*
*
*/
void MAGenerator::findFirstJpt(const int& subvernum, float*& subver, int*& subedge,
							   int& subfacenum,	float*& subparam,
							   int**& ver2face, int* ver2facenum,
							   MapArraySR& doubleface2sheet,  float* sheettab,MapArraySR& doubleface2edge,
							   intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
							   vector<MASeam>& seamtab,MapArraySR& tripleface2seam,
							   int*& ver2jpt)
{
	//find out the vertex with minimal degree
	int mindegree = ver2facenum[ 0 ];
	int minveri = 0;
	if( mindegree != 3 )	//if is 3, compute the first junction point from this vertex
	{
		for( int i = 1; i < subvernum; i ++)
		{
			if( ver2facenum[ i ] < mindegree)
			{
				mindegree = ver2facenum[ i ];
				minveri = i;
				if( mindegree == 3 )
					break;
			}
		}
	}
	

	MAJunctionPoint tjpt;
	MASeam tseam;
	float dir[ 3 ];	// save the seam direction starting from vertex minveri
	int keys[ 2 ];
	int sheetpos[ 2 ];
	for( int i = 0; i < 3; i ++)
		tseam.governor.push_back( ver2face[ minveri ][ i ]);
	if( mindegree == 3 )	//the minimal degree is 3!!!
	{
		//////////////////////////////////////////////////////////////////////////
//		cout<<"ver2face:"<<ver2face[ minveri ][ 0 ]
//		<<","<<ver2face[ minveri ][ 1 ]<<","
//			<<ver2face[ minveri ][ 2 ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		//int sheetpos[ 2 ];	
		if(!doubleface2sheet.getKeyVal( ver2face[ minveri ], 2, false, sheetpos[ 0 ] )
			|| !doubleface2sheet.getKeyVal( ver2face[ minveri ] + 1, 2, false, sheetpos[ 1 ]))
		{
			cout<<"In get first junction point! FAIL to find the sheet of two faces!"<<endl;
		}
		MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3, sheettab + 6*sheetpos[ 1 ]+3, dir);
		if( MyMath::dotProduct( dir, subparam + 4*ver2face[ minveri ][ 0 ]) < 0 )
		{
			MyMath::stretchVec( dir, -1);
		}
	}
	else					//the minimal degree is not 3!!!
	{
        //pick one edge with one vertex minveri and find the seam that has the smallest angle with the edge
		float dir2[ 3 ];
		int edgei = 0;
		if( !doubleface2edge.getKeyVal( ver2face[ minveri ], 2 , false, edgei ))
		{
			cout<<"In get first junction pint! FAIL to find the edge for double faces!"<<endl;
		}
		if( subedge[ edgei * 2 ] == minveri)
		{
			MyMath::getVec( subver + subedge[ edgei * 2 ] * 3, subver + subedge[ edgei*2 + 1 ]*3, dir2);
		}
		else
		{
			MyMath::getVec( subver + subedge[ edgei * 2 + 1 ] * 3, subver + subedge[ edgei * 2 ] * 3, dir2);
		}

        //go through all other faces, and find out the dir with minimal angle between dir2.
		bool first = true;
		float minval;
		float dir3[ 3 ];
		int tfacenum = ver2facenum[ minveri ];
		//int sheetpos[ 2 ];
		if(!doubleface2sheet.getKeyVal( ver2face[ minveri ], 2, false, sheetpos[ 0 ]))
		{
			cout<<"In get first junction point, FAIL to find the sheet for two faces!"<<endl;
		}
		keys[ 0 ] = ver2face[ minveri ][ 0 ];
		for( int i = 2; i < tfacenum; i ++)
		{
			keys[ 1 ] = ver2face[ minveri ][ i ];
			if( !doubleface2sheet.getKeyVal( keys, 2, false, sheetpos[ 1 ]))
			{
				cout<<"In get first junction point, FAIL to find the sheet for two faces!"<<endl;
			}
			MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3,sheettab + 6*sheetpos[ 1 ]+ 3,dir3);
			float val = MyMath::dotProduct( dir3, dir2 );
			if( first )
			{
				first = false;
				minval = val;
				memcpy( dir, dir3, sizeof( float )*3 );
				tseam.governor[ 2 ] = ver2face[ minveri ][ i ];
				continue;
			}
			
			if( abs( val ) < abs(minval) )
			{
				minval = val;
				memcpy( dir, dir3, sizeof(float) * 3 );
				tseam.governor[ 2 ] = ver2face[ minveri ][ i ];
			}
		}

		if( minval < 0 )
		{
			MyMath::stretchVec(dir, -1 );
		}
	}
	memcpy(tseam.dir, dir, sizeof(float)*3);

	//find the cutting plane
	for( int i = 0; i < 3; i ++)
		tjpt.governor.push_back( tseam.governor[ i ]);
	tjpt.governor.push_back( 0 );

	//the first cut face doesn't exist for the first junction point of
    //the seam, which is actually a vertex of the subspace
	tseam.cutface[ 0 ] =  -1 ;
	tseam.cutface[ 1 ] = 0;	//take up a place, to be replaced in the following code

	int* tfacemark = new int[ subfacenum ];
	for( int i = 0; i < subfacenum; i++)
		tfacemark[ i ] = 0;
	int tfacenum = ver2facenum[ minveri ];
	for( int i = 0; i < tfacenum; i++)
		tfacemark[ ver2face[ minveri ][ i ] ] = 1;
	float minval;
	bool first = true;
	keys[ 0 ] = ver2face[ minveri ][ 0 ];
	for( int i = 0; i < subfacenum; i++)
	{
		if( tfacemark[ i ] == 1 )
			continue;

		//compute the distance from vertex minveri to the intersection point between 
		//dir and the sheet between any face of the govenor of the seam and current face
		//along direction dir. find out the minimal one
		int tsheetpos = 0;
		keys[ 1 ] = i;
		if( !doubleface2sheet.getKeyVal( keys, 2, false, tsheetpos ))
		{
			cout<<"In get first junction point, FAIL to find the sheet for two faces!"<<endl;
		}
		float val = distPt2PlaneAlongDir( subver + minveri * 3 , dir,  true, sheettab + tsheetpos * 6,
			sheettab + tsheetpos * 6 + 3 , true);
		if( val <= 0 )	//this can't be a cutting plane
			continue;
		
		if( first )
		{
			first = false;
			minval = val;
			tjpt.governor[ 3 ] = i;
			MyMath::getPtOnRay( subver + minveri * 3, dir, minval, tjpt.pos );
			tseam.cutface[ 1 ] = i;
			continue;
		}
		if( minval > val )
		{
			minval = val;
			tjpt.governor[ 3 ] = i;
			MyMath::getPtOnRay( subver + minveri * 3, dir, minval, tjpt.pos );
			tseam.cutface[ 1 ] = i;
		}
	}

	// refresh activeJpts, jpttab,
	//seamtab, tripleface2seam,ver2jpt

	//jpttab
	MAJunctionPoint jpt2;
	memcpy( jpt2.pos, subver + 3 * minveri, sizeof(float) * 3 );
	jpt2.seam_pos.push_back( 0 );	//#0 seam
	jpt2.seam_pos.push_back( 0 );	//#0 junction point of the seam
	jpttab.push_back( jpt2 );
	tjpt.seam_pos.push_back( 0 );
	tjpt.seam_pos.push_back( 1 );
	jpttab.push_back( tjpt );
	
	tjpt.seam_pos.clear(); //clear temporary variable
	tjpt.governor.clear();
	jpt2.seam_pos.clear();

	//seamtab
	tseam.jpts[ 0 ] = 0;
	tseam.jpts[ 1 ] = 1;
	seamtab.push_back( tseam );

	tseam.governor.clear();	//temporary variable clear

	//tripleface2seam
	int ttfacei[ 3 ];
	for( int i = 0; i < 3; i++)
	{
		ttfacei[ i ] = seamtab[ 0 ].governor[ i ];
	}
	tripleface2seam.insertKeyVal( ttfacei, 3, false, 0);

	//ver2jpt
	ver2jpt[ 0 ] = 0;

	//activeJpts
	activeJpts.push_back( 1 );	//jpt 1
	activeJpts.push_back( 0 );	//the 0 th governor

//	writeJptSeamTab(jpttab, seamtab, 10000);
	
	delete []tfacemark;
}
void MAGenerator::writeSubspaceInfoWithFirstJpt(const char* fname,
										  int& subvernum, float*& subver,int& subedgenum, int*& subedge,int& subfacenum,int*& subfaceedgenum,
										  int**& subface,float*& subparam,
										  int*& subver2wver,  int*& subedge2wedge,
										 vector<MAJunctionPoint>& jpttab)
{
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file "<<fname<<" to write!";
		return;
	}

	//write ver
	fprintf( fout, "{{{%f,%f,%f}", subver[ 0 ], subver[ 1 ], subver[ 2 ]);
	for( int i = 1; i < subvernum ;i ++)
	{
		fprintf( fout, ",{%f,%f,%f}", subver[ 3*i ], subver[ 3*i + 1 ], subver[ 3*i + 2 ]);
	}

	//write edge
	fprintf( fout, "},{{%d,%d}", subedge[ 0 ]+1, subedge[ 1 ]+1);
	for( int i = 1; i < subedgenum; i ++)
	{
		fprintf( fout, ",{%d,%d}", subedge[ i * 2 ]+1, subedge[ i * 2 + 1 ]+1);
	}

	//write face
	fprintf( fout, "},{{%d", subface[ 0 ][ 0 ]+1 );
	for( int i = 1; i < subfaceedgenum[ 0 ]; i ++)
	{
		fprintf( fout, ",%d", subface[ 0 ][ i ]+1);
	}
	fprintf( fout, "}");
	for( int i = 1; i < subfacenum; i ++)
	{
		fprintf( fout, ",{%d", subface[ i ][ 0 ]+1);
		for( int j = 1; j < subfaceedgenum[ i ]; j ++)
		{
			fprintf( fout, ",%d", subface[ i ][ j ]+1);
		}
		fprintf(fout, "}");
	}

	//write planeparam
	fprintf( fout, "},{{%f,%f,%f,%f}", subparam[ 0 ], subparam[ 1 ], subparam[ 2 ], subparam[ 3 ]);
	for ( int i = 1; i < subfacenum; i ++)
	{
		fprintf( fout, ",{%f,%f,%f,%f}", subparam[ 4*i ], subparam[ 4*i + 1 ], subparam[ 4*i +  2 ], subparam[ 4*i +  3 ]);
	}

	//wirte vermap
	fprintf( fout, "},{%d", subver2wver[ 0 ]+1);
	for( int i = 1; i < subvernum ; i ++)
	{
		fprintf( fout, ",%d", subver2wver[ i ]+1);
	}
	//write edgemap	
	fprintf( fout, "},{%d", subedge2wedge[ 0 ]+1);
	for( int i = 1; i < subedgenum; i ++)
	{
		fprintf(fout, ",%d", subedge2wedge[ i ]+1);
	}
	fprintf( fout, "},{{%f,%f,%f},{%f,%f,%f}}}",
		jpttab[ 0 ].pos[ 0 ], jpttab[ 0 ].pos[ 1 ], jpttab[ 0 ].pos[ 2 ],
		jpttab[ 1 ].pos[ 0 ], jpttab[ 1 ].pos[ 1 ], jpttab[ 1 ].pos[ 2 ]);	

	fclose( fout );
}

/**
* Compute the seam for the three faces
*
*/
void MAGenerator::getSeam(float* subparam, MapArraySR& doubleface2sheet,  float* sheettab, int* triset, 
						  float dir[ 3 ] )
{
	int sheetpos[ 2 ];
	if( !doubleface2sheet.getKeyVal( triset, 2, false, sheetpos[ 0 ]) ||
		!doubleface2sheet.getKeyVal( triset+1, 2, false, sheetpos[ 1 ]))
	{
		cout<<"In get Seam! Fail to get sheet for double face!"<<endl;
	}
	MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3,sheettab + sheetpos[ 1 ] * 6 + 3,dir);
	if( MyMath::dotProduct( subparam + 4*triset[3], dir) < 0)
		MyMath::stretchVec( dir, -1);
}

/*
* decide if this is an old seam or not
* if yes, return the old seam's position in this MAJunctionPt saempos
* if not, return -1
*/
int MAGenerator::isOldSeam(int juncpti, float seamdir[ 3 ], vector<MAJunctionPoint>& jpttab,vector<MASeam>& seamtab)
{
	int seamnum = jpttab[ juncpti ].seam_pos.size()/2;
	int j = 0;
	//////////////////////////////////////////////////////////////////////////
	//cout<<"compareing with exisitng seams: #seam #ind"<<"jpt:"<<juncpti<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < seamnum; i ++)
	{
		int tseami = jpttab[ juncpti ].seam_pos[ j ++ ];
		int tposi = jpttab[ juncpti ].seam_pos[ j ++ ];
		//////////////////////////////////////////////////////////////////////////
		//cout<<tseami<<" "<<tposi<<",";
		//////////////////////////////////////////////////////////////////////////
	
		float dotval = MyMath::dotProduct( seamdir, seamtab[ tseami ].dir );
		if( tposi == 0)
		{
			if (MyMath::isEqualInToler( dotval, 1, TOLERANCE_SIX))
				return tseami;
		}
		else
		{
			if( MyMath::isEqualInToler( dotval, -1, TOLERANCE_SIX ))
				return tseami;
		}
	}
	return -1;
}

//2.2, the seam is new, compute the other junction point on this seam
//2.2.1	seam degnerates into one point
//2.2.2 seam is not a point
//2.2.3, the other junction point already exists
//2.2.4, the other junction point is totally new
void inline MAGenerator::computeNewJpt(int subfacenum,
									   MapArraySR& doubleface2sheet,  float* sheettab,
								int activejpti, float seamdir[ 3 ], int*& triset,
								intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
								vector<MASeam>& seamtab,MapArraySR& tripleface2seam)
{
	//2.2, the seam is new, compute the other junction point on this seam
	int* facemark = new int[ subfacenum ];
	for( int i = 0; i < subfacenum; i ++)
		facemark[ i ] = 0;
	for( int i = 0; i < 4; i ++)
		facemark[ triset[ i ]] = 1;
	int keys[ 2 ];
	keys[ 0 ] = triset[ 0 ];

	bool first = true;
	float minval;
	int njptgov[ 4 ];
	memcpy( njptgov, triset, sizeof(int)*3);
	for( int i = 0; i < subfacenum; i ++)
	{
		if( facemark[ i ] == 1)continue;
		keys[ 1 ] = i;
		int sheetpos = 0;
        if( !doubleface2sheet.getKeyVal( keys, 2, false, sheetpos ))
		{
			cout<<"In computeNewJpt, FAIL to find the sheet corresponding to double faces!"<<endl;
		}
		
		float val = distPt2PlaneAlongDir( jpttab[ activejpti ].pos, seamdir, true, sheettab + sheetpos*6,
			sheettab+sheetpos*6+3, true);
//////////////////////////////////////////////////////////////////////////
		//cout<<"val:"<<val;
		//////////////////////////////////////////////////////////////////////////
		if( val < 0 ) continue;
		if( val < TOLERANCE_TWO )	//the starting junction point is on this sheet
		{
			float tdotval = MyMath::dotProduct( seamdir, sheettab + sheetpos * 6 + 3 );
			//2.2.1	seam degenerates into one point
			if( (tdotval > 0 && (keys[ 1 ] > keys[ 0 ]))||
				(tdotval < 0 ) && (keys[ 1 ] < keys[ 0 ]))
			{
				//tripleface2seam
				//though no seam corresponds to the three faces, but they have already been checked
				//tripleface2seam.insertKeyVal( triset, 3, false, -1 );
				tripleface2seam.replaceKeyVal( triset, 3, false, -1 );

				//jpttab
				njptgov[ 3 ] = i;
				for( int j = 0; j < 4; j ++)
					jpttab[ activejpti ].governor.push_back( njptgov[ j ]);
				
				//activejpts
				activeJpts.push_back( activejpti );
				activeJpts.push_back( jpttab[ activejpti ].governor.size()/4 - 1 );
				delete []facemark;
				return;
			}
			continue;
		}
		//val must be > 0 and first time to set minval
		if( first)
		{
			first = false;
			minval = val;
			njptgov[ 3 ] = i;
			continue;
		}
		//not the first time
		if( val < minval )
		{
			minval = val;
			njptgov[ 3 ] = i;
		}
	}
	//no minval is set
	if( first )
	{
		delete []facemark;
		return;
	}

	//////////////////////////////////////////////////////////////////////////
	//cout<<"njptgov:"<<njptgov[ 0 ]<<" "<<njptgov[ 1 ]<<" "<<njptgov[ 2 ]<<" "<<njptgov[3 ]<<endl;
	//////////////////////////////////////////////////////////////////////////
	//2.2.2 seam is not a point( otherwise, already returned )
	float newjptpos[ 3 ];
	MyMath::getPtOnRay( jpttab[ activejpti ].pos, seamdir, minval, newjptpos );
	int tjptnum = jpttab.size();
	for( int i = 0; i < tjptnum; i ++)
	{		
		//2.2.3, the other junction point already exists
		if( pointEqualCube( jpttab[ i ].pos, newjptpos, TOLERANCE_TWO ))
		{
			//jpttab & activejpts
			int nseami = seamtab.size();
			jpttab[ activejpti ].seam_pos.push_back( nseami );
			jpttab[ activejpti ].seam_pos.push_back( 0 );
			jpttab[ i ].seam_pos.push_back( nseami );
			jpttab[ i ].seam_pos.push_back( 1 );

			int tgovsize = jpttab[ i ].governor.size();
			int j;
			for(j = 0;j < tgovsize; j += 4 )
			{
				if( isSetEqual( jpttab[ i ].governor, j, 4, njptgov ) )
					break;
			}
			if( j >= tgovsize )	//a new group of governor
			{
                for( int k = 0; k < 4; k ++)
				{
					jpttab[ i ].governor.push_back( njptgov[ k ]);
				}
				activeJpts.push_back( i );
				activeJpts.push_back( tgovsize/4 );
			}

			//seamtab
			MASeam nseam;
			seamtab.push_back( nseam );
			seamtab[ nseami ].jpts[ 0 ] = activejpti;
			seamtab[ nseami ].jpts[ 1 ] = i;
			memcpy( seamtab[ nseami ].dir, seamdir, sizeof(float)*3 );
			for( int j = 0; j< 3; j++)
			{
				seamtab[ nseami ].governor.push_back( triset[ j ]);
			}
			seamtab[ nseami ].cutface[ 0 ] = triset[ 3 ];
			seamtab[ nseami ].cutface[ 1 ] = njptgov[ 3 ];

			//tripleface2seam
			//tripleface2seam.insertKeyVal( triset, 3, false, nseami);
			tripleface2seam.replaceKeyVal( triset, 3, false, nseami);
		
			delete []facemark;
			return;
		}
	}
		//2.2.4, the other junction point is totally new
	int nseami = seamtab.size();

	//jpttab
	MAJunctionPoint npt;
	jpttab.push_back( npt );
	memcpy( jpttab[ tjptnum ].pos, newjptpos, sizeof(float)*3);
	for( int i = 0; i < 4; i++)
		jpttab[ tjptnum ].governor.push_back( njptgov[ i ]);
	jpttab[ tjptnum ].seam_pos.push_back( nseami );
	jpttab[ tjptnum ].seam_pos.push_back( 1 );
	jpttab[ activejpti ].seam_pos.push_back( nseami );
	jpttab[ activejpti ].seam_pos.push_back( 0 );

	//seamtab
	MASeam nsm;
	seamtab.push_back( nsm );
	seamtab[ nseami ].jpts[ 0 ] = activejpti;
	seamtab[ nseami ].jpts[ 1 ] = tjptnum;
	memcpy( seamtab[ nseami ].dir, seamdir, sizeof(float) * 3 );
	for( int i = 0; i < 3; i ++)
		seamtab[ nseami ].governor.push_back( njptgov[ i ]);
	seamtab[ nseami ].cutface[ 0 ] = triset[ 3 ];
	seamtab[ nseami ].cutface[ 1 ] = njptgov[ 3 ];

	//activejpts
	activeJpts.push_back( tjptnum );
	activeJpts.push_back( 0 );

	//tripleface2seam
	//tripleface2seam.insertKeyVal( triset, 3, false, nseami );
	tripleface2seam.replaceKeyVal( triset, 3, false, nseami );

	delete []facemark;
}
//put step2. 
//judge if the saem direction is new or old
//if new, compute the new junction point into this funciton.
void MAGenerator::traceSeamDir(
							   int subfacenum,
							   MapArraySR& doubleface2sheet,  float* sheettab,
							   int activejpti, float seamdir[ 3 ], int*& triset,
							   intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
							   vector<MASeam>& seamtab,MapArraySR& tripleface2seam)
{
	//step2. for this seam
	int tseami = isOldSeam( activejpti, seamdir, jpttab, seamtab);
	//	2.1, seam already exists!
	if( tseami != -1 )
	{
		//cout<<"is an old seam!"<<"activejpti:"<<activejpti<<"seami:"<<tseami<<endl;
		//tripleface2seam
		//tripleface2seam.insertKeyVal( triset, 3, false, tseami );
		tripleface2seam.replaceKeyVal( triset, 3 , false, tseami);

		//jpttab
		int newgov[ 4 ];
		memcpy( newgov, triset, sizeof(int)*3 );
		int tseampos = jpttab[ activejpti ].seam_pos[ tseami * 2 + 1 ];
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		int tjpti = seamtab[ tseami ].jpts[ 1 - tseampos ];
		newgov[ 3 ] = seamtab[ tseami ].cutface[ 1 - tseampos ];
		//cout<<"newgov4:"<<newgov[ 3 ]<<endl;
		int tgovsize = jpttab[ tjpti ].governor.size();
		int j = 0; 
		for(;j < tgovsize; j += 4 )
		{
			if( isSetEqual( jpttab[ tjpti ].governor, j, 4, newgov ))
			{
				break;
			}
		}
		if( j >= tgovsize )	//not break out, the governor does not exist!
		{
			for( int j = 0; j < 4; j ++)
				jpttab[ tjpti ].governor.push_back( newgov[ j ]);
			activeJpts.push_back( tjpti );
			activeJpts.push_back( tgovsize/4 );
		}

		//seamtab
		merge(seamtab[ tseami ].governor, triset, 3);
		return;
	}

	//  2.2, the seam is new, compute the other junction point on this seam
	//2.2.1	seam degnerates into one point
	//2.2.2 seam is not a point
	//2.2.3, the other junction point already exists
	//2.2.4, the other junction point is totally new
	//cout<<"copmuting new"<<endl;
	computeNewJpt(subfacenum,doubleface2sheet,  sheettab, 
		activejpti, seamdir, triset, activeJpts, jpttab,seamtab,tripleface2seam);

}
/**
* from the first junction point and seam, trace out all the seams and junction points
*
*/
void MAGenerator::traceMA(float* subver,float* subparam, int subfacenum,
						  MapArraySR& tripleface2ver, MapArraySR& doubleface2sheet,  float* sheettab,
						  intvector& activeJpts, vector<MAJunctionPoint>& jpttab,
						  vector<MASeam>& seamtab,MapArraySR& tripleface2seam,
						  int*& ver2jpt)
{
	//algorithm:
	//step1. find the governor of the seams
	//1.1 get the active juntion point out, and one group of governor
	//1.2 if the triple faces share one vertex
		//1.2.1 this vertex has not been added as junction point
		//1.2.2 this vertex has already been added
			//1.2.2.1 the seam has already been added
			//1.2.2.2 the seam is old
	//1.3 for each three out of the four, compute the seam

	//step2. for this seam
	//	2.1, seam already exists!
	//  2.2, the seam is new, compute the other junction point on this seam
		//2.2.1	seam degnerates into one point
		//2.2.2 seam is not a point
			//2.2.3, the other junction point already exists
			//2.2.4, the other junction point is totally new

	//////////////////////////////////////////////////////////////////////////
	int tcount = 0;
	//////////////////////////////////////////////////////////////////////////

	while( !activeJpts.empty())
	{
		int activejptgovi = activeJpts.back();
		activeJpts.pop_back();
		int activejpti = activeJpts.back();
		activeJpts.pop_back();


		//////////////////////////////////////////////////////////////////////////
		/*cout<<"Active JPt:"<<activejpti<<":";
		for( int j = 0; j < 4; j ++)
		{
			cout <<jpttab[ activejpti ].governor[ 4*activejptgovi + j ] <<" ";
		}
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////

		//step1. find the governor of the seams
		//1.1get the active juntion point out, and one group of governor
		vector<int*> triset;
		for( int i = 0; i < 4; i ++)
		{
			int* ttriset = new int[ 4 ];	//the last one is the cut face for this seam
			int k = 0;
			for( int j = 0; j < 4; j ++)
			{
				if( j == i )continue;
				ttriset[ k ] = jpttab[ activejpti ].governor[ 4*activejptgovi + j ];
				k++;
			}
			int tseami = 0;
			if( tripleface2seam.getKeyVal( ttriset, 3, false, tseami))	//exists!
			{
				delete []ttriset;
				continue;
			}
			ttriset[ 3 ] = jpttab[ activejpti ].governor[ 4*activejptgovi + i ];
			triset.push_back( ttriset );
			ttriset = NULL;
		}

		if( triset.size() == 0 ) //no need to process this active junction point
		{
			continue;
		}

		//////////////////////////////////////////////////////////////////////////
		//if( activejpti == 6 ) 
		//{
		////	for( int tk1 = 0; tk1 < triset.size(); tk1 ++ )
		//	{
		//		cout<<triset[ tk1 ][ 0 ]<<","<<triset[ tk1 ][ 1 ]<<","<<triset[ tk1 ][ 2 ]
		//		<<","<<triset[ tk1 ][ 3 ] <<endl;
		//	}
		//	cout<<endl;
		//	for( int tk1 = 0; tk1 < jpttab[ 3 ].governor.size(); tk1 ++ )
		//		cout<<jpttab[ 3 ].governor[ tk1 ]<<",";
		//	cout<<endl;
		//}
		
		//////////////////////////////////////////////////////////////////////////

		int ttrisetsize = triset.size();
		float seamdir[ 3 ];
		for( int i = 0; i < ttrisetsize; i++)
		{
			//////////////////////////////////////////////////////////////////////////
			//cout<<"Before processing this group:"<<endl;
			//cout<<"jpttab:"<<endl;
			//for( int ti = 0; ti < jpttab.size(); ti ++)
			//{
			//	cout<<"pos"<<jpttab[ti].pos[ 0 ]<<" "<<jpttab[ti ].pos[1]<<" "<<jpttab[ ti ].pos[2]<<endl;
			//	cout<<"governor:";//<<endl;
			//	for( int tj = 0; tj < jpttab[ ti ].governor.size(); tj ++)
			//	{
			//		cout<<jpttab[ ti ].governor[ tj ]<<" ";
			//	}				
			//	cout<<endl;
			//	cout<<"seampos:";//<<endl;
			//	for(int tj = 0; tj <jpttab[ ti ].seam_pos.size(); tj ++)
			//	{
			//		cout<<jpttab[ ti ].seam_pos[ tj ]<<" ";
			//	}
			//	cout<<endl;
			//}
			//cout<<"seamtab:"<<endl;
			//for( int ti = 0; ti < seamtab.size(); ti ++)
			//{
			//	cout<<"pts:"<<seamtab[ ti ].jpts[ 0 ]<<" "<<seamtab[ ti ].jpts[ 1 ]<<endl;
			//	cout<<"govenor";//<<endl;
			//	for(int tj = 0; tj < seamtab[ ti ].governor.size(); tj ++)
			//	{
			//		cout<<seamtab[ ti ].governor[ tj ]<<" ";
			//	}
			//	cout<<endl;
			//	cout<<"dir:";//<<endl;
			//	for( int tj = 0; tj < 3; tj ++)
			//	{
			//		cout<<seamtab[ ti ].dir[ tj ]<<"  ";
			//	}
			//	cout<<endl;
			//	cout<<"ctuface:"<<seamtab[ ti ].cutface[ 0 ]<<" "<<seamtab[ ti ].cutface[ 1 ]<<endl;
			//}
			//////////////////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////////////////
			/*for( int j = 0; j < 4;j  ++)
			{
				cout<<triset[ i ][ j ]<<" ";
			}
			cout<<endl;*/
			//////////////////////////////////////////////////////////////////////////

			//1.2 if the triple faces share one vertex	
			int tshareveri;
			if( tripleface2ver.getKeyVal( triset[ i ], 3, false, tshareveri ))
			{
				//////////////////////////////////////////////////////////////////////////
				//cout<<"subspace vertex!"<<" ";
				//////////////////////////////////////////////////////////////////////////
				//1.2.1 this vertex has not been added as junction point
				if( ver2jpt[ tshareveri ] == -1 )
				{
					//cout<<"the vertex hasn't been added!"<<endl;
					int njpti = jpttab.size();
					int nseami = seamtab.size();

					//add the vertex as a junction point
					MAJunctionPoint njpt;
					jpttab.push_back( njpt );
					memcpy(jpttab[ njpti ].pos, subver + tshareveri * 3, sizeof(float)*3);
					jpttab[ njpti ].seam_pos.push_back(nseami);
					jpttab[ njpti ].seam_pos.push_back( 1 );
					jpttab[ activejpti ].seam_pos.push_back( nseami );
					jpttab[ activejpti ].seam_pos.push_back( 0 );

					//add the seam
					MASeam nseam;
					seamtab.push_back( nseam );
					seamtab[ nseami ].jpts[ 0 ] = activejpti;
					seamtab[ nseami ].jpts[ 1 ] = njpti;
					MyMath::getVec( jpttab[ activejpti ].pos, jpttab[ njpti ].pos, seamdir);
					memcpy( seamtab[ nseami ].dir, seamdir, sizeof( float )* 3);
					for( int j = 0; j < 3; j ++)
						seamtab[ nseami ].governor.push_back( triset[ i ][ j ]);
					seamtab[ nseami].cutface[ 0 ] = triset[ i ][ 3 ];
					seamtab[ nseami].cutface[ 1 ] = -1;

					//tripleface2seam
					//tripleface2seam.insertKeyVal( triset[ i], 3, false, nseami);
					tripleface2seam.replaceKeyVal( triset[ i], 3, false, nseami);
					//ver2jpt
					ver2jpt[ tshareveri ] = njpti;
				}
				//1.2.2 this vertex has already been added
				else
				{
				//	cout<<"the vertex has been added!"<<endl;
					int tjpti = ver2jpt[ tshareveri ];
					bool isnew = true;
					int tseamsize = jpttab[ tjpti ].seam_pos.size()/2;
					int tseami;
					for( int j = 0; j <tseamsize; j ++)
					{
						tseami = jpttab[ tjpti ].seam_pos[ 2*j ];
						if( (seamtab[ tseami ].jpts[ 0 ] + seamtab[ tseami ].jpts[ 1 ] -
							tjpti) == activejpti)
						{
							isnew = false;
							break;
						}
					}
					//1.2.2.1 the seam is new
					if( isnew )
					{		
						int nseami = seamtab.size();

						//jpttab
						jpttab[ activejpti ].seam_pos.push_back( nseami );
						jpttab[ activejpti ].seam_pos.push_back( 0 );
						jpttab[ tjpti ].seam_pos.push_back( nseami );
						jpttab[ tjpti ].seam_pos.push_back( 1 );

						//seamtab
						MASeam tnseam;
						seamtab.push_back( tnseam );
						seamtab[ nseami ].jpts[ 0 ] = activejpti;
						seamtab[ nseami ].jpts[ 1 ] = tjpti;
						MyMath::getVec( jpttab[ activejpti ].pos, jpttab[ tjpti ].pos, seamdir );
						MyMath::normalize( seamdir );
						memcpy( seamtab[ nseami ].dir, seamdir, sizeof(float)*3 );
						for( int j = 0; j < 3; j ++)
							seamtab[ nseami ].governor.push_back( triset[ i ][ j ]);
						seamtab[ nseami ].cutface[ 0 ] = triset[ i ][ 3 ];
						seamtab[ nseami ].cutface[ 1 ] = -1;

						//tripleface2seam
						//tripleface2seam.insertKeyVal( triset[ i ], 3, false, nseami );
						tripleface2seam.replaceKeyVal( triset[ i], 3, false, nseami);
					
					}
					//1.2.2.2 the seam is old
					else
					{						
						//seamtab
						merge( seamtab[ tseami ].governor, triset[ i ], 3);

						//tripleface2seam
						tripleface2seam.replaceKeyVal( triset[ i], 3, false, tseami);
						//tripleface2seam.insertKeyVal( triset[ i ], 3, false, tseami );
					}
				}

				//check another direction of the seam
			/*	for( int ti1 = 0; ti1 < 3; ti1 ++ )
				{
					seamdir[ ti1 ] = -tseamdir[ ti1 ];
				}*/
				//////////////////////////////////////////////////////////////////////////
			//	tcount ++;
			//	writeJptSeamTab( jpttab, seamtab, tcount);
			//	cout<<"tshareveri:"<<tshareveri<<"ver2jpt:"<<ver2jpt[tshareveri]<<endl;
				//////////////////////////////////////////////////////////////////////////

				//MyMath::getVec( jpttab[ ver2jpt[ tshareveri ] ].pos, jpttab[ activejpti ].pos, seamdir );
				//MyMath::normalize( seamdir );				
				//traceSeamDir(subfacenum,doubleface2sheet,  sheettab, 
				//	activejpti, seamdir, triset[ i ], activeJpts, jpttab,seamtab,tripleface2seam);

				////////////////////////////////////////////////////////////////////////////
				//tcount ++;
				//writeJptSeamTab( jpttab, seamtab, tcount);
				////////////////////////////////////////////////////////////////////////////
				delete []triset[ i ];
				continue;
			}
			
			//1.3.0 before computing the seam
			//go through all the exisitng junction points
			//to see if any of them has three govenors the same as this seam
			unsigned int tempjpti = 0;
			int foundjpti = -1;
			int fourgov = -1;
			//bool found = false;
			for(; tempjpti < jpttab.size(); tempjpti ++ )
			{
				if( tempjpti == activejpti )
					continue;
				//////////////////////////////////////////////////////////////////////////
				/*if( (tempjpti == 3) && (i == 2) )
				{
					cout<<"should found!"<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////
				//check if there is a group of four include the three
				int govsize = jpttab[ tempjpti ].governor.size()/4;
			//	bool found = false;
				for( int tk = 0; tk < govsize; tk++ )
				{
					//bool found = true;
					int ti1 = 0;
					for( ; ti1 < 3; ti1 ++ )
					{
						int ti2 = 0;
						for(; ti2 < 4; ti2 ++ )
						{
							if( jpttab[ tempjpti ].governor[ 4*tk + ti2 ] == triset[ i ][ ti1 ])
								break;
						}
						if( ti2 == 4 ) //not break out, not found!
						{
							break;
						}
					}
					if( ti1 == 3 )	//naturally over, found!
					{
						//////////////////////////////////////////////////////////////////////////
						//cout<<"found!"<<endl;
						//////////////////////////////////////////////////////////////////////////
					//	found = true;
						foundjpti = tempjpti;
						//set the fourth governor for the new seam
						//which connects the active junction point and found one.
						int tmark[ 4 ]= {0,0,0};
						for( int ti2 = 0; ti2 < 4; ti2 ++ )
						{
							int tgov = jpttab[ tempjpti ].governor[ 4*tk + ti2 ];
							for(int ti1 = 0; ti1 < 3; ti1 ++ )
							{
								if( tgov == triset[ i ][ ti1 ] )	//found
								{
									tmark[ ti2 ] = 1;
									break;
								}
							}
							if( tmark[ ti2 ] == 0 ) //not marked
							{
								fourgov = tgov;
								break;
							}
						}
						//found, break out the finding loop
						break;
					}
				}
				if( foundjpti != -1 )	//the junction point is found!
					break;
			}
			// junction point foundjpti is the destination of the seam by the three govenor
			//in triset
			if( foundjpti != -1 )	
			{
				//jpttab & activejpts
				int nseami = seamtab.size();
				jpttab[ activejpti ].seam_pos.push_back( nseami );
				jpttab[ activejpti ].seam_pos.push_back( 0 );
				jpttab[ foundjpti ].seam_pos.push_back( nseami );
				jpttab[ foundjpti ].seam_pos.push_back( 1 );
				
				//seamtab
				MASeam nseam;
				seamtab.push_back( nseam );
				seamtab[ nseami ].jpts[ 0 ] = activejpti;
				seamtab[ nseami ].jpts[ 1 ] = foundjpti;
				float tseamdir[ 3 ];
				for(int ti1 = 0; ti1 < 3; ti1 ++ )
				{
					tseamdir[ ti1 ] = jpttab[ foundjpti ].pos[ ti1 ]
					- jpttab[ activejpti ].pos[ ti1 ];
				}
				MyMath::normalize( tseamdir );
				memcpy( seamtab[ nseami ].dir, tseamdir, sizeof(float)*3 );
				for( int j = 0; j< 3; j++)
				{
					seamtab[ nseami ].governor.push_back( triset[ i ][ j ]);
				}
				seamtab[ nseami ].cutface[ 0 ] = triset[ i ][ 3 ];
				seamtab[ nseami ].cutface[ 1 ] = fourgov;
				//tripleface2seam
				tripleface2seam.replaceKeyVal( triset[ i], 3, false, nseami);

				if( fourgov == -1 )
				{
					delete []triset[ i ];
					continue;
				}
				//tripleface2seam.insertKeyVal( triset[ i ], 3, false, nseami);

				//check another direction of the seam
				for( int ti1 = 0; ti1 < 3; ti1 ++ )
				{
					seamdir[ ti1 ] = -tseamdir[ ti1 ];
				}

				//////////////////////////////////////////////////////////////////////////
			//	tcount ++;
			//	writeJptSeamTab( jpttab, seamtab, tcount);
				//////////////////////////////////////////////////////////////////////////

				//traceSeamDir(subfacenum,doubleface2sheet,  sheettab, 
				//	activejpti, seamdir, triset[ i ], activeJpts, jpttab,seamtab,tripleface2seam);

				////////////////////////////////////////////////////////////////////////////
				//tcount ++;
				//writeJptSeamTab( jpttab, seamtab, tcount);
				////////////////////////////////////////////////////////////////////////////

				delete []triset[ i ];
				continue;
				
			}
			//1.3 for each three out of the four, compute the seam
			////////////////////////////////////////////////////////////////////////////
			//for( int j = 0; j < 4;j  ++)
			//{
			//	cout<<triset[ i ][ j ]<<" ";
			//}
			//cout<<endl;
			////////////////////////////////////////////////////////////////////////////
			//getSeam(subparam, doubleface2sheet, sheettab, triset[ i ], seamdir );
			int sheetpos[ 2 ];
			//float dir[ 3 ];
			if( !doubleface2sheet.getKeyVal( triset[ i ], 2, false, sheetpos[ 0 ]) ||
				!doubleface2sheet.getKeyVal( triset[ i ]+1, 2, false, sheetpos[ 1 ]))
			{
				cout<<"In get Seam! Fail to get sheet for double face!"<<endl;
			}
			MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3,sheettab + sheetpos[ 1 ] * 6 + 3,seamdir);			
			/*int sheetpos[ 2 ];
			if( !doubleface2sheet.getKeyVal( triset, 2, false, sheetpos[ 0 ]) ||
				!doubleface2sheet.getKeyVal( triset+1, 2, false, sheetpos[ 1 ]))
			{
				cout<<"In get Seam! Fail to get sheet for double face!"<<endl;
			}
			MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3,sheettab + sheetpos[ 1 ] * 6 + 3,dir);*/
			//if( MyMath::dotProduct( subparam + 4*triset[3], dir) < 0)
			//	MyMath::stretchVec( dir, -1);
			int tsheetpos = 0;
			doubleface2sheet.getKeyVal( triset[ i ] + 2, 2, false, tsheetpos);
			float tval = MyMath::dotProduct( sheettab + 6*tsheetpos + 3, seamdir);
			int kktest=triset.size();
			if( (tval > 0 && triset[ i ][ 3 ] > triset[ i ][ 2 ]) ||
				(tval < 0 && triset[ i ][ 3 ] < triset[ i ][ 2 ]))
					MyMath::stretchVec( seamdir, -1);
			traceSeamDir(subfacenum,doubleface2sheet,  sheettab, 
					activejpti, seamdir, triset[ i ], activeJpts, jpttab,seamtab,tripleface2seam);
			////1.3 for each three out of four, check the two directions of the seam
			////and compute the junction point if it exists, 
			////refresh are done in the called functions.
			//int sheetpos[ 2 ];
			////float dir[ 3 ];
			//if( !doubleface2sheet.getKeyVal( triset[ i ], 2, false, sheetpos[ 0 ]) ||
			//	!doubleface2sheet.getKeyVal( triset[ i ]+1, 2, false, sheetpos[ 1 ]))
			//{
			//	cout<<"In get Seam! Fail to get sheet for double face!"<<endl;
			//}
			//MyMath::crossProduct( sheettab + 6*sheetpos[ 0 ] + 3,sheettab + sheetpos[ 1 ] * 6 + 3,seamdir);
			//
			//for( int twice = 0; twice < 2; twice++)
			//{
			//	if( twice == 1 )
			//		MyMath::stretchVec( seamdir, -1 );
			//	traceSeamDir(subfacenum,doubleface2sheet,  sheettab, 
			//	activejpti, seamdir, triset[ i ], activeJpts, jpttab,seamtab,tripleface2seam);

			//	//////////////////////////////////////////////////////////////////////////
			//	tcount ++;
			//	writeJptSeamTab( jpttab, seamtab, tcount);
			//	//////////////////////////////////////////////////////////////////////////

			//}

			////step2. for this seam
			//int tseami = isOldSeam( activejpti, seamdir, jpttab, seamtab);
			////	2.1, seam already exists!
			//if( tseami != -1 )
			//{
			//	//cout<<"is an old seam!"<<"activejpti:"<<activejpti<<"seami:"<<tseami<<endl;
			//	//tripleface2seam
			//	tripleface2seam.insertKeyVal( triset[ i ], 3, false, tseami );

			//	//jpttab
			//	int newgov[ 4 ];
			//	memcpy( newgov, triset[ i ], sizeof(int)*3 );
			//	int tseampos = jpttab[ activejpti ].seam_pos[ tseami * 2 + 1 ];				
			//	int tjpti = seamtab[ tseami ].jpts[ 1 - tseampos ];
			//	newgov[ 3 ] = seamtab[ tseami ].cutface[ 1 - tseampos ];
			//	//cout<<"newgov4:"<<newgov[ 3 ]<<endl;
			//	int tgovsize = jpttab[ tjpti ].governor.size();
			//	int j = 0; 
			//	for(;j < tgovsize; j += 4 )
			//	{
			//		if( isSetEqual( jpttab[ tjpti ].governor, j, 4, newgov ))
			//		{
			//			break;
			//		}
			//	}
			//	if( j >= tgovsize )	//not break out, the governor does not exist!
			//	{
			//		for( int j = 0; j < 4; j ++)
			//			jpttab[ tjpti ].governor.push_back( newgov[ j ]);
			//		activeJpts.push_back( tjpti );
			//		activeJpts.push_back( tgovsize/4 );
			//	}
			//	
			//	//seamtab
			//	merge(seamtab[ tseami ].governor, triset[ i ], 3);
			//	delete []triset[ i ];
			//	continue;
			//}
			//
			////  2.2, the seam is new, compute the other junction point on this seam
			////2.2.1	seam degnerates into one point
			////2.2.2 seam is not a point
			////2.2.3, the other junction point already exists
			////2.2.4, the other junction point is totally new
			////cout<<"copmuting new"<<endl;
			//computeNewJpt(subfacenum,doubleface2sheet,  sheettab, 
			//	activejpti, seamdir, triset[ i ], activeJpts, jpttab,seamtab,tripleface2seam);

			delete []triset[ i ];
		}
		triset.clear();
	}	
}
void MAGenerator::generateMA(int planenum, float* planeparam,int ssvernum, float* ssver,int ssedgenum,
							 int* ssedge,int ssfacenum, int* ssfaceedgenum, int** ssface,int* ssface_planeindex, 
							 int ssspacenum,  int* ssspacefacenum,	int** ssspace, int** ssspace_planeside, int subspacei,
							//subspace info
							 int& subvernum, float*& subver, int& subedgenum,int*& subedge,int& subfacenum,
							 int*& subfaceedgenum, int**& subface, float*& subparam, int*& subver2wver,int*& subedge2wedge,
							 //ma info
							int& majptnum, float*& majpt, int& maseamnum, int*& maseam, 
							 MapArraySR& doubleface2sheet, int*& seamonsheetnum, int**& seamonsheet,  float*& sheettab,
							 int*& ver2jpt )
{ 	
	/*int subvernum;
	float* subver;
	int subedgenum;
	int* subedge;
	int subfacenum;
	int* subfaceedgenum;
	int** subface;
	float* subparam;
	int* subver2wver;
	int* subedge2wedge;*/

//	step1. get information of subspace
	getSubspaceInfo(planenum, planeparam,
		ssvernum,  ssver,ssedgenum,ssedge,ssfacenum, ssfaceedgenum, ssface,
		ssface_planeindex,ssspacenum,ssspacefacenum,ssspace, ssspace_planeside, 
		subspacei,
		subvernum, subver,subedgenum, subedge,subfacenum,subfaceedgenum,subface,subparam,
		subver2wver,subedge2wedge);

	/*subvernum = 6;
	subver = new float[ 18 ];
	subedgenum = 12;
	subedge = new int[ 24 ];
	subfacenum = 8;
	subfaceedgenum = new int[ 8 ];
	subface = new int*[ 8 ];
	subparam = new float[ 32 ];
	subver2wver = new int[ 8 ];
	subedge2wedge = new int[ 12 ];
	subver[ 0 ] = 0;	subver[ 1 ] = 0;	subver[ 2 ] = 1;
	subver[ 3 ] = 0;	subver[ 4 ] = -1;	subver[ 5 ] = 0;
	subver[ 6 ] = 1;	subver[ 7 ] = 0;	subver[ 8 ] = 0;
	subver[ 9 ] = 0;	subver[ 10 ] = 1;	subver[ 11 ] = 0;
	subver[ 12 ] = -1;	subver[ 13 ] = 0;	subver[ 14 ] = 0;
	subver[ 15 ] = 0;	subver[ 16 ] = 0;	subver[ 17 ] = -1;

	subedge[ 0 ] = 0;	subedge[ 1 ] = 1;
	subedge[ 2 ] = 0;	subedge[ 3 ] = 2;
	subedge[ 4 ] = 0;	subedge[ 5 ] = 3;
	subedge[ 6 ] = 0;	subedge[ 7 ] = 4;
	subedge[ 8 ] = 1;	subedge[ 9 ] = 2;
	subedge[ 10 ] = 2;	subedge[ 11 ] = 3;
	subedge[ 12 ] = 3;	subedge[ 13 ] = 4;
	subedge[ 14 ] = 4;	subedge[ 15 ] = 1;
	subedge[ 16 ] = 1;	subedge[ 17 ] = 5;
	subedge[ 18 ] = 2;	subedge[ 19 ] = 5;
	subedge[ 20 ] = 3;	subedge[ 21 ] = 5;
	subedge[ 22 ] = 4;	subedge[ 23 ] = 5;

	for( int i = 0; i < 8; i ++)
	{
		subfaceedgenum[ i ] = 3;
		subface[ i ] = new int[ 3 ];
	}
	subface[ 0 ][ 0 ] = 0;	subface[ 0 ][ 1 ] = 1;	subface[ 0 ][ 2 ] = 4;
	subface[ 1 ][ 0 ] = 1;	subface[ 1 ][ 1 ] = 2;	subface[ 1 ][ 2 ] = 5;
	subface[ 2 ][ 0 ] = 2;	subface[ 2 ][ 1 ] = 3;	subface[ 2 ][ 2 ] = 6;
	subface[ 3 ][ 0 ] = 3;	subface[ 3 ][ 1 ] = 0;	subface[ 3 ][ 2 ] = 7;
	subface[ 4 ][ 0 ] = 8;	subface[ 4 ][ 1 ] = 9;	subface[ 4 ][ 2 ] = 4;
	subface[ 5 ][ 0 ] = 9;	subface[ 5 ][ 1 ] = 10;	subface[ 5 ][ 2 ] = 5;
	subface[ 6 ][ 0 ] = 10;	subface[ 6 ][ 1 ] = 11;	subface[ 6 ][ 2 ] = 6;
	subface[ 7 ][ 0 ] = 11;	subface[ 7 ][ 1 ] = 8;	subface[ 7 ][ 2 ] = 7;
    
	subparam[ 0 ] = -1;	subparam[ 1 ] = 1;	subparam[ 2 ] = -1;	subparam[ 3 ] = -1;
	subparam[ 4 ] = -1;	subparam[ 5 ] = -1;	subparam[ 6 ] = -1;	subparam[ 7 ] = -1;
	subparam[ 8 ] = 1;	subparam[ 9 ] = -1;	subparam[ 10 ] = -1;	subparam[ 11 ] = -1;
	subparam[ 12 ] = 1;	subparam[ 13 ] = 1;	subparam[ 14 ] = -1;	subparam[ 15 ] = -1;
	subparam[ 16 ] = -1;	subparam[ 17 ] = 1;	subparam[ 18 ] = 1;	subparam[ 19 ] = -1;
	subparam[ 20 ] = -1;	subparam[ 21 ] = -1;	subparam[ 22 ] = 1;	subparam[ 23 ] = -1;
	subparam[ 24 ] = 1;	subparam[ 25 ] = -1;	subparam[ 26 ] = 1;	subparam[ 27 ] = -1;
	subparam[ 28 ] = 1;	subparam[ 29 ] = 1;	subparam[ 30 ] = 1;	subparam[ 31 ] = -1;
	for( int i = 0; i < 8; i ++)
	{
		float len = MyMath::vectorlen( subparam + 4*i );
		for( int j = 0; j< 4; j ++)
		{
			subparam[ 4*i + j ] = subparam[ 4*i + j ]/len;
		}
	}*/
	
	//////////////////////////////////////////////////////////////////////////
	//write out the subspace information
	/*char fname[ 1024 ];
	memset( fname, '\0', 1024);
	strcpy( fname, "mmdebug/onesubspace/");
	char numstr [ 10 ];
    itoa( subspacei, numstr, 10 );
	strcat( fname, numstr);
	strcat(fname,"subspace.txt");
	writeSubspaceInfo(fname,subvernum,subver,subedgenum, subedge,subfacenum,subfaceedgenum,subface,subparam,
		subver2wver,subedge2wedge);*/
	//////////////////////////////////////////////////////////////////////////

	//step2. get the topology of current subspace
	doubleface2sheet.setParam(subfacenum, true, 2);
	sheettab = new float[ subfacenum*(subfacenum-1)*3];

	//MapArraySR doubleface2sheet( subfacenum, true, 2);
	//select two faces from all the faces and 6 numbers for one sheet
	//float* sheettab = new float[ subfacenum*(subfacenum-1)*3];	
	MapArraySR tripleface2ver( subfacenum, true, 3);
	MapArraySR doubleface2edge( subfacenum, true, 2 );
	int** ver2face;
	int* ver2facenum;
	int* edge2face;
	int** face2ver;
	int* face2vernum;
	ver2face = new int*[ subvernum];
	ver2facenum = new int[ subvernum ];
	edge2face = new int[ 2*subedgenum ];
	face2ver = new int*[ subfacenum ];
	face2vernum = new int[ subfacenum ];
	gatherTopology(subvernum,subver,subedgenum, subedge,subfacenum,subfaceedgenum,subface,
		subparam,doubleface2sheet, sheettab,tripleface2ver, doubleface2edge,ver2face,ver2facenum,
		edge2face,face2ver, face2vernum );
	//////////////////////////////////////////////////////////////////////////
	//strcat(fname, ".top");
	//writeTopology(fname, subvernum, subedgenum, subfacenum,
	//	doubleface2sheet, sheettab, tripleface2ver,  doubleface2edge,
	//	ver2face, ver2facenum, edge2face, face2ver, face2vernum );
	//////////////////////////////////////////////////////////////////////////

	//step 3. find the first junction point
	intvector activeJpts;
	vector<MAJunctionPoint> jpttab;
	vector<MASeam> seamtab;
	MapArraySR tripleface2seam( subfacenum, true, 3);
	ver2jpt = new int[ subvernum ];
	//int* ver2jpt = new int[ subvernum ];
	for( int i = 0; i < subvernum; i ++)
		ver2jpt[ i ] = -1;
	findFirstJpt(subvernum,subver, subedge,subfacenum,subparam,
		ver2face,ver2facenum,doubleface2sheet, sheettab,doubleface2edge,
		activeJpts, jpttab,seamtab,tripleface2seam,ver2jpt);
	//////////////////////////////////////////////////////////////////////////
//	strcpy( fname,"3.fjpt\0");
//	writeSubspaceInfoWithFirstJpt(fname,subvernum,subver,subedgenum, subedge,subfacenum,subfaceedgenum,subface,subparam,
//				subver2wver,subedge2wedge,jpttab);
	//////////////////////////////////////////////////////////////////////////

	//step4.trace algorithm to find out all the junciton points and seams
	traceMA( subver,subparam, subfacenum,tripleface2ver, doubleface2sheet,   sheettab,
		 activeJpts,  jpttab,seamtab,tripleface2seam,ver2jpt);

	//step 5. post process, sort out all the information needed
	majptnum = jpttab.size();
	majpt = new float[ 3*majptnum ];
	for( int i = 0; i< majptnum; i++)
	{
		memcpy( majpt + 3*i, jpttab[ i ].pos, sizeof (float) * 3 );
	}
	
	//maseamnum = seamtab.size();
	//maseam = new int[ maseamnum * 2 ];
	maseamnum = seamtab.size() + subedgenum;
	maseam = new int[ maseamnum * 2 ];
	for(unsigned int i = 0; i < seamtab.size(); i ++)
	{
		memcpy( maseam + i*2, seamtab[ i ].jpts, sizeof ( int )*2 );
	}

	int ind = seamtab.size() * 2;
	for( int i = 0 ;i < subedgenum*2; i += 2)
	{
		maseam[ ind ++ ] = ver2jpt[subedge[ i ]];
		maseam[ ind ++ ] = ver2jpt[subedge[ i + 1 ]];
	}

	vector<intvector> sheetseams;
	int sheetnum = subfacenum * (subfacenum - 1)/2;
	sheetseams.resize( sheetnum );
	int sheeti;
	int facei[ 2 ];
	for(unsigned int i = 0; i < seamtab.size(); i++)
	{
		int govsize = seamtab[ i ].governor.size();
		//////////////////////////////////////////////////////////////////////////
		/*for( int j = 0; j <govsize; j ++)
			cout<<seamtab[ i ].governor[ j ]<<" ";
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////
		for( int j1 = 0; j1 < govsize; j1++)
		{
			facei[ 0 ] = seamtab[ i ].governor[ j1 ];
			for( int j2 = j1 + 1; j2 < govsize; j2++ )
			{
				facei[ 1 ] = seamtab[ i ].governor[ j2 ];
				doubleface2sheet.getKeyVal( facei, 2, false, sheeti );
				sheetseams[ sheeti ].push_back( i );
			}
		}
	}

	//push back the seams that are edges
	ind = seamtab.size();
	for( int i = 0; i < subedgenum ; i ++)
	{
		int* dface = edge2face + 2*i;
		doubleface2sheet.getKeyVal( dface, 2, false, sheeti );
		sheetseams[ sheeti ].push_back( ind++ );
		dface = NULL;
	}

	seamonsheet = new int*[ sheetnum ];
	seamonsheetnum = new int[ sheetnum ];
	for( int i = 0; i < sheetnum; i++)
	{
		seamonsheetnum[ i ]	= sheetseams[ i ].size();
		//////////////////////////////////////////////////////////////////////////
		//cout<<seamonsheetnum[ i ]<<"  ";
		if( seamonsheetnum[ i ] <= 1 )	//if equal or less than one seams on it, degenerate case.
		{
			seamonsheetnum[ i ] = 0;
			seamonsheet[  i ] = NULL;
		}
		seamonsheet[ i ] = new int[ seamonsheetnum[ i ]];
		for( int j = 0; j < seamonsheetnum[ i ]; j ++)
		{
			seamonsheet[ i ][ j ] = sheetseams[ i ][ j ];
		}
		sheetseams[ i ].clear();
	}
	sheetseams.clear();

	//clear temporary variables
	/*int** ver2face;
	int* ver2facenum;
	int* edge2face;
	int** face2ver;
	int* face2vernum;
	intvector activeJpts;
	vector<MAJunctionPoint> jpttab;
	vector<MASeam> seamtab;*/
	for( int i = 0; i < subvernum; i ++)
		delete []ver2face[ i ];
	delete []ver2face;
	delete []edge2face;

	for( int i = 0; i < subfacenum ; i++)
		delete []face2ver[ i ];
	delete []face2ver;

	delete []face2vernum;
	activeJpts.clear();
	for(unsigned int i = 0; i < jpttab.size(); i ++)
	{
		jpttab[ i ].governor.clear();
		jpttab[ i ].seam_pos.clear();
	}
	jpttab.clear();
    for(unsigned int i = 0; i < seamtab.size(); i++)
	{
		seamtab[ i ].governor.clear();
	}
	seamtab.clear();
}

void MAGenerator::writeJptSeamTab(vector<MAJunctionPoint>& jpttab,
					 vector<MASeam>& seamtab,
					 int count)
{
	char fname[ 30 ] = "jptseamtab";
	char tnum[ 5 ];
	itoa( count, tnum, 10 );
	strcat( fname, tnum );
	strcat( fname, ".txt");
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	int jptnum = jpttab.size();
	for( int i = 0; i < jptnum; i ++ )
	{
		if( i == 0 )
		{
			fprintf( fout, "{{{{%f,%f,%f},{", jpttab[ i].pos[ 0 ], jpttab[ i ].pos[1 ], jpttab[ i ].pos[2] );		
		}
		else
			fprintf( fout, "}},{{%f,%f,%f},{", jpttab[ i].pos[ 0 ], jpttab[ i ].pos[1 ], jpttab[ i ].pos[2] );
		int govsize = jpttab[ i ].governor.size();
		for( int j = 0; j < govsize; j++ )
		{
			fprintf( fout, "%d",jpttab[ i ].governor[ j ] + 1);
			if( j != govsize - 1)
				fprintf(fout, ",");
		}
		fprintf( fout, "},{");
		int seamsize = jpttab[ i ].seam_pos.size();
		for( int j = 0; j < seamsize; j ++ )
		{
			fprintf( fout, "%d", jpttab[ i ].seam_pos[ j ] + 1);
			if( j != seamsize - 1)
				fprintf( fout, ",");
		}
	}
		
	int seamnum = seamtab.size();
	for( int i = 0; i< seamnum; i ++ )
	{
		if( i == 0)
		{
			fprintf( fout, "}}},{{{%d,%d},{", seamtab[ i ].jpts[ 0 ] + 1, seamtab[ i ].jpts[ 1 ] + 1 );
		}
		else
		{	
			fprintf( fout, "}},{{%d,%d},{", seamtab[ i ].jpts[ 0 ] + 1, seamtab[ i ].jpts[ 1 ] + 1 );
		}
		int govsize = seamtab[ i ].governor.size();
		for( int j = 0; j < govsize; j ++ )
		{
			fprintf( fout, "%d", seamtab[ i ].governor[ j ] + 1);
			if( j != govsize - 1)
				fprintf( fout, ",");
		}
	}
	fprintf( fout ,"}}}}");

	fclose( fout );
}
void MAGenerator::writeMA(
						  const char* fname,
						  //subspace info
						  int& subvernum, float*& subver, int& subedgenum,int*& subedge,int& subfacenum,
						  int*& subfaceedgenum, int**& subface, 
						  //ma info
						  int& majptnum, float*& majpt, int& maseamnum, int*& maseam, 
						  MapArraySR& doubleface2sheet, int*& seamonsheetnum, int**& seamonsheet, 
						  int*& ver2jpt)
{
	FILE* fout = fopen( fname, "w");
	if( fout == NULL)
	{
		cout<<"Unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	//write vertex
	fprintf( fout, "{{{%f,%f,%f}", subver[ 0 ], subver[ 1 ], subver[ 2 ]);
	for( int i = 1; i < subvernum; i ++)
	{
		fprintf( fout, ",{%f,%f,%f}", subver[ i*3 ], subver[ i * 3 + 1 ] , subver[ i*3 + 2 ]);
	}

	//write edge
	fprintf( fout, "},{{%d,%d}", subedge[ 0 ]+1, subedge[ 1 ] + 1);
	for( int i = 1 ; i< subedgenum; i ++)
	{
		fprintf( fout, ",{%d,%d}", subedge[ 2*i ] + 1, subedge[ i * 2 + 1] + 1);
	}

	//write face
	fprintf( fout, "},{{%d", subface[ 0 ][ 0 ] + 1);
	for( int i = 1; i < subfaceedgenum[ 0 ]; i ++)
	{
		fprintf( fout, ",%d", subface[ 0 ][ i ] + 1 );
	}
	for( int i = 1; i < subfacenum; i ++)
	{
		fprintf(fout, "},{%d", subface[ i ][ 0 ] + 1);
		for( int j = 1; j < subfaceedgenum[ i ]; j++ )
			fprintf( fout, ",%d", subface[ i ][ j ] + 1);
	}
	
	//write junction points
	fprintf( fout, "}},{{%f,%f,%f}", majpt[ 0 ], majpt[ 1 ], majpt[ 2 ]);
	for( int i = 1; i < majptnum; i++)
	{
		fprintf( fout, ",{%f,%f,%f}", majpt[ 3*i ], majpt[ 3*i + 1 ], majpt[ 3*i + 2 ]);
	}

	//write seams
	fprintf( fout, "},{{%d,%d}", maseam[ 0 ] + 1 , maseam[ 1 ] + 1);
	for( int i = 1; i < maseamnum; i++)
	{
		fprintf(fout, ",{%d,%d}", maseam[ 2*i] + 1, maseam[ i*2 + 1] + 1);
	}
	
	//write doubleface2seam
	int sheeti;
	bool first = true;
	for( int i = 0; i < subfacenum; i ++)
	{
		for( int j = i + 1; j < subfacenum ; j ++)
		{
			int keys [2 ] = { i, j};
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti);
			if( first )
			{
				first = false;
				if( seamonsheetnum[ sheeti ] == 0)
				{
					fprintf( fout, "},{{");
					continue;
				}

				fprintf( fout, "},{{%d", seamonsheet[ sheeti ][ 0 ] + 1);
			}
			else
			{
				if( seamonsheetnum[ sheeti ] == 0)
				{
					fprintf( fout, "},{");
					continue;
				}
				fprintf( fout, "},{%d", seamonsheet[ sheeti][ 0 ] + 1);
			}
			for( int k = 1; k < seamonsheetnum[ sheeti ]; k++)
			{
				fprintf(fout, ",%d", seamonsheet[ sheeti ][ k ] + 1);
			}
		}
	}

	//ver2jpt
	fprintf( fout, "}},{%d", ver2jpt[ 0 ] + 1);
	for(int i = 1 ; i< subvernum; i++)
		fprintf( fout, ",%d", ver2jpt[ i ] + 1);
	fprintf( fout , "}}");

	fclose( fout );
}
MAGenerator::MAGenerator()
{

}
MAGenerator::~MAGenerator()
{

}