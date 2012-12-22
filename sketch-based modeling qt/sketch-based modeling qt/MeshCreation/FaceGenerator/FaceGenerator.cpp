#include "../FaceGenerator/FaceGenerator.h"

void FaceGenerator::get2DVer(
	floatvector& shtVpos_fvec,
	float xdir[ 3 ],
	float zdir[ 3 ],
	floatvector& vpos2D
)
{
	float ydir[ 3 ];
	MyMath::crossProductNotNorm( zdir, xdir,  ydir );

	int vnum = shtVpos_fvec.size()/3;
	vpos2D.resize( 2*vnum );
	for( int i = 0; i < vnum; i ++ )
	{
		float vpos[ 3 ];
		for( int j= 0; j < 3; j ++ )
			vpos[ j ] = shtVpos_fvec[ 3*i + j ];
		vpos2D[ 2*i ] = MyMath::dotProduct( vpos, xdir );
		vpos2D[ 2*i + 1 ] = MyMath::dotProduct( vpos, ydir );
	}
}
//sort the vertices out for the current region
void FaceGenerator::getCycle(
  sheetRegion& region, 
  //intvector& shtVposInMV_ivec,  floatvector& shtVpos_fvec,  
  intvector& shtEcmpos_ivec,
  int regioni,
  //float zdir[ 3 ],  float xdir[ 3 ],
  //floatvector& ver2D,  
  //intvector& vposinmesh, 
  vector<intvector>& cycles  )
{
	//floatvector& ver3d;
//	intvector& veri;	//index of the vertices in the cycles
//	int avnum = 0;
	int cyclenum = region.regions[ regioni ].size();
	cycles.resize( cyclenum );
	int bi;
	for( int i = 0; i < cyclenum; i ++ )
	{
		bi = region.regions[ regioni ][ i ];
		int vnum = region.boundaries[ bi ].size();	//also edge number in the boundary
		//////////////////////////////////////////////////////////////////////////
		//cout<<"vnum:"<<vnum<<endl;
		//////////////////////////////////////////////////////////////////////////
	//	avnum += vnum;
		cycles[ i ].resize( vnum );
		/*int veriosize = veri.size();*/
		//veri.resize( veriosize + vnum );
		for( int j = 0; j < vnum; j ++ )
		{
			int ei = region.boundaries[ bi ][ j ];
		/*	veri[ veriosize ] = shtEcmpos_ivec[ ei * 5 ];
			veriosize ++;*/

			cycles[ i ][ j ] = shtEcmpos_ivec[ ei * 5 ];
		}
	}
}
void FaceGenerator::writeVerCycle(
	floatvector& ver2d, 
	vector<intvector>& cycles, 
	int sheeti, int regioni)
{
	char fname[ 20 ] = "vercycle";
	char strnum[ 5 ];
	itoa( sheeti, strnum, 10);
	strcat( fname, strnum );
	itoa( regioni, strnum, 10 );
	strcat( fname, "_");
	strcat( fname, strnum);
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");

	if( fout == NULL )
	{
		cout<<"unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	//write out vertice first
	int vnum = ver2d.size()/2;
	for( int i = 0; i< vnum; i ++)
	{
		if( i == 0 )
		{
			fprintf( fout, "{{{%f,%f}", ver2d[ 2*i ], ver2d[ 2* i + 1]);
		}
		else
		{
			fprintf( fout, "{%f,%f}", ver2d[ 2*i ], ver2d[ 2*i + 1]);
		}
		if( i != vnum - 1)
			fprintf( fout, ",");
	}

	int cyclenum = cycles.size();
	for( int i = 0; i < cyclenum; i ++ )
	{
		if( i == 0 )
		{
			fprintf( fout, "},{");
		}
		
		int vnum = cycles[i].size();
		//////////////////////////////////////////////////////////////////////////
		//cout<<"in write: vnum: "<<vnum<<endl;
		//////////////////////////////////////////////////////////////////////////
		for( int j = 0; j < vnum ;j ++)
		{
			if( j == 0 )
				fprintf( fout, "{%d", cycles[ i ][ j ] + 1);
			else
				fprintf( fout, "%d", cycles[ i ][ j ] + 1);
			if( j != vnum - 1)
				fprintf( fout, ",");
		}

		if( i != cyclenum - 1)
			fprintf( fout, "},");
		else
			fprintf( fout, "}}}");
	}

	fclose( fout );
}

void FaceGenerator::GFaceOnSheets(
	intvector& meshFace,
	intvector*& shtVposInMV_arr_ivec,
	floatvector*& shtVpos_arr_fvec,
	intvector*& shtEcmpos_arr_ivec,
	sheetRegion*& region_vec,
	float*& sheettab,
	int facenum,
	int*& maseamnum,
	MapArraySR& doubleface2sheet)
{
	//step1. generate face on the masheets
	int keys[ 2 ];
	int sheeti;
	int tmat[ 2 ];
	bool verget;
	float zdir[ 3 ] ;
	float xdir[ 3 ];
	floatvector ver2d;
	vector<intvector> cycles;
	for( int facei = 0; facei < facenum -1 ; facei ++ )
	{
		keys[ 0 ] = facei;
		for( int facej = facei + 1; facej < facenum; facej ++ )
		{
			keys[ 1 ] = facej;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );

			if( maseamnum[ sheeti ] == 0 )	//no such sheet exists
				continue;

			verget = false;
			int regionnum = region_vec[ sheeti ].regions.size();
			int outregi = region_vec[ sheeti ].outregion;
			//////////////////////////////////////////////////////////////////////////
	//		cout<<"sheeti:"<<sheeti<<endl;
			//////////////////////////////////////////////////////////////////////////
			for( int regioni = 0; regioni < regionnum; regioni ++ )
			{
				if( outregi == regioni )
					continue;
				tmat[ 0 ] = region_vec[ sheeti ].mat[ regioni * 2 ];
				tmat[ 1 ] = region_vec[ sheeti ].mat[ regioni * 2 + 1 ];
				if( tmat[ 0 ] == tmat[ 1 ])		//no need to process this region
					continue;

				//1.1 get the 2D vertices list if the first time to process current sheet
				if(!verget )
				{
					//clear old one
					ver2d.clear();

					//get the 2d vertices of current sheet
					for( int tk = 0; tk < 3; tk ++ )
					{
						zdir[ tk ] = sheettab[ 6*sheeti + 3 + tk ];
					}
					xdir[ 0 ] = xdir[ 1 ] = xdir[ 2 ] = 0;
					if( abs( zdir[ 0 ] ) < TOLERANCE_SIX  )
						xdir[ 0 ] = 1;
					else if( abs( zdir[ 1 ] ) < TOLERANCE_SIX )
						xdir[ 1 ] = 1;
					else
					{
						xdir[ 0 ] = zdir[ 1 ];
						xdir[ 1 ] = -zdir[ 0 ];
					}
					get2DVer( shtVpos_arr_fvec[ sheeti ], xdir, zdir, ver2d);
					verget = true;
				}

				//1.2 get the cycles of the regions on sheet
				for(unsigned int i = 0; i < cycles.size(); i ++ )
					cycles[ i ].clear();
				cycles.clear();
				
				getCycle(region_vec[ sheeti ], 	shtEcmpos_arr_ivec[ sheeti ],
					regioni,cycles );

				//1.3 triangulate the regions and put the triangles into face
				intvector tri_vec;
				//////////////////////////////////////////////////////////////////////////
			//	cout<<"sheeti:"<<sheeti<<"regioni:"<<regioni<<endl;
			//	writeVerCycle(ver2d, cycles, sheeti, regioni);
				//////////////////////////////////////////////////////////////////////////
				Triangulation::triangulate( ver2d, cycles, tri_vec);
				//////////////////////////////////////////////////////////////////////////
				
				//////////////////////////////////////////////////////////////////////////
			
				int trinum = tri_vec.size()/3;
				int j = 0;
                for(int i = 0; i < trinum; i ++ )
				{
					meshFace.push_back( shtVposInMV_arr_ivec[ sheeti ][tri_vec[ j ++ ]]);
					meshFace.push_back( shtVposInMV_arr_ivec[ sheeti ][tri_vec[ j ++ ]]);
					meshFace.push_back( shtVposInMV_arr_ivec[ sheeti ][tri_vec[ j ++ ]] );
					meshFace.push_back( tmat[ 0 ] );
					meshFace.push_back( tmat[ 1 ] );
				}
			}		
		}
	}
	ver2d.clear();
	for(unsigned int i = 0; i < cycles.size() ; i++ )
		cycles[ i ].clear();
	cycles.clear();

}

void FaceGenerator::addCtrVerOneFace(
	floatvector& meshVer,
	SSPCTRVERVEC& ctrver,
	//SSPCTREDGEVEC& ctredge,
	int*& cvposinmv,
	//int*& ver2jpt,
	int*& jpt2ver,	//from junction to subspace vertex index
	int* njptreg,		//map from the junction point to the position of it in meshver		
	int spacedgenum, int maseamnum,		newSeamReg*& nseamReg,
	float*& ssver,		int*& ssedge,		//int ssfaceedgenum,		
//	int subspacedgenum,
	//int*& ssfaceedge,
	int*& edge2wedge,		intvector*& sverreg,	vector<intvector>*& sedgereg,
	int spaci
	)
{
	vector<intvector> ssedgevs;
	ssedgevs.resize( spacedgenum );
	int ctrvnum = ctrver.size();
	int seamnum = maseamnum - spacedgenum;
	for(int i = 0; i < ctrvnum; i ++ )
	{
		int type = ctrver[ i ].type;
		switch( type )
		{
		case VER_JPT:
			{
				//check if the junction point is one vertex
				//yes, then set the corresponding vertex in meshver
				//no, add the vertex and set the corresponding vertex
				int val = ctrver[ i ].val;	//the junction point index
				int veri = jpt2ver[ val ];	//ssver index
				if( veri != -1 )
				{
					cvposinmv[ i ] = njptreg[ val ];
					sverreg[ veri ].push_back( spaci );
					sverreg[ veri ].push_back( njptreg[ val ]);
					//sverreg[ veri ] = njptreg[ val ];	//set the registration info
					continue;
				}
				//not a vertex
				meshVer.push_back( ctrver[ i ].pos[ 0 ]);
				meshVer.push_back( ctrver[ i ].pos[ 1 ]);
				meshVer.push_back( ctrver[ i ].pos[ 2 ]);
				cvposinmv[ i ] = meshVer.size()/3 - 1;
			}
			break;
		case VER_SEAM:
			{
				int val = ctrver[ i ].val;	//the seam index
				//check if the seam is actually an edge
				//yes, remember it, process them later
				if ( val >= seamnum)	//is an edge
				{
					int ei = val - seamnum;
					ssedgevs[ ei ].push_back( i );	//the i th vertex is on ei
					continue;
				}				
				//no, add it into meshVer, and set crp info
				meshVer.push_back( ctrver[ i ].pos[ 0 ]);
				meshVer.push_back( ctrver[ i ].pos[ 1 ]);
				meshVer.push_back( ctrver[ i ].pos[ 2 ]);
				cvposinmv[ i ] = meshVer.size()/3 - 1;
			}
			break;
		case VER_SHEET:
			{
				//add the vertex into the meshVer, and set crsp info
				meshVer.push_back( ctrver[ i ].pos[ 0 ]);
				meshVer.push_back( ctrver[ i ].pos[ 1 ]);
				meshVer.push_back( ctrver[ i ].pos[ 2]) ;
				cvposinmv[ i ] = meshVer.size()/3 - 1;
			}
			break;
		default:
			break;
		}
	}

	//go through all the subspace edges, and set the crsp info
	for( int i= 0; i < spacedgenum; i ++ )
	{		
		int vnum = ssedgevs[ i ].size();

		//no registered vertex on this edge at all!
		if( vnum == 0 )
			continue;

	//	int sei = ssfaceedge[ i ];
		int sei = edge2wedge[ i ];	//from the ith edge in this subspace, which subspace edge in the whole edge list
		int svi[ 2 ] =  {ssedge[ 2*sei ], ssedge[ 2*sei + 1]};
		float* pver[ 2 ] = {ssver + 3*svi[ 0 ],  ssver + 3*svi[ 1 ]};
	
		//sort them according to the increasing order of the parameter
		float* param = new float[ vnum ];
		for( int j = 0; j < vnum; j ++ )
		{
			int vi = ssedgevs[ i ][ j ];
			param[ j ] = getParamOnSeg( pver[ 0 ], pver[ 1 ], ctrver[ vi ].pos );
		}
		pver[ 0 ] = pver[ 1 ] = NULL;

		//sort the contour vertices according to their parameters
		for(int i1 = 0; i1 < vnum; i1 ++ )
		{
			int mini = i1;
			for(int i2 = i1 + 1; i2 < vnum; i2 ++ )
			{
				if( param[ i2 ] < param[ mini ])
				{
					mini = i2;
				}
			}

			if( mini == i1 )
				continue;

			//swap the parameter
			float tv = param[ mini ];
			param[ mini ] = param[ i1 ];
			param[ i1 ] = tv;

			//swap the index
			int tvi = ssedgevs[ i ][ mini ];
			ssedgevs[ i ][ mini ] = ssedgevs[ i ][ i1 ];
			ssedgevs[ i ][ i1 ] = tvi;
		}

		//set the corresponding info
		int seami = maseamnum - spacedgenum  + i;
#ifdef debug
		if( nseamReg[ seami].vernum != vnum + 2)
		{
			cout<<"The added vertices on seam is not equal to the actual contour vertex on the edge!"
				<<"\nvertex# in seamedge and vertex# in the contour edge:\n"
				<<nseamReg[seami ].vernum - 2
				<<vnum<<endl;
		}
#endif
		//sedgereg[ ssfaceedge[ i ]].clear();	
		int realei = edge2wedge[ i ];
		if( sedgereg[ realei].size() == 0)
			sedgereg[ realei ].resize( vnum );
		for( int i1 = 0; i1 < vnum; i1 ++ )
		{
			int tvi = cvposinmv[ ssedgevs[ i ][ i1 ]] = nseamReg[ seami ].verPosInMeshVer[ i1 + 1];
			sedgereg[ realei ][ i1 ].push_back( spaci );
			sedgereg[ realei ][ i1 ].push_back( tvi );
		}
		delete []param;
	}

	//clear the temp var
	for( int i = 0; i < spacedgenum; i ++)
		ssedgevs[ i ].clear();
	ssedgevs.clear();
}

//generate faces by connecting projected edges on normal seams(not edges of subspace)
//and the contour edges
void FaceGenerator::GFaceNormalSeams(
									 intvector& meshFace,
									 vector<SSPCTREDGEVEC>& sspctredge_vec,
									 newSeamReg*& nseamReg,	int maseamnum,		int subedgenum,
									 int**& cvposinmesh,		int spaci,		intset*& sfacectrei,
									 int*& ssface,		//the face index in the whole subspace
									 int*& sfacespaci,	//for each facei, save two subspaces neighboring with it
									 vector<intvector>*& sfaceregface,
									 int*& faceside)
{
	int normalseamnum = maseamnum - subedgenum;
	for( int i= 0; i < normalseamnum; i ++ )
	{
		int edgenum = nseamReg[ i ].vernum - 1;
		for( int j = 0; j < edgenum; j ++)
		{
			int crspenum = nseamReg[ i ].edgelist[ j ].crspCtrEdges.size();
			if( crspenum == 0)
				continue;
			//go through all the faces and generate faces
			int* vi = nseamReg[ i ].verPosInMeshVer + j;
			for(int k = 0; k < crspenum; k ++ )
			{
				int facei = nseamReg[ i ].edgelist[ j ].crspCtrEdges[ k ].facei;
				int edgei = nseamReg[ i ].edgelist[ j ].crspCtrEdges[ k ].edgei;
				bool dir = nseamReg[ i ].edgelist[ j ].crspCtrEdges[ k ].samedirec;
			
				int* cvi = sspctredge_vec[ facei ][ edgei ].veris;
				int cvmpos[ 2 ];
				for( int tk = 0; tk < 2; tk ++ )
					cvmpos[ tk ] = cvposinmesh[ facei ][ cvi[ tk ]];
				int mpos[ 2 ];
				if( dir )
				{
					mpos[ 0 ] = vi[ 0 ];
					mpos[ 1 ] = vi[ 1 ];
				}
				else
				{
					mpos[ 0 ] = vi[ 1 ];
					mpos[ 1 ] = vi[ 0 ];
				}
				int nfacei = meshFace.size()/5;
				int mat[ 2 ] = { sspctredge_vec[ facei][ edgei ].mat[ 0 ],  sspctredge_vec[ facei][ edgei ].mat[ 1 ]};
				if( mpos[ 0 ] == cvmpos[ 0 ])	//the first vertices are the same
				{
					meshFace.push_back( cvmpos[ 0 ]);
					meshFace.push_back( cvmpos[ 1 ]  );
					meshFace.push_back( mpos[ 1 ] );
					if( faceside[ facei ] == 1 )
					{
						meshFace.push_back( mat[ 1 ]);
						meshFace.push_back( mat[ 0 ]);
					}
					else
					{
						meshFace.push_back( mat[ 0 ]);
						meshFace.push_back( mat[ 1 ]);
					}				
				}
				else if( mpos[ 1 ] == cvmpos[ 1 ])
				{
					meshFace.push_back( cvmpos[ 0 ]);
					meshFace.push_back( cvmpos [ 1 ]);
					meshFace.push_back( mpos[ 0 ]);
					if( faceside[ facei ] == 1 )
					{
						meshFace.push_back( mat[ 1 ]);
						meshFace.push_back( mat[ 0 ]);
					}
					else
					{
						meshFace.push_back( mat[ 0 ]);
						meshFace.push_back( mat[ 1 ]);
					}		
				}
				else
				{
					meshFace.push_back( cvmpos[ 0 ]);
					meshFace.push_back( cvmpos [ 1 ]);
					meshFace.push_back( mpos[ 0 ]);
					if( faceside[ facei ] == 1 )
					{
						meshFace.push_back( mat[ 1 ]);
						meshFace.push_back( mat[ 0 ]);
					}
					else
					{
						meshFace.push_back( mat[ 0 ]);
						meshFace.push_back( mat[ 1 ]);
					}		
					meshFace.push_back( cvmpos[ 1 ]);
					meshFace.push_back( mpos [ 1 ]);
					meshFace.push_back( mpos[ 0 ]);
					if( faceside[ facei ] == 1 )
					{
						meshFace.push_back( mat[ 1 ]);
						meshFace.push_back( mat[ 0 ]);
					}
					else
					{
						meshFace.push_back( mat[ 0 ]);
						meshFace.push_back( mat[ 1 ]);
					}			
				}
				int tsfacei = ssface[ facei ];
				int ti = 0;
				if( sfacespaci[ 2*tsfacei ] != spaci ) ti = 1;
				//find the position of the contour edge in the intset
				intset::iterator iter = sfacectrei[ tsfacei ].begin();
				int pos = 0;
				while( iter != sfacectrei[ tsfacei ].end())
				{
					if( *iter == sspctredge_vec[ facei ][ edgei ].ancestor)
						break;
					iter++;
					pos++;
				}
				//resize the registration vector for one face
				if( sfaceregface[ tsfacei ].size() == 0)
					sfaceregface[ tsfacei ].resize( 2*sfacectrei[ tsfacei].size());
				//////////////////////////////////////////////////////////////////////////
				cout<<sfacectrei[ tsfacei].size()<<endl;
				cout<<"the specified position is:"<< 2*pos + ti <<endl;
				//cout<<"t"
				//////////////////////////////////////////////////////////////////////////

				//add the face into the corresponding contour edge
				sfaceregface[ tsfacei ][ 2*pos + ti ].push_back( nfacei );
			}
			vi = NULL;
		}
	}
}
//generate faces by connecting projected edges on sheets and the contour edges
void FaceGenerator::GFaceSheet(
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
	int*& faceside	)
{
	//go through each sheet, and conect the projection with the original contour edge
	int sheeti;
	int keys[ 2];
	//////////////////////////////////////////////////////////////////////////
	//cout<<"for edges on sheets, connect the projected edge and the contour edge!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int tfacei = 0; tfacei < facenum - 1; tfacei ++)
	{
		keys[ 0 ] = tfacei;
		for( int tfacej = tfacei +1;tfacej < facenum; tfacej ++)
		{
			keys[ 1] = tfacej;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );
			if( seamonsheetnum[ sheeti ] == 0 )
				continue;
			if( nsheetReg[ sheeti ].size() == 0  )
				continue;

			//go through each edge
			int edgenum = nsheetReg[ sheeti ].size();
			//////////////////////////////////////////////////////////////////////////			
		//	cout<<"edges exist on sheet: "<<sheeti<<"! edgenum is:"<<edgenum<<"\n";
			//////////////////////////////////////////////////////////////////////////
			for( int i = 0;  i < edgenum; i ++ )
			{
				int ei = nsheetReg[ sheeti ][ i ].posInMeshEdge;
				int evi[ 2 ] = { meshEdge[ 2*ei ], meshEdge[ 2*ei + 1]};
				int crspnum = nsheetReg[ sheeti ][ i ].crspCtrEdges.size();
				for( int j = 0 ; j < crspnum; j ++ )
				{
					int facei = nsheetReg[ sheeti ][ i ].crspCtrEdges[ j ].facei;
					int edgei = nsheetReg[ sheeti ][ i ].crspCtrEdges[ j ].edgei;
					int dir = nsheetReg[ sheeti ][ i ].crspCtrEdges[ j ].samedirec;
					int* cepos = sspctredge_vec[ facei ][ edgei ].veris;
					int cvi[ 2 ];
					for( int k = 0; k < 2 ; k ++)
						cvi[ k ] = cvposinmesh[ facei ][ cepos[ k ]];
					cepos = NULL;
					int rvi[ 2 ];
					if ( dir ) 
					{
						rvi[ 0 ] = evi[ 0 ];
						rvi[ 1 ] = evi[ 1 ];
					}
					else
					{
						rvi[ 0 ] = evi [ 1 ];
						rvi[ 1 ] = evi[ 0 ];	
					}
					int nfacei = meshFace.size()/5;
					int* mat = sspctredge_vec[ facei ][ edgei ].mat;
                    if( rvi[ 0 ] == cvi[ 0 ])	//the first vertice are the same
					{
						meshFace.push_back( cvi[ 0 ]);
						meshFace.push_back( cvi[ 1]);
						meshFace.push_back( rvi[ 1 ]);
						if( faceside[ facei ] == 1 )
						{
							meshFace.push_back( mat[ 1 ]);
							meshFace.push_back( mat[ 0 ]);
						}
						else
						{
							meshFace.push_back( mat[ 0 ]);
							meshFace.push_back( mat[ 1 ]);
						}
					//	meshFace.push_back( mat[ 0 ]);
					//	meshFace.push_back( mat[ 1 ]);
					}
					else if( rvi[ 1 ] == cvi[ 1 ])
					{
						meshFace.push_back( cvi[ 0 ]);
						meshFace.push_back( cvi [1 ]);
						meshFace.push_back( rvi[ 0 ]);
						if( faceside[ facei ] == 1 )
						{
							meshFace.push_back( mat[ 1 ]);
							meshFace.push_back( mat[ 0 ]);
						}
						else
						{
							meshFace.push_back( mat[ 0 ]);
							meshFace.push_back( mat[ 1 ]);
						}
						
					//	meshFace.push_back( mat[ 0 ]);
					//	meshFace.push_back( mat[ 1 ]);
					}
					else
					{
						//two faces
						meshFace.push_back( cvi[ 0 ]);
						meshFace.push_back( cvi[ 1 ]);
						meshFace.push_back( rvi[ 0 ]);
						if( faceside[ facei ] == 1 )
						{
							meshFace.push_back( mat[ 1 ]);
							meshFace.push_back( mat[ 0 ]);
						}
						else
						{
							meshFace.push_back( mat[ 0 ]);
							meshFace.push_back( mat[ 1 ]);
						}
						//meshFace.push_back( mat[ 0 ]);
						//meshFace.push_back( mat[ 1 ]);
						meshFace.push_back( cvi[ 1 ]);
						meshFace.push_back( rvi[ 1 ]);
						meshFace.push_back( rvi[ 0 ]);
						if( faceside[ facei ] == 1 )
						{
							meshFace.push_back( mat[ 1 ]);
							meshFace.push_back( mat[ 0 ]);
						}
						else
						{
							meshFace.push_back( mat[ 0 ]);
							meshFace.push_back( mat[ 1 ]);
						}
					//	meshFace.push_back( mat[ 0 ]);
					//	meshFace.push_back( mat[ 1]);
					}
					// set the registration information
					int tsfacei = ssface[ facei ];
					int ti = 0;
					if( sfacespaci[ 2*tsfacei ] != spaci ) ti = 1;
					//find the position of the contour edge in the intset
					intset::iterator iter = sfacectrei[ tsfacei ].begin();
					int pos = 0;
					while( iter != sfacectrei[ tsfacei ].end())
					{
						if( *iter == sspctredge_vec[ facei ][ edgei ].ancestor)
						{
							//////////////////////////////////////////////////////////////////////////
						/*	cout<<"#ancestor in intset, and # of current contour edge:\n"
								<<*iter <<"\t"<<sspctredge_vec[ facei ][ edgei ].ancestor<<endl;
							*///////////////////////////////////////////////////////////////////////////
							break;
						}
						iter ++;
						pos++;
					}

					//////////////////////////////////////////////////////////////////////////
					/*if( pos == sfacectrei[ tsfacei ].size() )
					{
						cout<<"the contour edge was not found in the set!"<<endl;
						cout<<"ancesotr edge:"<<sspctredge_vec[ facei ][ edgei ].ancestor<<endl;
						cout<<"all possible edge list:"<<endl;
						iter = sfacectrei[ tsfacei ].begin();
						while( iter != sfacectrei[ tsfacei ].end() )
						{
							cout<<*iter<<" ";
							iter ++;
						}
						cout<<endl;						
					}*/
					//////////////////////////////////////////////////////////////////////////

					//resize the registration vector for one face
					if( sfaceregface[ tsfacei ].size() == 0)
						sfaceregface[ tsfacei ].resize( 2*sfacectrei[ tsfacei].size());
					//add the face into the corresponding contour edge
					//sfacectrei[ tsfacei ].insert( nfacei );
					//////////////////////////////////////////////////////////////////////////
				/*	cout<<"size of the vector, position to be set:\n"<<
						sfaceregface[ tsfacei ].size()<<"\t"<<
						2*pos + ti <<endl;*/
					//////////////////////////////////////////////////////////////////////////
					sfaceregface[ tsfacei ][ 2*pos + ti ].push_back( nfacei );
				}
			}
		}
	}
}
//void GFaceInOneSubspace(
//floatvector& meshVer,intvector& meshEdge, intvector& meshFace,
//intvector*& shtVposInMV_arr_ivec,
//intvector*& shtEposInME_arr_ivec,
//floatvector*& shtVpos_arr_fvec,
//intvector*& shtEcmpos_arr_ivec,
//intvector*& shtMat_arr_ivec,
//int*& seamedgenum_iarr,
//sheetRegion*& region_vec,
//	
//vector<SSPCTRVERVEC>& sspctrver_vec,
//vector<SSPCTREDGEVEC>& sspctredge_vec,
//
//int*& njptreg,
//newSeamReg*& nseamReg,
//EdgeReg*& nsheetReg,
//
//int facenum,
//int* ver2jpt,
//int majptnum,
//int maseamnum,
//int masheetnum,
//int*& maseamnum,
//MapArraySR& doubleface2sheet,
//float*& sheettab,
//
//int*& ssverreg,
//vector<intvector>& ssedgereg,
//vector<intvector>& ssfacereg,
//
//
//)
//{
//	//step1. generate all the faces on sheets
//	GFaceOnSheets(meshFace,
//		shtVpos_arr_fvec,shtEcmpos_arr_ivec,region_vec,
//		sheettab, facenum, maseamnum, doubleface2sheet);
//
//	//step2. add all the contour vertices into meshVer, and set the corresponding info
//    int** cvposinmesh;
//	cvposinmesh = new int*[ facenum ];
//	for(int i = 0; i < facenum; i ++ )
//	{
//		cvposinmesh[ i ] = new int[ sspctrver_vec[ i ].size() ];
//	}
//	int* jtp2ver = new int[ majptnum ];
//	for( int i= 0; i < ssvernum)
//	for( int i = 0; i < facenum; i ++)
//	{
//		addCtrVerOneFace(
//			meshVer,sspctrver_vec[ i ],
//			cvposinmesh,
//			jpt2ver,	//from junction to point
//			int* njptreg,		//map from the junction point to the position of it in meshver		
//			int maseamnum,
//			newSeamReg* nseamReg,
//			float*& ssver,
//			int*& ssedge,
//			int ssfaceedgenum,
//			int*& ssface,
//
//			int*& sverreg,	//for the subspace vertex, the corresponding vertex in meshVer
//			vector<intvector>& sedgereg	//for each subspace edge, 
//			//for all the vertices on it, the corresponding vertices on it.
//			)
//	}
//
//	//step3. genreate face by connecting projected edge on seam and contour edge
//
//	//step3. generate face by connecting projected edge on sheet and contour edge
//}

void FaceGenerator::writeSubMeshOut(
		floatvector& meshVer,intvector& meshFace,
		int spaci)

{
	char fname[ 20 ] = "mesh";
	char fnum[ 5 ];
	itoa( spaci + 1, fnum, 10 );
	strcat( fname, fnum );
	strcat( fname, ".txt");

	FILE*  fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	//write
	//vertex
	int vnum = meshVer.size()/3;
	for( int i = 0; i < vnum ; i ++)
	{
		if( i == 0 )
		{
			fprintf( fout, "{{{%f,%f,%f}", meshVer[ 0 ], meshVer[ 1], meshVer[ 2 ]);
		}
		else
		{
			fprintf( fout, "{%f,%f,%f}", meshVer[ 3*i],meshVer[ 3*i + 1], meshVer[ 3*i + 2]);
		}
		if( i != vnum - 1 )
			fprintf( fout, ",");
	}

	//face composition
	int facenum = meshFace.size()/5;
	if( facenum == 0 )
	{
		fprintf( fout, "},{");
	}
	for( int i = 0; i < facenum; i++ )
	{
		if( i == 0 )
		{
			fprintf(fout, "},{{%d,%d,%d}", meshFace[ 0 ]+1, meshFace[ 1 ]+1, meshFace[ 2 ]+1);
		}
		else
			fprintf( fout, "{%d,%d,%d}", meshFace[ 5*i ]+1, meshFace[ 5*i +1 ]+1, meshFace[ 5*i + 2]+1);
		if( i != facenum - 1)
			fprintf( fout, ",");
	}

	//face material
	if( facenum == 0 )
		fprintf( fout, "},{");
	for( int i= 0 ; i < facenum; i ++ )
	{
		if( i == 0 )
			fprintf(fout, "},{{%d,%d}", meshFace[ 3 ] + 1, meshFace [ 4 ] + 1  );
		else
			fprintf( fout, "{%d,%d}", meshFace[  5*i + 3 ] + 1, meshFace[ 5*i + 4 ] + 1);
		if( i != facenum - 1) 
			fprintf( fout, ",");
	}

	fprintf( fout, "}}");
	fclose( fout );
}