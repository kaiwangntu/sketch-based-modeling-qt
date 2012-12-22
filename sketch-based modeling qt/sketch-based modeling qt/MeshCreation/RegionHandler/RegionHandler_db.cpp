#include "../RegionHandler/regionHandler.h"

void regionHandler::writeRegion_db(
						   floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
						   intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
						   //for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
						   //	intvector& shtMat_ivec,
						   float zdir[ 3 ],
						   float xdir[ 3 ],
						   sheetRegion& region,
						   intvector& boundseg
						   )
{
	FILE* fout = fopen("regiondb.txt","w");
	if( fout == NULL )
	{
		cout<<"Unable to open file regiondb.txt to write! "<<endl;
		return;
	}

	//write xdir and zdir out
	fprintf( fout, "{{%f,%f,%f}",xdir[ 0 ], xdir[ 1 ] , xdir[ 2 ]) ;
	fprintf( fout, ",{%f,%f,%f}", zdir[ 0 ], zdir[ 1 ] , zdir[ 2]);

	//write vertex out
	int vnum = shtVpos_fvec.size()/3;
	for( int i = 0; i < vnum; i ++ )
	{
		if( i == 0 )
			fprintf( fout, ",{{%f,%f,%f}", shtVpos_fvec[ 0 ], shtVpos_fvec[ 1 ] , shtVpos_fvec[ 2 ]);
		else
			fprintf( fout,  "{%f,%f,%f}", shtVpos_fvec[ 3*i ], shtVpos_fvec[ 3*i + 1], shtVpos_fvec[ 3*i + 2 ]);
		if( i != vnum - 1 )
			fprintf( fout, ",");
	}

	//write edge out
	int edgenum = shtEcmpos_ivec.size() / 5;
	for(int i = 0; i < edgenum; i ++ )
	{
		if( i == 0 )
		{
			fprintf(fout, "},{{%d,%d}", shtEcmpos_ivec[ i * 5 ] + 1,  shtEcmpos_ivec[ i * 5 + 1] + 1);
		}
		else
			fprintf( fout, "{%d,%d}", shtEcmpos_ivec[ 5*i ] + 1, shtEcmpos_ivec[ 5*i + 1]  + 1) ;
		if( i != edgenum - 1 )
			fprintf(fout ,",");
	}
    
	//write boundaries out
	int bnum = region.boundaries.size();
	//fprintf( fout, "},{");
	for( int i = 0; i < bnum ;i ++ )
	{
		int bsize = region.boundaries[ i ].size();
		if( i == 0 )	
			fprintf(fout, "},{{");
		else
			fprintf( fout, "{");
		for( int j = 0; j < bsize; j ++ )
		{
			fprintf(fout, "%d", region.boundaries[ i ][ j ] + 1 );
			if( j != bsize - 1 )
				fprintf( fout, ",");
		}
		if( i != bnum - 1 )
			fprintf(fout, "},");
		else
			fprintf( fout, "}");
	}

	//write outregion hasn't been set, no, not write out!
	//fprintf( fout, "},%d", region.outregion);

	//write boundary seg out
	int boundsegsize = boundseg.size();
	for( int i = 0; i < boundsegsize; i ++ )
	{
		if( i == 0)
			fprintf( fout, "},{%d", boundseg[ 0 ]);
		else
			fprintf(fout, "%d", boundseg[ i ]);
		if( i != boundsegsize-  1 )
			fprintf( fout, ",");
	}
	fprintf( fout, "}}");

	fclose( fout );
}	

void regionHandler::writeVerEdgeOnOneSheet_db(
		intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,
		FILE* fout )
{
	int vnum = shtVposInMV_ivec.size();
	int edgenum = shtEposInME_ivec.size();

	//output vertex index in mesh
	fprintf( fout,"{{");
	for( int i = 0; i < vnum; i ++ )
	{
		fprintf( fout, "%d", shtVposInMV_ivec[ i ]);
		if( i != vnum - 1 )
			fprintf( fout, ",");
	}
	fprintf(fout, "},{");
	//output edge index in mesh
	for( int i = 0; i < edgenum ; i ++ )
	{
		fprintf(fout, "%d", shtEposInME_ivec[ i ]);
		if( i != edgenum - 1)
			fprintf( fout, ",");
	}
	fprintf( fout, "},{");
	//output the vertex position list
	for( int i = 0; i < vnum; i++ )
	{
		fprintf(fout, "{%f,%f,%f}", shtVpos_fvec[ 3*i ], shtVpos_fvec[ 3*i + 1], shtVpos_fvec[ 3*i + 2]);
		if( i != vnum - 1)
			fprintf( fout, ",");
	}
	fprintf(fout, "},{");
	//output the edge on the sheet
	for( int i = 0; i < edgenum; i++ )
	{
		fprintf( fout, "{%d,%d}", shtEcmpos_ivec[ 5*i ]+1, shtEcmpos_ivec[ 5*i + 1 ]+1);
		if( i != edgenum - 1 )
			fprintf( fout, ",");
	}
	fprintf( fout, "}}");
}
//write vertex, edge on sheets of one subspace
void regionHandler::writeVerEdgeOnSheet_db(
	intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
	intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
	floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
	intvector*& shtEcmpos_arr_ivec,
	int sheetnum,
	int spaci
								   )
{
	char fname[ 20 ] = "veredge";
	char fnum[ 5 ];
	itoa( spaci + 1, fnum, 10 );
	strcat( fname, fnum );
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file: "<<fname<<"to write!"<<endl;
		return;
	}

	fprintf( fout, "{");
	for( int i = 0 ;i < sheetnum; i ++ )
	{
		writeVerEdgeOnOneSheet_db(
			shtVposInMV_arr_ivec[ i ],	shtEposInME_arr_ivec[ i ],
			shtVpos_arr_fvec[ i ],shtEcmpos_arr_ivec[ i ], fout );
		if( i != sheetnum - 1)
			fprintf( fout, ",");
	}
	fprintf( fout, "}");
	fclose( fout );
}

//write one sheet of subspace
void regionHandler::writeVerEdgeRegionOneSht_db(
		intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
		intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
		sheetRegion& region,							 
		int seamedgenum,						//the number of edges on seam in the edge list
		FILE* fout 
							  )
{
	int vnum = shtVposInMV_ivec.size();
	int edgenum = shtEposInME_ivec.size() * 2;

	//output vertex index in mesh
	fprintf( fout,"{{");
	for( int i = 0; i < vnum; i ++ )
	{
		fprintf( fout, "%d", shtVposInMV_ivec[ i ] + 1);
		if( i != vnum - 1 )
			fprintf( fout, ",");
	}
	fprintf(fout, "},{");
	//output edge index in mesh
	for( int i = 0; i < edgenum/2 ; i ++ )
	{
		fprintf(fout, "%d", shtEposInME_ivec[ i ] + 1);
		if( i != edgenum/2 - 1)
			fprintf( fout, ",");
	}
	fprintf( fout, "},{");
	//output the vertex position list
	for( int i = 0; i < vnum; i++ )
	{
		fprintf(fout, "{%f,%f,%f}", shtVpos_fvec[ 3*i ], shtVpos_fvec[ 3*i + 1], shtVpos_fvec[ 3*i + 2]);
		if( i != vnum - 1)
			fprintf( fout, ",");
	}
	fprintf(fout, "},{");
	//output the edge on the sheet
	for( int i = 0; i < edgenum; i++ )
	{
		fprintf( fout, "{%d,%d}", shtEcmpos_ivec[ 5*i ]+1, shtEcmpos_ivec[ 5*i + 1 ]+1);
		if( i != edgenum - 1 )
			fprintf( fout, ",");
	}
	
	//write the number of the seam edge in the edge list
	fprintf( fout, "},%d,{{", seamedgenum);
	
	//write region of the sheet
	//boundary list
	//fprintf( fout, "{");	
	for(unsigned int i = 0; i < region.boundaries.size(); i ++ )
	{
		fprintf( fout , "{");
		for(unsigned int j= 0; j < region.boundaries[ i ].size(); j ++ )
		{
			fprintf( fout, "%d", region.boundaries[ i ][ j ] + 1);
			if( j != region.boundaries[ i ].size() - 1)
				fprintf(fout, ",");
		}
		fprintf(fout, "}");
		if( i != region.boundaries.size() -1 )
			fprintf( fout , ",");
	}

	//region list
	fprintf( fout, "},{");
	for(unsigned int i = 0; i < region.regions.size(); i ++ )
	{
		fprintf( fout, "{");
		for(unsigned int j = 0; j < region.regions[ i ].size(); j ++ )
		{
			fprintf( fout, "%d", region.regions[ i ][ j ] + 1);
			if( j != region.regions[ i ].size() - 1 )
                fprintf( fout, ",");
		}
		fprintf( fout, "}");
		if( i != region.regions.size() - 1)
			fprintf(fout, ",");
	}
	//outregion
	fprintf( fout, "},%d,{", region.outregion + 1);

	//region neighbors
	for(unsigned int i = 0; i < region.regionneighbrs.size(); i ++ )
	{
		fprintf( fout, "{");
		intset::iterator iter = region.regionneighbrs[ i ].begin() ;
		for(unsigned int j = 0; j < region.regionneighbrs[ i ].size(); j ++ )
		{			
			fprintf( fout, "%d", *iter + 1);
			iter ++;
			if( j != region.regionneighbrs[ i ].size() -1  )
				fprintf( fout, ",");
		}
		fprintf( fout, "}");
		if( i != region.regionneighbrs.size() - 1 )
			fprintf( fout, ",");
	}

	fprintf( fout, "},{");
	//mat of regions
    for(unsigned int i = 0; i < region.mat.size(); i += 2 )
	{
		fprintf( fout, "{%d,%d}", region.mat[i ] + 1, region.mat[ i + 1]  + 1);
		if( i!= region.mat.size() - 2 )
			fprintf( fout, ",");
	}
//	fprintf( fout, )
	fprintf( fout, "}}}");	//end of material, end of region, end of one sheet
}

//write vertex edge, and region on sheets of one subspace
void regionHandler::writeVerEdgeRegion_db(
		int spaci,
		int sheetnum,
		intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
		intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
		int*& seamedgenum_iarr,						//the number of edges on seam in the edge list
		sheetRegion*& region_vec
						)
{
	char fname[ 1024 ] = "mmdebug/onesubspace/veredgeregion";
	char fnum[ 5 ];
	itoa( spaci + 1, fnum, 10 );
	strcat( fname, fnum );
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file: "<<fname<<"to write!"<<endl;
		return;
	}

	fprintf( fout, "{");
	for( int i = 0 ;i < sheetnum; i ++ )
	{
		writeVerEdgeRegionOneSht_db(
			shtVposInMV_arr_ivec[ i ],	shtEposInME_arr_ivec[ i ],
			shtVpos_arr_fvec[ i ],shtEcmpos_arr_ivec[ i ], 
			region_vec[ i ],			seamedgenum_iarr[ i ], fout );
		if( i != sheetnum - 1)
			fprintf( fout, ",");
	}
	fprintf( fout, "}");
	fclose( fout );
}

void regionHandler::writeMat_db(
		  int spaci,
		  	int sheetnum,
		intvector*& shtMat_arr_ivec			  )
{
	char fname[ 30 ] = "mat";
	char fnum[ 5 ];
	itoa( spaci+ 1, fnum, 10 );
	strcat( fname, fnum );
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");

	fprintf( fout, "{");

	for( int i = 0; i < sheetnum; i ++ )
	{
		int size = shtMat_arr_ivec[ i ].size();
		fprintf( fout, "{");
		for( int j = 0; j < size; j += 4 )
		{
			fprintf( fout, "{{%d,%d},{%d,%d}}", shtMat_arr_ivec[ i ][ j ], shtMat_arr_ivec[ i ][ j +1 ],
				shtMat_arr_ivec[ i ][ j + 2], shtMat_arr_ivec[ i ][ j + 3]);
			if( j != size- 4)
				fprintf( fout, ",");
		}
		fprintf( fout, "}");
		if( i != sheetnum - 1)
			fprintf( fout, ",");
	}

	fprintf( fout, "}");
	fclose( fout );
}

void regionHandler::writeVerEdgeDirOneSheet(
	floatvector& shtver,
	intvector& shtedge,
	float dir[ 3 ],
	int spaci,
	int sheeti
	)
{
	char fname[ 1024 ] = "mmdebug/onesubspace/sheetve\0";
	char tnum[ 5 ];
	itoa( spaci, tnum, 10 );
	strcat( fname, tnum );
	strcat( fname, "_");
	itoa( sheeti, tnum, 10) ;
    strcat( fname, tnum );
	strcat(fname, ".txt");

	FILE* fout = fopen( fname, "w");

	if( fout == NULL )
	{
		cout<<"Unable to write out to the file :"<<fname<<"! Exit!"<<endl;
		return;
	}

	//dir
	fprintf( fout, "{{%f,%f,%f},", dir[ 0 ], dir[ 1 ] , dir[ 2 ]);

	//ver
	int vnum = shtver.size()/3; 
	for( int i = 0; i < vnum; i ++ )
	{
		if ( i == 0 )
		{
			fprintf( fout, "{{%f,%f,%f}" ,shtver[ 0 ], shtver[ 1 ], shtver[ 2 ]);
			//continue;
		}
		else
		{
			fprintf( fout, "{%f,%f,%f}", shtver[ i* 3 ] , shtver[ i *3 + 1], shtver[ i * 3 + 2 ]);
		}
		if( i != vnum - 1)
			fprintf( fout, ",");
	}

	int tenum = shtedge.size()/5;
	for( int i = 0; i < tenum; i ++ )
	{
		if( i == 0 )
		{
			fprintf(fout, "},{{%d,%d}", shtedge[ 0 ] + 1, shtedge[ 1 ] + 1);
		}
		else
			fprintf( fout, "{%d,%d}", shtedge[ i*5 ] + 1, shtedge[ i*5 + 1] + 1);
		if( i != tenum - 1 )
			fprintf(fout, ",");			
	}
	fprintf( fout, "}}");
	fclose( fout  );
}