#include "../RegionHandler/regionHandler.h"

void regionHandler::setMatOnOneSheet(
	intvector& shtEcmpos_ivec,
	intvector& shtMat_ivec,
	sheetRegion& region
					  )
{
	//set the region material from the edge
	int regnum = region.regions.size();
	region.mat.resize( 2*regnum, -1 );
	//////////////////////////////////////////////////////////////////////////
	/*for( int i = 0; i < 2*regnum; i ++ )
	{
		cout<<region.mat[ i ]<<" ";
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////
	bool upmatexist = false;
	bool downmatexist = false;
	
	//int edgenum = shtEcmpos_ivec.size()/10;
	int edgenum = shtMat_ivec.size()/4;
	for( int i = 0; i < edgenum; i ++ )
	{
		int twei = shtEcmpos_ivec[ 5*i + 2 ];
		int lrreg[ 2 ] = { shtEcmpos_ivec[ 5*i + 4], shtEcmpos_ivec[ 5* twei + 4 ]};
        
		//////////////////////////////////////////////////////////////////////////
		//cout<<"for facei, the left and right materials are:"<< shtMat_ivec[ 4*i ]<< shtMat_ivec[ 4*i + 1]<<endl;
		//cout<<"for facej, the left and right materials are:"<< shtMat_ivec[ 4*i + 2 ]<< shtMat_ivec[ 4*i + 3 ]
		//<<endl;
		////////////////////////////////////////////////////////////////////////////
        //set left region
		//up facej
		int mati = shtMat_ivec[ 4*i + 2 ];
		if( mati != -1 )
		{
		//	upregi = lrreg[ 0 ];	//the left region
			upmatexist = true;
			region.mat[ 2*lrreg[ 0 ] ] = mati;
		}
		//down facei
		mati =  shtMat_ivec[ 4*i ];
		if( mati != -1 )
		{
		//	downregi = lrreg[ 0 ];
			downmatexist = true;
			region.mat[ 2*lrreg[ 0 ] + 1 ] = mati;
		}

		//set right region
		//up facej
		mati = shtMat_ivec[ 4*i + 3 ];
		if( mati != -1 )
		{
			upmatexist = true;
			//upregi = lrreg[ 1 ];
			region.mat[ 2*lrreg[ 1 ]] = mati;
		}
		//down facei
		mati = shtMat_ivec[ 4*i + 1 ];
		if( mati != -1 )
		{
			downmatexist = true;
//			downregi = lrreg[ 1 ]; 
			region.mat[ 2*lrreg[ 1 ] + 1 ] = mati;
		}

	}

	//broadcast the material from one region to the next one
	if( !upmatexist && !downmatexist )	//no region has been set
		return;

	//for the upside material, broadcast from known to unknown
	//int* regmark = new int[ regnum ];
	//for( int i = 0; i < regnum; i++ )
	//	regmark[ i ] = 0;	//not visited
	stack<int> regstack;
	
	//broadcast from every region as far as possible
	if( upmatexist )
	{
		for( int i = 0; i < regnum; i ++ )
		{
			if( i == region.outregion )continue;

			int mati = region.mat[2 * i ];
			if( mati == -1 )
				continue;

			regstack.push( i );
			while( !regstack.empty() )
			{
				int curreg = regstack.top();
				regstack.pop();

				intset::iterator iter = region.regionneighbrs[ curreg ].begin();
				for(unsigned int j = 0; j < region.regionneighbrs[ curreg ].size(); j ++ )
				{
					int tregi = *iter;
					if( tregi == region.outregion )
					{
						iter++;
						continue;
					}
					if( region.mat[ 2*tregi ] == -1 )
					{
						region.mat[ 2*tregi ] = mati;
						regstack.push( tregi );
					}
					iter++;
				}
			}
		}
	}

	if( downmatexist )
	{
		for( int i = 0; i < regnum; i ++ )
		{
			if( i == region.outregion )
				continue;

			int mati = region.mat[2 * i + 1];
			if( mati == -1 )
				continue;

			regstack.push( i );
			while( !regstack.empty() )
			{
				int curreg = regstack.top();
				regstack.pop();
				intset::iterator iter = region.regionneighbrs[ curreg ].begin();
				for(unsigned int j = 0; j < region.regionneighbrs[ curreg ].size(); j ++ )
				{
					int tregi = *iter;
					if( tregi == region.outregion )
					{
						iter ++;
						continue;
					}
					if( region.mat[ 2*tregi + 1] == -1 )
					{
						region.mat[ 2*tregi + 1] = mati;
						regstack.push( tregi );
					}
					iter ++;
				}
			}
		}
	}
}

//void gatherSeamEdge(
//	intvector*& shtEposInME_arr_ivec,
//	int*& seamedgenum_iarr,
//	intset& seamedgeset,
//	int sheetnum
//					)
//{
//	for( int i = 0; i < sheetnum; i ++ )
//	{
//		for( int j = 0; j < seamedgenum_iarr[ i ]; j ++ )
//		{
//			seamedgeset.insert( shtEposInME_arr_ivec[ i ][ j ]);
//		}
//	}
//}

//pay attention: facei < facej
void regionHandler::insertNewSeamEdgeInfo(
	vector<intvector>& seamedgeinfo,
	int facei,int facej,int mati,int matj)
{
	int pos[ 2] ={ -1, -1 };
	int len1 = seamedgeinfo.size();
	for( int i = 0; i < len1; i ++ )
	{
		if( seamedgeinfo[ i ][ 0 ] == facei )
			pos[ 0 ] = i;
		else if( seamedgeinfo[ i ][ 0 ] ==facej )
			pos[ 1 ] = i;
	}

	if( (pos[ 0 ] == -1) && (pos[ 1 ] == -1) )
		seamedgeinfo.resize( len1 + 2 );
	else if( pos[ 0 ] == -1 )
		seamedgeinfo.resize( len1 + 1);
	else if( pos[ 1] == -1 )
		seamedgeinfo.resize( len1 + 1);

	//put the new ones into the seamedge info
	if( pos[ 0 ] == -1 )
	{
		seamedgeinfo[ len1 ].resize( 4 );
		seamedgeinfo[ len1 ][ 0 ] = facei;
		seamedgeinfo[ len1 ][ 1 ] = facej;
		seamedgeinfo[ len1 ][ 2 ] = 1;	//mati is the second material of the sheet
		seamedgeinfo[ len1 ][ 3 ] = mati;
	}
	else
	{
		int osize = seamedgeinfo[ pos[ 0 ]].size();
		seamedgeinfo[ pos[ 0 ] ].resize( osize + 3 );
		seamedgeinfo[ pos[ 0 ] ][ osize ] = facej;
		seamedgeinfo[ pos[ 0 ] ][ osize + 1 ] = 1;
		seamedgeinfo[ pos[ 0 ] ][ osize + 2 ] = mati;
	}

	if( pos[ 1 ] == -1 )
	{
		if( pos[ 0 ] == -1 )
			pos[ 1 ] = len1 + 1;
		else
			pos[ 1 ] = len1;
		seamedgeinfo[ pos[ 1 ] ].resize( 4 );
		seamedgeinfo[ pos[ 1 ]][ 0 ] = facej;
		seamedgeinfo[ pos[ 1 ]][ 1 ] = facei;
		seamedgeinfo[ pos[ 1 ]][ 2 ] = 0;	//matj is the first(upside) material of the sheet
		seamedgeinfo[ pos[ 1 ]][ 3 ] = matj;
	}
	else
	{
		int osize = seamedgeinfo[ pos[ 1 ]].size();
		seamedgeinfo[ pos[ 1 ]].resize( osize + 3);
		seamedgeinfo[ pos[ 1 ]][ osize ] = facei;
		seamedgeinfo[ pos[ 1 ]][ osize + 1] = 0;
		seamedgeinfo[ pos[ 1 ]][ osize + 2 ] = matj;
	}
}
void regionHandler::broadAcrossSeam(
	intvector*& shtEposInME_arr_ivec,
	intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
	int*& seamedgenum_iarr,
	sheetRegion*& region_vec,
//	intset& seamedgeset,
	int sheetnum,
	newSeamReg*& nseamReg,
	MapArraySR& doubleface2sheet,
	int*& seamonsheetnum,
	int facenum
		)
{
	//////////////////////////////////////////////////////////////////////////
	//cout<<"0 safe!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	//step1. gather all the subedges on seam and set map from edge - index in the list
	intset seamedgeset;
	for( int i = 0; i < sheetnum; i ++ )
	{
		for( int j = 0; j < seamedgenum_iarr[ i ]; j ++ )
		{
			//////////////////////////////////////////////////////////////////////////
			//cout<<shtEposInME_arr_ivec[ i ][ j ]<<" ";
			//////////////////////////////////////////////////////////////////////////
			seamedgeset.insert( shtEposInME_arr_ivec[ i ][ j ]);
			//////////////////////////////////////////////////////////////////////////
			//cout<<"OK. ";
			//////////////////////////////////////////////////////////////////////////
		}
	}
	//////////////////////////////////////////////////////////////////////////
	//cout<<"1 safe!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	//map from the seam edge index in mesh edge to the index in seamedgesset
	map<int, int> seamedge2ind;	
	int seamedgenum = seamedgeset.size();
	intset::iterator iter = seamedgeset.begin();
	for( int i = 0; i < seamedgenum; i ++ )
	{
		seamedge2ind[ *iter ] = i;
		iter++;
	}
	vector<intvector>* seamedgeinfo = new vector<intvector>[ seamedgenum ];
	//////////////////////////////////////////////////////////////////////////
	//cout<<"2 safe!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	//step2. go through each sheet, go through each seam on it
	//for each seam, generate the list as {facei, {facej, matforfacei, 0/1 }* }* 
	int keys[ 2 ];
	int sheeti;
	for( int facei = 0; facei < facenum - 1; facei ++ )
	{
		keys[ 0 ] = facei;
		for( int facej = facei + 1; facej < facenum; facej ++ )
		{
			keys[ 1 ] = facej;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );

			if( seamonsheetnum[ sheeti ] == 0 )
				continue;

            int tenum = seamedgenum_iarr[ sheeti ];
			int outri = region_vec[ sheeti ].outregion;
			for( int i = 0; i < tenum; i ++ )
			{
				int tei = shtEposInME_arr_ivec[ sheeti ][ i ];
				int tri = shtEcmpos_arr_ivec[ sheeti ][ 5*i + 4 ];
				if( tri == outri )
				{
					int twei = shtEcmpos_arr_ivec[ sheeti ][ 5*i + 2 ];
					tri = shtEcmpos_arr_ivec[ sheeti ][ 5*twei + 4 ];
				}
                
				//the index of the seam subedge in all the seam edge list
				int ti = seamedge2ind[ tei ];
                //insert facei -> facej, matforfacei, index_of_facei into seamedgeinfo
				//insert facej -> facei, matforfacej, index_of_Facej into seamedgeinfo
				//
				int mati = region_vec[ sheeti ].mat[ 2*tri + 1 ];
				int matj = region_vec[ sheeti ].mat[ 2*tri ];
				insertNewSeamEdgeInfo(seamedgeinfo[ ti ],facei,facej, mati,matj);
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"seamedgeinfo:"<<endl;
	for( int i= 0; i < seamedgenum; i ++ )
	{
		cout<<"seamedge "<<i<<endl;
		for( int j = 0; j < seamedgeinfo[ i ].size(); j ++ )
		{
			for( int k= 0; k < seamedgeinfo[ i ][ j ].size(); k++)
			{
				cout<<seamedgeinfo[ i ][ j ][ k ]<<" ";
			}
			cout<<endl;
		}
		cout<<"----------"<<endl;
	}*/
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
//	cout<<"Start broadcasting....."<<endl;
//	cout<<"seam edge number:"<<seamedgenum<<endl;
	//////////////////////////////////////////////////////////////////////////
	//step3. broadcast cross the seam
	//for each seam edge, broadcast as far as possible
	for( int i = 0; i < seamedgenum; i ++ )
	{
		//////////////////////////////////////////////////////////////////////////
	//	cout<<"broadcast from the "<<i<<"th edge!"<<endl;
		//////////////////////////////////////////////////////////////////////////
		stack<int> sestack;
		sestack.push( i );
		while( !sestack.empty())
		{
			int cureind = sestack.top();
			sestack.pop();

			//go through this subedge to check if it is able to broadcast
			int tlen = seamedgeinfo[ cureind ].size();
			if( tlen == 2 )	//able to broadcast from one side to the other side, actually it lies on the edge of subspace
			{
#ifdef debug
				if( (seamedgeinfo[ cureind ][ 0 ].size() !=  4) || ( seamedgeinfo[ cureind ][ 1 ].size() != 4 ) )
				{
					cout<<"Error! Info of seam edge(on edge of subspace) has two items, but size is not 4!"<<endl;
					continue;
				}
#endif				
				int tmat[ 2 ] = {seamedgeinfo[ cureind ][ 0 ][ 3 ], seamedgeinfo[ cureind ][ 1 ][ 3 ]};
				//both unknown or both known, no need to process
				if( ((tmat[ 0 ] == -1) && (tmat[ 1 ] == -1 )) ||  ((tmat[ 0 ] != -1) && (tmat[ 1 ] != -1 )))
					continue;

				//broadcast from one side to the other side
				int refsheeti;	//refresh sheet index
				int reffacei;		//refresh facei
				int refind;		//refresh index of the face in the two faces of the sheet
				int refmat;		//the mat value to set for the refface
				keys[ 0 ] = seamedgeinfo[ cureind ][ 0 ][ 0 ];
				keys[ 1 ] = seamedgeinfo[ cureind ][ 1 ][ 0 ];
				int nreffacei;	//not refreshed facei
				doubleface2sheet.getKeyVal( keys, 2, false, refsheeti );
				if( tmat[ 0 ] == -1)
				{
					reffacei = keys[ 0 ];
					refind = seamedgeinfo[ cureind ][ 0 ][ 2 ];
					seamedgeinfo[ cureind ][ 0 ][ 3 ] = refmat = tmat[ 1 ];
					nreffacei = keys[ 1];
				}
				else
				{
					reffacei = keys[ 1 ];
					refind = seamedgeinfo[ cureind ][ 1 ][ 2 ];
					seamedgeinfo[ cureind ][ 1 ][ 3 ] = refmat = tmat[ 0 ];
					nreffacei = keys[ 0 ] ;
				}
				//refresh the reffacei and push the seamedges on it to the stack
				//refresh
				int matlen = region_vec[ refsheeti ].mat.size();
				for( int j = refind; j < matlen ; j += 2)
					region_vec[ refsheeti ].mat[ j ] = refmat;
				//push stack
                int selen = seamedgenum_iarr[ refsheeti ];
				for ( int j = 0; j < selen; j ++ )
				{
					int mei = shtEposInME_arr_ivec[ refsheeti ][ j ];
					int ind = seamedge2ind[ mei ];
					//refresh seamedgeinfo
					int tlen1 = seamedgeinfo[ ind ].size();
					for( int k1 = 0; k1 < tlen1; k1 ++ )
					{
						if( seamedgeinfo[ ind ][ k1 ][ 0 ] != reffacei )
							continue ;
						
                        int tlen2 = (seamedgeinfo[ ind ][ k1 ].size() - 1)/3;
						for( int k2 = 0; k2 < tlen2; k2 ++ )
						{
							//seamedgeinfo[ ind ][ k1 ][ k2 ] = refmat;
							if( seamedgeinfo[ ind ][ k1 ][ 3*k2 + 1] == nreffacei )
							{
								seamedgeinfo[ ind ][ k1 ][ 3*k2 + 3 ] = refmat;
								break;
							}
						}
						break;
					}
					//push it to stack
					sestack.push( ind );
				}
				continue;
			}
			//the seam edge is not on edge of subspace
			for( int j = 0; j < tlen; j ++ )
			{
				int tmat[ 2 ] = { seamedgeinfo[ cureind ][ j ][ 3 ], seamedgeinfo[ cureind ][ j ][ 6 ] };
				//no need to refresh
				if ( ( (tmat[ 0 ] == -1) && (tmat[ 1 ] == -1 ) )||( (tmat[ 0 ] != -1) && (tmat[ 1 ] != -1 ) ))
					continue;
				int refsheeti;
				int reffacei;
			//	int reffacei = keys[ 0 ];
				int refind;
				int refmati;
				int keys[ 2 ] = {seamedgeinfo[ cureind ][ j ][ 0 ], -1 };
			//	int keys[ 2 ] = { seamedgeinfo[ cureind ][ j ][ 1 ], seamedgeinfo[ cureind ][ j ][ 4 ]};
			//	doubleface2sheet.getKeyVal( keys, 2, false, refsheeti);
				if( tmat[ 0 ] == -1 )
				{
					reffacei = keys[ 1 ] = seamedgeinfo[ cureind ][ j ][ 1 ];
					doubleface2sheet.getKeyVal( keys, 2, false, refsheeti);
					refind = seamedgeinfo[ cureind ][ j ][ 2 ];
					seamedgeinfo[ cureind ][ j ][ 3 ] = refmati = tmat[ 1 ];
				}
				else
				{
					reffacei = keys[ 1 ] = seamedgeinfo[ cureind ][ j ][ 4 ];
					doubleface2sheet.getKeyVal( keys, 2, false, refsheeti);
					refind = seamedgeinfo[ cureind ][ j ][ 5 ];
					 seamedgeinfo[ cureind ][ j ][ 6 ] = refmati = tmat[ 0 ];
				}

				//refresh
				for(unsigned int k1 = refind; k1 < region_vec[ refsheeti ].mat.size(); k1 += 2 )
				{
					region_vec[ refsheeti ].mat[ k1 ] = refmati;
				}

				//refresh seam edge info, and push the renewed ones into stack
				for( int k1 = 0; k1 < seamedgenum_iarr[ refsheeti ]; k1 ++ )
				{
					int mei = shtEposInME_arr_ivec[ refsheeti ][ k1 ];
					int indi = seamedge2ind[ mei ];
					
					int tlen1 = seamedgeinfo[ indi ].size();
					for( int k2 = 0;  k2 < tlen1; k2 ++ )
					{
					//	if( seamedgeinfo[ indi ][ k2 ][ 0 ]  != reffacei )
						if( seamedgeinfo[ indi ][ k2 ][ 0 ] != keys[ 0 ])
							continue;

						int tlen2 = (seamedgeinfo[ indi ][ k2 ].size() - 1)/3;
						for( int k3 = 0; k3 < tlen2 ; k3 ++ )
						{
							if(	seamedgeinfo[ indi ][ k2 ][ k3*3 + 1 ] == reffacei)
							{
								seamedgeinfo[ indi ][ k2 ][ k3*3 + 3 ] = refmati;
								break;
							}
						}
						break;

						/*int tlen2 = seamedgeinfo[ indi ][ k2 ].size();
						for( int k3 = 3; k3 < tlen2 ; k3 += 3 )
						{
							seamedgeinfo[ indi ][ k2 ][ k3 ] = refmati;
						}
						break;*/
					}

					//push the new edge into stack
					sestack.push( indi );
				}
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"seamedgeinfo:"<<endl;
	for( int i= 0; i < seamedgenum; i ++ )
	{
		cout<<"seamedge "<<i<<endl;
		for( int j = 0; j < seamedgeinfo[ i ].size(); j ++ )
		{
			for( int k= 0; k < seamedgeinfo[ i ][ j ].size(); k++)
			{
				cout<<seamedgeinfo[ i ][ j ][ k ]<<" ";
			}
			cout<<endl;
		}
		cout<<"----------"<<endl;
	}*/
	//////////////////////////////////////////////////////////////////////////
	//clear temp var
	for( int i = 0; i <seamedgenum; i ++ )
	{
		for(unsigned int j = 0; j < seamedgeinfo[ i ].size(); j ++ )
			seamedgeinfo[ i ][ j ].clear();
		seamedgeinfo[ i ].clear();
	}
	delete []seamedgeinfo;

	seamedgeset.clear();

	seamedge2ind.clear();

}