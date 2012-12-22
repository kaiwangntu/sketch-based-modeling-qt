#include "../RegionHandler/regionHandler.h"


//amplify the mesh by adding junction points into mesh vertex list
//change some of the mesh edge, whose one end point still set as -1
void regionHandler::amplifyMesh(
				 //mesh
				 floatvector& meshVer, intvector& meshEdge,
				 //vertex registration
				 int*& ajptReg,	//new junction point registration with all of them added into Meshver
				 int majptnum,
				 float*& majpt,
				 //edge registration
				 newSeamReg*& nseamReg, //new seam registration info
				 int maseamnum,
				 int*& maseam
				 )
{
	//step1. amplify the mesh first
	//add junction point and set the crsp mesh vertex index
	for( int i = 0; i < majptnum; i ++ )
	{
		if ( ajptReg[ i ] == -1 )	//not added yet
		{
			meshVer.push_back( majpt[ 3*i ]);
			meshVer.push_back( majpt[ 3*i + 1 ]);
			meshVer.push_back( majpt[3*i + 2]);
			ajptReg[ i ] = meshVer.size()/3 - 1;
		}
	}
	//go through each seam
	//cout<<"ind: #mesh edge #size"<<endl;
	for( int  i = 0; i < maseamnum; i ++ )
	{
		//two corresponding vertex in mesh vertex for the two endpoints of the seam
		int veri[ 2 ] = { 0, nseamReg[ i ].vernum - 1 };
		//		int posInMeshVer[ 2 ] = { nseamReg[ i ].verPosInMeshVer[ 0 ],
		//			nseamReg[ i ].verPosInMeshVer[ nseamReg[ i ].vernum - 1 ]};
		for( int j = 0; j < 2; j ++ )
		{
			int posInMeshVer = nseamReg[ i ].verPosInMeshVer[ veri[ j ]];
			if( posInMeshVer == -1 )
			{
				int meshvi =  ajptReg[ maseam[ 2*i + j ]];
				//set the verposinmesh for current junction point
				nseamReg[ i ].verPosInMeshVer[ veri[ j ]]  = meshvi;
				//refresh the vertex of the subedge in meshEdge
				int posInMeshEdge;
				if( j == 0 )
					posInMeshEdge = nseamReg[ i ].edgelist[  veri[ j ] ].posInMeshEdge;
				else
					posInMeshEdge = nseamReg[ i ].edgelist[  veri[ j ] - 1 ].posInMeshEdge;

				//////////////////////////////////////////////////////////////////////////
				//cout<<posInMeshEdge<<"\t"<<meshEdge.size()/2<<endl;
				//////////////////////////////////////////////////////////////////////////
				meshEdge[ 2*posInMeshEdge + j ] = meshvi;
			}
		}
	}
}



void regionHandler::setEdgeMat(
			  vector<SSPCTREDGEVEC>& sspctredge_vec,
			   EdgeRegItem& subedgereg,
			   bool dir[ 2 ],
			   int facei,
			   int facej,
			   int mat[ 4 ]
			   )
{
	//////////////////////////////////////////////////////////////////////////
//	cout<<"begin mat:"<<mat[ 0 ]<<" "<<mat[ 1 ]<<" "<<mat[2 ]<<" "<<mat[ 3 ]<<endl;
	//////////////////////////////////////////////////////////////////////////
	//mat[ 4 ] = { -1, -1, -1, -1};
	int twofaces[ 2 ] = {facei, facej };
	for(unsigned int k = 0; k < subedgereg.crspCtrEdges.size(); k ++ )
	{
		//facei is in corresponding edge list or not?
		int ctrfacei = subedgereg.crspCtrEdges[ k ].facei;
		int whichface = -1;
		if( ctrfacei == facei )
		{
			//////////////////////////////////////////////////////////////////////////
		//	cout<<"facei:"<<facei<<endl;
			//////////////////////////////////////////////////////////////////////////

			whichface = 0;
		}
		else if( ctrfacei == facej )
		{
			//////////////////////////////////////////////////////////////////////////
		//	cout<<"facej:"<<facej<<endl;
			//////////////////////////////////////////////////////////////////////////
			whichface = 1;
		}
		if( whichface == -1 )continue;
		
		int samdirec = subedgereg.crspCtrEdges[ k ].samedirec;
		if( samdirec )
			samdirec = dir[ whichface ];
		else
			samdirec = !dir[ whichface ];
		//samdirec = samdirec && dir[ whichface ];
		int ei = subedgereg.crspCtrEdges[ k ].edgei;
		if( samdirec )
		{
			mat[ 2*whichface ] = sspctredge_vec[ ctrfacei][ ei ].mat[ 0 ];
			mat[ 2*whichface + 1 ] = sspctredge_vec[ ctrfacei][ ei ].mat[ 1 ];
		}
		else
		{
			mat[ 2*whichface ] = sspctredge_vec[ ctrfacei][ ei ].mat[ 1 ];
			mat[ 2*whichface + 1 ] = sspctredge_vec[ ctrfacei][ ei ].mat[ 0 ];
		}
	}
	//////////////////////////////////////////////////////////////////////////
//	cout<<"mat:"<<mat[ 0 ]<<" "<<mat[ 1 ]<<" "<<mat[2 ]<<" "<<mat[ 3 ]<<endl;
	//////////////////////////////////////////////////////////////////////////
}
//get all the vertices and edges on one sheet
void regionHandler::getVerEdgeOnOneSheet(
	 intvector& shtVposInMV_ivec,		//each of them is the corresponding vertex index in mesh vertices
	intvector& shtEposInME_ivec,		//each of them is the corresopnding the edge index in mesh edges
	floatvector& shtVpos_fvec,		//each of them has the position of the vertices on one sheet
	intvector& shtEcmpos_ivec,		//each of them has a list of edges in it.
	//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
	intvector& shtMat_ivec,		//each of them has a list of material in it, four number corresponds to one edge 
	//left mat, right mat above, left mat, right mat, below
	int& seamedgenum,				//the number of edges on seam in the edge list
	floatvector& meshVer,		//mesh
	intvector& meshEdge,
	newSeamReg* nseamReg,	//seam reg
	EdgeReg& nsheetReg,		//sheet reg
	int*& seamonsheet,
	int seamonsheetnum,
	int*& faceside,
	int facei	,			//facei < facej
	int facej,
	//vector<SSPCTRVERVEC> sspctrver_vec,
	vector<SSPCTREDGEVEC> sspctredge_vec
								 )
{	
	bool dir[ 2 ];
	if( faceside[ facei ] == 1)	// for the edge on the sheet, if the material configuration for facei should be changed or not
		dir[ 0 ] = true;
	else
		dir[ 0 ] = false;
	if( faceside[ facej ] == 0 )
		dir[ 1 ] = true;
	else
		dir[ 1 ] = false;
	
	map<int, int> posinmv_posinshtv_map;
	int posinmv[ 2 ];
	int posinshtv[ 2 ];
	int posinme;
	//for each seam on this sheet, add all the subedges on seam into vertex and edge
    for( int i = 0; i < seamonsheetnum; i++ )
	{
		int seami = seamonsheet[ i ];
		for( int j = 0;  j < nseamReg[ seami ].vernum - 1; j ++ )
		{
			posinme = nseamReg[ seami ].edgelist[ j ].posInMeshEdge;
			posinmv[ 0 ] = meshEdge[ 2 * posinme ];
			posinmv[ 1 ] = meshEdge[ 2 * posinme + 1 ];
			//vertex
			for( int k = 0; k < 2; k ++ )
			{
				//the vertex has not been added yet
				if( posinmv_posinshtv_map.find( posinmv[ k ]) == posinmv_posinshtv_map.end() )
				{
					//add into vposinmv
					shtVposInMV_ivec.push_back( posinmv[ k ]);
					//add into vpos
					shtVpos_fvec.push_back( meshVer[ 3*posinmv[ k ]]);
					shtVpos_fvec.push_back( meshVer[ 3*posinmv[ k ] + 1]);
					shtVpos_fvec.push_back( meshVer[ 3*posinmv[ k ] + 2]);
					//set the map info
					posinshtv[ k ] = posinmv_posinshtv_map[ posinmv[ k ]] = shtVposInMV_ivec.size() - 1;
				}
				else
				{
					posinshtv[ k ] = posinmv_posinshtv_map[ posinmv[ k ]];
				}
			}
			
			//edge
			shtEposInME_ivec.push_back( posinme );
			shtEcmpos_ivec.push_back( posinshtv[ 0 ]);
			shtEcmpos_ivec.push_back( posinshtv[ 1 ]);
			shtEcmpos_ivec.push_back ( -1 );
			shtEcmpos_ivec.push_back( -1 );
			shtEcmpos_ivec.push_back( -1 );

			//material
			int mat[ 4 ] = {-1, -1, -1, -1};
			setEdgeMat(	sspctredge_vec, nseamReg[ seami ].edgelist[ j ], dir, facei, facej, mat);
			shtMat_ivec.push_back( mat[ 0 ] );
			shtMat_ivec.push_back( mat[ 1 ] );
			shtMat_ivec.push_back( mat[ 2 ] );
			shtMat_ivec.push_back( mat[ 3 ] ) ;
		}
	}
	//set the seam edge number on this sheet
	seamedgenum = shtEposInME_ivec.size();

	//for each edge on the sheet, add the vertices and edges
	for(unsigned int i = 0; i < nsheetReg.size(); i ++ )
	{
		posinme = nsheetReg[ i ].posInMeshEdge;
		posinmv[ 0 ] = meshEdge[ posinme * 2 ];
		posinmv[ 1 ] = meshEdge[ posinme*2 + 1 ];
		//////////////////////////////////////////////////////////////////////////
		if( posinmv[ 0 ] == posinmv[ 1 ])
		{
			cout<<"Warning! I find it!!!"<<"facei:"<<facei<<"facej:"<<facej<<endl;
			cout<<"the "<<i<<"th edge!"<<endl;
		}
		//////////////////////////////////////////////////////////////////////////
		
		//ver
		for( int j = 0; j < 2; j ++ )
		{
			//not added yet
			if( posinmv_posinshtv_map.find( posinmv[ j ] ) == posinmv_posinshtv_map.end())
			{
				//add sheetver, posinmeshver, and mapinfo
				shtVposInMV_ivec.push_back( posinmv[ j ]);
				shtVpos_fvec.push_back( meshVer[ 3*posinmv[ j ]]);
				shtVpos_fvec.push_back( meshVer[ 3*posinmv[ j ] + 1 ]);
				shtVpos_fvec.push_back( meshVer[ 3*posinmv[ j ] + 2 ]);
				posinshtv[ j ] =posinmv_posinshtv_map[ posinmv[ j ]] = shtVposInMV_ivec.size() - 1;
				
			}
			else
			{
				posinshtv[ j ] = posinmv_posinshtv_map[posinmv[ j ]];
			}
		}
        
		//edge
		shtEposInME_ivec.push_back( posinme );
		shtEcmpos_ivec.push_back( posinshtv[ 0 ]);
		shtEcmpos_ivec.push_back( posinshtv[ 1 ]);
		shtEcmpos_ivec.push_back( -1 );
		shtEcmpos_ivec.push_back( -1 );
		shtEcmpos_ivec.push_back( -1 );
			
		//material
		int mat[ 4 ] = {-1, -1, -1, -1};
		setEdgeMat(sspctredge_vec,nsheetReg[ i ],	dir, facei, facej,mat);
				
		shtMat_ivec.push_back( mat[ 0 ] );
		shtMat_ivec.push_back( mat[ 1 ] );
		shtMat_ivec.push_back( mat[ 2 ] );
		shtMat_ivec.push_back( mat[ 3 ] );
	}

	posinmv_posinshtv_map.clear();
}

//#using <mscorlib.dll>

//using namespace System;

//using namespace System;

bool regionHandler::findClosedRegionMatConfig(
		//mesh
		floatvector& meshVer, intvector& meshEdge,
		//vertex registration
		int*& ajptReg,	//new junction point registration with all of them added into Meshver
		int majptnum,
		float*& majpt,
		//edge registration
		newSeamReg*& nseamReg, //new seam registration info
		EdgeReg*& nsheetReg,
		int maseamnum,
		int*& maseam,
		int mafacenum,
		MapArraySR& doubleface2sheet,
		int masheetnum,
		int*& seamonsheetnum,
		int**& seamonsheet,
        int*& faceside,
		float*& sheettab, //6 for each, point and normal
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		intvector*& shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
		intvector*& shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
		floatvector*& shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
		intvector*& shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
		//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
		intvector*& shtMat_arr_ivec,		//each of them has a list of material in it, four number corresponds to one edge 
		//left mat, right mat above, left mat, right mat, below
		int*& seamedgenum_iarr,
		sheetRegion*& region_vec
		)
{
	//step1. amplify the mesh
	//step2. go through each sheet
		//2.1, sort out vertex and edge on this sheet
		//2.2 find closed region
			//2.2.1 find closed boundary
			//2.2.2 put boundaries together to form regions
		//2.3 find the out most region
		//2.4 set material: first round, broadcast only on one sheet
    //step3. broadcast across the maseam edges

	//step1. amplify the mesh
	amplifyMesh(
		meshVer,meshEdge,ajptReg, majptnum,majpt,
		nseamReg, maseamnum,maseam);

	//step2. go through each sheet
	//----------------
	//each of them corresponds to one sheet
	//intvector* shtVposInMV_arr_ivec;		//each of them is the corresponding vertex index in mesh vertices
	//intvector* shtEposInME_arr_ivec;		//each of them is the corresopnding the edge index in mesh edges
 //   floatvector* shtVpos_arr_fvec;		//each of them has the position of the vertices on one sheet
	//intvector* shtEcmpos_arr_ivec;		//each of them has a list of edges in it.
	//													//for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
	//intvector* shtMat_arr_ivec;			//each of them has a list of material in it, four number corresponds to one edge 
	//													//left mat, right mat above, left mat, right mat, below
	//int* seamedgenum_iarr;						//the number of edges on seam in the edge list
	//shtVposInMV_arr_ivec = new intvector[ masheetnum ];
	//shtEposInME_arr_ivec = new intvector[ masheetnum ];
	//shtVpos_arr_fvec = new floatvector[masheetnum]; 
	//shtEcmpos_arr_ivec = new intvector[ masheetnum ];
	//shtMat_arr_ivec = new intvector[ masheetnum ];
	//seamedgenum_iarr = new int[ masheetnum ];
	//sheetRegion* region_vec = new sheetRegion[ masheetnum ];
	//---------------------
	int keys[ 2 ];
	int sheeti;
	//////////////////////////////////////////////////////////////////////////
	bool suc = true;
	//////////////////////////////////////////////////////////////////////////
	for( int facei = 0; facei < mafacenum - 1; facei ++ )
	{
		keys[ 0  ] = facei;
		for( int facej = facei + 1; facej < mafacenum; facej ++ )
		{
			keys[ 1 ] = facej;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );
			if( seamonsheetnum[ sheeti ] == 0 )	//no such sheet
			{
				seamedgenum_iarr[ sheeti ] = 0;
				continue;
			}

			//////////////////////////////////////////////////////////////////////////
			//cout<<"sheeti:"<<sheeti<<endl;
			//////////////////////////////////////////////////////////////////////////

			//2.1, sort out vertex and edge on this sheet
			getVerEdgeOnOneSheet(shtVposInMV_arr_ivec[ sheeti ],shtEposInME_arr_ivec[ sheeti ], shtVpos_arr_fvec[ sheeti ],
			shtEcmpos_arr_ivec[ sheeti ],shtMat_arr_ivec[ sheeti ],
				seamedgenum_iarr[ sheeti ], meshVer,meshEdge, nseamReg,
				nsheetReg[ sheeti ],seamonsheet[ sheeti ],
				seamonsheetnum[ sheeti ],
				faceside,facei,facej,sspctredge_vec);

			//////////////////////////////////////////////////////////////////////////
			//write out the vertex and edge on current sheet
			/*writeVerEdgeDirOneSheet(
				shtVpos_arr_fvec[ sheeti ],shtEcmpos_arr_ivec[ sheeti ],
				sheettab + 6*sheeti + 3, 8, sheeti);*/
			//////////////////////////////////////////////////////////////////////////

			//2.2 find closed region
			float zdir[ 3 ];
			memcpy( zdir, sheettab + 6*sheeti + 3, sizeof( float )*3);
			float xdir[ 3 ] = {0,0,0};
			if( abs(zdir[ 0 ]) < 0.000001 )
			{
				xdir[ 0 ] = 1;
			}
			else if( abs(zdir[ 1 ]) < 0.000001)
			{
				xdir[ 1 ] = 1;
			}
			else
			{
				xdir[ 0 ] = zdir[ 1 ];
				xdir[ 1 ] = -zdir[ 0 ];
			}
			//////////////////////////////////////////////////////////////////////////
			/*tempshtVposInMV_ivec.clear();
			for( int kkk = 0; kkk < shtVposInMV_arr_ivec[ sheeti ].size(); kkk++ )
			{
				tempshtVposInMV_ivec.push_back( shtVposInMV_arr_ivec[ sheeti ][ kkk ]);
			}
			tempshtEposInME_ivec.clear();
			for( int kkk = 0; kkk < shtEcmpos_arr_ivec[ sheeti ].size(); kkk++ )
			{
				tempshtEposInME_ivec.push_back( shtEcmpos_arr_ivec[ sheeti ][ kkk ]);
			}*/
			//////////////////////////////////////////////////////////////////////////
			if(findClosedRegions(
				shtVpos_arr_fvec[ sheeti ],	shtEcmpos_arr_ivec[ sheeti ],				
				zdir,xdir,region_vec[ sheeti ],seamedgenum_iarr[ sheeti ],
				seamonsheetnum[ sheeti ],seamonsheet[ sheeti ],
				maseam, majpt ))
			{
				//////////////////////////////////////////////////////////////////////////
				//write out the regions on current sheet
				//////////////////////////////////////////////////////////////////////////
				//2.4 set material: first round, broadcast only on one sheet
				setMatOnOneSheet(shtEcmpos_arr_ivec[ sheeti ], shtMat_arr_ivec[ sheeti ], region_vec[sheeti]);
				suc = false;
		//		return false;
			}
			else
			{
				cout<<"sheet: "<<sheeti <<" fail!"<<endl;
			}
		
		//	if(!findClosedRegions(
		//		shtVpos_arr_fvec[ sheeti ],	shtEcmpos_arr_ivec[ sheeti ],				
		//		zdir,xdir,region_vec[ sheeti ],seamedgenum_iarr[ sheeti ],
		//		seamonsheetnum[ sheeti ],seamonsheet[ sheeti ],
		//		maseam, majpt ))
		//	{
		//		return false;
		//	}

		//	//2.4 set material: first round, broadcast only on one sheet
		//	setMatOnOneSheet(shtEcmpos_arr_ivec[ sheeti ], shtMat_arr_ivec[ sheeti ], region_vec[sheeti]);
		}
	}


	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////

	//step3. broadcast across the maseam edges
	//using namespace std;
	try
	{
		broadAcrossSeam(
			shtEposInME_arr_ivec,shtEcmpos_arr_ivec, seamedgenum_iarr,
			region_vec,	  masheetnum,
			nseamReg,doubleface2sheet, seamonsheetnum,mafacenum);
	}
	//catch (Exception* e)
	//{
	//	return false;			
	//}
//	catch (CFileException* e)
//	{
//	}
//kw	catch (CException*)
	catch (exception)
	{
		return false;			
	}	

	//////////////////////////////////////////////////////////////////////////
	//writeVerEdgeRegion_db(
	//	 4,
	//	masheetnum,
	//	shtVposInMV_arr_ivec,		//each of them is the corresponding vertex index in mesh vertices
	//	shtEposInME_arr_ivec,		//each of them is the corresopnding the edge index in mesh edges
	//	shtVpos_arr_fvec,		//each of them has the position of the vertices on one sheet
	//	shtEcmpos_arr_ivec,		//each of them has a list of edges in it.
	//	seamedgenum_iarr,						//the number of edges on seam in the edge list
	//	region_vec
	//	);
	//////////////////////////////////////////////////////////////////////////
	return true;

}
