#include "../Ctr2SufManager/Ctr2SufManager.h"
#include "../math/mymath.h"
#include "../KW_CS2Surf/KW_CS2Surf.h"
#include <sys/timeb.h>
#include <time.h>


void Ctr2SufManager::generateMA(int spaci,
								//subspace info
								int& subvernum, float*& subver, int& subedgenum,int*& subedge,int& subfacenum,
								int*& subfaceedgenum, int**& subface, float*& subparam, int*& subver2wver,int*& subedge2wedge,
								//ma info
								int& majptnum, float*& majpt, int& maseamnum, int*& maseam, 
								MapArraySR& doubleface2sheet, int*& seamonsheetnum, int**& seamonsheet,  float*& sheettab,
								int*& ver2jpt)
{
	MAGenerator::generateMA(planenum, pparam, ssvernum, ssver, ssedgenum, ssedge, ssfacenum, ssfaceedgenum,
		ssface, ssface_planeindex, ssspacenum, ssspacefacenum, ssspace, ssspace_planeside, spaci,
		//subspace info
		subvernum, subver,  subedgenum, subedge, subfacenum,
		 subfaceedgenum,  subface,  subparam,  subver2wver, subedge2wedge,
		//ma info
		majptnum,  majpt, maseamnum,  maseam, 
		doubleface2sheet, seamonsheetnum,  seamonsheet,  sheettab,
		ver2jpt);
}

void Ctr2SufManager::ctr2sufSubspaceProc(int spaci, floatvector& meshVer, intvector& meshEdge, intvector& meshFace)
{
	//gather the contour of current subspace
	//vertex
//	SUBSPACECTRVER** sspctrver;
//	SUBSPACECTREDGE** sspctredge;
	cout<<"****		Gathering contours	****"<<endl;
//	int sspctrvernum;
//	int sspctredgenum;
	vector<SSPCTRVERVEC> sspctrver_vec;
	vector<SSPCTREDGEVEC> sspctredge_vec;
	if( !gatherSubspaceCtr( sspctrver_vec, sspctredge_vec, spaci) )
	{
		cout<<"No contour in this subspace!"<<endl;
		return;	//no contour in this subspace, no need to process!
	}
	//////////////////////////////////////////////////////////////////////////
	//SSContour::writeGatheredCtr_DB(sspctrver_vec, sspctredge_vec, spaci, ssspacefacenum);
	//////////////////////////////////////////////////////////////////////////
	cout<<"===		GATHERING DONE!		==="<<endl;
	//generate MA
//	cout<<"****		Medial Axis		****"<<endl;
	int subvernum;
	float* subver;
	int subedgenum;
	int* subedge;
	int subfacenum;
	int* subfaceedgenum;
	int** subface;
	float* subparam;
	int* subver2wver;
	int* subedge2wedge;
	int majptnum;  
	float* majpt;
	int maseamnum; 
	int* maseam;
	MapArraySR doubleface2sheet;
	int* seamonsheetnum;
	int** seamonsheet; 
	float* sheettab;	//6 for each, point and normal
	int* ver2jpt;
	//generateMA( spaci,
	//subvernum, subver,  subedgenum, subedge, subfacenum,subfaceedgenum,  subface,  subparam,  subver2wver, subedge2wedge,//subspace
	//majptnum,  majpt, maseamnum,  maseam, 	doubleface2sheet, seamonsheetnum,  seamonsheet,  sheettab,ver2jpt);//ma
	MAGenerator::generateMA(planenum, pparam, ssvernum, ssver, ssedgenum, ssedge, ssfacenum, ssfaceedgenum,
		ssface, ssface_planeindex, ssspacenum, ssspacefacenum, ssspace, ssspace_planeside, spaci,
		//subspace info
		subvernum, subver,  subedgenum, subedge, subfacenum,
		subfaceedgenum,  subface,  subparam,  subver2wver, subedge2wedge,
		//ma info
		majptnum,  majpt, maseamnum,  maseam, 
		doubleface2sheet, seamonsheetnum,  seamonsheet,  sheettab,
		ver2jpt);

	this->SaveMAInfo(majptnum,majpt,maseamnum,maseam);


	SSContour::sortSheetSeams( maseam, subfacenum, doubleface2sheet, seamonsheetnum,seamonsheet);


	//////////////////////////////////////////////////////////////////////////
	//FILE* fout = fopen( "mmdebug/onesubspace/param.txt","w");
	//fprintf( fout, "{");
	//for( int i = 0; i < subfacenum ; i ++ )
	//{
	//	for(int j = 0; j < 4; j ++  )
	//	{
	//		if( j == 0 )
	//			fprintf( fout, "{%f", subparam[ 4*i + j ]);
	//		else
	//			fprintf( fout, "%f", subparam[ 4*i + j ]);
	//		if ( j != 3 )
	//			fprintf( fout, ",");
	//		else
	//			fprintf(fout, "}");
	//		//fprintf( fout, )
	//	}
	//	if( i != subfacenum-1 )
	//	{
	//		fprintf( fout, ",");
	//	}
	//	else
	//		fprintf( fout, "}");
	//}

	//fclose( fout );
	////////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	//char numstr[ 10 ];
	//itoa( spaci + 1, numstr, 10);

	//char fname[1024];
	//strcpy( fname, "mmdebug/onesubspace/");
	//strcat( fname, numstr );
	//strcat( fname, ".meda");
	//MAGenerator::writeMA( fname, subvernum, subver, subedgenum
	//	, subedge, subfacenum, subfaceedgenum, subface, majptnum, majpt, maseamnum,maseam,
	//	doubleface2sheet, seamonsheetnum, seamonsheet, ver2jpt);
	////////////////////////////////////////////////////////////////////////////
	cout<<"===		MA DONE!		==="<<endl;

	//return;

	//split contour on the face
	divideFaceContourByProjMA(spaci,
		sspctrver_vec,
		sspctredge_vec,
		//ma
		subvernum,subver,subedgenum,subedge,subfacenum,subfaceedgenum,subface,
		subparam,subver2wver,subedge2wedge,majptnum,majpt,maseamnum,
		maseam,doubleface2sheet,seamonsheetnum, seamonsheet,sheettab,ver2jpt);
	//////////////////////////////////////////////////////////////////////////
	//SSContour::writeGatheredCtr_DB(sspctrver_vec, sspctredge_vec, spaci, ssspacefacenum);
	//////////////////////////////////////////////////////////////////////////

	int masheetnum = subfacenum * (subfacenum - 1)/2;
	//project contour to MA sheet
	//////////////////////////////////////////////////////////////////////////
	//cout<<"projecting contours onto MA sheet!"<<endl;
	
	//////////////////////////////////////////////////////////////////////////
	int* jptReg;	//for each junction point, corresponding vertex in meshver
	intvector* seamVerReg;	//for each seam, the projected vertices
	EdgeReg* seamEdgeReg	;	//for each seam, the projected edges on it
	intvector* sheetVerReg;	//for each sheet, the projected vetices on it
	EdgeReg* sheetEdgeReg;	//..		...	, the projected edges on it
	jptReg = new int[ majptnum ];
	seamVerReg = new intvector[ maseamnum ];
	seamEdgeReg = new EdgeReg[ maseamnum ];
	sheetVerReg = new intvector[ masheetnum ];
	sheetEdgeReg = new EdgeReg[ masheetnum ];
	Projector::projectCtr(meshVer,
		meshEdge,jptReg,seamVerReg,seamEdgeReg,sheetVerReg,sheetEdgeReg,
		sspctrver_vec,sspctredge_vec,
		//ma
		subvernum,subver,subedgenum,subedge,subfacenum,subfaceedgenum,subface,
		subparam,subver2wver,subedge2wedge,majptnum,majpt,maseamnum,
		maseam,doubleface2sheet,seamonsheetnum, seamonsheet,sheettab,ver2jpt);
	//////////////////////////////////////////////////////////////////////////
	//cout<<"DONE!!! Projeting contours onto MA Sheet!"<<endl;
	//Projector::writeProjection_NoSplit(spaci, meshVer, meshEdge, majptnum, maseamnum, masheetnum, jptReg, seamVerReg, seamEdgeReg,
	//	sheetVerReg, sheetEdgeReg, sspctrver_vec,sspctredge_vec );
	////////////////////////////////////////////////////////////////////////////
	cout<<"===		Projection Done!		==="<<endl;

	
	//////////////////////////////////////////////////////////////////////////
	////gather the ancestor of the faces
	//for( int i = 0; i < ssspacefacenum[ spaci ]; i ++ )
	//{
	//	int facei = ssspace[ spaci ][ i ];
	//	int ind = 0;
	//	if( sfacespaci[ 2*facei ] != spaci )
	//		ind = 1;
	//	ind += 2*facei;
	//	for(int j = 0; j < sspctredge_vec[ i ].size(); j ++ )
	//	{
	//		ancestorlist[ ind ].insert( sspctredge_vec[ i ][ j ].ancestor );
	//	}
	//}
	//////////////////////////////////////////////////////////////////////////
//
//	//Split projected contours
	//cout<<"Splitting edges on sheets...."<<endl;
	newSeamReg* nseamReg;	//new seam registration, 
									//each has three arrays:
									//1bitmap 
									//2vertices list 
									//3edge info: pos in mesh edge and corresponding contour edges
	nseamReg = new newSeamReg[ maseamnum ];
	EdgeReg* nsheetReg;	//new sheet registration
								//each vector in it represents all the edges on this sheet
								//each edge has two info (1)pos in mesh edge (2) corresponding contour edges
	nsheetReg = new EdgeReg[ masheetnum ];
	ProjSplitter::SplitProjectedEdge(meshVer, meshEdge,
		nseamReg,sspctrver_vec, sspctredge_vec, nsheetReg,
		subfacenum, jptReg, seamVerReg, seamEdgeReg, sheetVerReg,
		sheetEdgeReg,majptnum, maseamnum, masheetnum,maseam, majpt, doubleface2sheet);
	//old registraion has already been cleared after splitting.
	//////////////////////////////////////////////////////////////////////////
	//ProjSplitter::WriteInfoAfterSplit(spaci, meshVer, meshEdge, majptnum,
	//	maseamnum, masheetnum, jptReg, nseamReg, nsheetReg, sspctrver_vec,
	//	sspctredge_vec);
	////////////////////////////////////////////////////////////////////////
	cout<<"===		Splitting Projected edges DONE!		==="<<endl;

//	//////////////////////////////////////////////////////////////////////////
//	for( int i = 0; i < ssspacefacenum[ spaci ]; i ++ )
//	{
//		int facei = ssspace[ spaci ][ i ];
//		int ind = 0;
//		if( sfacespaci[ 2*facei ] != spaci )
//			ind = 1;
//		ind += 2*facei;
//		for(int j = 0; j < sspctredge_vec[ i ].size(); j ++ )
//		{
//			ancestorlist_split[ ind ].insert( sspctredge_vec[ i ][ j ].ancestor );
//		}
//	}
//	//////////////////////////////////////////////////////////////////////////
//
	//find closed region and set material configuration
	intvector* shtVposInMV_arr_ivec;		//each of them is the corresponding vertex index in mesh vertices
	intvector* shtEposInME_arr_ivec;		//each of them is the corresopnding the edge index in mesh edges
	floatvector* shtVpos_arr_fvec;		//each of them has the position of the vertices on one sheet
	intvector* shtEcmpos_arr_ivec;		//each of them has a list of edges in it.
//	for each edge, it has 5 numbers, v1,v2,twin edge, next edge, region number
	intvector* shtMat_arr_ivec;			//each of them has a list of material in it, four number corresponds to one edge 
//	left mat, right mat above, left mat, right mat, below
	int* seamedgenum_iarr;						//the number of edges on seam in the edge list
	sheetRegion* region_vec;
	shtVposInMV_arr_ivec = new intvector[ masheetnum ];
	shtEposInME_arr_ivec = new intvector[ masheetnum ];
	shtVpos_arr_fvec = new floatvector[masheetnum]; 
	shtEcmpos_arr_ivec = new intvector[ masheetnum ];
	shtMat_arr_ivec = new intvector[ masheetnum ];
	seamedgenum_iarr = new int[ masheetnum ];
	region_vec = new sheetRegion[ masheetnum ];
	//sheetRegion* region_vec = new sheetRegion[ masheetnum ];
	int* ajptreg = new int[ majptnum ];
	memcpy( ajptreg, jptReg, majptnum * sizeof( int ));
	if(! regionHandler::findClosedRegionMatConfig(meshVer, meshEdge,
		ajptreg, majptnum, majpt, nseamReg, nsheetReg, maseamnum, maseam,
		subfacenum, doubleface2sheet, masheetnum, seamonsheetnum, seamonsheet, ssspace_planeside[ spaci ],
		sheettab,sspctredge_vec, shtVposInMV_arr_ivec, shtEposInME_arr_ivec, shtVpos_arr_fvec, 
		shtEcmpos_arr_ivec, shtMat_arr_ivec, seamedgenum_iarr,region_vec))
	{
		regionHandler::writeVerEdgeRegion_db(spaci, masheetnum, shtVposInMV_arr_ivec,
		shtEposInME_arr_ivec, shtVpos_arr_fvec, shtEcmpos_arr_ivec, seamedgenum_iarr,
		region_vec);	
		writeSheetTab(sheettab, masheetnum, spaci);
	/*	regionHandler::writeMat_db( spaci, masheetnum, shtMat_arr_ivec);
		regionHandler::writeVerEdgeOnSheet_db(shtVposInMV_arr_ivec, shtEposInME_arr_ivec,
			shtVpos_arr_fvec, shtEcmpos_arr_ivec,  masheetnum, spaci); */
//		cout<<"===		Closed region and material configuration DONE!		==="<<endl;

		cout<<":(unable to find closed region and set material for current subspace!"<<endl;
        return;
	}
	
	//////////////////////////////////////////////////////////////////////////
	////cout the edges registered on sheet0
	////facei, facej, side
	//cout<<"sheet 0 information.............."<<endl;
	//cout<<"facei side : "<<ssspace_planeside[ spaci ][ 0 ]<<endl;
	//cout<<"facej side : "<<ssspace_planeside[ spaci ][ 1 ] <<endl;
	//int regedgenum = nsheetReg[ 0 ].size();
	//for( int i = 0; i < regedgenum; i ++ )
	//{
	//	int posinme = nsheetReg[ 0 ][ i ].posInMeshEdge;
	//	int posvs[ 2 ] = {meshEdge[ 2*posinme ], meshEdge[ 2*posinme + 1 ]};
	//	//cout<<""
	//	cout<<"position of the two projected vertices:\n"
	//		<<"{"
	//		<<meshVer[ 3*posvs[ 0 ]]<<","<<meshVer[ 3*posvs[0] + 1]<<","
	//		<<meshVer[ 3*posvs[ 0 ] + 2 ]<<"}\t{"
	//		<<meshVer[ 3*posvs[ 1 ]]<<","<<meshVer[ 3*posvs[1] + 1]<<","
	//		<<meshVer[ 3*posvs[ 1 ] + 2 ]<<"}"<<endl;
	//	

	//	int crspnum = nsheetReg[ 0 ][ i ].crspCtrEdges.size();
	//	for( int j = 0; j < crspnum; j ++ )
	//	{
	//		int facei = nsheetReg[ 0 ][ i ].crspCtrEdges[ j ].facei ;
	//		int edgei =nsheetReg[ 0 ][ i ].crspCtrEdges[ j ].edgei ;
	//		
	//		cout<<"facei, edgei, dir:"
	//			<<nsheetReg[ 0 ][ i ].crspCtrEdges[ j ].facei<<" "
	//			<<nsheetReg[ 0 ][ i ].crspCtrEdges[ j ].edgei<<" "
	//			<<nsheetReg[ 0 ][ i ].crspCtrEdges[ j ].samedirec
	//			<<endl;
	//		cout<<"two mats:"<<sspctredge_vec[ facei ][ edgei ].mat[ 0 ]<<" "
	//			<<sspctredge_vec[ facei ][ edgei ].mat[ 1 ]<<endl;
	//		int* ctrvs = sspctredge_vec[ facei ][ edgei ].veris;

	//		cout<<"two vertices: {"
	//			<<sspctrver_vec[ facei ][ ctrvs[ 0 ] ].pos[ 0 ]<<","
	//			<<sspctrver_vec[ facei ][ ctrvs[ 0 ]].pos[1 ]<<","
	//			<<sspctrver_vec[ facei ][ ctrvs[ 0 ] ].pos[ 2 ]<<"}\t{"
	//			<<sspctrver_vec[ facei ][ ctrvs[ 1 ]].pos[ 0 ] <<","
	//			<<sspctrver_vec[ facei ][ ctrvs[ 1 ]].pos[ 1  ]<<","
	//			<<sspctrver_vec[ facei] [ctrvs[ 1 ]].pos[ 2 ]<<"}"<<endl;
	//	}
	//}
	//////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////
	/*regionHandler::writeVerEdgeRegion_db(spaci, masheetnum, shtVposInMV_arr_ivec,
		shtEposInME_arr_ivec, shtVpos_arr_fvec, shtEcmpos_arr_ivec, seamedgenum_iarr,
		region_vec);	
	writeSheetTab(sheettab, masheetnum, spaci);*/
	//////////////////////////////////////////////////////////////////////////

	//regionHandler::writeMat_db( spaci, masheetnum, shtMat_arr_ivec);
	//regionHandler::writeVerEdgeOnSheet_db(shtVposInMV_arr_ivec, shtEposInME_arr_ivec,
	//	shtVpos_arr_fvec, shtEcmpos_arr_ivec,  masheetnum, spaci);
	cout<<"===		Closed region and material configuration DONE!		==="<<endl;
	
	//add the contour vertices on faces into meshVer
	int** cvposinmesh = new int*[ subfacenum ];
	for(int i = 0; i < subfacenum; i ++ )
	{
		int vnum = sspctrver_vec[ i ].size();
		if( vnum == 0 )
		{
			cvposinmesh[ i ] = NULL;
			continue;
		}
		cvposinmesh[ i ] = new int[ vnum ];
		for( int j = 0; j < vnum; j ++ )
			cvposinmesh[ i ][ j ] = -1;		
	}
	int* jpt2ver = new int[ majptnum ];
	for( int i= 0; i < majptnum; i ++ )
	{
		jpt2ver[ i ] = -1;
	}
	for( int i = 0; i < subvernum; i ++ )
	{
		//////////////////////////////////////////////////////////////////////////
		//cout<<ver2jpt[ i ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		jpt2ver[ ver2jpt[ i ]] = subver2wver[ i ];
	}

	for( int i = 0; i< subfacenum; i ++ )
	{
		//no need to add current contour vertex and edges
		if( sspctrver_vec[ i ].size() == 0 )
		{
			//////////////////////////////////////////////////////////////////////////
			if( ssspace[ spaci ][ i ] == 58 )
			{
				cout<<"no contour vertex on face 58, not mention edge!"<<endl;
			}
			//////////////////////////////////////////////////////////////////////////

			continue;
		}
		//set the contour edge intset
		int facei = ssspace[ spaci ][ i ];
		if( sfacectrei[ facei ].size() == 0 )	//has already been set
		{		
			//////////////////////////////////////////////////////////////////////////
			/*cout<<"the contou redges on this face has alrady been set!"<<endl;
			intset::iterator iter = sfacectrei[ facei ].begin();
			while( iter != sfacectrei[ facei ].end() )
			{
			cout<<*iter<<" ";
			iter ++;
			}
			cout<<endl;	*/
			//////////////////////////////////////////////////////////////////////////
			//	continue;
			for(unsigned int j = 0; j < sspctredge_vec[ i ].size(); j ++ )
			{
				sfacectrei[ facei ].insert( sspctredge_vec[ i ][ j ].ancestor);
			}	
		}
		//////////////////////////////////////////////////////////////////////////
		if( facei == 58 )
		{
			cout<<"contour on face 58:"<<sfacectrei[ facei ].size()<<endl;
		}
		//////////////////////////////////////////////////////////////////////////

		//add the vertices on current face	
		FaceGenerator::addCtrVerOneFace(
			meshVer, sspctrver_vec[ i ],
			cvposinmesh[ i ],
			jpt2ver,ajptreg, subedgenum, maseamnum,
			nseamReg, ssver,ssedge,
			//subedgenum,
			//ssfaceedgenum[ ssspace[ spaci][ i ] ],
			//ssface[ ssspace[ spaci ][ i ]],
			subedge2wedge,
			sverreg, sedgereg, spaci);
	}
	cout<<"===		Add contour vertex into meshVer DONE!		==="<<endl;	

	//generate mesh in the current subspace according to the material configuration
	//generate face on sheets
	FaceGenerator::GFaceOnSheets(meshFace,
		shtVposInMV_arr_ivec,
		shtVpos_arr_fvec,shtEcmpos_arr_ivec,region_vec,
		sheettab, subfacenum, seamonsheetnum, doubleface2sheet);
	cout<<"===		Generate faces on sheets DONE!		==="<<endl;

	FaceGenerator::GFaceNormalSeams(
		meshFace,		sspctredge_vec,		 nseamReg,
		maseamnum, subedgenum,	cvposinmesh, spaci,
		sfacectrei, ssspace[ spaci ],	
		sfacespaci, sfaceregface,
		ssspace_planeside[ spaci ]);	
	cout<<"===		Generate faces for normal seam ctr edge DONE!		==="<<endl;
	
	FaceGenerator::GFaceSheet(
		meshEdge,	meshFace,		sspctredge_vec,
		cvposinmesh,	 nsheetReg, subfacenum,
		doubleface2sheet, seamonsheetnum,
		spaci,		ssspace[ spaci ],	 sfacespaci,		sfaceregface,
		sfacectrei	,
		ssspace_planeside[ spaci ]);	

	//////////////////////////////////////////////////////////////////////////
	//FaceGenerator::writeSubMeshOut( meshVer, meshFace, spaci);
	//////////////////////////////////////////////////////////////////////////
	cout<<"===		Generate faces for sheet ctr edge DONE!		==="<<endl;

	//clear the temp var for adding contour vertex into meshVer
	for( int i = 0; i < subfacenum; i++)
	{
		if( cvposinmesh[ i ] != NULL )
			delete []cvposinmesh[ i ];
	}
	delete []cvposinmesh;
	//clear the registration information
	delete []jptReg;
	delete []ajptreg;
	//set the subedge on edge seams, if in, mark it 1 else 0.
	int tei = maseamnum - subedgenum;
	int tei2 = 0;
	while( tei < maseamnum )
	{
		int sei = subedge2wedge[ tei2 ];
		if( sedgesubedgemark[ sei ].size() != 0 ) //already set		
		{
			tei ++;
			tei2++;
			continue;
		}
		int tenum = nseamReg[ tei ].vernum - 1;
		sedgesubedgemark[ sei ].resize( tenum );
		for( int j = 0; j < tenum; j ++ )
		{
			if( nseamReg[ tei ].edgelist[ j ].crspCtrEdges.size() == 0 )
				sedgesubedgemark[ sei ][ j ] = 0;
			else
				sedgesubedgemark[ sei ][ j ] = 1;
		}
		tei ++;
		tei2 ++;
	}

	//delete nseamreg
	for( int i = 0; i < maseamnum; i ++ )
	{
		int vnum = nseamReg[ i ].vernum;
		for( int j = 0; j < vnum - 1; j ++ )
			nseamReg[ i ].edgelist[ j ].crspCtrEdges.clear();
		delete []nseamReg[ i ].edgelist;
		delete []nseamReg[ i ].subEdgeIsIn;
		delete []nseamReg[ i ].verPosInMeshVer;
	}
	delete []nseamReg;
	
	//delete nsheetreg
	for( int i = 0; i < masheetnum; i ++)
	{
		int tenum = nsheetReg[ i ].size();
		for( int j = 0; j < tenum; j ++ )
			nsheetReg[ i ][ j ].crspCtrEdges.clear();
		nsheetReg[ i ].clear();
	}
	delete []nsheetReg;

	for( int i = 0; i < masheetnum; i ++ )
	{
		shtVposInMV_arr_ivec[ i ].clear();
		shtEposInME_arr_ivec[ i ].clear();	
		shtVpos_arr_fvec[ i ].clear();
		shtEcmpos_arr_ivec[ i ].clear();
		shtMat_arr_ivec[ i ].clear();
		int tsize = region_vec[ i ].boundaries.size();
		for( int j = 0; j < tsize;j ++ )
			region_vec[ i ].boundaries[ j ].clear();
		region_vec[ i ].boundaries.clear();
		region_vec[ i ].mat.clear();
		tsize = region_vec[ i ].regions.size();
		for( int j = 0; j < tsize; j++ )
			region_vec[ i ].regions[ j ].clear();
		region_vec[ i ].regions.clear();
		tsize = region_vec[ i ].regionneighbrs.size();
		for(int j = 0; j < tsize; j ++ )
			region_vec[ i ].regionneighbrs[ j ].clear();
		region_vec[ i ].regionneighbrs.clear();
	}
	delete []shtVposInMV_arr_ivec;
	delete []shtEposInME_arr_ivec;
	delete []shtVpos_arr_fvec;	
	delete []shtEcmpos_arr_ivec;
	delete []shtMat_arr_ivec;	
	delete []seamedgenum_iarr;	
	delete []region_vec;

	/*
	//already deleted during splitting!!
	for( int i = 0; i < maseamnum; i ++)
	{
		seamVerReg[ i ].clear();
		for( int j = 0; j < seamEdgeReg[ i ].size(); j ++)
		{
			seamEdgeReg[ i ][ j ].crspCtrEdges.clear();
		}
		seamEdgeReg[ i ].clear();
	}
	delete []seamVerReg;
	delete []seamEdgeReg;
	for( int i = 0; i < masheetnum; i ++ )
	{
		sheetVerReg[ i ].clear();

		for( int j = 0; j < sheetEdgeReg[ i ].size(); j ++)
		{
			sheetEdgeReg[ i ][ j ].crspCtrEdges.clear();
		}
		sheetEdgeReg[ i ].clear();
	}
	delete []sheetVerReg;
	delete []sheetEdgeReg;*/

	//clear the contour information
	for(unsigned int i = 0 ;i < sspctredge_vec.size(); i ++)
		sspctredge_vec[ i ].clear();
	sspctredge_vec.clear();
	for(unsigned int  i = 0; i < sspctrver_vec.size(); i++ )
		sspctrver_vec[ i ].clear();
	sspctrver_vec.clear();

	//the subspace and ma information
	delete []subver;
	delete []subedge;
	delete []subfaceedgenum;
	for( int i = 0; i < subfacenum; i ++)
		delete []subface[ i ];
	delete subface;
	delete []subparam;
	delete []subver2wver;
	delete []subedge2wedge;
	delete []majpt;
	delete []maseam;
	delete []seamonsheetnum;
	for( int i = 0; i < ( subfacenum-1 )* subfacenum/2; i++)
		delete []seamonsheet[ i ];
	delete []seamonsheet;
	delete []sheettab;
	delete []ver2jpt;
}

void Ctr2SufManager::ctr2sufProc(vector<vector<Point_3> >& MeshBoundingProfile3D,vector<Point_3>& vecTestPoint)
{
	clock_t   start   =   clock();   
	struct __timeb64 timebuffer;
	char *timeline;
	_ftime64( &timebuffer );
	timeline = _ctime64( & ( timebuffer.time ) );
	printf( "The time is %.19s.%hu %s", timeline, timebuffer.millitm, &timeline[20] );

	//common line case
	//compute the common line
	if( isComnCase )	
	{
		float tempparam[ 8 ];
		for( int i = 0; i < 8; i ++)
		{
			tempparam[ i  ] = pparam[ i ];
		}

		//////////////////////////////////////////////////////////////////////////
		/*for( int i = 0 ; i < 8; i ++ )
		{
			cout<<tempparam[ i ]<<",";
		}
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////

		computeComnLine( tempparam, tempparam + 4, comndir, comnpt);
		//////////////////////////////////////////////////////////////////////////
		/*for( int i = 0 ; i < 8; i ++ )
		{
			cout<<tempparam[ i ]<<",";
		}
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		//writeComnLineDB( tempparam, tempparam + 4, comndir, comnpt);
		//////////////////////////////////////////////////////////////////////////
	}

	//////////////////////////////////////////////////////////////////////////
	/*for( int i = 0; i < planenum + 4; i ++ )
	{
		cout<<pparam[ 4* i ]<<","
			<<pparam[ 4*i + 1]<<","
			<<pparam[ 4*i + 2]<<","
			<<pparam[ 4*i + 3]<<endl;
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//preprocess data first
	int** pctrvermark;

	if( !ispreproced || isComnCase)
	{
		pctrvermark = preProcData(isComnCase);
		//reset showcontourvers
		for( int i = 0; i < showplanenum; i ++)
			delete []showctrvers[ i ];
		delete []showctrvers;

		//copy the data from processed data, and reset size
		showplanenum = planenum;
		showctrvers = new float*[ showplanenum ];
		showctredgenum = new int[ showplanenum ];
		showctredges = new int*[ showplanenum ];
		for( int i = 0; i < showplanenum; i++ )
		{
			int versize = pctrvernum[ i ]*3; 
			showctrvers[ i ] = new float[ versize ];
			for( int j = 0 ; j < versize;  j ++ )
				showctrvers[ i ][ j ] = pctrvers[ i ][ j ]*DIM/PROCSIZE;

			showctredgenum[ i ] = pctredgenum[ i ];
			int edgesize = pctredgenum[ i ] * 2;			
			showctredges[ i ] = new int[ edgesize ];
			for( int  j = 0; j < showctredgenum[ i ]; j ++)
			{
				showctredges[ i ][ 2*j ] = pctredges[ i ][ 4*j ];
				showctredges[ i ][ 2*j + 1 ] = pctredges[ i ][ 4*j + 1];
			}
		}

		for( int i = 0; i < planenum; i ++)
		{
			int iVerIndBase=0;
			vector<Point_3> CurrentProfile3D;
			for( int j = 0;j < showctredgenum[ i ]; j++)
			{

				int v1 = showctredges[ i ][ 2*j ];
				int v2 = showctredges[ i ][ 2*j + 1 ];

				CurrentProfile3D.push_back(Point_3(showctrvers[ i ][ v1* 3],
					showctrvers[ i ][ v1* 3+1],
					showctrvers[ i ][ v1* 3+2]));

				if (v2==iVerIndBase)
				{
					iVerIndBase=iVerIndBase+CurrentProfile3D.size();
					MeshBoundingProfile3D.push_back(CurrentProfile3D);
					CurrentProfile3D.clear();
				}
			}
		}
		//for( int i = 0; i < planenum; i ++)
		//{	
		//	vector<Point_3> CurrentProfile3D;
		//	for( int j = 0;j < showctredgenum[ i ]; j++)
		//	{
		//		int v1 = showctredges[ i ][ 2*j ];
		//		glVertex3fv( &showctrvers[ i ][ v1* 3] );
		//		CurrentProfile3D.push_back(Point_3(showctrvers[ i ][ v1* 3],
		//								showctrvers[ i ][ v1* 3+1],
		//								showctrvers[ i ][ v1* 3+2]));
		//	}
		//	MeshBoundingProfile3D.push_back(CurrentProfile3D);
		//}

	}
	//////////////////////////////////////////////////////////////////////////
	//for( int i = 0; i < planenum + 4; i ++ )
	//{
	//	cout<<pparam[ 4* i ]<<","
	//		<<pparam[ 4*i + 1]<<","
	//		<<pparam[ 4*i + 2]<<","
	//		<<pparam[ 4*i + 3]<<endl;
	//}
	//cout<<endl;
	//////////////////////////////////////////////////////////////////////////
	//partition
//	cout<<"------------------ PARTITION ------------------"<<endl;
//	cout<<"Starting............"<<endl;
	partition();	

	///////////////////////////////////////////////////////////B///////////////
	/*writePartitionOut("mmdebug/partition.txt");*/
	//////////////////////////////////////////////////////////////////////////
//	cout<<"DONE!"<<endl<<endl;
	
	//process data
//	cout<<"------------------ PUT CONTOUR INTO FACE ------------------"<<endl;
	putContourIntoFace( pctrvermark );

	//SaveCtr2FaceInfo();


	if( isComnCase )
	{
		for( int i = 0; i < planenum; i ++ )
			delete []pctrvermark[ i ];
		delete []pctrvermark;
	}

	//////////////////////////////////////////////////////////////////////////
//	ContourHandler::writeContourInFace( ssfacenum, ctrfvernum, ctrfverpos, ctrfvertype, 
//		ctrfverval, ctrfedgenum, ctrfedge, ctrfedgetype, ctrfedgeval );	
	//////////////////////////////////////////////////////////////////////////
//	cout<<"DONE!"<<endl<<endl;

	
	//go through each subspace to generate mesh
//	cout<<"-------- GENERATING MESH IN EACH SUBSPACE ------------"<<endl;
	//kw noted: vertices, edges and faces in each subspace
	//stitch them together finally
	floatvector* subMeshVer;
	intvector* subMeshEdge;
	intvector* subMeshFace;
	subMeshVer = new floatvector[ ssspacenum ];
	subMeshEdge = new intvector[ ssspacenum ];
	subMeshFace = new intvector[ ssspacenum ];

	//initialize registration informaiton for stitching
	sverreg = new intvector[ ssvernum ];
	sedgereg = new vector<intvector>[ ssedgenum ];
	sedgesubedgemark = new intvector[ ssedgenum ];
	sfacectrei = new intset[ ssfacenum ];
	sfacespaci = new int[ ssfacenum * 2 ];
	sfaceregface = new vector<intvector>[ ssfacenum ];
	sfaceregver = new intvector[ ssfacenum ];
	sfaceregedgever = new intvector[ ssfacenum ];
	for( int i = 0; i < ssfacenum*2; i ++)
	{
		sfacespaci[ i ] = -1;
	}
	for( int spacei = 0; spacei < ssspacenum; spacei ++)
	{
		for( int facej = 0; facej < ssspacefacenum[ spacei ]; facej ++)
		{
			int facei = ssspace[ spacei ][ facej ];
			if( sfacespaci[ 2 * facei ] == -1)
				sfacespaci[ 2 * facei ] = spacei;
			else
				sfacespaci[ 2*facei + 1 ] = spacei;
		}
	}
	
	//////////////////////////////////////////////////////////////////////////
	/*ancestorlist.resize( ssfacenum * 2 );
	ancestorlist_split.resize( ssfacenum * 2 );*/
	//////////////////////////////////////////////////////////////////////////

	//return;

	dbSpaceNum = ssspacenum; 
	if( dbOneSpaceMode )
	{
		if( dbCurSpace < dbSpaceNum )
		{
			ctr2sufSubspaceProc( dbCurSpace, subMeshVer[ dbCurSpace ], subMeshEdge[ dbCurSpace ], subMeshFace[ dbCurSpace ]);

			//submeshedge is useless for stitching!
			subMeshEdge[ dbCurSpace ].clear();
		}
	}
	//kw: here can use multi-thread to compute each submesh in parallel
	else{
		//	for( int i = 7; i< 8; i ++)

		for( int i = 0; i < ssspacenum; i ++)
			//	for( int i = 2; i < 3; i ++)
			//	for( int i = 54; i < 55; i ++)
			//	for(int i = 0; i < 12; i ++)
			//	for( int i = 14; i < 15; i ++)
		{
					cout<<"--- subspace " << i <<endl;
			ctr2sufSubspaceProc( i, subMeshVer[ i ], subMeshEdge[ i ], subMeshFace[ i ]);

			//submeshedge is useless for stitching!
			subMeshEdge[ i ].clear();
		}
	}

	delete []subMeshEdge;

	//////////////////////////////////////////////////////////////////////////
	//cout<<"before splitting, ancestors:"<<endl;
	//for( int i = 0; i < ssfacenum*2; i +=2 )
	//{
	//	if( ancestorlist[ i ].size() == 0 )
	//	{
	//		if( ancestorlist[ i + 1].size() == 0 )
	//			continue;
	//	}
	//	intset::iterator iter = ancestorlist[ i ].begin();
	////	intset::iterator iter2 = ancestorlist[ i + 1].begin();
	//	while( iter != ancestorlist[ i ].end() )
	//	{
	//		cout<<*iter<<" ";
	//		iter++;
	//	}
	//	cout<<endl;
	//	iter = ancestorlist[ i+1 ].begin();
	//	while( iter != ancestorlist[ i + 1 ].end() )
	//	{
	//		cout<<*iter <<" ";
	//		iter ++;
	//	}
	//	cout<<endl;
	//	cout<<"\n";
	//}
	//cout<<"=============================\n";
	//cout<<"after splitting ancestors:"<<endl;
	//for( int i= 0; i < ssfacenum*2; i += 2 )
	//{
	//	if( ancestorlist_split[ i ].size() == 0 )
	//	{
	//		if( ancestorlist_split[ i + 1].size() == 0)
	//			continue;
	//	}
	//	intset::iterator iter = ancestorlist_split[ i ].begin();
	//	while( iter != ancestorlist_split[ i ].end() )
	//	{
	//		cout<<*iter <<" ";
	//		iter++;
	//	//	cout<<endl;
	//	}
	//	cout<<endl;
	//	iter = ancestorlist_split[ i + 1].begin();
	//	while( iter != ancestorlist_split[ i + 1].end() )
	//	{
	//		cout<<*iter <<" ";
	//		iter ++;
	//	}
	//	cout<<endl<<endl;
	//}


	//////////////////////////////////////////////////////////////////////////
	//cout the faces registered on the contour edges 
	/*for( int i = 0; i < ssfacenum; i ++ )
	{
		int ctrnum = sfaceregface[ i ].size()/2;
		if( ctrnum == 0 )
			continue;
		intset::iterator iter = ancestorlist_split[2* i ].begin();
		for( int j = 0; j < ctrnum; j ++ )
		{
			cout<<"("<<*iter<<":"<<sfaceregface[ i ][ 2*j ] .size()<<","<<
				sfaceregface[ i ][ 2*j +1 ].size()<<")  ";
			iter++;
		}
		cout<<endl;
	}*/
	//////////////////////////////////////////////////////////////////////////

	//for( int i = 0; i < ssfacenum; i ++)
	//{
	//	ancestorlist[ i].clear();
	//	ancestorlist_split[ i ].clear();
	//}
	//ancestorlist_split.clear();
	//ancestorlist.clear();
	//////////////////////////////////////////////////////////////////////////


	//stitch all the subspaces together
	stitchMesh(subMeshVer,subMeshFace);

	clock_t   endt   =   clock();
	cout<<"time difference is:"<<endt - start<<endl;
//	struct __timeb64 timebuffer;
//	char *timeline;
	_ftime64( &timebuffer );
	timeline = _ctime64( & ( timebuffer.time ) );
	printf( "The time is %.19s.%hu %s", timeline, timebuffer.millitm, &timeline[20] );


//	FaceGenerator::writeSubMeshOut( mver, mface, 99);

	//clear all the allocated space
	//for( int i = 0; i < ssspacenum; i ++)
	//{
	//	//subMeshVer[ i ].clear();
	////	subMeshEdge[ i ].clear();
	//	subMeshFace[ i ].clear();
	//}
	////delete []subMeshVer;
	////delete []subMeshEdge;
	//delete []subMeshFace;
	/*subMeshFace = NULL;
	subMeshVer = NULL;*/

	//clear the registration information for subspace vertex, edge and face
	//sverreg
	for( int i = 0; i < ssvernum; i ++)
		sverreg[ i ].clear();
	delete []sverreg;
	sverreg = NULL;

	//sedgereg
	for( int i= 0; i < ssedgenum; i ++)
	{
		for(unsigned int j = 0; j < sedgereg[ i ].size(); j ++)
			sedgereg[ i ][ j ].clear();
		sedgereg[ i ].clear();
	}
	delete []sedgereg;
	sedgereg = NULL;

	//sfacectrei, sfacespaci, sfaceregface, sfaceregver
	for( int i= 0; i < ssfacenum; i ++)
	{
		sfacectrei[ i ].clear();
		
	//	for( int j = 0; j < sfaceregface[ i ].size();  j ++ )
    //       sfaceregface[ i ][ j ].clear();
	//	sfaceregface[ i ].clear();
		sfaceregver[ i ].clear();
	}	
	delete []sfacectrei;
	delete []sfacespaci;
	//delete []sfaceregface;
	delete []sfaceregver;
	sfacectrei = NULL;
	sfacespaci = NULL;
	//sfaceregface = NULL;
	sfaceregver = NULL;

	//set the mesh
	//clear
	if( mesh!= NULL )
		delete mesh;

	//set
	mesh = new Mesh( mver, mface, ctrmedge, 
		center,  unitlen, PROCSIZE );
	
	////kw: get constrained vertices
	//set<int> ConstrVer;
	//for (unsigned int i=0;i<ctrmedge.size();i++)
	//{
	//	ConstrVer.insert(ctrmedge.at(i));
	//}
	//for (set<int>::iterator SetIter=ConstrVer.begin();SetIter!=ConstrVer.end();SetIter++)
	//{
	//	vecTestPoint.push_back(Point_3(mver.at(3*(*SetIter))*DIM/PROCSIZE,mver.at(3*(*SetIter)+1)*DIM/PROCSIZE,
	//		mver.at(3*(*SetIter)+2)*DIM/PROCSIZE));
	//}

	mesh->setGLParam( width, height, nearplane, farplane);

	//clear
	mver.clear();
	mface.clear();
	ctrmedge.clear();
}

//void Ctr2SufManager::clearSufMesh()
//{
//	mver.clear();
//	mface.clear();
//}

//render
//void Ctr2SufManager::renderSufMesh()
//{
//	//render mode
//	if( mesh != NULL )
//		mesh->render();    
//}