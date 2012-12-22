#include "../SubspaceContour/sscontour.h"

//////////////////////////////////////////////////////////////////////////
bool debugon = false;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//int tempface = -1;
//////////////////////////////////////////////////////////////////////////
//sort the seams on one sheet in round way, either clockwise or counterclockwise
void SSContour::sortSheetSeams(
int* maseam,int subfacenum,
MapArraySR& doubleface2sheet,int*& seamonsheetnum,int**& seamonsheet
									  )
{
	int keys[ 2];
	int sheeti;
	int seami;
	//////////////////////////////////////////////////////////////////////////
	/*for( int i = 0; i < subfacenum; i ++)
	{
		keys[ 0 ] = i;
		for(int j = i + 1; j < subfacenum; j ++)
		{
			keys[ 1 ] = j;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti );
			cout<<keys[ 0 ]<<","<<keys[1]<<", sheeti:"<<sheeti<<"numbers of the seam:"<<seamonsheetnum[ sheeti ]<<endl;
			int num = seamonsheetnum[ sheeti ];
			for( int k = 0; k < num; k++)
				cout<<seamonsheet[ sheeti ][ k ]<<" ";
			cout<<endl;
			cout<<"-------------------"<<endl;
		}
	}*/
	//////////////////////////////////////////////////////////////////////////
	
	for( int i = 0; i < subfacenum; i ++)
	{
		keys[ 0 ] = i;
		for( int j = i + 1; j <subfacenum; j++ )
		{
			keys[ 1 ] = j;
			doubleface2sheet.getKeyVal( keys, 2, true, sheeti);
			if( seamonsheetnum[ sheeti ] == 0 )
				continue;
			//////////////////////////////////////////////////////////////////////////
			//cout<<"--"<<keys[0]<<","<<keys[1]<<"sheet:"<<sheeti<<":"<<seamonsheetnum[ sheeti ]<<endl;
			//////////////////////////////////////////////////////////////////////////
			//sort all the seams in round way
			seami = seamonsheet[ sheeti ][ 0 ];
			int curjpt = maseam [ seami * 2 + 1];
			
			for(int k1 = 1; k1 < seamonsheetnum[ sheeti ] - 1 ; k1 ++)
			{
				for( int k2 = k1; k2 < seamonsheetnum[ sheeti ]; k2++)
				{
					seami = seamonsheet[ sheeti ][ k2 ];
					if( maseam[ 2*seami] == curjpt )
					{
						if( k2 != k1 )
						{
							int t = seamonsheet[ sheeti ][ k1 ];
							seamonsheet[ sheeti ][ k1 ] = seamonsheet[ sheeti ][ k2 ];
							seamonsheet[ sheeti ][ k2 ] = t;                            
						}
                        curjpt = maseam[ 2*seami + 1 ];
						break;
					}
					else if( maseam[ 2*seami + 1] == curjpt )
					{
						if( k2 != k1 )
						{
							int t = seamonsheet[ sheeti ][ k1 ];
							seamonsheet[ sheeti ][ k1 ] = seamonsheet[ sheeti ][ k2 ];
							seamonsheet[ sheeti ][ k2 ] = t;                            
						}
						curjpt = maseam[ 2*seami ];
						break;
					}
				}
			}
		}
	}
}

//gather the medial axis junction points, seams and sheets corresponding to this face
void SSContour::gatherMA2Face(
  int spacei, int facei,
  int subfacenum,int* maseam,MapArraySR& doubleface2sheet,int* seamonsheetnum,int** seamonsheet,
  //result
  int*& fjpts,int& fjptnum,
  int*& fseams, int& fseamnum,
  int*& fsheetis, int& fsheetinum)
{
	intvector fsheeti;
	intset fseam;
	intset fjpt;
	int sheeti;
	int keys[ 2 ];
	keys[ 0 ] = facei;
	//seams corresponding to this face
	for( int i = 0; i < subfacenum; i ++)
	{
		if( i == facei )continue;
		keys[ 1 ] = i;
		doubleface2sheet.getKeyVal(keys, 2, false, sheeti);
		if( seamonsheetnum[ sheeti ] == 0 )
			continue;
		fsheeti.push_back( sheeti );
		for( int j = 0; j < seamonsheetnum[ sheeti ]; j ++)
			fseam.insert( seamonsheet[ sheeti ][ j ]);
	}
	//junction points
	intset::iterator iter;
	iter = fseam.begin();
	while( iter != fseam.end() )
	{
		int seami = *iter;
		fjpt.insert( maseam[ 2*seami ]);
		fjpt.insert( maseam[ 2*seami + 1 ]);
		iter++;
	}
	fjptnum = fjpt.size();
	fjpts = new int[  fjptnum ];
	iter = fjpt.begin();
	for(unsigned int i = 0; i <fjpt.size(); i++)
	{
		fjpts[ i ] = *iter;
		iter ++;
	}
	fsheetinum = fsheeti.size();
	fsheetis = new int[fsheetinum ] ;
	for(unsigned int i = 0; i < fsheeti.size(); i ++)
	{
		fsheetis[ i ] = fsheeti[ i ];
	}
	fseamnum = fseam.size();
	fseams = new int[ fseamnum ];
	iter = fseam.begin();
	for(unsigned int i = 0; i <fseam.size(); i ++)
	{
		fseams[ i ] = *iter;
		iter++;
	}
	fseam.clear();
	fjpt.clear();
	fsheeti.clear();
}

void SSContour::projectionOfJpts(
int* fjpt, int* fjptmark, int fjptnum,
float fparam[ 4 ], float* majpt,float* subver,
//result
float* projpt			 )
{
	for( int i = 0; i < fjptnum; i ++)
	{
		if( fjptmark[ i ] != -1 )
		{
			//////////////////////////////////////////////////////////////////////////
			/*cout<<"subver "<<fjptmark[ i ]<<":";
			for( int k = 0; k < 3; k ++)
				cout<<subver[ 3*fjptmark[ i ] + k ]<<" ";
			cout<<endl;*/
			//////////////////////////////////////////////////////////////////////////
			memcpy( projpt+3*i, subver + 3*fjptmark[ i ], sizeof( float)*3 );
			//////////////////////////////////////////////////////////////////////////
			/*cout<<"after copy, the projection is"<<":";
			for( int k = 0; k < 3; k ++)
				cout<<projpt[ 3*i + k]<<" ";
			cout<<endl;*/
			//////////////////////////////////////////////////////////////////////////
			continue;
		}
		//compute the intersection point
		projectionPtOnPlane( majpt + 3*fjpt[ i ], fparam, true, projpt + 3*i );
	}
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"After returned!"<<endl;
	for( int i = 0; i < fjptnum; i ++)
		cout<<projpt[ 3*i ]<<"\t"<<projpt[ 3*i + 1] <<"\t"<<projpt[ 3*i + 2]<<endl;*/
	//////////////////////////////////////////////////////////////////////////
}
//when mark the property of the vertex, 
//if ver_jpt, set the value as the position of the junction point in fjpt
//if ver_seam, set the value as the position of the seam in fseam
//if ver_sheet, set the vlaue as the position of the sheet in fsheeti.
//so later on, should reset them as the real junction point index, seam index and sheet index!
void SSContour::getVerProp(
SUBSPACECTRVER& ctrver,
int* fjpt, int* fjptmark, float* projpt, int fjptnum,
int* fseam, int fseamnum,
int* fsheeti, int fsheetinum,
int subedgenum,
float*majpt, int* maseam, int maseamnum, int** maseamonsheet, int* maseamonsheetnum
						   )
{
	//step1. go through all the projection of junction points which are not subspace vertices 
	ctrver.val = findOverLapPoint( projpt, fjptmark, fjptnum, ctrver.pos, -1 );
	if ( ctrver.val != -1 )
	{
		ctrver.type = VER_JPT;
		return;
	}

	//step2. go through all the sheet, when checking every sheet, check if it lies on seam. 
	//in order to save time ,not handle them separately
	const int NOT_CAL = 1;			//the direction of Cross[p0seamv1, p0seamv2] is not computed
	const int CAL_ON_LINE_OUT = 2;	//the direction is computed, though p0 lies on the same line with the seam, but outside the seam
	const int CAL_NOT_ON_LINE = 3;	//the dreiction is computed, and it is not on the same line with the seam
//	const int NO_NEED_COMPUTE = 4;		//the seam is actually a subspace edge, so no need to compute the cross product
	const int EDGE_SEAM_NOT_CAL = 4;	//these seams are actually the edges of the subspace
	const int EDGE_SEAM_CAL = 5;	//these are the seams
	float* ptseamdir = new float[ 3*fseamnum ];
	int* seammark = new int[ fseamnum ];
	int* jpti2;
	int jpti2_fjpt[ 2 ];
	//bool onsameline;
	for( int i = 0; i < fseamnum; i ++)
	{
		if( fseam[ i ] >= maseamnum - subedgenum )
			seammark[ i ] = EDGE_SEAM_NOT_CAL;
		else
			seammark[ i ] = NOT_CAL;
	}
	bool first;
	int lastseami;
	int lastseami_fseam;
	for( int i = 0; i< fsheetinum; i++)
	{
		int sheeti = fsheeti[ i ];

		//////////////////////////////////////////////////////////////////////////
		if( sheeti != 14 )
			debugon = false;
		//check the property when the sheeti = 14 is checked.
		//////////////////////////////////////////////////////////////////////////

		if( maseamonsheetnum[ sheeti ] == 0)
			continue;
		
		first = true;

		//int lastjpt = maseam[2*maseamonsheet[ sheeti ][ 0 ] + 1];	//to see which direction current seam should be
		int j = 0;
		for( j = 0; j < maseamonsheetnum[ sheeti ]; j ++)
		{
			int seami = maseamonsheet[ sheeti ] [ j ];			
			jpti2 = maseam + 2*seami;
			jpti2_fjpt[ 0 ] = findPosInIncSortedArray( fjpt, fjptnum, jpti2[ 0 ]);
			jpti2_fjpt[ 1 ] = findPosInIncSortedArray( fjpt, fjptnum, jpti2[ 1 ]);
			//check if they are on the same line
			int seami_fseam = findPosInIncSortedArray( fseam, fseamnum, seami );
			
			//on the same line with the seam, but out of the seam
			//can't on this seam and can't be in the sheet
			if( seammark[ seami_fseam ] == CAL_ON_LINE_OUT )	
				break;
			//not calculated yet
			if( (seammark[ seami_fseam ] == NOT_CAL) || (seammark[ seami_fseam ] == EDGE_SEAM_NOT_CAL ))
			{
				//compute the crossproduct of p0v1 and p0v2
				float vec1[ 3 ];
				float vec2[ 3 ];
				/*if( dirright )
				{
				MyMath::getVec( ctrver.pos, majpt + 3*jpti2[ 0 ], vec1);
				MyMath::getVec( ctrver.pos, majpt + 3*jpti2[ 1 ], vec2  );					
				}
				else
				{
					MyMath::getVec( ctrver.pos, majpt + 3*jpti2[ 0 ], vec2);
					MyMath::getVec( ctrver.pos, majpt + 3*jpti2[ 1 ], vec1  );	
				}*/
				MyMath::getVec( ctrver.pos, projpt + 3*jpti2_fjpt[ 0 ], vec1);
				MyMath::getVec( ctrver.pos, projpt + 3*jpti2_fjpt[ 1 ], vec2  );
			//	MyMath::normalize( vec1 );	//safe, because it is not close enough to any of the two junction points!
			//	MyMath::normalize( vec2 );
				MyMath::crossProductNotNorm( vec1, vec2, ptseamdir + 3*seami_fseam );
				if(seammark[ seami_fseam ] == NOT_CAL)	//this is a non-edge seam
				{
					//if(MyMath::vectorlen( ptseamdir + 3*seami_fseam)<TOLERANCE_ANGLE )
					if( MyMath::vectorlen( ptseamdir + 3*seami_fseam)/MyMath::vectorlen(
						projpt+3*jpti2_fjpt[0], projpt+3*jpti2_fjpt[1]) < TOLERANCE_THREE )
						//area of the triangle/the base length=distance from the point to this seam!
					{
						//check if it lies on the seam or outside of the seam
						float t = getParamOnSeg( projpt + 3*jpti2_fjpt[ 0 ], projpt + 3*jpti2_fjpt[ 1 ], ctrver.pos );
						if( t > 0 && t < 1)	//the junction point is on this seam!!
						{
							//////////////////////////////////////////////////////////////////////////
							//FILE* fout = fopen("out.txt","a");
							//fprintf( fout, "returning from on one seam:%d\n", seami+1 );
							//for( int i = 0; i < fseamnum; i ++)
							//{
							//	if( seammark[ i ] == EDGE_SEAM_NOT_CAL )
							//		fprintf(fout, "%d:edge seam, not cal!\n",fseam[ i ]+1);
							//		//cout<<fseam[ i ]<<": edge seam, not cal!"<<endl;
							//	else if( seammark[ i ] == NOT_CAL )
							//		fprintf(fout, "%d: normal edge, not cal!\n",fseam[ i ]+1);
							//		//cout<<fseam[ i ]<<": normal edge, not cal!"<<endl;
							//	else if( seammark[ i ] == CAL_NOT_ON_LINE )
							//		fprintf(fout,"%d:%f,%f,%f NOT ON LINE!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
							//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
							//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//	//	" "<<ptseamdir[ 3*i + 2]<<" NOT ON LINE!"<<endl;
							//	else if( seammark[ i ] == CAL_ON_LINE_OUT )
							//		fprintf(fout,"%d:%f,%f,%f ON LINE BUT OUT!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
							//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
							//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//	//	" "<<ptseamdir[ 3*i + 2]<<" ON LINE BUT OUT!"<<endl;
							//	else if ( seammark [ i ] == EDGE_SEAM_CAL )
							//		fprintf(fout,"%d:%f,%f,%f EDGESEAMCAL!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
							//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
							//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//	//	" "<<ptseamdir[ 3*i + 2]<<" EDGE SEAM!"<<endl;
							//}
							///*cout<<"returning from on seam:"<<seami<<endl;
							//for( int i = 0; i < fseamnum; i ++)
							//{
							//	if( seammark[ i ] == EDGE_SEAM_NOT_CAL )
							//		cout<<fseam[ i ]<<": edge seam, not cal!"<<endl;
							//	else if( seammark[ i ] == NOT_CAL )
							//		cout<<fseam[ i ]<<": normal edge, not cal!"<<endl;
							//	else if( seammark[ i ] == CAL_NOT_ON_LINE )
							//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//		" "<<ptseamdir[ 3*i + 2]<<" NOT ON LINE!"<<endl;
							//	else if( seammark[ i ] == CAL_ON_LINE_OUT )
							//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//		" "<<ptseamdir[ 3*i + 2]<<" ON LINE BUT OUT!"<<endl;
							//	else if ( seammark [ i ] == EDGE_SEAM_CAL )
							//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
							//		" "<<ptseamdir[ 3*i + 2]<<" EDGE SEAM!"<<endl;
							//}*/
							//fclose(fout);
							//////////////////////////////////////////////////////////////////////////
							ctrver.type = VER_SEAM;
							ctrver.val = seami_fseam;
							delete ptseamdir;
							delete seammark;
							jpti2 = NULL;
							return;
						}
						//the point is on the seam , but out!, no need to process current sheet any more!
						seammark[ seami_fseam ] = CAL_ON_LINE_OUT;
						break;
					}
					//the point is not on the seam, but computed
					seammark[ seami_fseam ] = CAL_NOT_ON_LINE;
				}
				else	//is edge seam
				{
					seammark[ seami_fseam ] = EDGE_SEAM_CAL;	//has been calculated!
				}
			}
			//when come here, must have already been computed, if not, calculation is done in the previous step!
			if( first )
			{
				first = false;
				lastseami = seami;
				lastseami_fseam = seami_fseam;
			}
			else //compute the dot product with the last dir and current dir!
			{
				//the two directions are opposite
				//the contour vertex is not possible to in this sheet.
				float tdotp = MyMath::dotProduct( ptseamdir + 3*lastseami_fseam, ptseamdir + 3*seami_fseam );
				if(  ((tdotp > 0)&&( (maseam[2*lastseami] == jpti2[ 0 ]) ||( maseam[2*lastseami + 1] == jpti2[ 1 ]))) ||
					((tdotp < 0)&&((maseam[2*lastseami] == jpti2[ 1 ])||(maseam[2*lastseami + 1] == jpti2[ 0 ]))))
						break;				
				lastseami = seami;
				lastseami_fseam = seami_fseam;
			}
		}
		//not break out, but after all have been processed, so this vertex must lie in the sheet!
		if( j == maseamonsheetnum[ sheeti ])
		{
			//////////////////////////////////////////////////////////////////////////
			//FILE* fout = fopen("out.txt","a");
			////fprintf( fout, "returning from on one seam:%d\n", seami );
			//for( int i = 0; i < fseamnum; i ++)
			//{
			//	if( seammark[ i ] == EDGE_SEAM_NOT_CAL )
			//		fprintf(fout, "%d:edge seam, not cal!\n",fseam[ i ]+1);
			//	//cout<<fseam[ i ]<<": edge seam, not cal!"<<endl;
			//	else if( seammark[ i ] == NOT_CAL )
			//		fprintf(fout, "%d: normal edge, not cal!\n",fseam[ i ]+1);
			//	//cout<<fseam[ i ]<<": normal edge, not cal!"<<endl;
			//	else if( seammark[ i ] == CAL_NOT_ON_LINE )
			//		fprintf(fout,"%d:%f,%f,%f NOT ON LINE!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
			//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
			//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//	//	" "<<ptseamdir[ 3*i + 2]<<" NOT ON LINE!"<<endl;
			//	else if( seammark[ i ] == CAL_ON_LINE_OUT )
			//		fprintf(fout,"%d:%f,%f,%f ON LINE BUT OUT!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
			//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
			//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//	//	" "<<ptseamdir[ 3*i + 2]<<" ON LINE BUT OUT!"<<endl;
			//	else if ( seammark [ i ] == EDGE_SEAM_CAL )
			//		fprintf(fout,"%d:%f,%f,%f EDGESEAMCAL!" ,fseam[ i ]+1,ptseamdir[ 3*i ],
			//		ptseamdir[ 3*i + 1],ptseamdir[ 3*i + 2]);
			//	//	cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//	//	" "<<ptseamdir[ 3*i + 2]<<" EDGE SEAM!"<<endl;
			//}
			//fclose( fout );
			///*for( int i = 0; i < fseamnum; i ++)
			//{
			//	if( seammark[ i ] == EDGE_SEAM_NOT_CAL )
			//		cout<<fseam[ i ]<<": edge seam, not cal!"<<endl;
			//	else if( seammark[ i ] == NOT_CAL )
			//		cout<<fseam[ i ]<<": normal edge, not cal!"<<endl;
			//	else if( seammark[ i ] == CAL_NOT_ON_LINE )
			//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//		" "<<ptseamdir[ 3*i + 2]<<" NOT ON LINE!"<<endl;
			//	else if( seammark[ i ] == CAL_ON_LINE_OUT )
			//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//		" "<<ptseamdir[ 3*i + 2]<<" ON LINE BUT OUT!"<<endl;
			//	else if ( seammark [ i ] == EDGE_SEAM_CAL )
			//		cout<<fseam[ i ]<<":"<<ptseamdir[ 3*i ]<<" "<<ptseamdir[ 3*i + 1]<<
			//		" "<<ptseamdir[ 3*i + 2]<<" EDGE SEAM!"<<endl;
			//}*/
			//////////////////////////////////////////////////////////////////////////

			ctrver.type = VER_SHEET;
			ctrver.val = i;	//the ith sheet in fsheeti
			delete []ptseamdir;
			delete []seammark;
			jpti2 = NULL;

			return;
		}
	}
	//actually these two lines should never be reached!
	delete []ptseamdir;
	delete []seammark;
	jpti2 = NULL;
};

void SSContour::markVer(
SSPCTRVERVEC& sspctrver_vec,
int* fjpt, int* fjptmark, float* projpt, int fjptnum,
int* fseam, int fseamnum,
int* fsheeti, int fsheetinum,
int subvernum,int subedgenum,
float*majpt, int* maseam, int maseamnum, int** maseamonsheet, int* maseamonsheetnum,
int* ver2jpt, int* ver2wver, int* edge2wedge)
{
	//cout<<"subvernum:"<<subvernum<<endl;
	for(unsigned int i = 0; i < sspctrver_vec.size(); i ++)
	{
		//////////////////////////////////////////////////////////////////////////
		//cout<<i<<": type "<<sspctrver_vec[ i ].type<<" val:"<<sspctrver_vec[ i ].val<<endl;
		/*FILE* fout = fopen( "out.txt","a");
		fprintf ( fout, "%d :type %d val: %d \n",i+1,sspctrver_vec[ i ].type,sspctrver_vec[ i ].val);
		fclose( fout );*/
		//////////////////////////////////////////////////////////////////////////
		switch( sspctrver_vec[ i ].type )
		{
		case CTRTYPE_SUBVER:
			{
				int veri = sspctrver_vec[ i ].val;		//vertex index of all the vertices of all subspace
				veri = findPosInArray( ver2wver,subvernum, veri );	//index of the vertex in current subspace vertices
				if( veri == -1 )
				{
					cout<<"ERROR! Unable to find the val is CTRTYPE_SUBVER vertex in the subver!"<<endl;
					continue;
				}
				sspctrver_vec[ i ].type = VER_JPT;
				veri = findPosInIncSortedArray( fjpt, fjptnum, ver2jpt[ veri ]);	//the junction point position in fjpt
				sspctrver_vec[ i ].val = veri;
			}
			break;
		case CTRTYPE_SUBEDGE:
			{
				//////////////////////////////////////////////////////////////////////////
			//	cout<<"on edge:"<< sspctrver_vec[ i ].val<<" ";
					//////////////////////////////////////////////////////////////////////////
				int edgei = sspctrver_vec[ i ].val;
				int pos = findPosInArray(  edge2wedge, subedgenum, edgei );
				if( pos == -1 )
				{
					cout<<"ERROR! Unable to find the val is CTRTYPE_SUBEDGE vetex in ths subedge!"<<endl;
					continue;
				}
				sspctrver_vec[ i ].type = VER_SEAM;
				//because when computing medial axis, I put all the edges of the subspace at the end of the seams array, and put
				//them in the sequence of their order in subedge. that's why there correpondence is as follows
				int seami = maseamnum - subedgenum + pos;	
				//////////////////////////////////////////////////////////////////////////
				/*cout<<"seam index:"<<seami<<endl;
				cout<<"fseam:"<<endl;
				for( int tk = 0; tk < fseamnum; tk ++)
				{
					cout<<fseam[ tk ]<<" ";
				}
				cout<<endl;*/
				//////////////////////////////////////////////////////////////////////////
				seami = findPosInIncSortedArray( fseam, fseamnum, seami );	//the position of the seam in fseam
				//////////////////////////////////////////////////////////////////////////
				//cout<<"position in fseam:"<<seami<<endl;
				//////////////////////////////////////////////////////////////////////////

				sspctrver_vec[ i ].val =seami;
				
			}			
			break;
		case CTRTYPE_SUBFACE:
			{
				//////////////////////////////////////////////////////////////////////////
				if( i != 14 )
					debugon = false;	//turn it off!
				//////////////////////////////////////////////////////////////////////////

				getVerProp( sspctrver_vec[ i ], fjpt, fjptmark, projpt, fjptnum,fseam, fseamnum,fsheeti, fsheetinum,
					subedgenum,	majpt, maseam, maseamnum, maseamonsheet, maseamonsheetnum);
				//////////////////////////////////////////////////////////////////////////
				if(  debugon  )
				{
					if( i == 14 )
					{
						cout<<"type:"<<sspctrver_vec[ i ].type <<"val:"<<sspctrver_vec[ i ].val<<endl;
					}					
				}
				//////////////////////////////////////////////////////////////////////////
			}
			break;
		default:
			cout<<"ERROR! The type of the contour vertex is not marked propertly, unknown!"<<endl;
			break;
		}
		
	}
}
//if the property of the edge can be decided return true, otherwise, return false
bool SSContour::getEdgeProp(
SSPCTRVERVEC& sspctrver_vec,
SUBSPACECTREDGE& ctredge,
int* fjpt,
int* fseam,
int* fsheet,
int* jpt2seamnum,
int** jpt2seam,
int* jpt2sheetnum,
int** jpt2sheet,
int* seam2sheetnum,
int** seam2sheet )
{
	int type2[ 2 ] = {sspctrver_vec[ ctredge.veris[ 0 ] ].type,sspctrver_vec[ ctredge.veris[ 1 ] ].type};
	int mul =  type2[ 0 ] * type2[ 1 ];
	int val2[ 2 ] = { sspctrver_vec[ ctredge.veris[0] ].val,  sspctrver_vec[ ctredge.veris[1] ].val};
	
	int comseam;
	int comsheet;
	switch( mul )
	{
	case 1: //1*1 jpt jpt
		comseam = commonOfTwoIncSortedArray_PosArray(jpt2seam[ val2[ 0 ]], jpt2seamnum[ val2[ 0 ]],
			jpt2seam[val2[ 1]], jpt2seamnum[ val2[ 1 ]]	);
		if( comseam != -1 )	//exists!
		{
			ctredge.type = EDGE_SEAM;
			ctredge.val = fseam[ comseam ];
			return true;
		}
		//not exist
		comsheet = commonOfTwoIncSortedArray_PosArray( jpt2sheet[ val2[ 0 ]], jpt2sheetnum[ val2[ 0 ]],
			jpt2sheet[ val2[ 1 ]], jpt2sheetnum[ val2[ 1 ]]	);
		if( comsheet!=-1) //they lie in the same sheet!
		{
			ctredge.type = EDGE_SHEET;
			ctredge.val = fsheet[ comsheet ];
			return true;
		}
		ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
		return false;	//not able to set it!
	case 2: //1*2 jpt seam
		if( type2[ 0 ] = VER_JPT )
		{
			comseam = findPosInIncSortedArray( jpt2seam[ val2[ 0 ]], jpt2seamnum[ val2[ 0 ]],val2[ 1 ] );
			if( comseam != -1 )
				comseam = val2[ 1 ];
		}
		else
		{
			comseam =  findPosInIncSortedArray( jpt2seam[ val2[ 1 ]], jpt2seamnum[ val2[ 1 ]],val2[ 0 ] );
			if( comseam != -1 )
				comseam = val2[ 0 ];
		}
		if( comseam != -1 )//lie on this seam
		{
			ctredge.type = EDGE_SEAM;
			ctredge.val = fseam[ comseam ];
			return true;
		}
		if( type2[ 0 ] == VER_JPT)
		{
			comsheet = commonOfTwoIncSortedArray_PosArray( jpt2sheet[ val2[ 0 ]], jpt2sheetnum[ val2[ 0 ]],
				seam2sheet[ val2[ 1 ]], seam2sheetnum[ val2[ 1 ]]);
		}
		else
			comsheet = commonOfTwoIncSortedArray_PosArray( jpt2sheet[ val2[ 1 ]], jpt2sheetnum[ val2[ 1 ]],
			seam2sheet[ val2[ 0 ]], seam2sheetnum[ val2[ 0 ]]);
		if( comsheet != -1 )
		{
			ctredge.type = EDGE_SHEET;
			ctredge.val = fsheet[ comsheet ];
			return true;
		}
		ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
		return false;
	case 3: //1*3 jpt sheet
		if( type2[ 0 ] == VER_JPT)
		{
			comsheet = findPosInIncSortedArray( jpt2sheet[ val2[ 0 ] ], jpt2sheetnum[ val2[ 0 ]], val2[ 1 ]);
			if( comsheet != -1 )
				comsheet = val2[ 1 ];
		}
		else
		{
			comsheet = findPosInIncSortedArray( jpt2sheet[ val2[ 1 ] ], jpt2sheetnum[ val2[ 1 ]], val2[ 0 ]);
			if( comsheet != -1 )
				comsheet = val2[ 0 ];
		}
		if( comsheet != -1 )
		{
			ctredge.type = EDGE_SHEET;
			ctredge.val = fsheet[ comsheet ];
			return true;
		}
		ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
		return false;
	case 4: //2*2 seam seam
		if( val2[ 0 ] == val2[ 1 ])
		{
			ctredge.type = EDGE_SEAM;
			ctredge.val = fseam[ val2[ 0 ]];
			return true;
		}
		comsheet = commonOfTwoIncSortedArray_PosArray( seam2sheet[ val2[ 0 ]], seam2sheetnum[ val2[ 0 ]],
			seam2sheet[ val2[ 1 ]], seam2sheetnum[ val2[ 1 ]]);
		if( comsheet != -1 )
		{
			ctredge.type = EDGE_SHEET;
			ctredge.val = fsheet[ comsheet ];
			return true;
		}
		ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
		return false;
	case 6: //2*3 sheet seam
		if( type2[ 0 ] == VER_SEAM )
		{
			comsheet = findPosInIncSortedArray( seam2sheet[ val2[ 0 ]], seam2sheetnum[ val2[ 0 ]], val2[ 1 ] );
			if( comsheet != -1 )
				comsheet = val2[ 1 ];
		}
		else
		{
			comsheet = findPosInIncSortedArray( seam2sheet[ val2[ 1 ]], seam2sheetnum[ val2[ 1 ]], val2[ 0 ] );
			if( comsheet != -1 )
				comsheet = val2[ 0 ];
		}
		if( comsheet != -1 )
		{
			ctredge.type = EDGE_SHEET;
			ctredge.val = fsheet[ comsheet ] ;
			return true;
		}
		ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
		return false;
	case 9: //3*3 sheet
		if( val2[ 0 ] != val2[ 1 ])
		{
			ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
			return false;
		}
		ctredge.type = EDGE_SHEET;
		ctredge.val = fsheet[ val2[ 0 ]];
		return true;
	default:	//impossible
		break;
	}
	//actually won't be reached!
	ctredge.type = -1;	//not known--------not needed actually, for debug reason, i put here
	return false;	
}
void SSContour::markEdge(SSPCTREDGEVEC& sspctredge_vec, SSPCTRVERVEC& sspctrver_vec,
int subedgenum,int maseamnum,int* edge2wedge,int* fjpt,int* fseam,int* fsheet,int* jpt2seamnum,
int** jpt2seam,int* jpt2sheetnum,int** jpt2sheet,int* seam2sheetnum,int** seam2sheet,intvector& needsplitedge)
{
	for(unsigned int i = 0; i < sspctredge_vec.size(); i ++)
	{
		switch( sspctredge_vec[ i ].type )
		{
		case CTRTYPE_SUBEDGE:
			{
				int edgei = sspctredge_vec[ i ].val;
				edgei = findPosInArray(  edge2wedge, subedgenum, edgei );
				if( edgei == -1 )
				{
					cout<<"ERROR! Unable to find the val is CTRTYPE_SUBEDGE vetex in ths subedge!"<<endl;
					continue;
				}
				sspctredge_vec[ i ].type = VER_SEAM;
				//because when computing medial axis, I put all the edges of the subspace at the end of the seams array, and put
				//them in the sequence of their order in subedge. that's why there correpondence is as follows
				int seami = maseamnum - subedgenum + edgei;	
				sspctredge_vec[ i ].val =seami;
			}			
			break;
		case CTRTYPE_SUBFACE:
			if(!getEdgeProp(sspctrver_vec,sspctredge_vec[ i ],fjpt,fseam,fsheet,
				jpt2seamnum,jpt2seam,jpt2sheetnum,jpt2sheet,seam2sheetnum,seam2sheet ))
				needsplitedge.push_back( i );
			break;
		default:
			cout<<"ERROR! Unknown edge type!"<<endl;
			break;
		}
	}
}
void SSContour::processSplitEdge(
 intvector& needsplitedge,
SSPCTRVERVEC& sspctrver_vec,
SSPCTREDGEVEC& sspctredge_vec,
int* fjpt, float* projpt, int fjptnum,
int* fseam, int fseamnum,
int* fsheet,int subedgenum,
int* maseam, int maseamnum,
int* jpt2seamnum,
int** jpt2seam,
int* jpt2sheetnum,
int** jpt2sheet,
int* seam2sheetnum ,
int** seam2sheet,
float param[ 4 ])
{
	bool onsameline;
	int* ver2;
	int type2[ 2 ];
	int val2[ 2 ];
	int* jpt2;
	float t2[ 2 ];
//	int sjpt2[ 2 ];
	int jpt2_fjpt[ 2 ];
	int sjpt2_fjpt[ 2 ];
	while( !needsplitedge.empty())
	{
		int curedge = needsplitedge.back();
		needsplitedge.pop_back();

		ver2 = sspctredge_vec[ curedge ].veris;
		type2[ 0 ] = sspctrver_vec[ ver2[ 0 ]].type;
		type2[ 1 ] = sspctrver_vec[ ver2[ 1 ]].type;
		val2[ 0 ] = sspctrver_vec[ ver2[ 0 ]].val;
		val2[ 1 ] = sspctrver_vec[ ver2[ 1 ]].val;

		//////////////////////////////////////////////////////////////////////////
		/*if( tempface == 2 )
		{
			cout<<"the two vertices of the edge are:"<<ver2[ 0 ]<<","<<ver2[1]<<endl;
			cout<<"(type val):"<<type2[ 0 ]<<","<<val2[ 0 ]<<" |  "
				<<type2[1]<<","<<val2[1]<<endl;
		}*/

		//////////////////////////////////////////////////////////////////////////

		float bbox[ 6 ] = {
			min( sspctrver_vec[ ver2[ 0 ]].pos[ 0 ],sspctrver_vec[ ver2[ 1 ]].pos[ 0 ] ),
			min( sspctrver_vec[ ver2[ 0 ]].pos[ 1 ],sspctrver_vec[ ver2[ 1 ]].pos[ 1 ] ),
			min( sspctrver_vec[ ver2[ 0 ]].pos[ 2 ],sspctrver_vec[ ver2[ 1 ]].pos[ 2 ] ),
			max( sspctrver_vec[ ver2[ 0 ]].pos[ 0 ],sspctrver_vec[ ver2[ 1 ]].pos[ 0 ] ),
			max( sspctrver_vec[ ver2[ 0 ]].pos[ 1 ],sspctrver_vec[ ver2[ 1 ]].pos[ 1 ] ),
			max( sspctrver_vec[ ver2[ 0 ]].pos[ 2 ],sspctrver_vec[ ver2[ 1 ]].pos[ 2 ] ),
		};
		//go through all the seams to split current edge
		for( int i = 0; i < fseamnum; i ++)
		{
			int seami = fseam[ i ];
			if( seami >= maseamnum - subedgenum )	//is one of the subspace edge
				continue;
			jpt2 = maseam + 2*seami;
			jpt2_fjpt[ 0 ] = findPosInIncSortedArray( fjpt,fjptnum, jpt2[ 0 ]);
			jpt2_fjpt[ 1 ] = findPosInIncSortedArray( fjpt, fjptnum,jpt2[ 1 ]);
			float bboxs[ 6 ] = {
				min( projpt[ 3*jpt2_fjpt[ 0 ]], projpt[ 3*jpt2_fjpt[ 1 ]]),
				min( projpt[ 3*jpt2_fjpt[ 0 ] + 1 ], projpt[ 3*jpt2_fjpt[ 1 ] + 1]),
				min( projpt[ 3*jpt2_fjpt[ 0 ] + 2], projpt[ 3*jpt2_fjpt[ 1 ] + 2]),
				max( projpt[ 3*jpt2_fjpt[ 0 ]], projpt[ 3*jpt2_fjpt[ 1 ]]),
				max( projpt[ 3*jpt2_fjpt[ 0 ] + 1], projpt[ 3*jpt2_fjpt[ 1 ] + 1]),
				max( projpt[ 3*jpt2_fjpt[ 0 ] + 2], projpt[ 3*jpt2_fjpt[ 1 ] + 2]),
			};


			//////////////////////////////////////////////////////////////////////////
			/*if( tempface == 2 )
			{
				cout<<"current junction points of the seam:"<<endl;
				cout<< projpt[ 3*jpt2_fjpt[ 0 ]]<<","
					<<projpt[ 3*jpt2_fjpt[ 0 ] + 1]<<","
					<< projpt[ 3*jpt2_fjpt[ 0 ] + 2]<<"  "
					<< projpt[ 3*jpt2_fjpt[ 1 ]]<<","
					<< projpt[ 3*jpt2_fjpt[ 1 ] + 1]<<","
					<< projpt[ 3*jpt2_fjpt[ 1 ] + 2]<<endl;
			}*/
			//////////////////////////////////////////////////////////////////////////

			//first check bounding box of them
			/*if(( bboxs[ 0 ] > bbox[ 3 ] + TOLERANCE_THREE ) ||( bboxs[ 1 ] > bbox[ 4 ] + TOLERANCE_THREE ) 
				||( bboxs[ 2 ] > bbox[ 5 ] + TOLERANCE_THREE) ||( bboxs[ 3 ] + TOLERANCE_THREE < bbox[ 0 ]  ) 
				||( bboxs[ 4 ]  + TOLERANCE_THREE< bbox[ 1 ]) ||( bboxs[ 5 ] + TOLERANCE_THREE < bbox[ 2 ]) )*/
			if(( bboxs[ 0 ] > bbox[ 3 ] + TOLERANCE_POS_ONE ) ||( bboxs[ 1 ] > bbox[ 4 ] + TOLERANCE_POS_ONE ) 
				||( bboxs[ 2 ] > bbox[ 5 ] + TOLERANCE_POS_ONE) ||( bboxs[ 3 ] + TOLERANCE_POS_ONE < bbox[ 0 ]  ) 
				||( bboxs[ 4 ]  + TOLERANCE_POS_ONE< bbox[ 1 ]) ||( bboxs[ 5 ] + TOLERANCE_POS_ONE < bbox[ 2 ]) )
			{
				//////////////////////////////////////////////////////////////////////////
				/*if( tempface == 2 )
				{
					cout<<"bounding box fails!"<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////
				continue;				
			}

			//the first vertex of the edge lies on the seam, either one junction point or on this seam
			if( ((type2[ 0 ] == VER_JPT) && ((fjpt[ val2[ 0 ]] == jpt2[ 0 ])||( fjpt[ val2[ 0 ]] == jpt2[ 1 ])))||
				((type2[ 0 ] == VER_SEAM )&&(val2[ 0 ] == i )))
			{
				onsameline = onSameLine( projpt + 3*jpt2_fjpt[ 0 ], projpt + 3*jpt2_fjpt[ 1 ], sspctrver_vec[ ver2[ 1 ]].pos );
				if( !onsameline )	//current seam is not able to cut the contour edge
					continue;
			}
			//the second vertex...
			else if( ((type2[ 1 ] == VER_JPT) && ((fjpt[ val2[ 1 ]] == jpt2[ 1 ])||( fjpt[ val2[ 1 ]] == jpt2[ 0 ])))||
				((type2[ 1 ] == VER_SEAM )&&(val2[ 1 ] == i )))
			{
				onsameline = onSameLine( projpt + 3*jpt2_fjpt[ 0 ], projpt + 3*jpt2_fjpt[ 1 ], sspctrver_vec[ ver2[ 0 ]].pos  );
				if( !onsameline)	//no intersection point between current seam
					continue;
			}
			else
			{
				onsameline = onSameLine( projpt + 3*jpt2_fjpt[ 0 ], projpt + 3*jpt2_fjpt[ 1 ], sspctrver_vec[ ver2[ 0 ]].pos  )
					&&onSameLine( projpt + 3*jpt2_fjpt[ 0 ], projpt + 3*jpt2_fjpt[ 1 ], sspctrver_vec[ ver2[ 1 ]].pos  );
			}
			if( onsameline)
			{
				//compute the parameters of the two vertices
                int maxind = 0;
				if( abs(sspctrver_vec[ ver2[ 0 ]].pos[1] - sspctrver_vec[ ver2[ 1 ]].pos[ 1 ]) > abs(sspctrver_vec[ ver2[ 0 ]].pos[ maxind ] - sspctrver_vec[ ver2[ 1 ]].pos[ maxind ]) )
					maxind = 1;
				if( abs(sspctrver_vec[ ver2[ 0 ]].pos[2] - sspctrver_vec[ ver2[ 1 ]].pos[ 2 ]) > abs(sspctrver_vec[ ver2[ 0 ]].pos[ maxind ] - sspctrver_vec[ ver2[ 1 ]].pos[ maxind ]) )
					maxind = 2;
                t2[ 0 ] = ( projpt[ 3*jpt2_fjpt[ 0 ] + maxind]-sspctrver_vec[ ver2[ 0 ]].pos[ maxind ])/
					( sspctrver_vec[ ver2[ 1 ]].pos[ maxind ] - sspctrver_vec[ ver2[ 0 ]].pos[ maxind ]);
				t2[ 1 ] = ( projpt[ 3*jpt2_fjpt[ 1 ] + maxind]-sspctrver_vec[ ver2[ 0 ]].pos[ maxind ])/
					( sspctrver_vec[ ver2[ 1 ]].pos[ maxind ] - sspctrver_vec[ ver2[ 0 ]].pos[ maxind ]);
				//rightdir = true;
				
				if ( t2[ 1 ] < t2[ 0 ])
				{
				//	sjpt2[ 0 ] = jpt2[ 1 ];
				//	sjpt2[ 1 ] = jpt2[ 0 ];
					sjpt2_fjpt[ 0 ] = jpt2_fjpt[ 1 ];
					sjpt2_fjpt[ 1 ] = jpt2_fjpt[ 0 ];
					//rightdir = false;
					float tt = t2[ 1 ];
					t2[ 1 ] = t2[ 0 ];
					t2[ 0 ] = tt;
				}
				else
				{
				//	sjpt2[ 0 ] = jpt2[ 0 ];
				//	sjpt2[ 1 ] = jpt2[ 1 ];
					sjpt2_fjpt[ 0 ] = jpt2_fjpt[ 0 ];
					sjpt2_fjpt[ 1 ] = jpt2_fjpt[ 1 ];
				}

				//because of tolerance, so both of them are out of range are not processed first.
				//the first contour vertex lies on junction point
				if( (type2[ 0 ] == VER_JPT) && ( val2[ 0 ] == sjpt2_fjpt[ 0 ]))
				{
					//add new contour vertex
					int nveri = sspctrver_vec.size();
					sspctrver_vec.resize( nveri + 1 );
                    memcpy( sspctrver_vec[ nveri ].pos, projpt + sjpt2_fjpt[ 1 ]*3, sizeof(float)*3);
					sspctrver_vec[ nveri ].type = VER_JPT;
					//int posi_fjpt = findPosInIncSortedArray( fjpt, jpt2[ 1 ]);
					sspctrver_vec[ nveri ].val = sjpt2_fjpt[ 1 ];

					//add one new contour edges and change the old one
                    //new
					int nedgei = sspctredge_vec.size();
					/*sspctredge_vec.resize( nedgei + 1 );*/
					sspctredge_vec.push_back( sspctredge_vec[ curedge ]);	//make a copy
					sspctredge_vec[ nedgei ].veris[ 0 ] = nveri;
					//sspctredge_vec[ nedgei ].veris[ 1 ] = sspctredge_vec[ curedge ].veris[ 1 ];
					//sspctredge_vec[ nedgei ].mat[ 0 ] = sspctredge_vec[ curedge ].mat[ 0 ];
					//sspctredge_vec[ nedgei ].mat[ 1 ] = sspctredge_vec[ curedge ].mat[ 1 ];
					if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei] , fjpt, fseam, fsheet, jpt2seamnum
						,jpt2seam, jpt2sheetnum, jpt2sheet,seam2sheetnum, seam2sheet))
						needsplitedge.push_back( nedgei );
					//old
					sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
					sspctredge_vec[ curedge ].type = EDGE_SEAM;
					sspctredge_vec[ curedge ].val = seami;
					break;	//go to the next contour edge to process
				}
				//the second contour vertex lies on junction point
				else if((type2[ 1 ] == VER_JPT) &&( sjpt2_fjpt[ 1 ] == val2[ 1 ]))
				{
					//add new contour vertex
					int nveri = sspctrver_vec.size();
					sspctrver_vec.resize( nveri + 1 );
					memcpy( sspctrver_vec[ nveri ].pos, projpt + sjpt2_fjpt[ 0 ]*3, sizeof(float)*3);
					sspctrver_vec[ nveri ].type = VER_JPT;
					//int posi_fjpt = findPosInIncSortedArray( fjpt, jpt2[ 1 ]);
					sspctrver_vec[ nveri ].val =sjpt2_fjpt[ 0 ];

					//add new contour edge and change the old one
					int nedgei = sspctredge_vec.size();
					sspctredge_vec.push_back( sspctredge_vec[ curedge ]);
					//sspctredge_vec.resize( nedgei + 1 );
					//sspctredge_vec[ nedgei ].veris[ 0 ] = sspctredge_vec[ curedge ].veris[ 0 ];
					sspctredge_vec[ nedgei ].veris[ 1 ] = nveri;
					//sspctredge_vec[ nedgei ].mat[ 0 ] = sspctredge_vec[ curedge ].mat[ 0 ];
					//sspctredge_vec[ nedgei ].mat[ 1 ] = sspctredge_vec[ curedge ].mat[ 1 ];
					if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei] , fjpt, fseam, fsheet, jpt2seamnum
						,jpt2seam, jpt2sheetnum, jpt2sheet,seam2sheetnum, seam2sheet))
						needsplitedge.push_back( nedgei );
					//old
					sspctredge_vec[ curedge ].veris[ 0 ] = nveri;
					sspctredge_vec[ curedge ].type = EDGE_SEAM;
					sspctredge_vec[ curedge ].val = seami;
					break;
				}
				//the two ts are too small or big, don't forget t2[ 1] > t2[ 0 ]
				else if( t2[ 1 ] < 0 || t2[ 0 ] > 1 )	
				{	
					continue;	//go to the next seam to process
				}
				//the two are all in range [ 0, 1 ], cut the contour edge into 3 pieces
				else if( (t2[ 0 ] > 0) && (t2[ 1 ] < 1) )
				{
					//add two new contour vertex
					int nveri = sspctrver_vec.size();					
					sspctrver_vec.resize( nveri + 2);
					memcpy(sspctrver_vec[ nveri ].pos, projpt + sjpt2_fjpt[ 0 ]*3, sizeof(float)*3);
					memcpy(sspctrver_vec[ nveri + 1 ].pos, projpt + sjpt2_fjpt[ 1 ]*3, sizeof(float)*3);
					sspctrver_vec[ nveri ].val =sjpt2_fjpt[ 0 ];
					sspctrver_vec[ nveri + 1].val = sjpt2_fjpt[ 1 ];
					sspctrver_vec[ nveri ].type = sspctrver_vec[ nveri +  1].type = VER_JPT;
					
					//add new two contour edges, and change the old one
					int nedgei = sspctredge_vec.size();
					sspctredge_vec.push_back( sspctredge_vec[ curedge ]);
					sspctredge_vec.push_back( sspctredge_vec[ curedge ]);
					//sspctredge_vec.resize( nedgei + 2 );
					sspctredge_vec[ nedgei ].veris[ 0 ] = nveri;
					sspctredge_vec[ nedgei ].veris[ 1 ] = nveri + 1;
					sspctredge_vec[ nedgei ].type = EDGE_SEAM;
					sspctredge_vec[ nedgei ].val = seami;
					sspctredge_vec[ nedgei + 1 ].veris[ 0 ] = nveri + 1;
					//sspctredge_vec[ nedgei + 1 ].veris[ 1 ] = sspctredge_vec[ curedge ].veris[ 1 ];
					if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei + 1], fjpt,
						fseam, fsheet, jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet) )
						needsplitedge.push_back( nedgei + 1);
					sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
					if( !getEdgeProp(sspctrver_vec, sspctredge_vec[ curedge ], fjpt,
						fseam, fsheet, jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet) )
						needsplitedge.push_back( curedge );
					break;
				}
				//the larger t is in the range [0,1], smaller t must be larger than 0, or else, last if would have processed it!
				else if( t2[ 1 ] < 1 )	//sjpt0 ver0 sjpt1 ver1
				{
                    //if the first contour vertex doesn't lie on the seam, bad judgement
					if( !( (type2[ 0 ] == VER_SEAM) && ( val2[ 0 ] == i ) ))	//bad judgement
						continue;

					//ok, split the edge into two
					//new contour vertex
					int nveri = sspctrver_vec.size();
					sspctrver_vec.resize( nveri + 1 );
					memcpy( sspctrver_vec[ nveri ].pos, projpt + sjpt2_fjpt[ 1 ]*3, sizeof(float)*3);
					sspctrver_vec[ nveri ].type = VER_JPT;
					//int posi_fjpt = findPosInIncSortedArray( fjpt, jpt2[ 1 ]);
					sspctrver_vec[ nveri ].val = sjpt2_fjpt[ 1 ];

					//add new contour edge and change the old one
					int nedgei = sspctredge_vec.size();
					sspctredge_vec.push_back( sspctredge_vec[curedge] );
					//sspctredge_vec.resize( nedgei + 1 );
					sspctredge_vec[ nedgei ].veris[ 0 ] = nveri;
					//sspctredge_vec[ nedgei ].veris[ 1 ] = sspctredge_vec[ curedge ].veris[ 1 ];
					if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei] , fjpt, fseam, fsheet, jpt2seamnum
						,jpt2seam, jpt2sheetnum, jpt2sheet,seam2sheetnum, seam2sheet))
						needsplitedge.push_back( nedgei );
					//old
					sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
					sspctredge_vec[ curedge ].type = EDGE_SEAM;
					sspctredge_vec[ curedge ].val = seami;
					break;
				}
				// the smaller t is in the range[ 0, 1 ], ver0 sjpt0, ver1 sjpt1
				else if( t2[ 0 ] > 0 )
				{
					//check if the second contour vertex is on the seam
					if( !( (type2[ 1 ] == VER_SEAM) && ( val2[ 1 ] == i )))	//bad judgement
						continue;

					//split it 
					//new contour vertex
					int nveri = sspctrver_vec.size();
					sspctrver_vec.resize( nveri + 1 );
					memcpy( sspctrver_vec[ nveri ].pos, projpt + sjpt2_fjpt[ 0 ]*3, sizeof(float)*3);
					sspctrver_vec[ nveri ].type = VER_JPT;
					//int posi_fjpt = findPosInIncSortedArray( fjpt, jpt2[ 1 ]);
					sspctrver_vec[ nveri ].val = sjpt2_fjpt[ 0 ];

					//add new contour edge and change the old one
					int nedgei = sspctredge_vec.size();
					//sspctredge_vec.resize( nedgei + 1 );
					sspctredge_vec.push_back( sspctredge_vec[ curedge ] );
					sspctredge_vec[ nedgei ].veris[ 0 ] =  nveri;
				//	sspctredge_vec[ nedgei ].veris[ 1 ] =  sspctredge_vec[ curedge ].veris[ 1 ]; 
					if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei] , fjpt, fseam, fsheet, jpt2seamnum
						,jpt2seam, jpt2sheetnum, jpt2sheet,seam2sheetnum, seam2sheet))
						needsplitedge.push_back( nedgei );
					//old
					sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
					sspctredge_vec[ curedge ].type = EDGE_SEAM;
					sspctredge_vec[ curedge ].val = seami;
					break;
				}	

				//////////////////////////////////////////////////////////////////////////
				/*if( tempface == 2 )
				{
					cout<<"the seam and the edge share some vertex!!"<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////

				continue;
			}

			//not on the same line, then check if they have intersection point
			//ver0 and ver1 must lie on different sides of the seam
			float planedir[ 3 ];
			float vec[ 3 ];
			MyMath::getVec( projpt + 3*jpt2_fjpt[ 0 ], projpt + 3*jpt2_fjpt[ 1 ], vec);
			MyMath::crossProduct( vec, param, planedir );
			int sidesign[ 2 ];
			for( int j = 0; j < 2; j ++)
			{
				MyMath::getVec(  projpt + 3*jpt2_fjpt[ 0 ], sspctrver_vec[ ver2[ j ]].pos, vec );
				if( MyMath::dotProduct( vec, planedir ) > 0 )
					sidesign[ j ] = 1;
				else
					sidesign[ j ] = -1;
				//impossible to be 0, otherwise, the vertex lies on the seam, which is not possible from previous process
			}

			//no intersection point at all
			if( sidesign[ 0 ] * sidesign[ 1 ] != -1 )
			{
				//////////////////////////////////////////////////////////////////////////
				/*if( tempface == 2 )
				{
					cout<<"in checking of the saem and the edge intersects with each other!!! FAIL!!"<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////
				continue;
			}
			
			//jpt0 and jpt1 must lie on different sides of the edge,
			MyMath::getVec( sspctrver_vec[ ver2[ 0 ]].pos,sspctrver_vec[ ver2[ 1 ]].pos, vec );
			MyMath::crossProduct( vec, param, planedir);
			
			//////////////////////////////////////////////////////////////////////////
			/*cout<<"pos1:"<<sspctrver_vec[ ver2[ 0 ]].pos[ 0 ]<<" "<<
				sspctrver_vec[ ver2[ 0 ]].pos[ 1 ]<<" "
				<<sspctrver_vec[ ver2[ 0 ]].pos[ 2 ]<<endl;
			
			cout<<"pos1:"<<sspctrver_vec[ ver2[ 1 ]].pos[ 0 ]<<" "
				<<sspctrver_vec[ ver2[ 1 ]].pos[ 1 ]<<" "
				<<sspctrver_vec[ ver2[ 1 ]].pos[ 2 ]<<endl;

			cout<<"dir of the edge:"<<vec[ 0  ]<<" "<<vec[1 ]<<" "<<vec[2]<<endl;
			
			cout<<"face normal:"<<param[ 0 ]<<" "<<param[ 1 ]<<" "<<param[2]<<endl;
			cout<<"dir of the plane:"<<planedir[ 0] <<" "<<planedir[1]<<" "<<planedir[2]<<endl;*/

			//////////////////////////////////////////////////////////////////////////
			int jptonctredge = -1; //no such jpt
			float dotp[ 2 ];
			float normvec[ 3 ];
			for( int j = 0; j < 2; j ++)
			{
				MyMath::getVec( sspctrver_vec[ ver2[ 0 ]].pos, projpt + 3*jpt2_fjpt[j],  vec);
				MyMath::normalize( vec, normvec );
				float t = MyMath::dotProduct( normvec, planedir );
				if( abs(t) < TOLERANCE_FOUR )	//this jpt is on the contour edge
				{
					jptonctredge = j;	//the jth one is on the contour edge
					break;
				}
				dotp[ j ] = MyMath::dotProduct( vec, planedir );
				if( t > 0 )
					sidesign[ j ] = 1;
				else
					sidesign[ j ] = -1;
			}
			//one of the junction point lies on current edge
			if( jptonctredge != -1 )
			{
				//////////////////////////////////////////////////////////////////////////
				/*if( tempface == 2 )
				{
					cout<<"one junction point is on the edge! it is"<<jptonctredge<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////

				//add the junction point into contour vertex
				int nveri = sspctrver_vec.size();
				sspctrver_vec.resize( nveri + 1);
				memcpy(sspctrver_vec[ nveri ].pos, projpt + 3*jpt2_fjpt[ jptonctredge ], sizeof(float )*3);
				sspctrver_vec[ nveri ].type = VER_JPT;
				sspctrver_vec[ nveri ].val = jpt2_fjpt[ jptonctredge ];

				//split the edge
				int nedgei = sspctredge_vec.size();
				sspctredge_vec.push_back( sspctredge_vec[ curedge ]);
				//sspctredge_vec.resize( nedgei + 1);
				sspctredge_vec[ nedgei ].veris[ 0 ] = nveri;
				//sspctredge_vec[ nedgei ].veris[ 1 ] = sspctredge_vec[ curedge ].veris[ 1 ];
				if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ nedgei ], fjpt,fseam, fsheet,
					jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet ))
					needsplitedge.push_back( nedgei );
				sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
				if( !getEdgeProp( sspctrver_vec, sspctredge_vec[ curedge ], fjpt,fseam, fsheet,
					jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet ))
					needsplitedge.push_back( curedge );
				break;
			}
			
			//on different sides
			if( sidesign[ 0 ]*sidesign[1] == -1)
			{
				//////////////////////////////////////////////////////////////////////////
				/*if( tempface == 2 )
				{
					cout<<"the two vertices do lie on different sides!"<<endl;
				}*/
				//////////////////////////////////////////////////////////////////////////
				//compute the new intersection point, and split the contour edge
				float interpt[ 3 ];
				for( int k = 0; k < 3; k++)
				{
					interpt[ k ] = (dotp[ 1 ] * projpt[ 3*jpt2_fjpt[ 0 ] + k ] - 
						dotp[ 0 ]*projpt[ 3*jpt2_fjpt[ 1 ] + k])/(dotp[ 1 ] - dotp[ 0 ]);
				}
				int nveri = sspctrver_vec.size();
				sspctrver_vec.resize( nveri + 1);
				memcpy(sspctrver_vec[ nveri ].pos, interpt, sizeof( float )*3);
				sspctrver_vec[ nveri ].type = VER_SEAM;
				sspctrver_vec[ nveri ].val = i;	//the ith seam in fseam

				//new contour edge
				int nedgei = sspctredge_vec.size();
				sspctredge_vec.push_back( sspctredge_vec[ curedge ]);
				//sspctredge_vec.resize( nedgei + 1);
				sspctredge_vec[ nedgei ].veris[ 0 ] = nveri;
				//sspctredge_vec[ nedgei ].veris[ 1 ] = sspctredge_vec[ curedge ].veris[ 1 ];
				if( !getEdgeProp(sspctrver_vec, sspctredge_vec[ nedgei ], fjpt,fseam, fsheet,
					jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet ))
					needsplitedge.push_back( nedgei );
				sspctredge_vec[ curedge ].veris[ 1 ] = nveri;
				if( !getEdgeProp(sspctrver_vec, sspctredge_vec[ curedge ], fjpt,fseam, fsheet,
					jpt2seamnum, jpt2seam, jpt2sheetnum, jpt2sheet, seam2sheetnum, seam2sheet ))
					needsplitedge.push_back( curedge );
				break;
			}
		}
	}	
	ver2 = NULL;
	jpt2 = NULL;
}
void SSContour::divideOneFaceContour(
	int spacei,
	int facei,
	//contour
	SSPCTRVERVEC& sspctrver_vec,
	SSPCTREDGEVEC& sspctredge_vec,
	//ma
	int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
	float* subparam,int* subver2wver,int* subedge2wedge,int majptnum,float* majpt,int maseamnum,
	int* maseam,MapArraySR& doubleface2sheet,int* maseamonsheetnum,int** maseamonsheet,int* ver2jpt
	)
{
	//////////////////////////////////////////////////////////////////////////
	//tempface = facei;
	//////////////////////////////////////////////////////////////////////////

	//step1. get all the junction points that will project to this face
	int* fjpt;
	int fjptnum;
	int* fseam;
	int fseamnum;
	int* fsheeti;
	int fsheetinum;
	gatherMA2Face(spacei, facei, subfacenum, maseam, doubleface2sheet, maseamonsheetnum,maseamonsheet,
		fjpt, fjptnum, fseam, fseamnum, fsheeti, fsheetinum);
	int* fjptmark = new int[ fjptnum ];
	for( int i = 0; i < fjptnum; i ++)
	{
		fjptmark[ i ] = findPosInArray(ver2jpt, subvernum, fjpt[ i ] );
	}

    //projection of these junction points to the face
	float* projpt = new float[ fjptnum*3 ];
	projectionOfJpts(fjpt, fjptmark, fjptnum,subparam + 4*facei, majpt,subver, projpt);
	//////////////////////////////////////////////////////////////////////////
	/*cout<<"After returned!"<<endl;
	for( int i = 0; i < fjptnum; i ++)
		cout<<projpt[ 3*i ]<<"\t"<<projpt[ 3*i + 1] <<"\t"<<projpt[ 3*i + 2]<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//step2. mark all the properties of the vertices
	//////////////////////////////////////////////////////////////////////////
	if( (spacei == 5) && (facei == 5) )
	{
		debugon = true;
	}
	//////////////////////////////////////////////////////////////////////////

	markVer(sspctrver_vec,
		fjpt, fjptmark, projpt, fjptnum,
		fseam, fseamnum,
		fsheeti, fsheetinum,
		subvernum,subedgenum,
		majpt, maseam, maseamnum, maseamonsheet, maseamonsheetnum,
		ver2jpt, subver2wver, subedge2wedge);

	//////////////////////////////////////////////////////////////////////////
	debugon = false;
	//////////////////////////////////////////////////////////////////////////

	//step3. gather topology information,needed when decide the edge property of the edge
	//the position of the junction point in fjpt to the position of the seams in fseam that are incident with this junction point
	vector<intset> jpt2seam;
	//similar to the one above
	vector<intset> jpt2sheet;
	//similar to the one above
	vector<intset> seam2sheet;
	seam2sheet.resize( fseamnum );
	jpt2sheet.resize( fjptnum );
	jpt2seam.resize( fjptnum );
	intset::iterator iter;
	for( int i = 0; i < fsheetinum; i++)
	{
		int sheeti = fsheeti[ i ];
		//	iter = seam2sheet[ sheeti ].begin();
		for( int j = 0 ;j < maseamonsheetnum[ sheeti ]; j++)
		{
			//		int seami = *iter;
			//		iter++;
			int seami = maseamonsheet[ sheeti ][ j ];
			seami = findPosInIncSortedArray( fseam, fseamnum, seami );
			seam2sheet[ seami ].insert( i );
		}
	}
	for( int i = 0; i < fseamnum; i ++ )
	{
		int seami = fseam[ i ];
		for( int j = 0; j < 2; j ++)
		{
			int jpti = maseam[ 2*seami + j ];
			jpti = findPosInIncSortedArray(fjpt, fjptnum, jpti);
			jpt2seam[ jpti ].insert( i );

			iter = seam2sheet[ i ].begin();
			while( iter != seam2sheet[ i ].end() )
			{
				jpt2sheet[ jpti ].insert( *iter );
				iter++;
			}
		}
	}
	int* jpt2seamnum;
	int** ajpt2seam;
	int* jpt2sheetnum;
	int** ajpt2sheet;
	int* seam2sheetnum;
	int** aseam2sheet;
	jpt2seamnum = new int[ fjptnum ];
	jpt2sheetnum = new int[ fjptnum ];
	seam2sheetnum = new int[ fseamnum ];
	ajpt2seam = new int*[ fjptnum ];
	ajpt2sheet = new int*[ fjptnum ] ;
	aseam2sheet = new int*[ fseamnum ];

	for( int i = 0; i < fjptnum; i ++)
	{
		jpt2seamnum[ i  ] = jpt2seam[i].size();
		ajpt2seam[ i ] = new int[ jpt2seamnum[ i ]];
		iter = jpt2seam[ i ].begin();
		for(unsigned int j = 0; j < jpt2seam[ i ].size(); j ++)
		{
			ajpt2seam[ i ][ j ] = *iter;
			iter++;
		}
		jpt2sheetnum[ i ] = jpt2sheet[ i ].size();
		ajpt2sheet[ i ] = new int[ jpt2sheetnum[ i ]];
		iter = jpt2sheet[ i ].begin();
		for(unsigned int j = 0; j < jpt2sheet[ i ].size(); j ++)
		{
			ajpt2sheet[ i ][ j ] = *iter;
			iter++;
		}
	}
	for( int i = 0; i < fseamnum; i++)
	{
		seam2sheetnum[ i ] = seam2sheet[ i ].size();
		aseam2sheet[ i ] = new int[ seam2sheetnum[ i ]];
		iter = seam2sheet[ i ].begin();
		for( int j = 0; j < seam2sheetnum[ i ]; j ++)
		{
			aseam2sheet[ i ][ j ] = *iter;
			iter++;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//writeTopJptSeamSheet_DB(projpt, maseam, fjpt,fjptmark,fjptnum, fseam,  fseamnum, fsheeti, fsheetinum, ajpt2seam, jpt2seamnum,ajpt2sheet,
	//	jpt2sheetnum,aseam2sheet, seam2sheetnum, spacei, facei);
	////////////////////////////////////////////////////////////////////////////

	//step4. mark all the properties of the edges.
	intvector needsplitedge;	//set when setting properties of the edges, cleared when processing all the needing splitting edges
	markEdge(sspctredge_vec, sspctrver_vec,
		subedgenum,maseamnum,subedge2wedge,fjpt,fseam,fsheeti,jpt2seamnum,
		ajpt2seam,jpt2sheetnum,ajpt2sheet,seam2sheetnum,aseam2sheet,needsplitedge);

	//////////////////////////////////////////////////////////////////////////
	/*if( tempface == 2)
	{
		cout<<"the needed process edges are:"<<endl;
		for( int i = 0 ; i < needsplitedge.size(); i ++ )
		{
			cout<<needsplitedge[ i ]<<" ";
		}
		cout <<endl;
	}*/
	//////////////////////////////////////////////////////////////////////////

	////step5. split all the edges that cross many sheets!
	processSplitEdge(
		needsplitedge,sspctrver_vec,sspctredge_vec,fjpt,projpt, fjptnum,fseam, fseamnum,fsheeti,subedgenum, maseam, maseamnum,
		jpt2seamnum,ajpt2seam,jpt2sheetnum,ajpt2sheet,seam2sheetnum ,aseam2sheet,subparam + 4*facei);

	//step6. go through all the contour vertices, and set their property values to the right jpt, seam and sheet
	for(unsigned int i = 0; i < sspctrver_vec.size(); i++ )
	{
		switch( sspctrver_vec[i].type)
		{
		case VER_JPT:
			sspctrver_vec[ i ].val = fjpt[ sspctrver_vec[ i ].val];
			break;
		case VER_SEAM:
			sspctrver_vec[ i ].val = fseam[ sspctrver_vec[ i ].val];
			break;
		case VER_SHEET:
			sspctrver_vec[ i ].val = fsheeti[ sspctrver_vec[ i ].val];
			break;
		default:
			break;
		}
	}	

	//needsplitedge.clear();

	for( int i = 0; i < fjptnum; i ++)
	{
		delete []ajpt2seam [ i ];
		delete []ajpt2sheet [ i ];
	}
	delete []ajpt2sheet;
	delete []ajpt2seam;
	delete []jpt2seamnum;
	delete []jpt2sheetnum;
	for( int i = 0; i < fseamnum; i ++)
	{
		delete []aseam2sheet[ i ];
	}
	delete []aseam2sheet;
	delete []seam2sheetnum;

	delete []fjpt;
	delete []fseam;
	delete []fsheeti;
	delete []fjptmark;
}

//write out the topology information of current subspace
void SSContour::writeTopJptSeamSheet_DB(
	float* majpt,int* maseam,
	int* fjpt,int* fjptmark, int fjptnum,
	int* fseam, int fseamnum,
	int* fsheet, int fsheetnum,
	int** jpt2seam, int* jpt2seamnum,
	int** jpt2sheet, int* jpt2sheetnum,
	int** seam2sheet, int * seam2sheetnum,
	int spacei, int facei )
{
	char fname[ 1024 ];
	strcat( fname, "mmdebug/onesubspace/");

	char numstr[ 10 ];
	itoa( spacei+1, numstr, 10 );
	strcat( fname, numstr);

	memset( numstr, '\0', 10 );

	itoa( facei+1, numstr, 10);
	strcat( fname, "_f");
	strcat( fname, numstr);
	strcat(fname,"ma.txt");
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	//position of these projected junction points
	fprintf( fout, "{{{%f,%f,%f}",majpt[ 0 ], majpt[ 1], majpt[ 2] );
	for( int i = 1; i < fjptnum; i ++)
		fprintf( fout, ",{%f,%f,%f}", majpt[ 3*i ], majpt[ 3*i + 1], majpt[ 3*i + 2]);
//	fprintf( fout, "{{{%f,%f,%f}",majpt[ 3*fjpt[ 0 ]], majpt[ 3*fjpt[ 0 ] + 1], majpt[ 3*fjpt[ 0 ] + 2] );
//	for( int i = 1; i < fjptnum; i ++)
//		fprintf( fout, ",{%f,%f,%f}", majpt[ 3*fjpt[ i ]], majpt[ 3*fjpt[ i ] + 1], majpt[ 3*fjpt[ i ] + 2]);
	
	//seams 
	for( int i = 0; i < fseamnum; i ++)
	{
		int seami = fseam[ i ];
		int vers[ 2 ];
		for( int j = 0; j < 2; j ++)
			vers[ j ] = findPosInIncSortedArray( fjpt, fjptnum, maseam[ 2*fseam[ i ] + j]);
        if( i == 0 )
			fprintf( fout, "},{{%d,%d}", vers[ 0] + 1, vers[ 1 ] + 1);
		else
			fprintf( fout, ",{%d,%d}", vers[ 0 ] + 1, vers[ 1 ] + 1);
	}
	
	//fjpt
	fprintf( fout, "},{%d", fjpt[ 0 ] + 1);
	for( int i = 1; i < fjptnum; i ++)
		fprintf( fout, ",%d", fjpt[ i ] + 1);

	//fjptmark
	fprintf( fout, "},{%d", fjptmark[  0 ]);
	for( int i = 1; i < fjptnum; i++)
		fprintf( fout, ",%d", fjptmark[ i ]);
	//fseam
	fprintf( fout, "},{%d", fseam[ 0 ] + 1);
	for( int i = 1;  i < fseamnum; i ++)
		fprintf( fout, ",%d", fseam[ i ] + 1);

	//fsheet
	fprintf( fout, "},{%d", fsheet[ 0 ] + 1);
	for ( int i = 1; i < fsheetnum; i ++)
	{
		fprintf( fout, ",%d", fsheet[ i ] + 1);
	}

	//jpt2seam
	for( int i = 0; i < fjptnum; i ++)
	{
		if( i == 0 )
			fprintf( fout, "},{{%d", jpt2seam[ i ][ 0 ] + 1);
		else
			fprintf( fout, "},{%d", jpt2seam[ i ][ 0 ] + 1);
		for( int j = 1; j < jpt2seamnum[ i ]; j ++)
		{
			fprintf( fout, ",%d", jpt2seam[ i ][ j ] + 1);
		}
	}
	
	//jpt2sheet
	for( int i = 0; i < fjptnum; i ++)
	{
		if( i == 0 )
			fprintf( fout, "}},{{%d", jpt2sheet[ i ][ 0 ]+1 );
		else
			fprintf( fout, "},{%d", jpt2sheet[ i ][ 0 ] + 1);
		for( int j = 1; j < jpt2sheetnum[ i ]; j++)
		{
			fprintf( fout, ",%d", jpt2sheet[ i ][ j ] + 1);
		}
	}

	//seam2sheet
	for( int i = 0; i < fseamnum; i++)
	{
		if( i == 0 )
			fprintf( fout, "}},{{%d", seam2sheet[ i ][ 0 ] + 1);
		else
			fprintf( fout, "},{%d", seam2sheet[ i ][ 0 ] + 1);
		for( int j = 1; j < seam2sheetnum[ i ]; j ++)
		{
			fprintf( fout, ",%d", seam2sheet[ i ][ j ] + 1);
		}
	}
	fprintf( fout, "}}}");

	fclose( fout );
}
void SSContour::writeGatheredCtr_DB(vector<SSPCTRVERVEC>& sspctrver_vec,vector<SSPCTREDGEVEC>& sspctredge_vec, int spacei,
									int* ssspacefacenum)
{
	char numstr[ 10 ];
	itoa( spacei + 1, numstr, 10 );

	char fname[ 1024 ];
	strcpy ( fname, "mmdebug/onesubspace/");
	strcat( fname, numstr);
	strcat( fname, "ctrgather.txt");
	FILE* fout = fopen( fname, "w");

	if( fout == NULL )
	{
		cout<<"Unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	for( int i = 0; i < ssspacefacenum[ spacei ]; i ++)
	{
		if( sspctrver_vec[ i ].size() == 0)
		{
			if ( i == 0 )
				fprintf( fout, "{{{},{},{},{");
			else
				fprintf( fout, "}},{{},{},{},{");
			continue;
		}

		//cout<<i<<":"<<sspctrver_vec[ i ].size()<<endl;
		//write out vertex
		if( i == 0 )
			fprintf( fout, "{{{{%f,%f,%f}", sspctrver_vec[ i ][ 0 ].pos[ 0 ], sspctrver_vec[ i ][ 0 ].pos[1], sspctrver_vec[ i ][ 0 ].pos[ 2 ]);
		else
			fprintf( fout, "}},{{{%f,%f,%f}", sspctrver_vec[ i ][ 0 ].pos[ 0 ], sspctrver_vec[ i ][ 0 ].pos[1], sspctrver_vec[ i ][ 0 ].pos[ 2 ]);

		//positions of the vertices
		for(unsigned int j = 1; j < sspctrver_vec[ i ].size(); j ++)
		{
			fprintf( fout, ",{%f,%f,%f}", sspctrver_vec[ i ][ j ].pos[ 0 ], sspctrver_vec[ i ][ j ].pos[1], 
				sspctrver_vec[ i ][ j ].pos[2]);
		}

		//properties of the vertices
		fprintf( fout, "},{{%d,%d}", sspctrver_vec[ i ][ 0 ].type, sspctrver_vec[ i ][ 0 ].val + 1);
		for(unsigned int j = 1; j < sspctrver_vec[ i ].size(); j ++)
			fprintf(fout, ",{%d,%d}", sspctrver_vec[ i ][ j ].type, sspctrver_vec[ i ][ j ].val + 1);

		//write out edges
		//composition and material
		fprintf( fout, "},{{{%d,%d},{%d,%d}}", sspctredge_vec[ i ][ 0 ].veris[ 0 ]+1, sspctredge_vec[ i ][ 0 ].veris[ 1 ]+1,
			sspctredge_vec[ i ][ 0 ].mat[ 0 ]+1, sspctredge_vec[ i ][ 0 ].mat[1]+1);
		for(unsigned int j = 1 ; j <sspctredge_vec[ i ].size(); j ++)
		{
			fprintf( fout, ",{{%d,%d},{%d,%d}}",sspctredge_vec[ i ][ j ].veris[ 0 ]+1, sspctredge_vec[ i ][ j ].veris[ 1 ]+1,
				sspctredge_vec[ i ][ j ].mat[ 0 ]+1, sspctredge_vec[ i ][ j ].mat[1]+1);
		}

		//properties
		fprintf( fout, "},{{%d,%d}", sspctredge_vec[ i ][ 0 ].type, sspctredge_vec[ i ][ 0 ].val + 1);
		for(unsigned int j = 1; j < sspctredge_vec[ i ].size(); j ++)
		{
			fprintf( fout, ",{%d,%d}", sspctredge_vec[ i ][ j ].type, sspctredge_vec[ i ][ j ].val + 1);
		}		
	}
	fprintf( fout, "}}}");
	fclose( fout );
}