#include "../Projection/Projector.h"
#include "../SubspaceContour/sscontour.h"

void Projector::projectCtr(
		floatvector& meshVer,
		intvector& meshEdge,
	//	intvector& meshFace,
		int* jptReg,				//for each junction point, corresponding vertex in meshver
		intvector* seamVerReg,		//for each seam, the projected vertices
		EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
		intvector* sheetVerReg,		//for each sheet, the projected vetices on it
		EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it

		//all the vertices and edges in the subspace
		vector<SSPCTRVERVEC>& sspctrver_vec,
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//ma
		int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
		float* subparam,int* subver2wver,int* subedge2wedge,int majptnum,float* majpt,int maseamnum,
		int* maseam,MapArraySR& doubleface2sheet,int* seamonsheetnum,int** seamonsheet,float* sheettab,int* ver2jpt)
{
	//const int VER_JPT = 1;	//the contour vertex will be projected to a junction point
	//const int VER_SEAM = 2;	
	//const int VER_SHEET = 3;
	//const int EDGE_SEAM = 2;
	//const int EDGE_SHEET = 3;
	//struct SUBSPACECTRVER
	//{
	//	float pos[ 3 ];
	//	int type;	//which kind of element of the medial axis this contour vertex will project to.
	//	int val;	//the value of the element
	//};
	//struct SUBSPACECTREDGE
	//{
	//	int veris[ 2 ];	//the two vertices of the edge
	//	int mat[ 2 ];	//material on the left side and the right side
	//	int type;	//which kind of element of the medial axis this edge is going to project to
	//	int val;	//the value of the element.
	//	int ancestor;	//the original edge index in the plane, useful when stitching.
	//};

	//initialize
	for( int i = 0; i < majptnum; i ++)
		jptReg[ i ] = -1;	//corresponding vertex in mesh is not set yet!
	
	int ctrvernum;
	int ctredgenum;
	int* vermap = NULL ;
	//float squareTol = TOLERANCE_SAME_VER * TOLERANCE_SAME_VER;
	float squareTol = TOLERANCE_ONE * TOLERANCE_ONE;
	int* projjptindex = new int[ majptnum ];
	float* projjpt = new float[ majptnum * 3 ];
	//floatvector projjpt;
	float projpt[ 3 ];
//	int twofaces[ 2 ];

	for( int i = 0; i < subfacenum;i ++)
	{
		//step0, initialize the projected junction point, only computed when need
		for( int j = 0; j < majptnum; j ++)
			projjptindex[ j ] = -1;
//		twofaces[ 0 ] = i;

		//step1 project all the vertices
		//refresh
		//1. jptreg, 2. seamverreg, 3. sheetverreg
		ctrvernum = sspctrver_vec[ i ].size();
		if( ctrvernum == 0 )
			continue;
		vermap = new int[ ctrvernum ];
		for( int j = 0; j < ctrvernum; j ++ )
			vermap[ j ] = -1;
		//go through each vertex and project it to the right junction point, seam or sheet
		for( int j = 0; j < ctrvernum; j ++)
		{
			//////////////////////////////////////////////////////////////////////////
			/*if( (i == 3) || (i ==4) )
			{
				cout
			}*/
			//////////////////////////////////////////////////////////////////////////
			switch( sspctrver_vec[ i ][ j ].type )
			{
			case VER_JPT:
				{
					int jpti = sspctrver_vec[ i ][ j ].val;
					//check if the juncton point has already been added
					if( jptReg[ jpti ] == -1 )	//not added
					{
						meshVer.push_back( majpt[ 3*jpti ] );
						meshVer.push_back( majpt[ 3*jpti + 1 ]);
						meshVer.push_back( majpt[ 3*jpti + 2 ] );
						vermap[ j ] = jptReg[ jpti ] = meshVer.size()/3 - 1;
					}
					else
						vermap[ j ] = jptReg[ jpti ];
				}
				break;
			case VER_SEAM:
				{
					int seami = sspctrver_vec[ i ][ j ].val;
					//step1. compute the projected vertex for this vertex
					//check if the projection of the two endpoints have been computed or not
					int jpti[ 2 ] = { maseam[ seami * 2 ], maseam[ seami * 2 + 1 ]};
					for(int k = 0; k < 2; k ++)
					{						
						if( projjptindex[ jpti[ k ] ] == -1 )	//not computed yet
						{
							projectionPtOnPlane( majpt + 3*jpti[ k ], subparam + 4*i , true, projjpt + 3*jpti[ k ]);
						}
					}
					float param = getParamOnSeg( projjpt + 3 * jpti[ 0 ], projjpt + 3* jpti[ 1 ], sspctrver_vec[ i ][ j ].pos);
					MyMath::getPtOnSeg( majpt + 3*jpti[ 0 ], majpt + 3*jpti[ 1 ], param, projpt );

					//////////////////////////////////////////////////////////////////////////
				/*	if( i == 2 )
					{
						cout<<"seamval:"<<seami<<endl;
						cout<<"jpts are:{"
							<<majpt[ 3*jpti[ 0 ]] <<","
							<<majpt[ 3*jpti[ 0 ] + 1] <<","
							<<majpt[ 3*jpti[ 0 ] + 2] <<"}\t{"
							<<majpt[ 3*jpti[ 1 ]] <<","
							<<majpt[ 3*jpti[ 1 ] + 1] <<","
							<<majpt[ 3*jpti[ 1 ] + 2] <<"}\n";

						cout<<"projected junction points:{"
							<<projpt[ 3*jpti[ 0 ]] <<","
							<<projjpt[ 3*jpti[ 0 ] + 1] <<","
							<<projjpt[ 3*jpti[ 0 ] + 2] <<"}\t{"
							<<projjpt[ 3*jpti[ 1 ]] <<","
							<<projjpt[ 3*jpti[ 1 ] + 1] <<","
							<<projjpt[ 3*jpti[ 1 ] + 2] <<"}\n";

						cout<<"the vertex position:"<<"{"
							<<sspctrver_vec[ i ][ j ].pos[ 0 ]<<","
							<<sspctrver_vec[ i ][ j ].pos[ 1 ]<<","
							<<sspctrver_vec[ i ][ j ].pos[ 2 ]<<"}\n";

						cout<<"the computed position is:{"
							<<projpt[ 0 ]<<","
							<<projpt[ 1 ]<<","
							<<projpt[ 2 ]<<"}\n";
					}*/
					//////////////////////////////////////////////////////////////////////////


					//////////////////////////////////////////////////////////////////////////
					//if (( (i == 3) && (( j==1) || ( j == 9 ))) ||
					//	( (i == 4) &&  ((j == 1)||(j == 5)) ))
					//{
					//	cout<<"face "<<i<<" vertex "<<j<<":"<<endl;
					//	cout<<"seam:"<<seami<<"jpti:"<<jpti[ 0 ]<<"  "<<jpti[1]<<endl;
					//	cout<<"projected junction point:"
					//		<<projjpt[ 3*jpti[ 0 ]]<<","<<projjpt[ 3*jpti[ 0 ]+1]<<","<<projjpt[ 3*jpti[ 0 ]+2]<<"   "
					//		<<projjpt[ 3*jpti[ 1 ]]<<","<<projjpt[ 3*jpti[ 1 ]+1]<<","<<projjpt[ 3*jpti[ 1 ]+2]<<endl;
					//	cout<<"junction point:"
					//		<<majpt[ 3*jpti[ 0 ]]<<","<<majpt[ 3*jpti[ 0 ]+1]<<","<<majpt[ 3*jpti[ 0 ]+2]<<"   "
					//		<<majpt[ 3*jpti[ 1 ]]<<","<<majpt[ 3*jpti[ 1 ]+1]<<","<<majpt[ 3*jpti[ 1 ]+2]<<endl;
					//	
					//	cout<<"param:"<<param<<"projpt on seam:"
					//		<<projpt[ 0 ]<<","<<projpt[1]<<","<<projpt[2]<<endl;

					//}
					////////////////////////////////////////////////////////////////////////////

					//step2. check if the vertex has already been added or not
					for(unsigned int k = 0; k < seamVerReg[ seami ].size(); k ++)
					{
						float dlen = 0;
						for( int k2 = 0; k2 < 3; k2 ++)
						{
							dlen += pow(meshVer[ 3*seamVerReg[ seami ][ k ] + k2 ] - projpt[ k2 ], 2);
						}
						if ( dlen < squareTol )	//same!
						{
							vermap[ j ] = seamVerReg[ seami ][ k ];
							break;
						}
					}
					if( vermap[ j ] == -1 )	//no same vertex
					{
						//push the projected vertex into meshVer					
						meshVer.push_back( projpt[ 0 ] );
						meshVer.push_back( projpt[ 1 ] );
						meshVer.push_back( projpt[ 2 ] );
											
						//set the map information and the registration information
						vermap[ j ] = meshVer.size()/3 - 1; 
						seamVerReg[ seami ].push_back( vermap[ j ] );

						//////////////////////////////////////////////////////////////////////////
						/*if (( (i == 3) && (( j==1) || ( j == 9 ))) ||
							( (i == 4) &&  ((j == 1)||(j == 5)) ))
						{
							cout<<"posiiton in mesh vertex:"<<vermap[ j ]<<endl;
						}*/
						//////////////////////////////////////////////////////////////////////////
					}
				}
				break;
			case VER_SHEET:
				{
					
					//step1. compute the projected vertex on the sheet
				//	int sheeti ;
				//	twofaces[ 1 ] = sspctrver_vec[ i ][ j ].val;
				//	doubleface2sheet.getKeyVal(twofaces, 2, false, sheeti );
					int sheeti = sspctrver_vec[ i ][ j ].val;
					float t = distPt2PlaneAlongDir( sspctrver_vec[ i ][ j ].pos, subparam + 4*i, true,
						sheettab + 6*sheeti,  sheettab+ 6*sheeti + 3, true);
					for( int k = 0; k < 3; k ++ )
					{
						projpt[ k ] = sspctrver_vec[ i ][ j ].pos[ k ] + t * subparam[ 4*i + k ];
					}
					//////////////////////////////////////////////////////////////////////////
					/*if ( (i == 3) && ( j==0))						
					{
						cout<<"face "<<i<<" vertex "<<j<<":"<<endl;						
						cout<<"sheet pt:"<<sheettab[6*sheeti]<<","
							<<sheettab[6*sheeti+1]<<","
							<<sheettab[6*sheeti+2]<<"   "
							<<"norm:"<<sheettab[6*sheeti+3]<<","
							<<sheettab[6*sheeti+4]<<","
							<<sheettab[6*sheeti+5]<<endl;
						cout<<"projpt on seam:"
							<<projpt[ 0 ]<<","<<projpt[1]<<","<<projpt[2]<<endl;

					}*/
					//////////////////////////////////////////////////////////////////////////

					//////////////////////////////////////////////////////////////////////////
					if( (i == 2) && (j == 28) )
					{
						cout<<"position of the contour vertex:"
							<<sspctrver_vec[ i ][ j ].pos[0 ] <<","
							<<sspctrver_vec[ i ][ j ].pos[ 1]<<","
							<<sspctrver_vec[ i ][ j ].pos[ 2 ]<<endl;
						cout<<"plane param:"
							<<subparam[ 4*i ]<<","
							<<subparam[ 4*i + 1]<<","
							<<subparam[ 4*i + 2]<<","
							<<subparam[ 4*i + 3 ]<<"\n";
						cout<<"sheet param:"
							<<sheettab[ 6*sheeti ]<<","
							<<sheettab[ 6*sheeti + 1 ]<<","
							<<sheettab[ 6*sheeti + 2 ]<<"  "
							<<sheettab[ 6*sheeti + 3 ]<<","
							<<sheettab[ 6*sheeti + 4 ]<<","
							<<sheettab[ 6*sheeti + 5 ]<<endl;
						cout<<"computed position for it:"
							<<projpt[ 0 ] <<","
							<<projpt[ 1 ] <<","
							<<projpt[ 2 ] <<"\n";


					}
					//////////////////////////////////////////////////////////////////////////


					//step2. go through all the vertices on sheet, to see if it is close to some point on it
					//float dlen = 0;
					//////////////////////////////////////////////////////////////////////////
					//cout<<"sheetverreg size:"<<sheetVerReg[ sheeti ].size()<<endl;
					//////////////////////////////////////////////////////////////////////////
					for(unsigned int k = 0; k < sheetVerReg[ sheeti ].size(); k ++ )
					{
						//check if the two vertices are too close
						int veri = sheetVerReg[ sheeti ][ k ];
						float dlen = 0;
						for( int k1 = 0; k1 < 3; k1 ++ )
						{
							dlen += pow( meshVer[ 3*veri + k1 ] - projpt[ k1 ], 2);
						}
						if( dlen < squareTol )
						{
							vermap[ j ] = veri;
							break;
						}
					}
					
					if( vermap[ j ] == -1 )	//no overlapped vertex on the sheet
					{
						//add the projected vertex into meshVer
						meshVer.push_back( projpt[ 0 ]);
						meshVer.push_back( projpt[ 1 ]);
						meshVer.push_back( projpt[ 2 ]);

						//set the vermap and the sheetVerReg
						vermap[ j ] = meshVer.size() /3 - 1;
						sheetVerReg[ sheeti ].push_back( vermap[ j ]);
					}
				}
				break;
			default:
#ifdef debug
				cout<<"ERROR: Type of #"<<j<<" Vertex on face# "<<i<<"is not set yet!"<<endl;
#endif
				break;
			}
		}

		//step2 project all the edges
		//refresh:
		//1. meshEdge 2. seamedgereg,3.sheetedgereg
		ctredgenum = sspctredge_vec[ i ].size();
		bool added = false;
		for( int j = 0; j < ctredgenum; j ++ )
		{			
			switch( sspctredge_vec[ i ][ j ].type )
			{
			case EDGE_SEAM:
				{
					int seami = sspctredge_vec[ i ][ j ].val;
					int nvers[ 2 ] = {vermap[ sspctredge_vec[ i ][ j ].veris[ 0 ]], 
						vermap[ sspctredge_vec[ i ][ j ].veris[ 1 ] ] };

					added = false;
					//step1, go through all the registered edges to see if it is already in
					for(unsigned int k = 0; k < seamEdgeReg[ seami ].size(); k ++)
					{
						int edgei = seamEdgeReg[ seami ][ k ].posInMeshEdge;
						//case1, already added, and in the same direction
						if( (meshEdge[ 2*edgei ] == nvers[ 0 ] ) && (meshEdge[ 2*edgei + 1 ] == nvers[ 1 ] ))
						{
							correspnCtrEdge tmp;
							tmp.facei = i;
							tmp.edgei = j;
							tmp.samedirec = true;
							seamEdgeReg[ seami ][ k ].crspCtrEdges.push_back( tmp );
							added = true;
							break;
						}
						//case2, already added, but in opposite direction
						else if ((meshEdge[ 2*edgei ] == nvers[ 1 ] ) && (meshEdge[ 2*edgei + 1 ] == nvers[ 0 ] ))
						{
							correspnCtrEdge tmp;
							tmp.facei = i;
							tmp.edgei = j;
							tmp.samedirec = false;
							seamEdgeReg[ seami ][ k ].crspCtrEdges.push_back( tmp );
							added = true;
							break;
						}					
					}	
					//case3, not added yet	
					if( !added )
					{
						meshEdge.push_back( nvers[ 0 ]);
						meshEdge.push_back( nvers[ 1 ] );
						EdgeRegItem tedgeregitem;
						tedgeregitem.posInMeshEdge = meshEdge.size() / 2 - 1;
						correspnCtrEdge tcrspctredge;
						tcrspctredge.facei = i;
						tcrspctredge.edgei = j;
						tcrspctredge.samedirec = true;
						tedgeregitem.crspCtrEdges.push_back( tcrspctredge );
						seamEdgeReg[ seami ].push_back( tedgeregitem );		
					}
				}
				break;
			case EDGE_SHEET:
				{
				//	twofaces[ 1 ] = sspctredge_vec[ i ][ j ].val;
				//	int sheeti;
				//	doubleface2sheet.getKeyVal( twofaces, 2, false, sheeti );
					int sheeti = sspctredge_vec[ i ][ j ].val;
					int nvers[ 2 ] = {vermap[ sspctredge_vec[ i ][ j ].veris[ 0 ]],
					vermap[ sspctredge_vec[ i ][ j ].veris[ 1 ]]};
					//go through all the registered edge, to see if it is in or not
					added = false;
					for(unsigned int k = 0; k < sheetEdgeReg[ sheeti ].size(); k ++)
					{
						int edgei = sheetEdgeReg[ sheeti ][ k ].posInMeshEdge;
						//case1, already added, and in the same direction
						if( (meshEdge[ 2*edgei ] == nvers[ 0 ] ) && (meshEdge[ 2*edgei + 1 ] == nvers[ 1 ] ))
						{
							correspnCtrEdge tmp;
							tmp.facei = i;
							tmp.edgei = j;
							tmp.samedirec = true;
							sheetEdgeReg[ sheeti ][ k ].crspCtrEdges.push_back( tmp );
							added = true;
							break;
						}
						//case2, already added, but in opposite direction
						else if ((meshEdge[ 2*edgei ] == nvers[ 1 ] ) && (meshEdge[ 2*edgei + 1 ] == nvers[ 0 ] ))
						{
							correspnCtrEdge tmp;
							tmp.facei = i;
							tmp.edgei = j;
							tmp.samedirec = false;
							sheetEdgeReg[ sheeti ][ k ].crspCtrEdges.push_back( tmp );
							added = true;
							break;
						}					
					}
					if( !added )
					{
						meshEdge.push_back( nvers[ 0 ]);
						meshEdge.push_back( nvers[ 1 ] );
						EdgeRegItem tedgeregitem;
						tedgeregitem.posInMeshEdge = meshEdge.size() / 2 - 1;
						correspnCtrEdge tcrspctredge;
						tcrspctredge.facei = i;
						tcrspctredge.edgei = j;
						tcrspctredge.samedirec = true;
						tedgeregitem.crspCtrEdges.push_back( tcrspctredge );
						sheetEdgeReg[ sheeti ].push_back( tedgeregitem );
						//////////////////////////////////////////////////////////////////////////
						//clear the temporary variable
						tedgeregitem.crspCtrEdges.clear();
					}
				}
				break;
			default:
#ifdef debug
				cout<<"ERROR: Type of #"<<j<<" Edge on face# "<<i<<"is not set yet!"<<endl;
#endif
				break;
			}
		}
        
		//clear all the temporary vars
		delete []vermap;
		vermap = NULL;
	}
	delete []projjptindex;
	delete []projjpt;
}


void Projector::writeProjection_NoSplit(
							 int spacei,
							 //mesh
							 floatvector& meshVer,
							 intvector& meshEdge,

							 //jptreg seamreg sheetreg
							 int majptnum,	int maseamnum,	int masheetnum,
							 int* jptReg,				//for each junction point, corresponding vertex in meshver
							 intvector* seamVerReg,		//for each seam, the projected vertices
							 EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
							 intvector* sheetVerReg,		//for each sheet, the projected vetices on it
							 EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it
							 //dcontour
							 vector<SSPCTRVERVEC>& sspctrver_vec,
							 vector<SSPCTREDGEVEC>& sspctredge_vec)
{
    char fname[ 1024 ];
	strcpy( fname,  "mmdebug/onesubspace/projnosplit");

	char numstr[ 10 ];
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
		
		//ver type and val
		if( sspctrver_vec[ i ].size() == 0 )
		{
			fprintf( fout, "},{");
		}
		else
		{
			fprintf( fout, "},{{%d,%d}", sspctrver_vec[ i ][ 0 ].type, sspctrver_vec[ i ][ 0 ].val + 1);
			for(unsigned int j = 1; j < sspctrver_vec[ i ].size(); j ++ )
			{
				fprintf( fout, ",{%d,%d}", sspctrver_vec[ i ][ j ].type,
					sspctrver_vec[ i ][ j ].val + 1);
			}
		}

		//edge type and val
		if( sspctredge_vec[ i ].size() == 0 )
		{
			fprintf(fout, "},{");
		}
		else
		{
			fprintf( fout, "},{{%d,%d}", sspctredge_vec[ i ][ 0 ].type, sspctredge_vec[ i ][ 0 ].val + 1 );
			for(unsigned int j= 1; j < sspctredge_vec[ i ].size(); j ++ )
			{
				fprintf( fout, ",{%d,%d}", sspctredge_vec[ i ][ j ].type,
					sspctredge_vec[ i ][ j ].val + 1);
			}
		}

		fprintf( fout, "}}");
	}

	//jptreg
	fprintf( fout, "},{%d", jptReg[ 0 ] + 1);
	for( int i = 1; i < majptnum; i ++)
		fprintf( fout, ",%d", jptReg[ i ] + 1);

	//seamverreg
	for( int i = 0; i < maseamnum; i ++ )
	{
		if( seamVerReg[ i ].size() == 0 )
		{
			if( i == 0 )
				fprintf( fout, "},{{");
			else
				fprintf( fout,"},{");
			continue;
		}

		if( i == 0 )
			fprintf( fout, "},{{%d", seamVerReg[ i ][ 0 ] + 1);
		else
			fprintf( fout, "},{%d", seamVerReg[ i ][ 0 ] + 1);
		for(unsigned int j = 1;j < seamVerReg[ i ].size(); j ++ )
		{	
			fprintf( fout, ",%d", seamVerReg[ i ][ j ] + 1);
		}
	}	

	//seamedgereg
	int dir = 1;
	for( int i = 0; i < maseamnum; i ++ )
	{
		if( seamEdgeReg[ i ].size() == 0 )
		{
			if( i == 0 )
				fprintf( fout, "}},{{");
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
					fprintf( fout, "}},{{{%d,{", seamEdgeReg[ i ][ 0 ].posInMeshEdge+1);
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

	//sheetverreg
	for( int i = 0; i < masheetnum; i ++ )
	{
		if( sheetVerReg[ i ].size() == 0 )
		{
			if( i == 0 )
				fprintf(fout, "}},{{");
			else
				fprintf( fout, "},{");
			continue;
		}

		for(unsigned int j = 0; j < sheetVerReg[ i ].size(); j ++ )
		{
			if( j == 0 )
			{
				if ( i == 0 )
					fprintf( fout, "}},{{%d",sheetVerReg[ i ][ j ] + 1);
				else
					fprintf( fout, "},{%d", sheetVerReg[ i ][ j ] + 1 );
				continue;
			}
			fprintf( fout, ",%d", sheetVerReg[ i ][ j ] + 1);
		}
	}
	
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
void Projector::writeProjection_NoSplit_bak(
	 int spacei,
	 //mesh
	 floatvector& meshVer,
	 intvector& meshEdge,
	 
	//jptreg seamreg sheetreg
	int majptnum,	int maseamnum,	int masheetnum,
	int* jptReg,				//for each junction point, corresponding vertex in meshver
	intvector* seamVerReg,		//for each seam, the projected vertices
	EdgeReg* seamEdgeReg,		//for each seam, the projected edges on it
	intvector* sheetVerReg,		//for each sheet, the projected vetices on it
	EdgeReg* sheetEdgeReg,		//..		...	, the projected edges on it
	//dcontour
	vector<SSPCTRVERVEC>& sspctrver_vec,
	vector<SSPCTREDGEVEC>& sspctredge_vec)
{
	char fname[ 20 ] = "projnosplit";
	char numstr[ 3 ];
	itoa( spacei, numstr, 10);
	strcat( fname, numstr);
	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"Unable to open the file "<<fname<<" to write!"<<endl;
		return;
	}

	//mesh
	fprintf(fout, "{{{{%f,%f,%f}", meshVer[ 0 ], meshVer[ 1 ], meshVer[ 2 ]);
	for(unsigned int i = 1; i < meshVer.size()/3; i ++ )
	{
		fprintf( fout, ",{%f,%f,%f}", meshVer[ 3*i ], meshVer[ 3*i + 1 ], meshVer[ 3*i + 2 ]);
	}
	fprintf( fout, "},{{%d,%d}", meshEdge[ 0 ] + 1, meshEdge[ 1 ] + 1);
	for(unsigned int i = 1; i < meshEdge.size()/2; i ++)
	{
		fprintf( fout, ",{%d,%d}", meshEdge[ 2*i ] + 1, meshEdge[ 2*i + 1 ] + 1);
	}

	//jptreg
	fprintf( fout, "}},{%d", jptReg[ 0 ] + 1 );
	for( int i = 1; i < majptnum; i ++)
	{
		fprintf( fout, ",%d", jptReg[ i ] + 1 );
	}

	//seamreg
	for( int i = 0; i < maseamnum; i ++)
	{
		//current seamregistration is empty
		if( seamVerReg[ i ].size() == 0 )
		{
			if( i == 0 )
				fprintf( fout, "},{{{},{{");
			else
				fprintf( fout, "}}},{{},{{");
		}
		else
		{
			//current seamregistration is not empty
			//seamverreg
			if( i == 0 )	//it is the first seam
				fprintf( fout, "},{{{%d", seamVerReg[ i ][ 0 ] + 1);
			else
				fprintf( fout, "}}},{{%d", seamVerReg[ i ][ 0 ] + 1);
			for(unsigned int j = 1; j < seamVerReg[ i ].size(); i ++ )
			{
				fprintf(fout, ",%d", seamVerReg[ i ][ j ] + 1 );
			}
		}		
		
		//seamedgereg
		//edge composition information
		if( seamEdgeReg[ i ].size() == 0)	//no projected edge on the saem
		{
			fprintf( fout, "},{{},{");
			continue;
		}
		int meshepos = seamEdgeReg[ i ][ 0 ].posInMeshEdge;
		fprintf( fout, "},{{{%d,%d}", meshEdge[ 2*meshepos ] + 1, meshEdge[ 2*meshepos + 1 ] + 1);
		for(unsigned int j = 1; j < seamEdgeReg[ i ].size(); j ++ )
		{
			meshepos = seamEdgeReg[ i ][ j ].posInMeshEdge;
			fprintf( fout, ",{%d,%d}", meshEdge[ 2*meshepos ] + 1, meshEdge[ 2*meshepos + 1] + 1);
		}
		//correponding edge information
		//the first edge on it
		//position in mesh edge
		fprintf( fout , "},{{%d,{", seamEdgeReg[ i ][ 0 ].posInMeshEdge + 1);
		//corresponding edges in contour edges
		int dir = 1;
		if( !(seamEdgeReg[ i ][ 0 ].crspCtrEdges[ 0 ].samedirec ))
			dir = 0;
		fprintf( fout, "{%d,%d,%d}", seamEdgeReg[ i ][ 0 ].crspCtrEdges[ 0 ].facei + 1,
			seamEdgeReg[ i ][ 0 ].crspCtrEdges[ 0 ].edgei + 1,
			dir	);
		for(unsigned int j = 1; j < seamEdgeReg[ i ][ 0 ].crspCtrEdges.size(); j ++)
		{
			dir = 1;
			if(!(seamEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].samedirec ))
				dir = 0;
			fprintf( fout, ",{%d,%d,%d}", seamEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].facei + 1,
				seamEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].edgei + 1,
				dir	);
		}
		//other edges on current seam
		for(unsigned int j = 1; j < seamEdgeReg[ i ].size(); j ++ )
		{
			//the position in mesh edge
			fprintf( fout, "}},{%d,{", seamEdgeReg[ i ][ j ].posInMeshEdge + 1 );
			//the corresponding contour edges
			dir = 1;
			if( ! (seamEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].samedirec) )
				dir = 0;
			fprintf( fout, "{%d,%d,%d}", 
				seamEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].facei + 1, 
				seamEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].edgei + 1,
				dir	);
			for(unsigned int k = 1; k <seamEdgeReg[ i ][ j ].crspCtrEdges.size(); k ++)
			{
				dir = 1;
				if(!(seamEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].samedirec) )
					dir = 0;
				fprintf( fout, "{%d,%d,%d}", 
					seamEdgeReg[ i ][ j ].crspCtrEdges[ k ].facei + 1, 
					seamEdgeReg[ i ][ j ].crspCtrEdges[ k ].edgei + 1,
					dir	);
			}
		}
		fprintf( fout ,"}}");
	}

	//sheetreg
	//vertex on sheet
	for( int i = 0; i < masheetnum; i ++)
	{
		//write vertex out
		if( sheetVerReg[ i ].size() == 0 )
		{
			if( i == 0 )	//the first sheet
			{
				fprintf( fout, "}}},{{{");
			}
			else
				fprintf( fout, "}},{{");
		}
		else
		{
			//there exists point on this sheet
			//write out the verti
			int tveri = sheetVerReg[ i ][ 0 ];
			if( i == 0 )
			{
				fprintf( fout, "}}},{{{{%f,%f,%f}", meshVer[ 3*tveri ],
					meshVer[ 3*tveri + 1], meshVer[ 3*tveri + 2 ]);
			}
			else
				fprintf( fout, "}},{{{%f,%f,%f}",meshVer[ 3*tveri ],
				meshVer[ 3*tveri + 1], meshVer[ 3*tveri + 2 ]);
			for(unsigned int j = 1; j < sheetVerReg[ i ].size(); j ++)
			{
				tveri = sheetVerReg[ i ][ j ];
				fprintf( fout, ",{%f,%f,%f}", meshVer[ 3*tveri ],
					meshVer[ 3*tveri + 1 ], meshVer[ 3*tveri +  2 ]	);
			}
		}
		
		//write edges out
        if( sheetEdgeReg [ i ].size() == 0 )	
		{
			fprintf(fout, "}},{{},{");
			continue;
		}
		//write the composition of the edge out
		//write the first edge
		int tedgei = sheetEdgeReg[ i ][ 0 ].posInMeshEdge;
		fprintf( fout, "}},{{{%d,%d}", meshEdge[ tedgei * 2 ] + 1, meshEdge[ tedgei * 2 + 1] + 1);
		//write other edges
		for(unsigned int j = 1; j < sheetEdgeReg[ i ].size(); j ++)
		{
			tedgei = sheetEdgeReg[ i ][ j ].posInMeshEdge;
			fprintf( fout,",{%d,%d}", meshEdge[ tedgei * 2 ] + 1, meshEdge[ tedgei * 2 + 1] + 1);
		}

		//write corresponding edges and pos in mesh edge
		//write the first edge out
		fprintf( fout, "},{{%d", sheetEdgeReg[ i ][ 0 ].posInMeshEdge + 1);
		int dir = 1;
		if( !( sheetEdgeReg[ i ][ 0 ].crspCtrEdges[ 0 ].samedirec ))
			dir = 0;
		fprintf( fout, ",{{%d,%d,%d}", sheetEdgeReg[ i ][ 0 ].crspCtrEdges[ 0 ].facei + 1,
			sheetEdgeReg[ i ][ 0 ].crspCtrEdges[  0 ].edgei + 1,
			dir	);
		for(unsigned int j = 1; j < sheetEdgeReg[ i ][ 0 ].crspCtrEdges.size(); j ++)
		{
			dir = 1;
			if(!( sheetEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].samedirec ))
				dir = 0;
			fprintf( fout, ",{{%d,%d,%d}", sheetEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].facei + 1,
				sheetEdgeReg[ i ][ 0 ].crspCtrEdges[ j ].edgei + 1,
				dir	);
		}
        //write the other edges out
		for(unsigned int j = 1; j < sheetEdgeReg[ i ].size(); j ++)
		{
			//edge position in mesh
			fprintf(fout, "}},{%d", sheetEdgeReg[ i ][ j ].posInMeshEdge + 1 );
			//crspCtrEdges
			dir = 1;
			if( !( sheetEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].samedirec ))
				dir = 0;
			fprintf(fout, ",{{%d,%d,%d}",sheetEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].facei + 1,
				sheetEdgeReg[ i ][ j ].crspCtrEdges[ 0 ].edgei + 1 ,dir	);
			for(unsigned int k = 1; k < sheetEdgeReg[ i ][ j ].crspCtrEdges.size(); k ++ )
			{
				dir = 1;
				if( !( sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].samedirec))
					dir = 0;
				fprintf( fout, ",{%d,%d,%d}", sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].facei + 1,
					sheetEdgeReg[ i ][ j ].crspCtrEdges[ k ].edgei + 1, dir	);
			}
		}
		fprintf( fout, "}}");
	}
	
	//dcontour
	//print out all the divided contours out
	

	//jptnum
	//seamnum
	//facenum

	fclose( fout );
}