#include "../Ctr2SufManager/Ctr2SufManager.h"


/************************************************************************/
/* Preprocess the data, to resize the data to show and resize the data to 1000
for process to avoid numerical issue, and make uniform possible
*/
/************************************************************************/
void Ctr2SufManager::resize()
{
	inresize = true;

//	clearContour();	//clear all the read in contour
	clearPData();	//clear the prcessed contour: show contour and preprocessed contour

	float maxlen = 0;
	for( int i = 0; i < 3; i++)
	{
		center[ i ]  = (bbox[ i ] + bbox[ i + 3 ])/2;
		float len = bbox[ i + 3 ] - bbox[ i ];
		if( len > maxlen )
			maxlen = len;
	}
	unitlen = maxlen / 2;
	//kw added,important in scaling!!
	unitlen=1.0;

	planenum = ctrvers.size();

	showplanenum = planenum;
	showctrvers = new float*[ planenum ];
	showctredges = new int*[ planenum ];
	showctredgenum = new int[ planenum ];

	pctrvers = new float* [ planenum ];
	pctrvernum = new int[ planenum ];
	pctredges = new int*[ planenum ];
	pctredgenum = new int[ planenum ];
	//kw: plus the parameters for the 6 faces of the bounding box
	pparam = new float[ planenum * 4 + 24 ]; 

	//////////////////////////////////////////////////////////////////////////
//	cout<<"planenum:"<<planenum<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < planenum; i ++)
	{
		int len = ctrvers[ i ].size();
		pctrvernum[ i ] = len/3;
		//////////////////////////////////////////////////////////////////////////
	//	cout<<len<<" ";
		//////////////////////////////////////////////////////////////////////////
		pctrvers[ i ] = new float[ len ];
		showctrvers[ i ] = new float[ len ];
		for( int j = 0; j < len; j ++)
		{
			showctrvers[ i ][ j ] = ctrvers[ i ][ j ];
			if( (j + 1)%3 == 0 )
			{
				for( int k = 0; k < 3; k ++)
				{
					int ti = j - 2 + k;
					showctrvers[ i ][ ti ] = (showctrvers[ i ][ ti ] - center[ k ])/unitlen;
					//showctrvers[ i ][ ti ] = (showctrvers[ i ][ ti ] - center[ k ]);///unitlen;//kw modified
				}
			}
		}
		memcpy( pctrvers[ i ], showctrvers[ i ], len * sizeof( float ));

		//resize them to their own size
		for( int j = 0; j < len; j ++)
		{
			showctrvers[ i ][ j ] = showctrvers[ i ][ j ] * DIM;
			pctrvers[ i ][ j ] = pctrvers[ i ][ j ] * PROCSIZE;
		}

		//edge
		len = ctredges[ i ].size();
		showctredgenum[ i ] = pctredgenum[ i ] = len/4;		
		pctredges[ i ] = new int[ len ];
		for( int j = 0; j < len ;j ++)
		{
			pctredges[ i ][ j ] = ctredges[ i ][ j ];
		}
		int tlen = len / 2;
		showctredges[ i ] = new int[ tlen ];
		for( int j = 0; j < tlen/2; j ++ )
		{
			showctredges[ i ][ 2*j ] = pctredges[ i ][ 4 * j ];
			showctredges[ i ][ 2*j + 1] = pctredges[ i ][ 4 * j + 1 ];
		}

		//param
		for( int j = 0; j < 3;j ++)
		{
			pparam[ 4*i + j ] = param[ 4*i + j ];
		}
	//	MyMath::normalize( pparam + 4*i );
		//for the d value, from 10 first points
		pparam[ 4*i + 3] = MyMath::dotProduct( pparam+4*i, pctrvers[ i ]);
		/*float d = 0;
		int ptNum = min( 50, pctrvernum[ i ]);
		for( int j = 0; j < ptNum; j ++ )
		{
			d += MyMath::dotProduct( pparam + 4*i , pctrvers[ i ] + 3* j )/ptNum;
		}
		pparam[ 4*i + 3 ] = d;*/

		//project all the contour vertices onto the plane.
		//this is caused by the precision of the plane is not good enough
		/*for( int j = 0; j < pctrvernum[ i ]; j ++ )
		{
			float dis = MyMath::dotProduct( pparam + 4*i , pctrvers[ i ] + 3*j ) - pparam[ 4*i + 3 ];
			MyMath::getPtOnSeg( pctrvers[ i ] + 3*j , pparam + 4*i , -dis, pctrvers[ i ] + 3*j );
		}*/

		float paramlen = MyMath::vectorlen( pparam + 4*i );
		for( int j = 0; j< 4; j++)
			pparam[ 4*i + j ] = pparam[ 4*i + j ]/paramlen;
	}

	for( int i = 0; i < 3; i ++)
	{
		pbbox [ i ] = (bbox[ i ] - center[ i ])*PROCSIZE/unitlen;
		pbbox[ i + 3 ] = ( bbox[ i + 3 ] - center[ i ] )*PROCSIZE/unitlen;
	}

	//////////////////////////////////////////////////////////////////////////
	//cout<<"exist resize safely!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	inresize = false;
}

void Ctr2SufManager::clearPData()
{
	//data process
	//planenum = 0;
	if( pparam != NULL )
	{
		delete []pparam;
		for( int i = 0; i < planenum; i ++)
		{
			//may have already been cleaned in Ctr2SufManager::clearContour()
			if (showctrvers!=NULL)
			{
				delete []showctrvers[ i ];
			}
			delete []showctredges[ i ];
			delete []pctrvers[ i ];
			delete []pctredges[ i ];
		}
		delete []showctrvers;
		delete []showctredges;
		delete []showctredgenum;

		delete []pctredges;
		delete []pctrvers;
		delete []pctrvernum;
		delete []pctredgenum;
		pparam = NULL;
		showctrvers = NULL;
		showctredgenum = NULL;
		showctredges = NULL;
		pctredgenum = NULL;
		pctredges = NULL;
		pctrvers = NULL;
		pctrvernum =  NULL;

		if( ver2planelistpos != NULL )
		{
			for( int i =0; i < planenum; i++)
				delete []ver2planelistpos[ i ];
			delete []ver2planelistpos;
		}

		for(unsigned int i = 0; i < ver2planelist.size(); i++)
			ver2planelist[ i ].clear();
		ver2planelist.clear();
	}
	planenum = 0;
	ispreproced = false;
}

//write the preprocessed data out!
void Ctr2SufManager::writePreProcData()
{
	FILE* fout = fopen( "preprocdata.txt", "w");
	if( fout == NULL )
	{
		cout<<"Unable to open the preprocdata.txt to write!"<<endl;
		return;
	}

	//planenum
	fprintf( fout, "%d\r\n", planenum );
	
	//for each vertex, if ver2planelistpos is not -1, output vertex position and the planelist
	for( int i = 0; i < planenum; i ++ )
	{
		fprintf(fout, "//plane %d\r\n", i );
		for( int j = 0; j < pctrvernum[ i ]; j ++)
		{
			int tpos = ver2planelistpos[ i ][ j ] ;
			if ( tpos != -1 )
			{
				fprintf( fout, "%d %d %f %f %f\r\n", 
				j, tpos, pctrvers[ i ][ 3*tpos ], pctrvers[ i ][ 3*tpos + 1 ], pctrvers[ i ][ 3*tpos + 2 ]);
				fprintf( fout, "%d", ver2planelist[ tpos ][ 0 ]);
				for(unsigned int k = 1; k < ver2planelist[ tpos ].size(); k ++ )
					fprintf( fout, " %d", ver2planelist[ tpos ][ k ]);
				fprintf( fout, "\r\n");
			}
		}
	}

	fclose( fout );
}
//set the ver2planelist
int** Ctr2SufManager::preProcData(bool ISCOMMONLINE /* = false */)
{
	ispreproced = true;
	//set the ver2planelistpos, and ver2planelist
	if( !ISCOMMONLINE )
	{
		ContourHandler::preProcDataSingleMat(planenum,pparam,pctrvers,pctrvernum,pctredges,pctredgenum,
		 ver2planelistpos, ver2planelist);
		return NULL;
	}
	else
	{
		return ContourHandler::makeConsistentSingleMatCommonLine(
			planenum,  pparam, pctrvernum, pctrvers, pctredgenum,  pctredges,
			comndir, comnpt);
	}	
}

/************************************************************************/
/* Put contours into right faces
/************************************************************************/
void Ctr2SufManager::putContourIntoFace(int** ctrverMark)
{	
	//initialize these data
	this->ctrfvernum = new int[ ssfacenum ];
	this->ctrfverpos = new float*[ ssfacenum ];
	this->ctrfvertype = new int*[ ssfacenum ];
	this->ctrfverval = new int*[ ssfacenum ] ;

	this->ctrfedgenum = new int[ ssfacenum ];
	this->ctrfedge = new int*[ ssfacenum ];
	this->ctrfedgetype = new int*[ ssfacenum ];
	this->ctrfedgeval = new int*[ ssfacenum];
	this->ctrfedgeancestor = new int*[ ssfacenum ];
	for( int i = 0; i < ssfacenum; i++)
	{
		this->ctrfvernum [ i ] = this->ctrfedgenum[ i ] = 0;
		this->ctrfverpos[ i ] = NULL;
		this->ctrfvertype[ i ] = NULL;
		this->ctrfverval[ i ] = NULL;

		this->ctrfedge[ i ] = NULL;
		this->ctrfedgetype[ i ] = NULL;
		this->ctrfedgeval[ i ] = NULL;
		this->ctrfedgeancestor[ i ] = NULL;
	}

	if( !isComnCase)
		ContourHandler::putContourIntoFace(planenum, ssvernum, 
		ssver, ssedgenum, ssedge, ssfacenum, ssfaceedgenum, ssface, ssface_planeindex, ssspacenum,
		ssspacefacenum, ssspace, pctrvernum, pctrvers, pctredgenum, pctredges, pparam, ver2planelistpos,ver2planelist,
		ctrfvernum, ctrfverpos, ctrfvertype, ctrfverval, ctrfedgenum, ctrfedge, ctrfedgetype, ctrfedgeval, ctrfedgeancestor);
	else
		ContourHandler::putContourIntoFaceCMNL(planenum, ssvernum, ssver,
		ssedgenum, ssedge, ssfacenum, ssfaceedgenum, ssface, ssface_planeindex, ssspacenum, ssspacefacenum, 
		ssspace, pctrvernum, pctrvers, pctredgenum, pctredges, pparam, ctrverMark, comndir,
		comnpt, comnedgei, comnveri, ctrfvernum, ctrfverpos, ctrfvertype, ctrfverval, 
		ctrfedgenum, ctrfedge, ctrfedgetype, ctrfedgeval, ctrfedgeancestor	);
}

void Ctr2SufManager::clearFaceContour()
{
	if( ctrfvernum != NULL )
	{
		for( int i = 0; i < ssfacenum; i ++)
		{
			if( ctrfverpos[ i ] != NULL) 
				delete []ctrfverpos[ i ];
			if( ctrfvertype[ i ] != NULL )
				delete []ctrfvertype[ i ];
			if( ctrfverval[ i ] != NULL )
				delete []ctrfverval[ i ];
			if( ctrfedge[ i ] != NULL )
				delete []ctrfedge[ i ];
			if( ctrfedgetype[ i ] != NULL )
				delete []ctrfedgetype[ i ];
			if( ctrfedgeval[ i ] != NULL )
				delete []ctrfedgeval[ i ] ;
			if( ctrfedgeancestor[ i ] != NULL )
				delete []ctrfedgeancestor[ i ];
		}
		delete []ctrfvernum;
		delete []ctrfverpos;
		delete []ctrfvertype;
		delete []ctrfverval;
		delete []ctrfedgenum;
		delete []ctrfedge;
		delete []ctrfedgetype;
		delete []ctrfedgeval;
		delete []ctrfedgeancestor;
		ctrfvernum = NULL ;
		ctrfverpos = NULL ;
		ctrfvertype = NULL ;
		ctrfverval = NULL ;
		ctrfedgenum = NULL ;
		ctrfedge = NULL ;
		ctrfedgetype = NULL ;
		ctrfedgeval = NULL ;
		ctrfedgeancestor = NULL;
	}
}

/************************************************************************/
/*  
Process contour of each face
*/
/************************************************************************/
//return true if the contours in this subspace is not null
bool Ctr2SufManager::gatherSubspaceCtr(vector<SSPCTRVERVEC>& sspctrver_vec,vector<SSPCTREDGEVEC>& sspctredge_vec, int spacei )
{
	bool isnotnull = false; 
	sspctrver_vec.resize( ssspacefacenum[ spacei ]);
	sspctredge_vec.resize( ssspacefacenum[ spacei ]);
	for ( int i = 0; i < ssspacefacenum[ spacei ]; i++)
	{
		 int facei = ssspace[ spacei ][ i ];
		 if( ctrfvernum[ facei ] == 0 )
		 {
			 sspctrver_vec[ i ].resize( 0 );
			 continue;
		 }
		 isnotnull = true;
		 //push down all the vertices
		 sspctrver_vec[ i ].resize( ctrfvernum[ facei ]);
		 for( int j = 0; j < ctrfvernum[ facei ] ; j ++ )
		 {
			 for( int k = 0; k < 3; k++)
			 {
				 sspctrver_vec[ i ][ j ].pos[ k ] = ctrfverpos[ facei ][ 3*j + k];
			 }
			sspctrver_vec[ i ][ j ].type = ctrfvertype[ facei ][ j ];
			sspctrver_vec[ i ][ j ].val = ctrfverval[ facei ][ j ];
		 }

		 //push down all the edges
		 sspctredge_vec[ i ].resize( ctrfedgenum[ facei ]);
		 for( int j = 0; j < ctrfedgenum[ facei ]; j ++)
		 {
			 for(int k = 0; k <2; k ++)
			 {
				sspctredge_vec[ i ][ j ].veris[ k ] = ctrfedge[ facei ][ j*4 + k];
				sspctredge_vec[ i ][ j ].mat[ k ] = ctrfedge[ facei ][ j*4 + 2 + k ];
			 }
             sspctredge_vec[ i ][ j ].type = ctrfedgetype[ facei ][ j ];
			 sspctredge_vec[ i ][ j ].val = ctrfedgeval[ facei ][ j ];
			 sspctredge_vec[ i ][ j ].ancestor = ctrfedgeancestor[ facei ][ j ];
		 }
	}
	return isnotnull;
}

void Ctr2SufManager::divideFaceContourByProjMA(
		int spacei,
		//contour
	//	SUBSPACECTRVER**& sspctrver,int& sspctrvernum,	//final result
	//	SUBSPACECTREDGE**& sspctredge,int& sspctredgenum,
		vector<SSPCTRVERVEC>& sspctrver_vec,	//temporary result
		vector<SSPCTREDGEVEC>& sspctredge_vec,
		//ma
		int subvernum,float* subver,int subedgenum,int* subedge,int subfacenum,int* subfaceedgenum,int** subface,
		float* subparam,int* subver2wver,int* subedge2wedge,int majptnum,float* majpt,int maseamnum,
		int* maseam,MapArraySR& doubleface2sheet,int* seamonsheetnum,int** seamonsheet,float* sheettab,int* ver2jpt)
{
	//divide each face
	for( int i = 0 ; i < ssspacefacenum[ spacei ]; i++)
	{
		if( sspctredge_vec[ i ].size()!=0)
			SSContour::divideOneFaceContour(spacei, i, sspctrver_vec[ i ], sspctredge_vec[ i ], subvernum, subver, subedgenum,
		subedge, subfacenum, subfaceedgenum, subface, subparam, subver2wver, subedge2wedge, majptnum, majpt, maseamnum,
		maseam, doubleface2sheet, seamonsheetnum, seamonsheet, ver2jpt);
	}
}