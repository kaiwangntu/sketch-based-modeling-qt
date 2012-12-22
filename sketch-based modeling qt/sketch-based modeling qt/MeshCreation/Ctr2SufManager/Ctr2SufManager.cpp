#include "../Ctr2SufManager/Ctr2SufManager.h"

Ctr2SufManager::Ctr2SufManager()
{
	mesh = NULL;
	inresize = false;

	//render
	showctrvers = NULL;
	showctredges = NULL;
	showctredgenum = NULL;

	//contour
	bboxset = false;
	showContour = true;

	//data process
	center[ 0 ] = center[ 1 ] = center[ 2 ] = 0;
	unitlen = 1;
	uniform = true;
	expectlen = 20;
	planenum = 0;
	pparam = NULL;
	pctrvers = NULL;		
	pctrvernum = NULL;
	pctredges = NULL;
	pctredgenum = NULL;
	ispreproced = false;
	ver2planelistpos = NULL;

	//partition
//	enlargeratio[ 0 ] = 1.2;
//	enlargeratio[ 1 ] = 1.2;
//	enlargeratio[ 2 ] = 100;
	enlargeratio[ 0 ] = enlargeratio[ 1 ] = enlargeratio[ 2 ] = 2;//2;//2;
	ssver = NULL;
	ssedge = ssface_planeindex = ssfaceedgenum = ssspacefacenum = NULL;
	ssface = ssspace = ssspace_planeside = NULL;
	showPartition = true;

	//contour in face
	ctrfvernum = NULL ;
	ctrfverpos = NULL ;
	ctrfvertype = NULL ;
	ctrfverval = NULL ;
	ctrfedgenum = NULL ;
	ctrfedge = NULL ;
	ctrfedgetype = NULL ;
	ctrfedgeval = NULL ;

	//////////////////////////////////////////////////////////////////////////
	dbOneSpaceMode = false;
	dbCurSpace = 0;
	dbSpaceNum = 0;
	//////////////////////////////////////////////////////////////////////////

	//common line case
	isComnCase = false;

	//kw added
	this->bRenderSS=false;
}

Ctr2SufManager::~Ctr2SufManager()
{
	//mesh
	delete mesh;
	
	//Contour
	clearOldContour();
	clearContour();

	//clear processed data
	clearPData();

	//clear partition data
	clearPartition();

	//clear face contour data
	clearFaceContour();
}

void Ctr2SufManager::render()
{
	if( showContour )
		renderContour();

	//////////////////////////////////////////////////////////////////////////
//	cout<<"in render!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	if( mesh != NULL )
	{		
		//////////////////////////////////////////////////////////////////////////
//		cout<<"in render, mesh is not nulll!"<<endl;
			//////////////////////////////////////////////////////////////////////////
		mesh->render();
	}
}

//write the sheettab of one subspace
void Ctr2SufManager::writeSheetTab(
			float* masheettab,
			int masheetnum,
			int spaci
								   )
{
	char fname[ 1024 ] = "mmdebug/onesubspace/sheettab";
	char fnum[ 5 ];
	itoa( spaci+1, fnum, 10 );
	strcat( fname, fnum );
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");
	if( fout == NULL )
	{
		cout<<"unable to open file: "<<fname<<" to write!"<<endl;
		return;
	}
	
	fprintf( fout, "{");
	for( int i = 0; i < masheetnum; i ++ )
	{
		fprintf( fout, "{{%f,%f,%f},{%f,%f,%f}}", masheettab[6*i],
			masheettab[ 6*i+1],masheettab[ 6*i+2], masheettab[ 6*i + 3],
			masheettab[ 6*i + 4], masheettab[ 6*i + 5]);

		if( i != masheetnum - 1)
			fprintf( fout, ",");
	}
	fprintf( fout, "}");
	fclose( fout );	
}