#include "../Ctr2SufManager/Ctr2SufManager.h"

//clear the contour to show
void Ctr2SufManager::clearContour()
{
	if( showctrvers != NULL )
	{	
		delete []showctrvers;
	//	delete []showctredges
	}
	showctrvers = NULL;
}

//clear already added contour
void Ctr2SufManager::clearOldContour()
{
	bboxset = false;
	param.clear();				//the parameters of the planes
	int layer = ctrvers.size();
	for( int i = 0; i < layer; i ++)
	{
		ctrvers[ i ].clear();
		ctredges[ i ].clear();
	}
	ctrvers.clear();
	ctredges.clear();

//	clearContour();
//	showctrvers = NULL;
}
void Ctr2SufManager::readContour(const int filenum, char** fnames)
{
	clearOldContour();
	////clear processed data
	//clearPData();

	////clear partition data
	//clearPartition();

	////clear face contour data
	//clearFaceContour();
	//
	//clear mesh
	delete mesh;
	mesh = NULL;

	addContour(filenum, fnames);

}

void Ctr2SufManager::readContourFromVec(vector<CurveNetwork> vecCurveNetwork,float* fPreSetBBox,vector<vector<Point_3>>& MeshBoundingProfile3D)
{
	clearOldContour();
	//clear mesh
	delete mesh;
	mesh = NULL;

	addContourFromVec(vecCurveNetwork,fPreSetBBox,MeshBoundingProfile3D);
}

void Ctr2SufManager::readContourFromFile(char* fnames,vector<vector<Point_3>>& MeshBoundingProfile3D)
{
	clearOldContour();
	//clear mesh
	delete mesh;
	mesh = NULL;

	ContourHandler::readContourFromFile(fnames, param, ctrvers, ctredges, bbox, bboxset);
	resize();


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

			//kw noted: one closed cross section is totally read
			//may have problem for non-self-defined data
			if (v2==iVerIndBase)
			{
				iVerIndBase=iVerIndBase+CurrentProfile3D.size();
				MeshBoundingProfile3D.push_back(CurrentProfile3D);
				CurrentProfile3D.clear();
			}
		}
	}
}

void Ctr2SufManager::addContour(const int filenum, char** fnames)
{
	ContourHandler::readContour( filenum, fnames, param, ctrvers, ctredges, bbox, bboxset);
	resize();

	//////////////////////////////////////////////////////////////////////////
	//cout<<"exit addcontour safely!"<<endl;
	//////////////////////////////////////////////////////////////////////////
}

void Ctr2SufManager::addContourFromVec(vector<CurveNetwork> vecCurveNetwork,float* fPreSetBBox,vector<vector<Point_3>>& MeshBoundingProfile3D)
{
	ContourHandler::readContourFromVec(vecCurveNetwork,param, ctrvers, ctredges, bbox, bboxset);
	
	//kw: set the bbox to the preset value,instead of the one calculated
	memcpy(bbox,fPreSetBBox,sizeof(float)*6);

	resize();

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
}

void Ctr2SufManager::renderContour()
{
	if( inresize )
		return;
	///////////////////////////////////////////////////////////////////////////
	/*cout<<"rendrecontour now!"<<endl;
	cout<<"planenum:"<<planenum<<endl;
	for( int i = 0; i < planenum; i ++ )
		cout<<showctredgenum[ i ]<<" ";
	cout<<endl;*/
	
	//////////////////////////////////////////////////////////////////////////

//	int ctrnum = ctrvers.size();
	//////////////////////////////////////////////////////////////////////////
	//	cout<<"ctrnum:"<<ctrnum<<endl;
	//////////////////////////////////////////////////////////////////////////
	if( planenum == 0)
		return;

	//show the edges
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height,nearplane, farplane * 3 );
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
	glLineWidth( 5 ) ;
	glColor3f(1,1,0);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	//////////////////////////////////////////////////////////////////////////
	//cout<<"planenum:"<<planenum<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < planenum; i ++)
	{	
		//////////////////////////////////////////////////////////////////////////
		//cout<<"edgenum:"<<showctredgenum[ i ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		for( int j = 0;j < showctredgenum[ i ]; j++)
		{
			int v1 = showctredges[ i ][ 2*j ];
			int v2 = showctredges[ i ][ 2*j + 1 ];
			//		glVertex3fv( &ctrvers[ i ][ v1*3 ]);
			//		glVertex3fv( &ctrvers[ i ][ v2*3 ]);
			//////////////////////////////////////////////////////////////////////////
			//cout<<"("<<v1<<","<<v2<<")"<<" ";
			//////////////////////////////////////////////////////////////////////////
			glVertex3fv( &showctrvers[ i ][ v1* 3] );
			glVertex3fv( &showctrvers[ i ][ v2* 3] );
		}
		//////////////////////////////////////////////////////////////////////////
		//cout<<endl;
		//////////////////////////////////////////////////////////////////////////
	}
	glEnd();
	//glPointSize(6);
	//glColor3f(0,0,0);
	//glBegin(GL_POINTS);
	//for( int i =0; i < ctrplanenum; i ++)
	//{
	//	for( int j = 0;  j< ctrvernum[ i ]; j ++)
	//	{
	//		glVertex3fv(&ctrver[i][3*j]);
	//	}
	//}
	//glEnd();
	glEnable(GL_LIGHTING);
	// restore the farplane
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height, nearplane, farplane);
	// get back to GL_MODELVIEW matrix
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
	glLineWidth( 1 );

	//glDisable(GL_LIGHTING);
	//glBegin(GL_LINES);
	//glColor3f( 1, 0, 1);
	//glLineWidth( 2 );
	////show conotur one by one
	//for( int i = 0; i < planenum; i ++)
	//{	
	//	for( int j = 0;j < pctredgenum[ i ]; j++)
	//	{
	//		int v1 = pctredges[ i ][ 4*j ];
	//		int v2 = pctredges[ i ][ 4*j + 1 ];
	//		//		glVertex3fv( &ctrvers[ i ][ v1*3 ]);
	//		//		glVertex3fv( &ctrvers[ i ][ v2*3 ]);
	//		glVertex3fv( &showctrvers[ i ][ v1* 3] );
	//		glVertex3fv( &showctrvers[ i ][ v2* 3] );
	//	}
	//}
	//glEnd();
	//glEnable(GL_LIGHTING);
}