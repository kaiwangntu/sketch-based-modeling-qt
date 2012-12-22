#include "mesh.h"
Mesh::Mesh()
{
	//color
	for( int i = 0; i< 18; i++)
	{
		for( int j = 0; j < 3; j++)
			fcols[i][j] = cols[i][j]/255;
	}

	//data
	memset( center, 0, sizeof(float)*3);
	unitlen = 1;
	ctrplanenum = 0;
	ctrplaneparam = NULL;
	ctrver = NULL;
	ctrvernum = NULL;
	ctredge = NULL;
	ctredgenum = NULL;
	ctrtriconfig = NULL;
	ctrtrinum = NULL;

	sufvernum = 0;
	suffacenum = 0;
	sufver = NULL;
	sufface = NULL;
	sufmat = NULL;

	stdverlist = NULL;
	stdvernum = 0;
	stdfacelist = NULL;
	stdfacenum = 0;
	showStdMesh = true;

	interpolate = true;

	//render
	smoothshading = false;
	wireframe = false;
	//render option
	showContour = true;
	showMesh = true;
	showAll = true;
	flipOut = false;
	curmat = -1;
	showOutline = true;
	//debug
	debugpts.clear();
	curdebugpt = 0;
}
void Mesh::setGLParam(int width, int height, int nearplane, int farplane)
{
	this->width = width;
	this->height = height;
	this->nearplane = nearplane;
	this->farplane = farplane;
}

void Mesh::renderContour()
{
	//draw all the triangles
	if(!wireframe)
	{
		glBegin(GL_TRIANGLES);
		float norm[3];
		for( int i = 0; i< ctrplanenum; i ++)
		{
			//		if( (i == 2) ||( i == 3))continue;
			for( int j = 0; j < ctrtrinum[ i ]; j ++)
			{
				int mati = ctrtriconfig[ i ][ 4*j + 3 ];
				glColor3fv( fcols[(mati)%18]);
				memcpy(norm, &ctrplaneparam[i*4], sizeof(float)*3);
				MyMath::normalize(norm);
				glNormal3fv( norm );
				//	glNormal3f( ctrplaneparam[i*4], ctrplaneparam[i*4+1], ctrplaneparam[i*4+2]);
				for( int k = 0; k < 3; k ++)
				{
					glVertex3fv( &ctrver[i][ 3*ctrtriconfig[i][j*4+k] ]);
				}
			}
			norm[ 0 ] = - norm[0]; norm[1] = - norm[1]; norm[2] = -norm[2];
			for( int j = 0; j < ctrtrinum[ i ]; j ++)
			{
				int mati = ctrtriconfig[ i ][ 4*j + 3 ];
				glColor3fv( fcols[(mati)%18]);
				glNormal3fv(norm);
				//	glNormal3f( -ctrplaneparam[i*4], -ctrplaneparam[i*4+1], -ctrplaneparam[i*4+2]);
				for( int k = 2; k >= 0; k --)
				{
					glVertex3fv( &ctrver[i][ 3*ctrtriconfig[i][j*4+k] ]);
				}
			}
		}
		glEnd();
	}
	//show the edges
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height,nearplane, farplane * 3 );
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
	glLineWidth( 5 ) ;
	glColor3f(1,0,1);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	for( int i = 0; i < ctrplanenum; i++)
	{
		//glColor3fv( fcols[i%18] );
		for(int j = 0 ; j < ctredgenum[i]; j++)
		{

			for( int k = 0; k < 2; k++)
			{
				glVertex3fv( &ctrver[i][3 * ctredge[ i ][ j * 2 + k]]);
			}
		}
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
}
void Mesh::renderMeshAll()
{
	//////////////////////////////////////////////////////////////////////////
	//cout<<"in mesh rendermeshall!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	if( wireframe )
	{
		glMatrixMode(GL_MODELVIEW);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	else
	{
		glMatrixMode(GL_MODELVIEW);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	glBegin(GL_TRIANGLES);
	float norm[ 3 ];
	if(flipOut)
	{
		for( int i = 0; i < suffacenum; i ++)
		{
			glColor3fv( fcols[ sufmat[ i * 2 + 1 ]]);
			glNormal3fv( &suffacenorm[ i*3 ]);
			for( int k = 0; k < 3; k ++)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
		for( int i = 0; i < suffacenum; i ++)
		{
			glColor3fv( fcols[ sufmat[ i * 2 ]]);
			memcpy( norm, &suffacenorm[ i*3 ], sizeof( float )* 3);
			MyMath::getNeg( norm );
			glNormal3fv(norm);
			for( int k = 2; k >= 0; k --)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
	}
	else
	{
		for( int i = 0; i < suffacenum; i ++)
		{
			glColor3fv( fcols[ sufmat[ i * 2  ]]);
			glNormal3fv( &suffacenorm[ i*3 ]);
			for( int k = 0; k < 3; k ++)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
		for( int i = 0; i < suffacenum; i ++)
		{
			glColor3fv( fcols[ sufmat[ i * 2 + 1]]);
			memcpy( norm, &suffacenorm[ i*3 ], sizeof( float )* 3);
			MyMath::getNeg( norm );
			glNormal3fv(norm);
			for( int k = 2; k >= 0; k --)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
	}
	glEnd();

	/***show if contour edge is loaed correclty****/
	/*	glColor3f(0,0,0);
	glBegin(GL_LINES);
	for( int i = 0; i < sufctredgenum; i ++)
	{
	for( int j = 0; j < 2; j ++)
	{
	glVertex3fv( &sufver[sufctredge[ i * 2 + j] * 3] );
	}
	}
	glEnd();*/
	//show outline
	if(wireframe) return;
	if( !showOutline )return;
	glColor3f(0,0,0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height,nearplane, farplane * 3 );
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
	glLineWidth( 1 ) ;

	// draw here
	glDisable(GL_LIGHTING);
	HashMap ver2edge;
	int edgenum = 0;
	glBegin(GL_LINES);
	for( int i = 0; i < suffacenum; i ++)
	{
		//	glColor3fv( fcols[ sufmat[ i * 2 + 1 ]]);
		//	glNormal3fv( &suffacenorm[ i*3 ]);
		for( int k = 0; k < 3; k ++)
		{
			if( ver2edge.findInsertSort(sufface[ i * 3 + k] , sufface[ i * 3 + (k+1)%3 ], edgenum) == edgenum ){
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
				glVertex3fv( &sufver[sufface[ i * 3 + (k+1)%3 ] * 3]);
				edgenum++;
			}
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
	// restore the farplane
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height, nearplane, farplane);
	// get back to GL_MODELVIEW matrix
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
}
void Mesh::renderMeshCurComponent()
{
	if( curmat < 0 || curmat >= matlist.size())
	{
//		fl_alert("Currentmat is not valid!");
		return;
	}
	glBegin(GL_TRIANGLES);
	float norm[ 3 ];
	if(flipOut)
	{
		for( int i = 0; i < suffacenum; i ++)
		{
			if (sufmat[ i * 2 + 1 ] != matlist[curmat] && sufmat[ i * 2 ] != matlist[curmat])
				continue;
			glColor3fv( fcols[ sufmat[ i * 2 + 1 ]]);
			glNormal3fv( &suffacenorm[ i*3 ]);
			for( int k = 0; k < 3; k ++)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
		for( int i = 0; i < suffacenum; i ++)
		{
			if (sufmat[ i * 2 + 1 ] != matlist[curmat]&& sufmat[ i * 2 ] != matlist[curmat])
				continue;
			glColor3fv( fcols[ sufmat[ i * 2 ]]);
			memcpy( norm, &suffacenorm[ i*3 ], sizeof( float )* 3);
			MyMath::getNeg( norm );
			glNormal3fv(norm);
			for( int k = 2; k >= 0; k --)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
	}
	else
	{
		for( int i = 0; i < suffacenum; i ++)
		{
			if (sufmat[ i * 2 + 1 ] != matlist[curmat] && sufmat[ i * 2 ] != matlist[curmat])
				continue;
			glColor3fv( fcols[ sufmat[ i * 2  ]]);
			glNormal3fv( &suffacenorm[ i*3 ]);
			for( int k = 0; k < 3; k ++)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
		for( int i = 0; i < suffacenum; i ++)
		{
			if (sufmat[ i * 2 + 1 ] != matlist[curmat] && sufmat[ i * 2 ] != matlist[curmat])
				continue;
			glColor3fv( fcols[ sufmat[ i * 2 + 1]]);
			memcpy( norm, &suffacenorm[ i*3 ], sizeof( float )* 3);
			MyMath::getNeg( norm );
			glNormal3fv(norm);
			for( int k = 2; k >= 0; k --)
			{
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
			}
		}
	}
	glEnd();

	//show outline
	if(wireframe) return;
	if( !showOutline )return;
	glColor3f(0,0,0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height,nearplane, farplane * 3 );
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_LINE );
	glLineWidth( 1 ) ;
	glColor3f(0,0,0);

	HashMap ver2edge;
	int edgenum = 0;
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	for( int i = 0; i < suffacenum; i ++)
	{
		if (sufmat[ i * 2 + 1 ] != matlist[curmat] && sufmat[ i * 2 ] != matlist[curmat])
			continue;
		for( int k = 0; k < 3; k ++)
		{
			if( ver2edge.findInsertSort(sufface[ i * 3 + k] , sufface[ i * 3 + (k+1)%3 ], edgenum) == edgenum ){
				glVertex3fv( &sufver[sufface[ i * 3 + k] * 3]);
				glVertex3fv( &sufver[sufface[ i * 3 + (k+1)%3] * 3]);
				edgenum++;
			}
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
	// restore the farplane
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (float)width/(float)height, nearplane, farplane);
	// get back to GL_MODELVIEW matrix
	glMatrixMode(GL_MODELVIEW);
	glPolygonMode ( GL_FRONT_AND_BACK, GL_FILL );
}
void Mesh::renderMesh()
{	
	glLineWidth(1);
	if( showAll )
		renderMeshAll();
	else
		renderMeshCurComponent();
}
void Mesh::renderStdMesh( )
{
	if( stdvernum == 0 )
		return;
	glDisable( GL_LIGHTING );
	glPointSize( 3 );
	glColor3f(1, 0, 0);
	glBegin( GL_POINTS);
	for( int i = 0; i < stdvernum; i ++)
	{
		glVertex3fv( stdverlist + 3 * i );
	}
	glEnd();

	glEnable( GL_LIGHTING);
}
void Mesh:: renderSelectedVer()
{
	glPointSize(2);
	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	for(unsigned int i = 0; i < selectVerList.size(); i++)
	{
		glVertex3fv( &sufver[selectVerList[ i ] * 3] );
	}
	glEnd();
}
//render the debug stuff
//void Mesh::renderDBStuff()
//{
//	glDisable(GL_LIGHTING);
//	glColor3f(1,0,1);
//	glLineWidth(4);
//	glBegin(GL_LINES);
//	for( int i = 0; i < dbnmedgelist.size(); i ++)
//	{
//		int edgei = dbnmedgelist[ i ] ;
//		int vers[ 2 ]  = { dbedgelist[ edgei * 2 ], dbedgelist[ edgei * 2 + 1] };
//		glVertex3fv( &sufver[ 3 * vers[ 0 ]]);
//		glVertex3fv( &sufver[ 3* vers[1]]);
//	}
//	glEnd();
//	glEnable(GL_LIGHTING);
//}
void Mesh::render()
{
	//render mesh
	if( showMesh && sufver != NULL )
		renderMesh();
	//render contour
	if( showContour && ctrver != NULL)
		renderContour();
	if( showStdMesh)
		renderStdMesh();

	glDisable(GL_LIGHTING);
	renderSelectedVer();
	/*------------------------*/
	if( debugpts.size() > 0 )
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		//	glColor3f(1,0,0);
		glColor3f(0,0,1);
		glVertex3fv( &sufver[ 3*debugpts[0]] );
		//	glColor3f(0,0,1);
		//cout<<"total number:"<<debugpts.size()<<endl;
		for(unsigned int i = 1; i < debugpts.size(); i ++)
		{
			glVertex3fv( &sufver[ 3*debugpts[i]] );
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);

//	renderDBStuff();
	//	glDisable(GL_LIGHTING);
	//	if(dbnmedgelist.size() > 0)
	//	{
	////		cout<<"to render edge number:"<<dbnmedgelist.size() <<endl;
	//		glLineWidth(5);
	//		glColor3f(1,0 , 0);
	//		glBegin(GL_LINES);
	//		
	//		for( int i = 0; i < dbnmedgelist.size()/2; i ++)
	//		{
	//			glVertex3fv(&sufver[3*dbnmedgelist[i*2]]);
	//			glVertex3fv(&sufver[ 3*dbnmedgelist[i*2+1]]);
	//			
	//		//	cout<<sufver[3*dbnmedgelist[i*2]]<<" "<<sufver[3*dbnmedgelist[i*2]+1]<<" "<<sufver[3*dbnmedgelist[i*2]+2]<<endl;
	//		//	cout<<sufver[3*dbnmedgelist[i*2+1]]<<" "<<sufver[3*dbnmedgelist[i*2+1]+1]<<" "<<sufver[3*dbnmedgelist[i*2+1]+2]<<endl;
	//		}
	//		glEnd();
	//	//	glLineWidth(2);
	//	}
	//
	//	/*glColor3f(1,0,0);
	//	glBegin(GL_LINES);
	//	for( int i = 0; i < sufctredgenum; i ++)
	//	{
	//		glVertex3fv( &sufver[ 3*sufctredge[ i*2 ] ]);
	//		glVertex3fv( &sufver[ 3*sufctredge[i * 2 + 1]]);
	//	}
	//	glEnd();*/
	//
	//	glEnable(GL_LIGHTING);

	/*------------------------*/
	/*-------------------*/
	//render different type of vertex
	/*	glPointSize(8);
	glBegin(GL_POINTS);
	for( int i = 0; i < vermark.size(); i++)
	{
	if(vermark[ i ] == 1)
	{
	glColor3f(1, 0, 0);
	glVertex3fv( &sufver[ 3*i ]);
	}
	else if( vermark[ i ] == 2)
	{
	glColor3f(0,1,0);
	glVertex3fv( &sufver[3*i]);
	}
	else if( vermark[ i ] == 3)
	{
	glColor3f( 0, 0, 1);
	glColor3fv( &sufver[3*i]);
	}
	}
	glEnd();*/
	//render all the different types of edges
	//////////////////////////////////////////////////////////////////////////
	/*	glColor3f( 1, 0, 0);
	glLineWidth( 4 );
	glDisable(GL_LIGHTING);
	glBegin( GL_LINES);
	//	glVertex3f(0,0,0);
	//	glVertex3f(1,1,1);
	for(int i = 0; i < ctredgelist.size(); i ++)	//contour edge
	{
	glVertex3fv(&sufver[ edgelist[ ctredgelist[ i ] * 2 ] * 3 ]);
	glVertex3fv(&sufver[ edgelist[ ctredgelist[ i ] * 2 + 1 ] * 3 ]);
	}
	glColor3f( 0, 1, 0);
	for(int i = 0; i < nmedgelist.size(); i ++)	//contour edge
	{
	glVertex3fv(&sufver[ edgelist[ nmedgelist[ i ] * 2 ] * 3 ]);
	glVertex3fv(&sufver[ edgelist[ nmedgelist[ i ] * 2 + 1 ] * 3 ]);
	}
	glEnd();
	glEnable(GL_LIGHTING);*/
	//////////////////////////////////////////////////////////////////////////
	/*-------------------*/
}
void Mesh::renderSelectableShapes()
{
	//render all the vertices
	glPointSize( 2 ) ;
	glColor3f( 1, 0, 0 ) ;
	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		glLoadName( i ) ;		
		glBegin( GL_POINTS ) ;
		glVertex3f(sufver[ i * 3 ], sufver[ i * 3 + 1 ], sufver[ i * 3 + 2 ] ) ;	
		glEnd() ;
	}
}
void Mesh::toggleShading()
{
	smoothshading = !smoothshading;
}
void Mesh::toggleWireframe(bool wireframe)
{
	this->wireframe = wireframe;
}
void Mesh::toggelShowContour()
{
	showContour = !showContour;
}
void Mesh::toggleShowMesh()
{
	showMesh = !showMesh;
}
void Mesh::toggleShowAll(bool showAll)
{
	this->showAll = showAll;
}
void Mesh::toggleFlipOut()
{
	flipOut = !flipOut;
}
void Mesh::toggleShowOutline(int showoutline)
{
	if( showoutline == 0)
		this->showOutline = false;
	else
		this->showOutline = true;
}
void Mesh::toggleNext()
{
	if( matlist.size() == 0)
		return;
	curmat = (curmat+1)%matlist.size();
}
void Mesh::writeMeshJu(const char* fname)
{
	FILE* fout = fopen( fname, "w");
	cout<<"writing ..... "<<fname;
	if( fout == NULL)
	{
		cout<<"Unable to open the file!"<<endl;
		return;
	}
	fprintf(fout, "%d\n", sufvernum);
	for( int i = 0; i < sufvernum; i ++)
	{
		fprintf(fout, "%f %f %f\n", sufver[ 3*i ]*unitlen+center[0], sufver[ 3*i + 1]*unitlen+center[1], sufver[3*i+2]*unitlen+center[2]);
	}
	fclose(fout);
	cout<<"done!"<<endl;
}
//void Mesh::writeMeshJu(const char* fname)
//{
//	cout<<"writing mesh out....\t";
//	//write out face by using vertices positions
//	FILE* fout = fopen(fname, "w");
//	if( fout  == NULL)
//	{
//		fl_alert("Unable write the mesh!");
//		return;
//	}
//
//	fprintf(fout, "{{");
//	//write the first face out
//	fprintf(fout, "{{%f,%f,%f}", sufver[3*sufface[0]], sufver[3*sufface[0] + 1], sufver[3*sufface[0] + 2]);
//	for(int i = 1; i < 3; i ++)
//	{
//		fprintf(fout, ",{%f,%f,%f}", sufver[ 3*sufface[i]], sufver[ 3*sufface[i] + 1], sufver[ 3*sufface[i] + 2]);
//	}
//	fprintf(fout, "}");		//for the end of first face
//	for( int i = 1; i< suffacenum;  i ++)
//	{
//		fprintf(fout, ",");		//the separater
//
//		//first vertex of the face
//		fprintf(fout, "{{%f,%f,%f}", sufver[3*sufface[ 3* i ]], sufver[3*sufface[  3* i ] + 1], sufver[3*sufface[  3* i ] + 2]);
//		for(int j = 1; j < 3; j ++)
//		{
//			fprintf(fout, ",{%f,%f,%f}", sufver[ 3*sufface[ 3* i + j ]], sufver[ 3*sufface[ 3*i + j ] + 1], sufver[ 3*sufface[ 3* i + j ] + 2]);
//		}
//		fprintf(fout, "}");		//for the end of current face
//	}
//	fprintf(fout, "},{");			//for the end of all the faces and the begin of the contour vertex
//
//	//write out contour vertex by using its position
//	int* vermark = new int[ sufvernum ];
//	memset(vermark, 0, sizeof(int)*sufvernum);
//	for( int i = 0; i < sufctredgenum*2; i++)
//	{
//		vermark[ sufctredge[i] ] = 1;
//	}
//	//first contour vertex
//	bool first = true;
//	for( int i = 0; i < sufvernum; i ++)
//	{
//		if( vermark[ i ] == 0) continue;
//		if( first)
//		{
//			first = false;
//			fprintf(fout, "{%f,%f,%f}", sufver[ 3*i ], sufver[ 3* i + 1 ], sufver[ 3*i + 2 ] );
//		}
//		else
//		{
//			fprintf( fout, ",{%f,%f,%f}", sufver[ 3*i ], sufver[ 3* i + 1 ], sufver[ 3*i + 2 ] );
//		}
//	}
//	fprintf(fout, "}}");
//	fclose(fout);
//	cout<<"done\n";
//}
Mesh::~Mesh()
{
	if( ctrplanenum != 0)
	{
		for( int i = 0; i < ctrplanenum; i ++)
		{
			delete []ctrver[i];
			delete []ctredge[i];
			delete []ctrtriconfig[i];
		}
		delete []ctrver;
		delete []ctrplaneparam;
		delete []ctrvernum;
		delete []ctredge;
		delete []ctredge;
		delete []ctredgenum;
		delete []ctrtrinum;
		delete []ctrtriconfig;
	}
	if( sufver != NULL)
		delete []sufver;
	if( sufface != NULL)
		delete []sufface;
	if( sufmat != NULL)
		delete []sufmat;
	matlist.clear();

	if( stdverlist != NULL )
		delete []stdverlist;
	if( stdfacelist != NULL )
		delete []stdfacelist;

	//debug
	/*dbnmedgelist.clear();
	selectVerList.clear();
	debugpts.clear();
	dbedgelist.clear();*/
}