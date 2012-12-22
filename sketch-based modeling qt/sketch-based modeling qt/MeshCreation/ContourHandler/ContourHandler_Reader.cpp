#include "../ContourHandler/ContourHandler.h"
#include "../math/mymath.h"

/**
* Function read the data from .contour file
* @param filename: the path of the file to read in
* @param param: the parameters of the planes
* @param ctrvers: all the vertices of the contours
* @param ctredges: all the edges of the contours
*/
void ContourHandler::readOneContour(char* filename, 
						   floatvector& param,	vector<floatvector>& ctrvers,vector<intvector>& ctredges,
						   float bbox[ 6 ], bool& bboxset)
{
	FILE* fin = fopen( filename, "r");
//	FILE* fin = fopen( "sshape.contour", "r");

	if( fin == NULL )
	{
		cout<<"Unable to open file:" <<filename<<endl;
		return;
	}

	//contour number
	int ctrnum = 0;
	fscanf( fin, "%d", &ctrnum );

	//////////////////////////////////////////////////////////////////////////
//	cout<<"ctrnum:"<<ctrnum<<endl;
	//////////////////////////////////////////////////////////////////////////

	//read contour one by one
	float tparam[ 4 ];
	float tver[ 3 ];
	int tedge[ 4 ];
	int vernum, edgenum;
	floatvector tvers;
	intvector tedges;
	for( int i = 0; i < ctrnum; i ++)
	{
		//plane parameter
		fscanf( fin, "%f %f %f %f\r\n", tparam, tparam + 1, tparam+2, tparam+3);
		//////////////////////////////////////////////////////////////////////////
		//cout<<"parma:"<<tparam[ 0 ]<<" "<<tparam[ 1 ]<<" "<<tparam[ 2 ]<<" "<<tparam[ 3 ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		param.push_back( tparam[ 0 ]);
		param.push_back( tparam[ 1 ]);
		param.push_back( tparam[ 2 ]);
		param.push_back( tparam[ 3 ]);
			
		//vertex number, edge number
		fscanf( fin, "%d %d\r\n", &vernum,&edgenum);
		
		//////////////////////////////////////////////////////////////////////////
	//	cout<<"vernum"<<vernum<<"edgenum"<<edgenum<<endl;
		//////////////////////////////////////////////////////////////////////////
		//vertices
		for( int j = 0; j < vernum; j ++)
		{
			fscanf( fin, "%f %f %f\r\n", tver, tver+1, tver+2);
			tvers.push_back( tver[ 0 ]);
			tvers.push_back( tver[ 1 ]);
			tvers.push_back( tver[ 2 ]);

			if( !bboxset )
			{
				bbox[ 0 ] = bbox[ 3 ] = tver[ 0 ];
				bbox[ 1 ] = bbox[ 4 ] = tver[ 1 ];
				bbox[ 2 ] = bbox[ 5 ] = tver[ 2 ];
				bboxset = true;
			}
			else
			{
				if( bbox[ 0 ] > tver[ 0 ])
				{
					bbox[ 0 ] = tver[ 0 ];
				}
				else if( bbox[ 3 ] < tver[ 0 ])
				{
					bbox[ 3 ] = tver[ 0 ];
				}
				if( bbox[ 1 ] > tver[ 1 ])
				{
					bbox[ 1 ] = tver[ 1 ];
				}
				else if( bbox[ 4 ] < tver[ 1 ])
				{
					bbox[ 4 ] = tver[ 1 ];
				}
				if( bbox[ 2 ] > tver[ 2 ])
				{
					bbox[ 2 ] = tver[ 2 ];
				}
				else if( bbox[ 5 ] < tver[ 2 ])
				{
					bbox[ 5 ] = tver[ 2 ];
				}				
			}
			//////////////////////////////////////////////////////////////////////////
		//	cout<<tver[ 0 ]<<" "<<tver[ 1 ]<<" "<<tver[ 2 ]<<endl;
			//////////////////////////////////////////////////////////////////////////
		}
		ctrvers.push_back( tvers );

		//edges
		for( int j = 0; j < edgenum; j ++ )
		{
			fscanf( fin, "%d %d %d %d\r\n", tedge, tedge + 1, tedge + 2, tedge + 3 );
			tedges.push_back( tedge[ 0 ]);
			tedges.push_back( tedge[ 1 ]);
			//tedges.push_back( tedge[ 2 ]);
			//tedges.push_back( tedge[ 3 ]);
			tedges.push_back( 1);
			tedges.push_back( 2);

			//////////////////////////////////////////////////////////////////////////
		//	cout<<tedge[ 0 ]<<" "<<tedge[ 1]<<" "<<tedge[ 2 ] <<" "<<tedge[3]<<endl;
			//////////////////////////////////////////////////////////////////////////
		}
		ctredges.push_back( tedges );

		tvers.clear();
		tedges.clear();
	}

	fclose( fin );
}

void ContourHandler::readOneContourFromVec(vector<CurveNetwork> vecCurveNetwork, 
										floatvector& param, vector<floatvector>& ctrvers,vector<intvector>& ctredges,
										float bbox[ 6 ], bool& bboxset)
{
	//read contour one by one
	float tparam[ 4 ];
	float tver[ 3 ];
	int tedge[ 4 ];
	int vernum, edgenum;
	floatvector tvers;
	intvector tedges;
	
	for (unsigned int iPlane=0;iPlane<vecCurveNetwork.size();iPlane++)
	{
		//plane parameter
		tparam[0]=vecCurveNetwork.at(iPlane).plane.a();tparam[1]=vecCurveNetwork.at(iPlane).plane.b();
		tparam[2]=vecCurveNetwork.at(iPlane).plane.c();tparam[3]=vecCurveNetwork.at(iPlane).plane.d();
		//////////////////////////////////////////////////////////////////////////
		//cout<<"parma:"<<tparam[ 0 ]<<" "<<tparam[ 1 ]<<" "<<tparam[ 2 ]<<" "<<tparam[ 3 ]<<endl;
		//////////////////////////////////////////////////////////////////////////
		param.push_back( tparam[ 0 ]);
		param.push_back( tparam[ 1 ]);
		param.push_back( tparam[ 2 ]);
		param.push_back( tparam[ 3 ]);

		//kyewong
		int iEdgeIndBase=0;

		for (unsigned int iCurve=0;iCurve<vecCurveNetwork.at(iPlane).Profile3D.size();iCurve++)
		{
			//vertex number, edge number
			vernum=edgenum=vecCurveNetwork.at(iPlane).Profile3D.at(iCurve).size();
			//////////////////////////////////////////////////////////////////////////
			//	cout<<"vernum"<<vernum<<"edgenum"<<edgenum<<endl;
			//////////////////////////////////////////////////////////////////////////
			//vertices
			for( int j = 0; j < vernum; j ++)
			{
				//			fscanf( fin, "%f %f %f\r\n", tver, tver+1, tver+2);
				tver[0]=vecCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(j).x();
				tver[1]=vecCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(j).y();
				tver[2]=vecCurveNetwork.at(iPlane).Profile3D.at(iCurve).at(j).z();
				tvers.push_back( tver[ 0 ]);
				tvers.push_back( tver[ 1 ]);
				tvers.push_back( tver[ 2 ]);

				if( !bboxset )
				{
					bbox[ 0 ] = bbox[ 3 ] = tver[ 0 ];
					bbox[ 1 ] = bbox[ 4 ] = tver[ 1 ];
					bbox[ 2 ] = bbox[ 5 ] = tver[ 2 ];
					bboxset = true;
				}
				else
				{
					if( bbox[ 0 ] > tver[ 0 ])
					{
						bbox[ 0 ] = tver[ 0 ];
					}
					else if( bbox[ 3 ] < tver[ 0 ])
					{
						bbox[ 3 ] = tver[ 0 ];
					}
					if( bbox[ 1 ] > tver[ 1 ])
					{
						bbox[ 1 ] = tver[ 1 ];
					}
					else if( bbox[ 4 ] < tver[ 1 ])
					{
						bbox[ 4 ] = tver[ 1 ];
					}
					if( bbox[ 2 ] > tver[ 2 ])
					{
						bbox[ 2 ] = tver[ 2 ];
					}
					else if( bbox[ 5 ] < tver[ 2 ])
					{
						bbox[ 5 ] = tver[ 2 ];
					}				
				}
			}
				//////////////////////////////////////////////////////////////////////////
				//	cout<<tver[ 0 ]<<" "<<tver[ 1 ]<<" "<<tver[ 2 ]<<endl;
				//////////////////////////////////////////////////////////////////////////
				//edges
				for( int j = 0; j < edgenum; j ++ )
				{
					//			fscanf( fin, "%d %d %d %d\r\n", tedge, tedge + 1, tedge + 2, tedge + 3 );
					if (j==edgenum-1)
					{
						tedge[0]=j+iEdgeIndBase;tedge[1]=0+iEdgeIndBase;
					}
					else
					{
						tedge[0]=j+iEdgeIndBase;tedge[1]=j+iEdgeIndBase+1;
					}
					tedges.push_back( tedge[ 0 ]);
					tedges.push_back( tedge[ 1 ]);
					//tedges.push_back( tedge[ 2 ]);
					//tedges.push_back( tedge[ 3 ]);
					tedges.push_back( 1);
					tedges.push_back( 2);

					//////////////////////////////////////////////////////////////////////////
					//	cout<<tedge[ 0 ]<<" "<<tedge[ 1]<<" "<<tedge[ 2 ] <<" "<<tedge[3]<<endl;
					//////////////////////////////////////////////////////////////////////////
				}
				iEdgeIndBase=iEdgeIndBase+edgenum;
		}
		ctrvers.push_back( tvers );
		ctredges.push_back( tedges );
		tvers.clear();
		tedges.clear();
	}
}


/**
* Function read the data from .contour file
* @param filenum: number of the files to read contour in
* @param filenames: the paths of the files to read in
* @param param: the parameters of the planes
* @param ctrvers: all the vertices of the contours
* @param ctredges: all the edges of the contours
*/
void ContourHandler::readContour(const int filenum, char** filenames, 
			floatvector& param,	vector<floatvector>& ctrvers,vector<intvector>& ctredges,
			float bbox[ 6 ], bool& bboxset)
{
	//read file one by one
	for( int i = 0; i < filenum; i ++)
	{
		readOneContour(filenames[ i ], param,ctrvers,ctredges,bbox,bboxset);
	}

	//normalize the normals of the planes
	int planenum = param.size()/4;
	for( int i = 0; i < planenum; i ++)
	{
		float veclen = MyMath::vectorLen( param[ i * 4 ], param[ i*4 + 1 ], param[ i*4+ 2 ]);
		for(int j = 0; j < 4; j ++)
			param[ 4*i + j] = param[ 4*i + j]/veclen;
	}
}

void ContourHandler::readContourFromFile(char* filenames, floatvector& param, 
										 vector<floatvector>& ctrvers,vector<intvector>& ctredges, float bbox[ 6 ], bool& bboxset)
{
	readOneContour(filenames, param,ctrvers,ctredges,bbox,bboxset);

	//normalize the normals of the planes
	int planenum = param.size()/4;
	for( int i = 0; i < planenum; i ++)
	{
		float veclen = MyMath::vectorLen( param[ i * 4 ], param[ i*4 + 1 ], param[ i*4+ 2 ]);
		for(int j = 0; j < 4; j ++)
			param[ 4*i + j] = param[ 4*i + j]/veclen;
	}
}

void ContourHandler::readContourFromVec(vector<CurveNetwork> vecCurveNetwork,
										floatvector& param,vector<floatvector>& ctrvers,vector<intvector>& ctredges,
										float bbox[ 6 ], bool& bboxset)
{
	//read file one by one
	readOneContourFromVec(vecCurveNetwork, param,ctrvers,ctredges,bbox,bboxset);

	//normalize the normals of the planes
	int planenum = param.size()/4;
	for( int i = 0; i < planenum; i ++)
	{
		float veclen = MyMath::vectorLen( param[ i * 4 ], param[ i*4 + 1 ], param[ i*4+ 2 ]);
		for(int j = 0; j < 4; j ++)
			param[ 4*i + j] = param[ 4*i + j]/veclen;
	}
}