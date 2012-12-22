#include "../Ctr2SufManager/Ctr2SufManager.h"

void Ctr2SufManager::sortFace(intvector& meshFace,intvector& regface,int*& sregface)
{
	//////////////////////////////////////////////////////////////////////////
//	cout<<"safe 1"<<endl;
	//////////////////////////////////////////////////////////////////////////
	int facen = regface.size();
	if( facen == 1 )
	{
		sregface[ 0 ] = regface[ 0 ];
		return;
	}
	int* vfi[ 3 ];
	for( int i = 0; i < 3; i ++ )
		vfi[ i ] = new int[ facen + 1 ];
	for(int i = 0; i < 3; i ++ )
	{
		for( int j = 0;j <= facen; j ++)
		{
			vfi[ i ][ j ] = -1;
		}
	}

	for( int i = 0; i < facen; i++ )
	{
		int v1 = meshFace[ 5*regface[ i ] ] ;	//first vertex of the edge
		int v2 = meshFace[ 5*regface[ i ] + 1];	//second vertex of the edge
		//find the position for the edge for v1.
		for( int j = 0; j <= facen; j ++ )
		{
			if( vfi[ 0 ][ j ] == -1 )
			{
				vfi[ 0 ][ j ] = v1;	//the vertex index
				vfi[ 1 ][ j ] = i;	//index of the edge starting from this vertex
				break;
			}
			if( vfi[ 0 ][ j ] == v1 )
			{
				vfi[ 1 ][ j ] = i;
				break;
			}
		}
		for( int j= 0; j <= facen; j ++ )
		{
			if( vfi [ 0 ][ j ] == -1 )
			{
				vfi[ 0 ][ j ] = v2;
				vfi[ 2 ][ j ] = i;
				break;
			}
			if( vfi[ 0 ][ j ] == v2 )
			{
				vfi[ 2 ][ j ] = i;
				break;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//cout<<"safe 2"<<endl;
	//////////////////////////////////////////////////////////////////////////

	//find the starting face
	int startfi = -1;
	int* nexti = new int[ facen ];
	for( int i = 0; i <= facen; i++ )
	{
		if( vfi[ 2 ][ i ] == -1 )	//only one edge, whose head is this vertex
		{
			startfi = vfi[ 1 ][ i ];	//the starting face
			continue;
		}
		nexti[ vfi[ 2 ][ i ]] = vfi[ 1 ][ i ];
	}

////////////////////////////////////////////////////////////////////////
	//cout all the first two vertices of the faces
	//cout the two 3 rows vfi
	//cout<<"face composition........."<<endl;
	//for( int i = 0; i < facen; i++ )
	//{
	//	int v1 = meshFace[ 5*regface[ i ] ] ;	//first vertex of the edge
	//	int v2 = meshFace[ 5*regface[ i ] + 1];	//second vertex of the edge
	//	cout<<"("<<v1<<","<<v2<<")  ";
	//}
	//cout<<endl;
	//cout<<"pre next table..............."<<endl;
	//for( int i = 0; i <= facen; i ++ )
	//{
	//	cout<<vfi[ 0 ][ i ]<<"   ";
	//}
	//cout<<endl;
	//for( int i = 0; i <= facen; i ++ )
	//{
	//	cout<<vfi[ 1 ][ i ]<<"   ";
	//}
	//cout<<endl;
	//for( int i = 0; i <= facen; i ++ )
	//	cout<<vfi[ 2 ][ i ]<<"   ";
	//cout<<endl;

	//cout<<"next for every face........................"<<endl;
	//for( int i = 0; i < facen; i ++ )
	//{
	//	cout<<nexti[ i ]<<"   ";
	//}
	//cout<<endl;
	//cout<<"start fi: "<<startfi<<endl;
	////////////////////////////////////////////////////////////////////////
	sregface[ 0 ] = regface[ startfi ];
	int curfi = startfi;
	for( int i = 1; i < facen; i ++ )
	{
		curfi = nexti[ curfi ];
		sregface[ i ] = regface[ curfi ];
	}

	//////////////////////////////////////////////////////////////////////////	
	/*cout<<"sorted face list........"<<endl;
	for( int i = 0; i < facen; i ++ )
	{
		cout<<sregface[ i ]<<"  ";
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//cout<<"safe 3"<<endl;
	//////////////////////////////////////////////////////////////////////////

	for( int i = 0; i < 3; i ++ )
		delete []vfi[ i ];

	//////////////////////////////////////////////////////////////////////////
//	cout<<"safe 4"<<endl;
	//////////////////////////////////////////////////////////////////////////
}
void Ctr2SufManager::splitCtrEdgeOnFace(
										floatvector*& subMeshVer,
										intvector*& subMeshFace)
{
	//go through each face
	for( int sfacei = 0; sfacei < ssfacenum; sfacei ++ )
	{	
		int ctrenum = sfaceregface[ sfacei ].size()/2;

		if( ctrenum == 0 )
			continue;

		int curspaci[ 2 ] = {sfacespaci[ 2* sfacei ], sfacespaci[2*sfacei+1]};

		for( int ctri = 0; ctri < ctrenum;  ctri ++ ) 
		{
			int regfacenum[ 2 ] = { sfaceregface[ sfacei ][ 2*ctri ].size(), sfaceregface[ sfacei ][ 2*ctri + 1].size()};

			/*--Because now my project has problem, I have to add this! :(------*/
			if( (regfacenum[ 0 ] == 0) || (regfacenum[ 1 ] == 0 ) )
				continue;
			/*--------*/
			//both of them only has one complete edge
			if( (regfacenum[ 0 ] == 1) && ( regfacenum[ 1 ] == 1))
			{
				int regface[ 2 ] = { sfaceregface[ sfacei ][ 2*ctri ][ 0 ], sfaceregface[ sfacei][ 2*ctri+1][0] };
				//in sfaceregver, one pair includes one vertex index from one subspace and 
				//the other is from the other subspace, the sequence depends on sfacespaci!
				sfaceregver[ sfacei ].push_back( subMeshFace[ curspaci[ 0 ]][ 5*regface[ 0 ]] );
				sfaceregver[ sfacei ].push_back(subMeshFace[ curspaci[ 1 ]][ 5*regface[ 1 ]] );
				sfaceregver[ sfacei ].push_back( subMeshFace[ curspaci[ 0 ]][ 5*regface[ 0 ] + 1]);
				sfaceregver[ sfacei ].push_back( subMeshFace[ curspaci[ 1 ]][ 5*regface[ 1 ] + 1]);

				//set the two vers in them in sfaceregedgever
				sfaceregedgever[ sfacei ].push_back( sfaceregver[ sfacei ].size() - 4);
				sfaceregedgever[ sfacei ].push_back( sfaceregver[ sfacei ].size() - 2);
				continue;
			}

			//save the indices in the two lists of the faces corresponding to the contour edge
			int* sregface[ 2 ];
			sregface[ 0 ] = new int[regfacenum[ 0 ] ];
			sregface[ 1 ] = new int[regfacenum[ 1 ] ];

			//sort out the two subedge list of the two subspace
			sortFace(subMeshFace[curspaci[ 0 ]], sfaceregface[ sfacei][ 2*ctri ], sregface[ 0 ]);
			sortFace(subMeshFace[curspaci[ 1 ]], sfaceregface[ sfacei][ 2*ctri +1 ], sregface[ 1 ]);

			//for the two arrays of faces
			int inspaci[ 2 ] = { curspaci[ 0 ], curspaci[ 1 ]};
			if( regfacenum[ 0 ] > regfacenum[ 1 ])	//always split the first subspace
			{
				inspaci[ 0 ] = curspaci[ 1 ];
				inspaci[ 1 ] = curspaci[ 0 ];
				int* tp = sregface[ 0 ];
				sregface[ 0  ] = sregface[ 1 ];
				sregface[ 1 ] = tp;
				int ti = regfacenum[ 0 ];
				regfacenum[ 0 ] = regfacenum[ 1 ];
				regfacenum[ 1 ] = ti;
			}

			int* vilist[ 2 ];
			vilist[ 0 ] = new int[ regfacenum[ 1 ] + 1 ];
			vilist[ 1 ] = new int[ regfacenum[ 1 ] + 1 ];			
			if( regfacenum[ 1 ] > regfacenum[ 0 ])
			{				
				//remeber the position for all the vertices
				int osize = subMeshVer[ inspaci[ 0 ]].size()/3;
				int addsize = regfacenum[1] - regfacenum[ 0 ];
				subMeshVer[ inspaci[ 0 ]].resize( (osize + addsize) * 3 );
				int midi = regfacenum[ 0 ]/2;
				for( int k = 0; k <= midi; k ++ )
				{
					vilist[ 0 ][ k ] = subMeshFace[ inspaci[ 0 ] ][ 5*sregface[ 0 ][ k ]];
				}
				for( int k = midi + 1; k < midi + 1 + addsize; k ++ )
				{
					vilist[ 0 ][ k ] = osize;
					osize ++;
				}
				for( int k = midi + 1 + addsize, k2 = midi + 1; k < regfacenum[ 1 ]; k ++, k2++ )
				{
					vilist[ 0 ][ k ] = subMeshFace[ inspaci[ 0 ]][5*sregface[ 0 ][ k2 ]];
				}
				vilist[ 0 ][ regfacenum[ 1 ]] = subMeshFace[ inspaci[ 0 ]][ 5*sregface[ 0 ][ regfacenum[ 0 ] -1 ] + 1];

				//split the middle face with new vertices added the midi th face is split
				//add new faces
                int tspac = inspaci[ 0 ];
				int ofsize = subMeshFace[ tspac ].size();
				subMeshFace[ tspac ].resize( ofsize + addsize * 5 );
				int tfi =  sregface[ 0 ][ midi ];				
				int mat2[ 2 ] = {subMeshFace[ tspac ][ tfi * 5 + 3 ],
					subMeshFace[ tspac ][ tfi * 5 + 4 ]};
				int thirdvi = subMeshFace[ tspac ][ tfi * 5 + 2 ];				
				int tind = ofsize;
				for( int k = 0, k2 = midi + 1; k < addsize; k ++, k2++ )
				{
					subMeshFace[ tspac][ tind++ ] = vilist[ 0 ][ k2 ];
					subMeshFace[ tspac][ tind++ ] = vilist[ 0 ][ k2 + 1];
					subMeshFace[ tspac][ tind++ ] = thirdvi;
					subMeshFace[ tspac][ tind++ ] = mat2[ 0 ];
					subMeshFace[ tspac][ tind++ ] = mat2[ 1 ];
				}
				//change the old face
				subMeshFace[ tspac ][ 5*tfi + 1 ] = vilist[ 0 ][ midi + 1 ];
			}
			else
			{
				for( int k = 0; k < regfacenum[ 1 ]; k++ )
				{
					vilist[ 0 ][ k ] = subMeshFace[ inspaci[ 0 ] ][ 5*sregface[ 0 ][ k ]];					
				}
				vilist[ 0 ][ regfacenum[ 0 ]] = subMeshFace[ inspaci[ 0 ]][ 5*sregface[ 0 ][ regfacenum[ 0 ] -1 ] + 1];

			}

			for( int k = 0; k < regfacenum[ 1 ]; k++ )
			{
				vilist[ 1 ][ k ] = subMeshFace[ inspaci[ 1 ] ][ 5*sregface[ 1 ][ k ]];
			}
			vilist[ 1 ][ regfacenum[ 1 ]] = subMeshFace[ inspaci[ 1 ]][ 5*sregface[ 1 ][ regfacenum[ 1 ] -1 ] + 1];

			//register the vertices pair for subspace faces.
			int tcurnum = sfaceregver[ sfacei ].size();

			if( inspaci[ 0 ] == curspaci[ 0 ])	//didn't change the order
			{
				for( int k = 0; k <= regfacenum[ 1 ]; k ++ )
				{
					sfaceregver[ sfacei ].push_back( vilist[ 0 ][ k ]);
					sfaceregver[ sfacei ].push_back( vilist[ 1 ][ k ]);

					//push down the contour edge
					if( k != 0 )
					{
						sfaceregedgever[ sfacei ].push_back( tcurnum );
						sfaceregedgever[ sfacei ].push_back( tcurnum + 2 );
						tcurnum += 2;
					}
				}
			}
			else
			{
				for( int k= 0; k <= regfacenum[ 1 ]; k ++ )
				{
					sfaceregver[ sfacei ].push_back( vilist[ 1 ][ k ]);
					sfaceregver[ sfacei ].push_back( vilist[ 0 ][ k ]);

					//push down the contour edge
					if( k != 0 )
					{
						sfaceregedgever[ sfacei ].push_back( tcurnum );
						sfaceregedgever[ sfacei ].push_back( tcurnum + 2 );
						tcurnum += 2;
					}
				}

			}

			//adjust all the vertices between the two endpoints
			float endpt [ 2 ][ 3 ];
			for( int k = 0; k <3; k++)
			{
				endpt[ 0 ][ k ] = subMeshVer[ inspaci[ 1 ]][ vilist[ 1 ][ 0 ] * 3 + k ];
				endpt[ 1 ][ k ] = subMeshVer[ inspaci[ 1 ]][ vilist[ 1 ][ regfacenum[ 1 ]] * 3 + k ];
			}
			for( int k = 1; k < regfacenum[ 1 ]; k ++ )
			{
				//compute the position, reset the position in meshVer
				float param = ((float)k)/regfacenum[ 1 ];
				float pt[ 3 ];
				MyMath::getPtOnSeg( endpt[ 0 ], endpt[ 1 ], param, pt );
				//set the two vertices in the two mesh
				for(int k1 = 0; k1 < 3; k1 ++ )
				{
					subMeshVer[ inspaci[ 0 ]][ 3*vilist[ 0 ][ k ] + k1 ] = pt[ k1 ];
					subMeshVer[ inspaci[ 1 ]][ 3*vilist[ 1 ][ k ] + k1 ] = pt[ k1 ];
				}
			}

			delete []sregface[ 0 ];
			delete []sregface[ 1 ];
			delete []vilist[ 0 ];
			delete []vilist[ 1 ];
		}

		for( int ctri = 0; ctri < ctrenum*2; ctri++)
		{
			sfaceregface[ sfacei ][ ctri ].clear();
		}
		sfaceregface[ sfacei ].clear();
	}
	delete []sfaceregface;
	sfaceregface = NULL;
}
//stitch meshes from subspaces
void Ctr2SufManager::stitchMesh(
								floatvector*& subMeshVer,
								intvector*& subMeshFace)
{	
	//step1. for the edges registered in faces. split the corresponding faces if necessary
	splitCtrEdgeOnFace( subMeshVer,subMeshFace);

	//step2. mark all the vertices of the subsapces.	-2 -> -1 if used, -1 -> index in mver
	//step2.1. mark all the vertices from -2 to -1, if the vertex is used by the subspaces
	//step2.2, add all the vertices registered for subspace vertex
	//step2.3 add all the vertices registered for subsapce edges
	//step2.4 add all the vertices registered for subspace faces.
	int** vermark = new int*[ ssspacenum ] ;
	for( int spaci = 0; spaci < ssspacenum; spaci++)
	{
		int vnum = subMeshVer[ spaci ].size()/3;
		vermark[ spaci ] = new int[ vnum ];
		for( int i = 0 ;i < vnum; i ++ )
			vermark[ spaci ][ i ] = -2;	//marked as not used
	}

	//step2.1. mark all the vertices from -2 to -1, if the vertex is used by the subspaces
	for( int spaci = 0; spaci < ssspacenum; spaci ++ )
	{
		int facesize = subMeshFace[ spaci ].size();
		for( int i = 0; i < facesize; i++ )
		{
			//material index
			if ( i % 5 >= 3 )continue;
			//vertex index, set they are used!
			vermark[ spaci ][ subMeshFace[ spaci ][ i ]] = -1;	//used
		}
	}

	//step2.2, add all the vertices registered for subspace vertex
	for( int sveri = 0; sveri < ssvernum; sveri++ )
	{
		if( sverreg[ sveri ].size() == 0 )
			continue;
		//put the vertex into the final mesh
		int spaci = sverreg[ sveri ][ 0 ];
		int vi = sverreg[ sveri ][ 1 ];
		int mvpos = mver.size()/3;
		mver.push_back( subMeshVer[ spaci ][ 3*vi ]);
		mver.push_back( subMeshVer[ spaci ][ 3*vi + 1 ]);
		mver.push_back( subMeshVer[ spaci ][ 3*vi + 2 ]);

		int pairnum = sverreg[ sveri ].size()/2;
		for( int i = 0; i < pairnum; i ++ )
		{
			int spaci = sverreg[ sveri ][ 2*i ];
			int vi = sverreg[ sveri ][ 2*i + 1];
			vermark[ spaci ][ vi ] = mvpos;	//has already been put into final mver, set the position
		}
	}
	
	//step2.3 add all the vertices registered for subsapce edges
	for( int i = 0; i < ssedgenum; i++ )
	{
		int vnum = sedgereg[ i ].size();
		
		int* tmverpos = new int[ vnum ];
		for( int ti = 0; ti < vnum; ti++ )
			tmverpos[ ti ] = -1;

		for( int j = 0;  j < vnum; j ++ )
		{
			//add the vertices into the mver, and set the crsp info
			int pairnum = sedgereg[ i ][ j ].size()/2;

			//add vertex
			if( pairnum == 0 )		//no need to add at all
				continue;

			int mvpos = mver.size()/3;
			int curspaci = sedgereg[ i ][ j ][ 0 ];
			int curvi = sedgereg[ i ][ j ][ 1 ];
			
			if( vermark[ curspaci ][ curvi ] == -2 )	//no need to add!
				continue;

			mver.push_back(subMeshVer[ curspaci ][ 3*curvi ]);
			mver.push_back( subMeshVer[ curspaci][ 3*curvi + 1 ]);
			mver.push_back( subMeshVer[ curspaci][ 3*curvi + 2 ]);
            
			for( int k = 0; k < pairnum; k ++ )
			{
				curspaci = sedgereg[ i ][ j ][ 2*k  ];
				curvi = sedgereg[ i ][ j ][ 2 * k + 1 ];
				vermark[ curspaci ][ curvi ] = mvpos;
			}
			tmverpos[ j ] = mvpos;
		}

		//set the material contour edge on this subspace edge
		for( int j = 0; j < vnum - 1; j ++ )
		{
			if( sedgesubedgemark[ i ][ j ] == 1 )	//is in
			{
				ctrmedge.push_back( tmverpos[ j ]);
				ctrmedge.push_back( tmverpos[ j + 1 ]);
			}
		}
		//clear the mark
		sedgesubedgemark[ i ].clear();
		delete []tmverpos;

	}
	delete []sedgesubedgemark;
	sedgesubedgemark = NULL;
	
	//step2.4 add all the vertices registered for subspace faces.
	for( int i = 0; i < ssfacenum; i ++ )
	{
		int spacei[ 2 ] = { sfacespaci[ 2* i ], sfacespaci[ 2*i + 1 ]};

		int vnum = sfaceregver[ i ].size()/2;
		if( vnum == 0 )	//no contour vertex on this subspace face
		{
			continue;
		}
		for( int j = 0; j < vnum; j ++ )
		{
			int vis[ 2 ] = {sfaceregver[ i ][ j * 2 ], sfaceregver[ i ][ j * 2 + 1 ]};
			if( vermark[ spacei[ 0 ]][ vis[ 0 ]] != -1 )	//already added!
			{
				//remember the new position in mver in sfaceregver
				sfaceregver[ i ][ j * 2 ] = vermark[ spacei[ 0 ]][ vis[ 0 ]];
				continue;
			}

			//add and set the crsp info
			int vpos = mver.size() / 3;
			mver.push_back ( subMeshVer[ spacei[ 0 ]][ 3*vis[ 0 ]]);
			mver.push_back( subMeshVer[ spacei[ 0 ]][ 3*vis[ 0 ] + 1]);
			mver.push_back( subMeshVer[ spacei[ 0 ]][ 3*vis[ 0 ] + 2 ]);
			vermark[ spacei[ 0 ]][ vis [ 0 ]] = vpos;
			vermark[ spacei[ 1 ]][ vis[ 1 ]] = vpos;
			//remeber the new position in mver in sfaceregver
			sfaceregver[ i ][ j * 2 ] = vpos;
		}

		//add all the contour edges on this face
		int tenum = sfaceregedgever[ i ].size();
		for( int j = 0; j < tenum; j ++ )
		{
			int tpos = sfaceregedgever[ i ][ j ];
			ctrmedge.push_back( sfaceregver[ i ][ tpos ] );
		}

		sfaceregedgever[ i ].clear();
	}
	delete []sfaceregedgever;

	//step3. go through each subspace, add the remaiing vertices, with mark -1, and reset the vertices for the faces
	int mvpos = mver.size() / 3;
	for( int i = 0; i < ssspacenum; i++ )
	{
		//add the vertices in the subspace into mver, and set crsp
		int vnum = subMeshVer[ i ].size() / 3;
		 
		for( int j = 0; j < vnum; j ++ )
		{
			//no need to add, either already added, or not used at all!
			if( vermark [ i ][ j ] != -1  )continue;
				
			mver.push_back( subMeshVer[ i ][ 3*j ]);
			mver.push_back( subMeshVer[ i ][ 3*j + 1 ]);
			mver.push_back ( subMeshVer[ i ][  3* j + 2 ]);

			vermark[ i ][ j ] = mvpos; 
			mvpos++;
		}

		subMeshVer[ i ].clear();
	}
	delete []subMeshVer;

	//add faces of the subsapce into the final faces: mface
	int mfacesize = 0;
	int tind = 0;
	for( int i = 0; i <ssspacenum; i ++ )
	{
        int fnum = subMeshFace[ i ].size();		//five is a group
		mfacesize += fnum;
		fnum /= 5;
		mface.resize( mfacesize );
		int tind2 = 0;
        for( int j = 0; j< fnum; j ++ )
		{
			//set the face vertices indices
			for( int k = 0; k < 3;k ++)
                mface[ tind++  ] = vermark[ i ][ subMeshFace[ i ][ tind2 ++ ]];
            
			//set material for the face
			for( int k = 3; k < 5; k ++ )
				mface[ tind++ ] = subMeshFace[ i ][ tind2 ++];
		}

		subMeshFace[ i ].clear();
	}
	////////////////////////////////////////////////////////////////////////////
	//for( int i = 0; i< mface.size()/5; i++ )
	//{
	//	cout<<mface[ 5*i + 3 ]<<","<<mface[ 5*i + 4] <<" | ";
	//}
	//cout<<endl;
	//////////////////////////////////////////////////////////////////////////

	delete []subMeshFace;
}