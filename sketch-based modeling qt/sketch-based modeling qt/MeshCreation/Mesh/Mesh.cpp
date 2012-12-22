#include "../Mesh/mesh.h"
//#include "../config.h"
//#include "Common/glut.h"

//Mesh::~Mesh()
//{
//	//clear all the infomation in mesh
//	//ctr
//	for( int i = 0 ;i < ctrplanenum;	i ++)
//	{
//		delete []ctrver[ i ];
//		delete []ctredge[ i ];
//		delete []ctrtriconfig[ i ];
//	}
//	delete []ctrplaneparam;
//	delete []ctrver;
//	delete []ctrvernum;
//	delete []ctredge;
//	delete []ctredgenum;
//	delete  []ctrtrinum;
//	delete  []ctrtriconfig;	
//
//	//mesh
//	delete []sufver;
//	delete []sufface;
//	delete []suffacenorm;
//	delete []sufmat;
//	delete []sufctredge;
//	matlist.clear();
//}
void Mesh::gatherEdgeInfo(vector<int>& edgelist,vector<intvector>& edge2facelist,
						  HashMap& ver2edgehash,vector<int>& ctredgelist,vector<int>&	nmedgelist,
						  vector<int>& normedgelist,vector<float>& edgelenlist)
{
	/*---------------------*/
	/*edgelist.clear();
	for(int i = 0; i < edge2facelist.size(); i ++)
	edge2facelist[ i ].clear();
	edge2facelist.clear();
	ctredgelist.clear();
	nmedgelist.clear();
	normedgelist.clear();
	edgelenlist.clear();*/
	/*---------------------*/

	//edgelist and edge2facelist and ver2edgehash
	int edgelen = 0;
	int edgeindex;
	for( int i = 0;  i < suffacenum; i++)
	{
		for( int j = 0; j < 3; j ++)
		{
			edgeindex = ver2edgehash.findInsertSort(sufface[ i*3 + j ], sufface[ i*3 + (j+1)%3 ], edgelen);
			if(edgeindex == edgelen)
			{
				edgelen ++;
				edgelist.push_back(sufface[ i*3 + j ]);
				edgelist.push_back(sufface[ i*3 + (j+1)%3 ]);
				intvector facelist;
				facelist.push_back( i );	//i face
				facelist.push_back( j );	//j edge in i face
				edge2facelist.push_back( facelist );	//in c++, copy and push back
				facelist.clear();		//clear temp variable, avoid memory leakage
			}
			else
			{
				edge2facelist[ edgeindex ].push_back( i );	//face i
				edge2facelist[ edgeindex ].push_back( j );	//edge j in face i
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
	//	cout << "edgelen:"<<edgelen<<"edgelist size:"<<edgelist.size()/2<<endl;
	//////////////////////////////////////////////////////////////////////////
	//ctredgelist nmedgelist normedgelist
	int* edgemark = new int[ edgelen ];		// 0 - normal edge 1 - contour edge 2 - nonmanifold edge
	for( int i= 0; i < edgelen; i++)
		edgemark[ i ] = 0;
	//////////////////////////////////////////////////////////////////////////
	//	cout<<"sufctredgenum:"<<sufctredgenum<<endl;
	for( int i = 0; i < sufctredgenum; i++)
	{
		edgeindex = ver2edgehash.findInsertSort( sufctredge[ i * 2], sufctredge[i * 2 +1], edgelen);
		//cout<<"edgeindex:"<<edgeindex<<endl;
		if( edgeindex == edgelen )
		{
			cout<<sufctredge[ i * 2]<<"\t"<<sufctredge[i * 2 +1]<<endl;
			cout<<"not valid input!"<<endl;
			continue;
		}
		edgemark[edgeindex] = 1;
	}
	for( int i = 0; i < edgelen; i ++)
	{
		if( edge2facelist[ i ].size() > 4)
		{
			edgemark[ i ] = 2;
		}
	}
	for(int i = 0; i < edgelen; i ++)
	{
		switch( edgemark[i] )
		{
		case 0:
			normedgelist.push_back( i );
			break;
		case 1:
			ctredgelist.push_back( i );
			break;
		case 2:
			nmedgelist.push_back( i );
			break;
		default:
			break;
		}
	}
	//////////////////////////////////////////////////////////////////////////
	//cout<<"contour edge number:"<<ctredgelist.size()<<"\t";
	//cout<<"non manifold edge number:"<<nmedgelist.size()<<endl;
	delete []edgemark;

	//edgelenlist
	edgelenlist.resize(edgelen,0);
	//	for( int i = 0; i < edgelen; i ++)
	//		edgelenlist[i] = 0;
	for(unsigned int i = 0; i < ctredgelist.size(); i++)
	{
		edgelenlist[ ctredgelist[i]] = MyMath::vectorlen(&sufver[edgelist[ctredgelist[i]*2]*3],
			&sufver[edgelist[ctredgelist[i]*2 + 1]*3]);
	}	
}
void Mesh::gatherVerInfo(vector<float>& verlist,vector<float>& verattrlist, vector<int>&edgelist, vector<float>&edgelenlist,
						 vector<int>&nmedgelist, vector<int>&normedgelist, vector<int>&ctredgelist )
{
	//verlist
	verlist.resize(sufvernum*3, 0);
	for( int i = 0; i < sufvernum * 3; i ++)
		verlist[ i ] = sufver[ i ];

	//verattrilist: the expected lengh at that vertex
	//	cout<<"sufvernum:"<<sufvernum;
	verattrlist.resize(sufvernum, 0);
	int* vermark = new int[ sufvernum ];	//0 - normal ver 1 - ver on contour edge 2 - ver on non manifold edge not on contour edge
	int* vercount = new int[ sufvernum ];
	for( int i = 0; i < sufvernum; i ++)
	{
		vermark[ i ] = 0;
		vercount[ i ] = 0;
		verattrlist[ i ] = 0;
	}
	//cout<<"ctredgelist.size:"<<ctredgelist.size()<<endl;
	for(unsigned int i = 0;i < ctredgelist.size(); i ++)
	{
		vermark[ edgelist[ ctredgelist[ i ] * 2 ] ] = 1;
		vermark[ edgelist[ ctredgelist[ i ] * 2 + 1]] = 1;
	}
	for(unsigned int i = 0; i < nmedgelist.size(); i ++)
	{
		if( vermark[ edgelist[ nmedgelist[ i ]* 2 ] ] == 0 )
			vermark[ edgelist[ nmedgelist[ i ]* 2 ] ] = 2;
		if( vermark[ edgelist[ nmedgelist[ i ]* 2  + 1] ] == 0 )
			vermark[ edgelist[ nmedgelist[ i ]* 2  + 1] ] = 2;
	}

	int veri = 0;
	//	cout<<"ctredgelist.size:"<<ctredgelist.size()<<endl;
	for(unsigned int i = 0; i < ctredgelist.size(); i ++)
	{
		for( int j = 0;  j < 2; j ++)
		{
			veri = edgelist[ctredgelist[ i ]*2 + j ];
			//	cout<<"veri:"<<veri<<endl;
			//	cout<<"ctredgelist[ i ]:"<<ctredgelist[ i ]<<endl;
			//	cout<<"edgelenlist[ ctredgelist[ i ] ]"<<edgelenlist[ ctredgelist[ i ] ]<<endl;
			verattrlist[ veri ] = verattrlist[ veri ] + edgelenlist[ ctredgelist[ i ] ];
			//	cout<<edgelenlist[ ctredgelist[ i ] ]<<"\t";
			vercount[ veri ] ++;
		}
	}
	//	cout<<endl;
	for( int i = 0; i < sufvernum; i ++)
	{
		if( vermark[ i ] == 1)			//contour vertex
		{
			verattrlist[ i ] /= vercount[ i ];
			vercount[ i ] = 0;
		}
		//cout<<verattrlist[ i ]<<"\t";
	}
	//attribute for other vertices: ring by ring propagating the attributes
	int veri1, veri2;
	int edgenum = edgelist.size()/2;
	bool again = true;
	while( again )
	{
		//	memset(vercount, 0, sizeof(int)*sufvernum);
		again = false;
		for( int i = 0; i < edgenum; i ++)
		{
			veri1 = edgelist[ i * 2 ];
			veri2 = edgelist[ i * 2 + 1 ];
			if( ((vercount[ veri1 ]== 0) && (verattrlist[ veri1 ] != 0) ) && ((verattrlist[ veri2 ] == 0) ||(vercount[veri2] != 0 ) ) )		//veri1 is one that has attri, while veri2 not
			{
				verattrlist[ veri2 ] += verattrlist[ veri1 ];
				vercount[ veri2 ] ++;
			}
			else if( ((vercount[ veri2 ]== 0) && (verattrlist[ veri2 ] != 0) ) && ((verattrlist[ veri1 ] == 0) ||(vercount[veri1] != 0 ) )  )	//veri2 has while veri1 not
			{
				verattrlist[ veri1 ] += verattrlist[ veri2 ];
				vercount[ veri1 ] ++;
			}
		}

		for( int i = 0; i < sufvernum; i ++)
		{
			if( vercount[ i ] != 0 )
			{
				again = true;
				verattrlist[ i ] /= vercount[ i ];
				vercount[ i ] = 0;
			}
		}
	}
	//attribute for other vertices: old implementation
	//	int veri2 = 0;
	////	cout<<"normedgelist.size:"<<normedgelist.size()<<endl;
	//	for( int i = 0; i < normedgelist.size(); i ++)
	//	{
	//		veri = edgelist[ normedgelist[i] * 2];
	//		veri2 = edgelist[ normedgelist[i] * 2 + 1];
	//		if( vermark[ veri ] == 1)
	//		{
	//			verattrlist[ veri2 ] += verattrlist[ veri ];
	//			vercount[ veri2 ] ++;
	//		}
	//		if( vermark[ veri2 ] == 1)
	//		{
	//			verattrlist[ veri ] += verattrlist[ veri2 ];
	//			vercount[ veri ] ++;
	//		}
	//	}
	////	cout<<"nmedgelist.size:"<<nmedgelist.size()<<endl;
	//	for( int i = 0; i < nmedgelist.size(); i++)
	//	{
	//		veri = edgelist[ nmedgelist[i] * 2];
	//		veri2 = edgelist[ nmedgelist[i] * 2 + 1];
	//		if( vermark[ veri ] == 1)
	//		{
	//			verattrlist[ veri2 ] += verattrlist[ veri ];
	//			vercount[ veri2 ] ++;
	//		}
	//		if( vermark[ veri2 ] == 1)
	//		{
	//			verattrlist[ veri ] += verattrlist[ veri2 ];
	//			vercount[ veri ] ++;
	//		}
	//	}
	//	for( int i = 0; i < sufvernum; i ++)
	//	{
	//		if( vermark[ i ] != 1 && vercount[ i ] != 0 )
	//			verattrlist[ i ] /= vercount[ i ];
	//
	//	//	cout<<verattrlist[i]<<"\t";
	//	}
	//	float maxattr,minattr;
	//	maxattr = minattr = verattrlist[ edgelist[ctredgelist[ 0 ] * 2 ]];
	//	for( int i = 0; i < sufvernum; i ++)
	//	{
	//		if( vercount[ i ]!=0)
	//		{
	//			if( verattrlist[ i ] < minattr)
	//				minattr = verattrlist[ i ];
	//			else if( verattrlist[ i ] > maxattr )
	//				maxattr = verattrlist[ i ];
	//		}
	//	}
	//	//maxattr = (maxattr + minattr)/2;
	////	cout<<"maxattr"<<maxattr<<endl;
	//	for( int i = 0; i < sufvernum; i ++)
	//	{
	//		if( vercount[ i ] == 0)
	//			verattrlist[ i ] = maxattr;
	//	}
	//	cout<<endl;	
	delete []vermark;
}
void Mesh::gatherVerInfo(vector<float>& verlist,vector<float>& verattrlist,vector<int>&vermark,  vector<int>&edgelist, vector<float>&edgelenlist,
						 vector<int>&nmedgelist, vector<int>&normedgelist, vector<int>&ctredgelist )
{
	//verlist
	verlist.resize(sufvernum*3, 0);
	for( int i = 0; i < sufvernum * 3; i ++)
		verlist[ i ] = sufver[ i ];

	//verattrilist: the expected lengh at that vertex
	//	cout<<"sufvernum:"<<sufvernum;
	verattrlist.resize(sufvernum, 0);
	//int* vermark = new int[ sufvernum ];	//0 - normal ver 1 - ver on contour edge 2 - ver on non manifold edge not on contour edge
	vermark.resize(sufvernum);
	int* vercount = new int[ sufvernum ];
	for( int i = 0; i < sufvernum; i ++)
	{
		vermark[ i ] = 0;
		vercount[ i ] = 0;
		verattrlist[ i ] = 0;
	}
	//cout<<"ctredgelist.size:"<<ctredgelist.size()<<endl;
	for(unsigned int i = 0;i < ctredgelist.size(); i ++)
	{
		vermark[ edgelist[ ctredgelist[ i ] * 2 ] ] = 1;
		vermark[ edgelist[ ctredgelist[ i ] * 2 + 1]] = 1;
	}
	for(unsigned int i = 0; i < nmedgelist.size(); i ++)
	{
		if( vermark[ edgelist[ nmedgelist[ i ]* 2 ] ] == 0 )
			vermark[ edgelist[ nmedgelist[ i ]* 2 ] ] = 2;
		if( vermark[ edgelist[ nmedgelist[ i ]* 2  + 1] ] == 0 )
			vermark[ edgelist[ nmedgelist[ i ]* 2  + 1] ] = 2;
	}

	int veri = 0;
	//	cout<<"ctredgelist.size:"<<ctredgelist.size()<<endl;
	for(unsigned int i = 0; i < ctredgelist.size(); i ++)
	{
		for( int j = 0;  j < 2; j ++)
		{
			veri = edgelist[ctredgelist[ i ]*2 + j ];
			//	cout<<"veri:"<<veri<<endl;
			//	cout<<"ctredgelist[ i ]:"<<ctredgelist[ i ]<<endl;
			//	cout<<"edgelenlist[ ctredgelist[ i ] ]"<<edgelenlist[ ctredgelist[ i ] ]<<endl;
			verattrlist[ veri ] = verattrlist[ veri ] + edgelenlist[ ctredgelist[ i ] ];
			//	cout<<edgelenlist[ ctredgelist[ i ] ]<<"\t";
			vercount[ veri ] ++;
		}
	}
	//	cout<<endl;
	for( int i = 0; i < sufvernum; i ++)
	{
		if( vermark[ i ] == 1)			//contour vertex
		{
			verattrlist[ i ] /= vercount[ i ];
			vercount[ i ] = 0;
		}
		//cout<<verattrlist[ i ]<<"\t";
	}
	//attribute for other vertices: ring by ring propagating the attributes
	int veri1, veri2;
	int edgenum = edgelist.size()/2;
	bool again = true;
	while( again )
	{
		//	memset(vercount, 0, sizeof(int)*sufvernum);
		again = false;
		for( int i = 0; i < edgenum; i ++)
		{
			veri1 = edgelist[ i * 2 ];
			veri2 = edgelist[ i * 2 + 1 ];
			if( ((vercount[ veri1 ]== 0) && (verattrlist[ veri1 ] != 0) ) && ((verattrlist[ veri2 ] == 0) ||(vercount[veri2] != 0 ) ) )		//veri1 is one that has attri, while veri2 not
			{
				verattrlist[ veri2 ] += verattrlist[ veri1 ];
				vercount[ veri2 ] ++;
			}
			else if( ((vercount[ veri2 ]== 0) && (verattrlist[ veri2 ] != 0) ) && ((verattrlist[ veri1 ] == 0) ||(vercount[veri1] != 0 ) )  )	//veri2 has while veri1 not
			{
				verattrlist[ veri1 ] += verattrlist[ veri2 ];
				vercount[ veri1 ] ++;
			}
		}

		for( int i = 0; i < sufvernum; i ++)
		{
			if( vercount[ i ] != 0 )
			{
				again = true;
				verattrlist[ i ] /= vercount[ i ];
				vercount[ i ] = 0;
			}
		}
	}
	//	delete []vermark;
}
void Mesh::gatherFaceInfo(vector<int>& facelist)
{
	facelist.resize(suffacenum*5, 0);
	for( int i = 0; i < suffacenum; i ++)
	{
		for( int j = 0; j < 3; j ++)
		{
			facelist[ i * 5 + j ] = sufface[ i * 3 + j];
		}
		for( int j = 0; j < 2; j ++)
			facelist[ i * 5 + j + 3 ] = sufmat[ i * 2 + j];
	}
}
void Mesh::splitOneNMEdge(int segnum, int i, vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
						  vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist, HashMap& ver2edgehash)
{
	int verpos[2] = {edgelist[nmedgelist[i]*2],edgelist[nmedgelist[i]*2 + 1]};
	//	cout<<"verpos are:"<<verpos[0]<<"   "<<verpos[1]<<endl;
	float newver[3];
	int verlen = verlist.size()/3;
	for( int j = 1; j < segnum; j++ )
	{
		MyMath::getPtOnSeg(&verlist[verpos[0]*3], &verlist[verpos[1]*3], j/(float)segnum, newver);
		verlist.push_back( newver[ 0 ]) ;
		verlist.push_back( newver[ 1 ]);
		verlist.push_back( newver[ 2 ]);
		verattrlist.push_back( (1 - j/(float)segnum)*verattrlist[verpos[0]] +  j/(float)segnum*verattrlist[verpos[1]] );
	}
	//------------//
	//	cout<<"vertex added!"<<endl;
	//new edge on the old non-manifold edge
	int edgelen = edgelist.size()/2;
	int oldnmedgelen = nmedgelist.size();
	nmedgelist.resize(oldnmedgelen + segnum - 1);
	for( int j = 1; j < segnum - 1; j ++)
	{
		edgelist.push_back( verlen + j - 1);
		edgelist.push_back( verlen + j);
		nmedgelist[ oldnmedgelen + j - 1] = edgelen;
		ver2edgehash.findInsertSort( verlen + j -1, verlen + j, edgelen);
		edgelen ++;
	}
	nmedgelist[ oldnmedgelen + segnum - 2] = edgelen;
	edgelist.push_back( verlen + segnum - 2);	//the last segment
	edgelist.push_back( verpos[1] );
	ver2edgehash.findInsertSort( verlen + segnum - 2, verpos[1], edgelen);
	edgelen++;
	edgelist[ nmedgelist[ i ]*2 + 1 ] = verlen;	//second point of the first segment

	int facenum = facelist.size()/5;	//old face number
	//edgelen = edgelist.size()/2;		//edge number after adding new edges on non-manifold edge
	//------------//
	//	cout<<"new edge added!"<<endl;
	//every face
	int curfacenum = edge2facelist[ nmedgelist[ i ]].size()/2;	//incident faces number
	//intvector& curfacelist = edge2facelist[ nmedgelist[i]];		//incident faces list	/*-------*/

	int oldnormedgelen = normedgelist.size();
	normedgelist.resize( oldnormedgelen + curfacenum*(segnum - 1));
	for( int j = 0; j < (segnum - 1)*curfacenum; j++)
		normedgelist[ oldnormedgelen + j] = edgelen + j;
	edge2facelist.resize(edgelen + curfacenum * (segnum - 1));

	intvector& curfacelist = edge2facelist[ nmedgelist[i]];		//incident faces list	/*-------*/
	//------------//
	//		cout<<"incident face number:"<<curfacenum<<endl;
	for( int j = 0; j < curfacenum; j ++)
	{
		//	cout<<"curface:"<<curfacelist[2*j]<<endl;
		int edgepos = curfacelist[ 2*j+1 ];
		int thirdverpos = facelist[5 * curfacelist[ 2*j ]+(edgepos + 2)%3];
		//	cout<<"current face:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
		//			" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
		//			<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
		//	cout<<"edgepos"<<edgepos<<"\t";
		//	cout<<"thirdver:"<<thirdverpos<<"\n";
		bool iscw = (verpos[ 0 ] == facelist[ 5 * curfacelist[ 2*j ] + edgepos ]);
		int mat[2] = {facelist[ 5 * curfacelist[ 2*j ] + 3], facelist[ 5 * curfacelist[ 2*j ] + 4]};

		//add new edge, add new face, refresh edge2facelist
		//add new edge
		for( int k = 0; k < segnum - 1; k++ )
		{
			edgelist.push_back( thirdverpos );
			edgelist.push_back( verlen + k );
			ver2edgehash.findInsertSort( thirdverpos, verlen + k,  edgelen + k + j * (segnum-1) );
		}
		//------------//
		//	cout<<"new edge added for current face!"<<endl;
		//new face
		if( iscw )
		{
			for( int k = 0; k < segnum - 2; k ++ )
			{
				facelist.push_back( thirdverpos );
				facelist.push_back( verlen + k );
				facelist.push_back( verlen + k + 1);
				facelist.push_back( mat[ 0 ]);
				facelist.push_back( mat[ 1 ]);
			}
			for( int k = 0; k < 5; k ++)
				facelist.push_back( facelist[ 5 * curfacelist[ 2*j ] + k ]);
			facelist[ ( facenum + segnum - 2 ) * 5 + edgepos ] = verlen + segnum - 2;
			facelist[ 5 * curfacelist[ 2*j ] + (edgepos + 1)%3 ] = verlen;
		}
		else
		{
			for( int k = 0; k < segnum - 2; k ++)
			{
				facelist.push_back( thirdverpos );
				facelist.push_back( verlen + k + 1);
				facelist.push_back( verlen + k );					
				facelist.push_back( mat[ 0 ]);
				facelist.push_back( mat[ 1 ]);
			}
			for( int k = 0; k < 5; k ++)
				facelist.push_back( facelist[ 5 * curfacelist[ 2*j ] + k ]);
			facelist[ ( facenum + segnum - 2 ) * 5 + (edgepos + 1)%3] = verlen + segnum - 2;
			facelist[ 5 * curfacelist[ 2*j ] + edgepos ] = verlen;
		}
		//---------------------//
		/*cout<<"faces:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
		" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
		<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
		for( int k = 0; k < segnum - 1; k++ )
		{
		cout<<"faces:"<<facelist[5 * (facenum+k)]<<" "<<facelist[5 *(facenum+k) + 1]<<
		" "<<facelist[5 *(facenum+k) + 2]<<" "<<facelist[5 * (facenum+k)+ 3]<<" "
		<<facelist[5 * (facenum+k)+4]<<endl;
		}*/
		facenum += (segnum - 1);

		//edge2facelist
		//new edge on the nmedge
		int edgelen2 = edgelist.size()/2;
		edge2facelist[ edgelen - 1].push_back( facenum - 1);	//the last new edge
		edge2facelist[ edgelen - 1].push_back( edgepos );
		for( int k = 2; k < segnum; k ++)
		{
			edge2facelist[ edgelen - k ].push_back( facenum - k);
			edge2facelist[ edgelen - k].push_back( 1 );
		}
		//old edge in the old face
		int oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
		//if( iscw)
		//{
		//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
		//	//////////////////////////////////////////////////////////////////////////
		//	if( oldedge == 25 )
		//	{
		//		cout<<"cw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[1]<<endl;
		//	}
		//}
		//else
		//{
		//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 0 ], edgelen2);
		//	//////////////////////////////////////////////////////////////////////////
		//	if( oldedge == 25 )
		//	{
		//		cout<<"ccw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[0]<<endl;
		//	}
		//}

		if(oldedge == edgelen2 )
		{
			cout<<"trying to find the old edge fails!!!"<<endl;
		}
		else
		{
			for(unsigned int k = 0; k < edge2facelist[ oldedge ].size(); k++)
			{
				if( edge2facelist[ oldedge ][ k * 2 ] == curfacelist[ j * 2 ])
				{
					edge2facelist[ oldedge ][ k * 2 ] = facenum - 1;
					break;
				}
			}
		}
		//new edge on the face
		int pos = edgelen + (segnum - 1) * j;
		for( int k = 0; k < segnum - 2; k++)
		{
			edge2facelist[ pos + k ].push_back( facenum - segnum + k + 1);
			if( iscw )
				edge2facelist[ pos + k ].push_back( 0 );
			else
				edge2facelist[ pos + k  ].push_back( 2 );
		}
		//last new edge
		edge2facelist[ pos + segnum - 2].push_back( facenum - 1 );
		if( iscw )
			edge2facelist[ pos + segnum - 2].push_back( (edgepos + 2)%3 );
		else
			edge2facelist[ pos + segnum - 2].push_back( (edgepos + 1)%3 );
		for( int k = segnum - 2; k >= 1; k --)
		{
			edge2facelist[ pos + k ].push_back( facenum - segnum + k);
			if( iscw )
				edge2facelist[ pos + k ].push_back( 2 );
			else
				edge2facelist[ pos + k ].push_back( 0 );
		}
		edge2facelist[ pos ].push_back( curfacelist[ j * 2 ] );
		if( iscw )
			edge2facelist[ pos ].push_back( (edgepos + 1)%3 );
		else
			edge2facelist[ pos ].push_back( (edgepos + 2)%3 );
		//	cout<<"current face is done!"<<endl;
	}
}

void Mesh::splitNMEdge(vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
					   vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist, HashMap& ver2edgehash)
{
	//go through each non-manifold edge, if the length of it is longer than parameter, split it
	int nmedgelen = nmedgelist.size();
	float curlen;	//current non-manifold edge length
	float param;	//the bigger parameter of the two endpoints of current non-manifold edge
	int segnum;		//how many segments it's going to be cut into
	//for debug
	//	for( int i = 0; i < 4; i ++)
	for( int i = 0; i < nmedgelen; i++)
	{
		curlen = MyMath::vectorlen(&verlist[edgelist[nmedgelist[i]*2]*3],	&verlist[edgelist[nmedgelist[i]*2 + 1]*3]);
		param = verattrlist[ edgelist[ nmedgelist[ i ]* 2]];
		if( verattrlist[edgelist[nmedgelist[i] * 2  + 1]] > param )
			param = verattrlist[ edgelist[nmedgelist[i]*2 + 1]];
		//for debug
		//segnum = 4;
		//	cout<<"param"<<param<<"\t"<<curlen<<endl;
		//	segnum = (int)(curlen / param + 0.6);
		segnum = curlen/param;
		if( segnum <= 1)continue;
		//split current edge into segnum segments.
		//add new vertex: refresh verlist, and verattrilist
		//add new edge: refresh edgelist, edge2facelist, nmedgelist, normedgelist, and ctredgelist
		//add new face: refresh facelist
		//ver
		//------------//
		//	cout<<"segnum"<<segnum<<endl;

		splitOneNMEdge(segnum,  i,  verlist, verattrlist,edgelist, edge2facelist,nmedgelist, normedgelist,facelist,  ver2edgehash);
	}
	//!!!old implementation below!!!
	//		int verpos[2] = {edgelist[nmedgelist[i]*2],edgelist[nmedgelist[i]*2 + 1]};
	//	//	cout<<"verpos are:"<<verpos[0]<<"   "<<verpos[1]<<endl;
	//		float newver[3];
	//		int verlen = verlist.size()/3;
	//		for( int j = 1; j < segnum; j++ )
	//		{
	//			MyMath::getPtOnSeg(&verlist[verpos[0]*3], &verlist[verpos[1]*3], j/(float)segnum, newver);
	//			verlist.push_back( newver[ 0 ]) ;
	//			verlist.push_back( newver[ 1 ]);
	//			verlist.push_back( newver[ 2 ]);
	//			verattrlist.push_back( (1 - j/(float)segnum)*verattrlist[verpos[0]] +  j/(float)segnum*verattrlist[verpos[1]] );
	//		}
	//		//------------//
	//	//	cout<<"vertex added!"<<endl;
	//		//new edge on the old non-manifold edge
	//		int edgelen = edgelist.size()/2;
	//		int oldnmedgelen = nmedgelist.size();
	//		nmedgelist.resize(oldnmedgelen + segnum - 1);
	//		for( int j = 1; j < segnum - 1; j ++)
	//		{
	//			edgelist.push_back( verlen + j - 1);
	//			edgelist.push_back( verlen + j);
	//			nmedgelist[ oldnmedgelen + j - 1] = edgelen + j - 1;
	//		}
	//		nmedgelist[ oldnmedgelen + segnum - 2] = edgelen + segnum - 2;
	//		edgelist.push_back( verlen + segnum - 2);	//the last segment
	//		edgelist.push_back( verpos[1] );
	//		edgelist[ nmedgelist[ i ]*2 + 1 ] = verlen;	//second point of the first segment
	//        
	//		int facenum = facelist.size()/5;	//old face number
	//		edgelen = edgelist.size()/2;		//edge number after adding new edges on non-manifold edge
	//		//------------//
	//	//	cout<<"new edge added!"<<endl;
	//		//every face
	//		int curfacenum = edge2facelist[ nmedgelist[ i ]].size()/2;	//incident faces number
	//		intvector& curfacelist = edge2facelist[ nmedgelist[i]];		//incident faces list
	//
	//		int oldnormedgelen = normedgelist.size();
	//		normedgelist.resize( oldnormedgelen + curfacenum*(segnum - 1));
	//		for( int j = 0; j < (segnum - 1)*curfacenum; j++)
	//			normedgelist[ oldnormedgelen + j] = edgelen + j;
	//		edge2facelist.resize(edgelen + curfacenum * (segnum - 1));
	//		//------------//
	////		cout<<"incident face number:"<<curfacenum<<endl;
	//		for( int j = 0; j < curfacenum; j ++)
	//		{
	//		//	cout<<"curface:"<<curfacelist[2*j]<<endl;
	//			int edgepos = curfacelist[ 2*j+1 ];
	//			int thirdverpos = facelist[5 * curfacelist[ 2*j ]+(edgepos + 2)%3];
	//		//	cout<<"current face:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
	//	//			" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
	//	//			<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
	//		//	cout<<"edgepos"<<edgepos<<"\t";
	//		//	cout<<"thirdver:"<<thirdverpos<<"\n";
	//			bool iscw = (verpos[ 0 ] == facelist[ 5 * curfacelist[ 2*j ] + edgepos ]);
	//			int mat[2] = {facelist[ 5 * curfacelist[ 2*j ] + 3], facelist[ 5 * curfacelist[ 2*j ] + 4]};
	//
	//			//add new edge, add new face, refresh edge2facelist
	//			//add new edge
	//            for( int k = 0; k < segnum - 1; k++ )
	//			{
	//				edgelist.push_back( thirdverpos );
	//				edgelist.push_back( verlen + k );
	//			}
	//			//------------//
	//		//	cout<<"new edge added for current face!"<<endl;
	//			//new face
	//			if( iscw )
	//			{
	//				for( int k = 0; k < segnum - 2; k ++ )
	//				{
	//					facelist.push_back( thirdverpos );
	//					facelist.push_back( verlen + k );
	//					facelist.push_back( verlen + k + 1);
	//					facelist.push_back( mat[ 0 ]);
	//					facelist.push_back( mat[ 1 ]);
	//				}
	//				for( int k = 0; k < 5; k ++)
	//					facelist.push_back( facelist[ 5 * curfacelist[ 2*j ] + k ]);
	//				facelist[ ( facenum + segnum - 2 ) * 5 + edgepos ] = verlen + segnum - 2;
	//				facelist[ 5 * curfacelist[ 2*j ] + (edgepos + 1)%3 ] = verlen;
	//			}
	//			else
	//			{
	//				for( int k = 0; k < segnum - 2; k ++)
	//				{
	//					facelist.push_back( thirdverpos );
	//					facelist.push_back( verlen + k + 1);
	//					facelist.push_back( verlen + k );					
	//					facelist.push_back( mat[ 0 ]);
	//					facelist.push_back( mat[ 1 ]);
	//				}
	//				for( int k = 0; k < 5; k ++)
	//					facelist.push_back( facelist[ 5 * curfacelist[ 2*j ] + k ]);
	//				facelist[ ( facenum + segnum - 2 ) * 5 + (edgepos + 1)%3] = verlen + segnum - 2;
	//				facelist[ 5 * curfacelist[ 2*j ] + edgepos ] = verlen;
	//			}
	//			//---------------------//
	//			/*cout<<"faces:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
	//				" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
	//				<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
	//			for( int k = 0; k < segnum - 1; k++ )
	//			{
	//				cout<<"faces:"<<facelist[5 * (facenum+k)]<<" "<<facelist[5 *(facenum+k) + 1]<<
	//					" "<<facelist[5 *(facenum+k) + 2]<<" "<<facelist[5 * (facenum+k)+ 3]<<" "
	//					<<facelist[5 * (facenum+k)+4]<<endl;
	//			}*/
	//			facenum += (segnum - 1);
	//		
	//			//edge2facelist
	//			//new edge on the nmedge
	//			int edgelen2 = edgelist.size()/2;
	//			edge2facelist[ edgelen - 1].push_back( facenum - 1);	//the last new edge
	//			edge2facelist[ edgelen - 1].push_back( edgepos );
	//			for( int k = 2; k < segnum; k ++)
	//			{
	//				edge2facelist[ edgelen - k ].push_back( facenum - k);
	//				edge2facelist[ edgelen - k].push_back( 1 );
	//			}
	//			//old edge in the old face
	//			int oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
	//			//if( iscw)
	//			//{
	//			//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
	//			//	//////////////////////////////////////////////////////////////////////////
	//			//	if( oldedge == 25 )
	//			//	{
	//			//		cout<<"cw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[1]<<endl;
	//			//	}
	//			//}
	//			//else
	//			//{
	//			//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 0 ], edgelen2);
	//			//	//////////////////////////////////////////////////////////////////////////
	//			//	if( oldedge == 25 )
	//			//	{
	//			//		cout<<"ccw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[0]<<endl;
	//			//	}
	//			//}
	//
	//			if(oldedge == edgelen2 )
	//			{
	//				cout<<"trying to find the old edge fails!!!"<<endl;
	//			}
	//			else
	//			{
	//				for( int k = 0; k < edge2facelist[ oldedge ].size(); k++)
	//				{
	//					if( edge2facelist[ oldedge ][ k * 2 ] == curfacelist[ j * 2 ])
	//					{
	//						edge2facelist[ oldedge ][ k * 2 ] = facenum - 1;
	//						break;
	//					}
	//				}
	//			}
	//			//new edge on the face
	//			int pos = edgelen + (segnum - 1) * j;
	//			for( int k = 0; k < segnum - 2; k++)
	//			{
	//				edge2facelist[ pos + k ].push_back( facenum - segnum + k + 1);
	//				if( iscw )
	//					edge2facelist[ pos + k ].push_back( 0 );
	//				else
	//					edge2facelist[ pos + k  ].push_back( 2 );
	//			}
	//			//last new edge
	//			edge2facelist[ pos + segnum - 2].push_back( facenum - 1 );
	//			if( iscw )
	//				edge2facelist[ pos + segnum - 2].push_back( (edgepos + 2)%3 );
	//			else
	//				edge2facelist[ pos + segnum - 2].push_back( (edgepos + 1)%3 );
	//			for( int k = segnum - 2; k >= 1; k --)
	//			{
	//				edge2facelist[ pos + k ].push_back( facenum - segnum + k);
	//				if( iscw )
	//					edge2facelist[ pos + k ].push_back( 2 );
	//				else
	//					edge2facelist[ pos + k ].push_back( 0 );
	//			}
	//			edge2facelist[ pos ].push_back( curfacelist[ j * 2 ] );
	//			if( iscw )
	//				edge2facelist[ pos ].push_back( (edgepos + 1)%3 );
	//			else
	//				edge2facelist[ pos ].push_back( (edgepos + 2)%3 );
	//		//	cout<<"current face is done!"<<endl;
	//		}
	//	}
}
void Mesh::splitOneNormEdge(int segnum, int i, vector<float>& verlist,vector<float>& verattrlist,vector<int>& edgelist,vector<intvector>& edge2facelist,
							vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist, HashMap& ver2edgehash)
{
	int verpos[2] = {edgelist[normedgelist[i]*2],edgelist[normedgelist[i]*2 + 1]};
	//	cout<<"verpos are:"<<verpos[0]<<"   "<<verpos[1]<<endl;
	float newver[3];
	int verlen = verlist.size()/3;
	for( int j = 1; j < segnum; j++ )
	{
		MyMath::getPtOnSeg(&verlist[verpos[0]*3], &verlist[verpos[1]*3], j/(float)segnum, newver);
		verlist.push_back( newver[ 0 ]) ;
		verlist.push_back( newver[ 1 ]);
		verlist.push_back( newver[ 2 ]);
		verattrlist.push_back( (1 - j/(float)segnum)*verattrlist[verpos[0]] +  j/(float)segnum*verattrlist[verpos[1]] );
	}
	//------------//
	//	cout<<"vertex added!"<<endl;
	//new edge on the old normal edge	don't feel weird by following name, changed from splitNMEdge, but you know, they are similar, i am lazy to rewrite it.
	int edgelen = edgelist.size()/2;
	int oldnmedgelen = normedgelist.size();
	normedgelist.resize(oldnmedgelen + segnum - 1);
	for( int j = 1; j < segnum - 1; j ++)
	{
		edgelist.push_back( verlen + j - 1);
		edgelist.push_back( verlen + j);
		normedgelist[ oldnmedgelen + j - 1] = edgelen + j - 1;
		ver2edgehash.findInsertSort( verlen + j -1 , verlen + j, edgelen + j - 1);
	}
	normedgelist[ oldnmedgelen + segnum - 2] = edgelen + segnum - 2;
	edgelist.push_back( verlen + segnum - 2);	//the last segment
	edgelist.push_back( verpos[1] );
	ver2edgehash.findInsertSort( verlen + segnum -2 , verpos[ 1 ], edgelen + segnum - 2 );
	edgelist[ normedgelist[ i ]*2 + 1 ] = verlen;	//second point of the first segment

	int facenum = facelist.size()/5;	//old face number
	edgelen = edgelist.size()/2;		//edge number after adding new edges on non-manifold edge
	//------------//
	//	cout<<"new edge added!"<<endl;
	//every face
	int curfacenum = edge2facelist[ normedgelist[ i ]].size()/2;	//incident faces number

	int oldnormedgelen = normedgelist.size();
	normedgelist.resize( oldnormedgelen + curfacenum*(segnum - 1));
	for( int j = 0; j < (segnum - 1)*curfacenum; j++)
		normedgelist[ oldnormedgelen + j] = edgelen + j;
	edge2facelist.resize(edgelen + curfacenum * (segnum - 1));

	intvector* curfacelist = &edge2facelist[ normedgelist[i]];		//incident faces list
	//------------//
	//cout<<"curedge:"<<normedgelist[i]<<endl;
	//cout<<"incident face number:"<<curfacenum<<endl;
	//for(int j = 0; j < curfacenum*2; j ++) 
	//{
	//	cout<<edge2facelist[normedgelist[i]][j]<<" ";
	//	cout<<(*curfacelist)[j]<<" ";
	//}
	//cout<<endl;

	for( int j = 0; j < curfacenum; j ++)
	{
		//	cout<<"curface:"<<curfacelist[2*j]<<endl;
		int edgepos = (*curfacelist)[ 2*j+1 ];
		int thirdverpos = facelist[5 * (*curfacelist)[ 2*j ]+(edgepos + 2)%3];
		//	cout<<"current face:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
		//			" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
		//			<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
		//	cout<<"edgepos"<<edgepos<<"\t";
		//	cout<<"thirdver:"<<thirdverpos<<"\n";
		bool iscw = (verpos[ 0 ] == facelist[ 5 * (*curfacelist)[ 2*j ] + edgepos ]);
		int mat[2] = {facelist[ 5 * (*curfacelist)[ 2*j ] + 3], facelist[ 5 * (*curfacelist)[ 2*j ] + 4]};

		//add new edge, add new face, refresh edge2facelist
		//add new edge
		for( int k = 0; k < segnum - 1; k++ )
		{
			edgelist.push_back( thirdverpos );
			edgelist.push_back( verlen + k );
			ver2edgehash.findInsertSort( thirdverpos, verlen + k, (segnum - 1) * j + k + edgelen  );
		}
		//------------//
		//	cout<<"new edge added for current face!"<<endl;
		//new face
		if( iscw )
		{
			for( int k = 0; k < segnum - 2; k ++ )
			{
				facelist.push_back( thirdverpos );
				facelist.push_back( verlen + k );
				facelist.push_back( verlen + k + 1);
				facelist.push_back( mat[ 0 ]);
				facelist.push_back( mat[ 1 ]);
			}
			for( int k = 0; k < 5; k ++)
				facelist.push_back( facelist[ 5 *(* curfacelist)[ 2*j ] + k ]);
			facelist[ ( facenum + segnum - 2 ) * 5 + edgepos ] = verlen + segnum - 2;
			facelist[ 5 * (*curfacelist)[ 2*j ] + (edgepos + 1)%3 ] = verlen;
		}
		else
		{
			for( int k = 0; k < segnum - 2; k ++)
			{
				facelist.push_back( thirdverpos );
				facelist.push_back( verlen + k + 1);
				facelist.push_back( verlen + k );					
				facelist.push_back( mat[ 0 ]);
				facelist.push_back( mat[ 1 ]);
			}
			for( int k = 0; k < 5; k ++)
				facelist.push_back( facelist[ 5 * (*curfacelist)[ 2*j ] + k ]);
			facelist[ ( facenum + segnum - 2 ) * 5 + (edgepos + 1)%3] = verlen + segnum - 2;
			facelist[ 5 * (*curfacelist)[ 2*j ] + edgepos ] = verlen;
		}
		//---------------------//
		/*cout<<"faces:"<<facelist[5 * curfacelist[ 2*j ]]<<" "<<facelist[5 * curfacelist[ 2*j ] + 1]<<
		" "<<facelist[5 * curfacelist[ 2*j ] + 2]<<" "<<facelist[5 * curfacelist[ 2*j ]+ 3]<<" "
		<<facelist[5 * curfacelist[ 2*j ]+4]<<endl;
		for( int k = 0; k < segnum - 1; k++ )
		{
		cout<<"faces:"<<facelist[5 * (facenum+k)]<<" "<<facelist[5 *(facenum+k) + 1]<<
		" "<<facelist[5 *(facenum+k) + 2]<<" "<<facelist[5 * (facenum+k)+ 3]<<" "
		<<facelist[5 * (facenum+k)+4]<<endl;
		}*/
		facenum += (segnum - 1);

		//edge2facelist
		//new edge on the nmedge
		int edgelen2 = edgelist.size()/2;
		edge2facelist[ edgelen - 1].push_back( facenum - 1);	//the last new edge
		edge2facelist[ edgelen - 1].push_back( edgepos );
		for( int k = 2; k < segnum; k ++)
		{
			edge2facelist[ edgelen - k ].push_back( facenum - k);
			edge2facelist[ edgelen - k].push_back( 1 );
		}
		//old edge in the old face
		int oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
		//if( iscw)
		//{
		//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 1 ], edgelen2);
		//	//////////////////////////////////////////////////////////////////////////
		//	if( oldedge == 25 )
		//	{
		//		cout<<"cw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[1]<<endl;
		//	}
		//}
		//else
		//{
		//	oldedge = ver2edgehash.findInsertSort( thirdverpos, verpos[ 0 ], edgelen2);
		//	//////////////////////////////////////////////////////////////////////////
		//	if( oldedge == 25 )
		//	{
		//		cout<<"ccw oldedge is :"<<oldedge<<" "<<thirdverpos<<" "<<verpos[0]<<endl;
		//	}
		//}

		if(oldedge == edgelen2 )
		{
			cout<<"trying to find the old edge fails!!!"<<endl;
		}
		else
		{
			for(unsigned int k = 0; k < edge2facelist[ oldedge ].size(); k++)
			{
				if( edge2facelist[ oldedge ][ k * 2 ] == (*curfacelist)[ j * 2 ])
				{
					edge2facelist[ oldedge ][ k * 2 ] = facenum - 1;
					break;
				}
			}
		}
		//new edge on the face
		int pos = edgelen + (segnum - 1) * j;
		for( int k = 0; k < segnum - 2; k++)
		{
			edge2facelist[ pos + k ].push_back( facenum - segnum + k + 1);
			if( iscw )
				edge2facelist[ pos + k ].push_back( 0 );
			else
				edge2facelist[ pos + k  ].push_back( 2 );
		}
		//last new edge
		edge2facelist[ pos + segnum - 2].push_back( facenum - 1 );
		if( iscw )
			edge2facelist[ pos + segnum - 2].push_back( (edgepos + 2)%3 );
		else
			edge2facelist[ pos + segnum - 2].push_back( (edgepos + 1)%3 );
		for( int k = segnum - 2; k >= 1; k --)
		{
			edge2facelist[ pos + k ].push_back( facenum - segnum + k);
			if( iscw )
				edge2facelist[ pos + k ].push_back( 2 );
			else
				edge2facelist[ pos + k ].push_back( 0 );
		}
		edge2facelist[ pos ].push_back((*curfacelist)[ j * 2 ] );
		if( iscw )
			edge2facelist[ pos ].push_back( (edgepos + 1)%3 );
		else
			edge2facelist[ pos ].push_back( (edgepos + 2)%3 );
		//	cout<<"current face is done!"<<endl;
	}
}

void Mesh::splitNCtrEdgeTwoCtrVer(vector<float>& verlist,vector<float>& verattrlist,vector<int>&vermark, vector<int>& edgelist,vector<intvector>& edge2facelist,
								  vector<int>&nmedgelist, vector<int>& normedgelist,vector<int>&facelist,		HashMap& ver2edgehash, const int oldedgenum)
{
	cout<<"oldedgenumber is :"<<oldedgenum<<endl;
	const int oldverlen = vermark.size();	//possible contour vertex
	cout<<"old ver number is:"<<oldverlen<<endl;
	int veri1, veri2;
	for(unsigned int i = 0; i <nmedgelist.size(); i ++)
	{
		if( nmedgelist[ i ] >= oldedgenum) break;
		veri1 = edgelist[ nmedgelist[i] * 2 ];
		if( veri1 >= oldverlen )continue;
		veri2 = edgelist[ nmedgelist[i]*2 + 1 ];
		if( veri2 >= oldverlen )continue;

		if(!( (vermark[ veri1 ] == 1) && (vermark[ veri2 ] == 1) ))continue;

		//split the non-manifold edge
		splitOneNMEdge(2,   i, verlist,  verattrlist, edgelist, edge2facelist, nmedgelist, normedgelist,facelist, ver2edgehash);
	}
	for(unsigned int i = 0; i < normedgelist.size(); i++)
	{
		if(  normedgelist[i]  >= oldedgenum ) break;
		veri1 = edgelist[ normedgelist[i] * 2 ];
		if( veri1 >= oldverlen )continue;
		veri2 = edgelist[ normedgelist[i]*2 + 1 ];
		if( veri2 >= oldverlen )continue;

		if(!(( (vermark[ veri1 ] == 1)) && (vermark[ veri2 ] == 1) ))continue;
		//cout<<"veri1:"<<veri1<<"mark:"<<vermark[veri1]<<"  "<<"veri2:"<<veri2<<"mark:"<<vermark[veri2]<<endl;
		//split the normal edge
		splitOneNormEdge(2,  i,  verlist,verattrlist,edgelist,edge2facelist, nmedgelist,  normedgelist, facelist, ver2edgehash);
	}
}
const int MAXTIMES = 1024;
void Mesh::swapEdge(vector<float>&verlist,vector<int>& edgelist,vector<intvector>& edge2facelist,vector<int>&normedgelist, vector<int>&facelist, HashMap& ver2edgehash)
{
	int edgelen = edgelist.size()/2;
	//	HashMap ver2edgehash;
//	int temp;
	/*	for( int i = 0; i < edgelen; i ++)
	{
	if( ver2edgehash.findInsertSort( edgelist[ i * 2 ], edgelist[ i * 2 + 1], i ) != i)
	cout<<"Error occurs when building hash map for edges!"<<endl;
	}*/
	bool again = true;
	int count = 0;
	edgelen = normedgelist.size();

	//////////////////////////////////////////////////////////////////////////
	/*	FILE* fout = fopen("out.txt","w");
	for( int i = 0; i < edgelen; i ++)
	{
	fprintf( fout, "%d\t%d\t%d\n",i,edgelist[ 2*i ], edgelist[ 2*i+1 ]);
	//all the faces
	for( int j = 0; j < edge2facelist[ i ].size()/2; j++)
	{
	int facei = edge2facelist[ i ][ 2*j ];
	fprintf(fout, "%d\t%d\t%d\t%d\n", facelist[5*facei], facelist[5*facei+1],facelist[5*facei+2],
	edge2facelist[i][j*2+1]);
	}
	}
	fclose(fout);*/
	//////////////////////////////////////////////////////////////////////////
	while( again && count < MAXTIMES)
	{
		again = false;
		count ++;

		/////////////////////////check/////////////////////////////////////////////////
		/*for( int i = 0;  i< edgelen ; i ++)
		{
		for( int j = 0; j < edge2facelist[i].size()/2; j++)
		{
		int facei = edge2facelist[i][ j*2 ];
		int pos = edge2facelist[i][j*2+1];
		bool issame = false;
		issame = ((edgelist[ 2*i ] == facelist[ 5*facei + pos ]) && (edgelist[2*i+1]==facelist[5*facei+(pos+1)%3]));
		if(issame)continue;
		issame = ((edgelist[ 2*i ] == facelist[ 5*facei +(pos+1)%3 ]) && (edgelist[2*i+1]==facelist[5*facei+ pos]));
		if( issame )continue;
		cout<<"edge :"<<i<<" "<<edgelist[ 2*i ]<<edgelist[ 2*i+1 ]<<"pos:"<<pos<<endl;
		cout<<"face:"<<facelist[5*facei]<<" "<<facelist[5*facei+1]<<" "<<facelist[5*facei+2]<<endl;
		}
		}*/
		//go through each interior edge, swap it if needed
		//for( int i = 0; i < edgelen; i ++)
		//	cout<<edgelen;
		for( int i = 0; i < edgelen; i ++)
		{			
			intvector& facepair = edge2facelist[ normedgelist[ i ]];
			if( facepair.size() != 4)
			{
				/*-------*/
				if( facepair.size() > 4)
					cout<<"incident faces number is over 4, error!"<<endl;
				else 
					cout<<"incident faces number is smaller than 4, error!"<<endl;
				continue;	
			}

			int verpos[2] = {edgelist[ normedgelist[ i ]*2], edgelist[ normedgelist[ i ]*2+1]};
			int thirdver[ 2 ] = { facelist[ 5 * facepair[ 0 ] + (facepair[ 1 ] + 2)%3 ], 
				facelist[ 5 * facepair[ 2 ] + (facepair[ 3 ] + 2)%3 ]};
			//	cout<<verpos[0]<<" "<<verpos[1]<<" "<<thirdver[0]<<" "<<thirdver[1]<<endl;
			//	cout<<"face:"<<facepair[0]<<"edge:"<<facepair[1]<<" "<<facelist[ 5 * facepair[ 0 ]]<<" "<<facelist[ 5 * facepair[ 0 ]+1]<<" "<<facelist[ 5 * facepair[ 0 ]+2]<<endl;
			//	cout<<"face:"<<facepair[2]<<"edge:"<<facepair[3]<<" "<<facelist[ 5 * facepair[ 2 ]]<<" "<<facelist[ 5 * facepair[ 2 ]+1]<<" "<<facelist[ 5 * facepair[ 2 ]+2]<<endl;

			//compute the angle of the third angles
			float cosab[2] ;
			cosab[ 0 ] = MyMath::getCosOfAngle(&verlist[3*verpos[0]], &verlist[3*verpos[1]], &verlist[ 3 * thirdver[0]]);
			cosab[ 1 ] = MyMath::getCosOfAngle(&verlist[3*verpos[0]], &verlist[3*verpos[1]], &verlist[ 3 * thirdver[1]]);
			//////////////////////////////////////////////////////////////////////////
			/*if( cosab[0]<0)
			cout<<"cos1:"<<cosab[0];
			if( cosab[1]<0)
			cout<<"\tcos2:"<<cosab[1]<<endl;*/
			//////////////////////////////////////////////////////////////////////////
			/*if( cosab[0] >= 1 || cosab[ 0 ] <= -1)
			{
			cout<<"pt1:"<<verlist[3*verpos[0]]<<", "<<verlist[3*verpos[0]+1]<<", "<<verlist[3*verpos[0]+2]<<endl;
			cout<<"pt2:"<<verlist[3*verpos[1]]<<", "<<verlist[3*verpos[1]+1]<<", "<<verlist[3*verpos[1]+2]<<endl;
			cout<<"pt0:"<<verlist[ 3 * thirdver[0]]<<", "<<verlist[3 * thirdver[0]+1]<<", "<<verlist[ 3 * thirdver[0]+2]<<endl;
			cout<<MyMath::getCosOfAngle(&verlist[3*verpos[0]], &verlist[3*verpos[1]], &verlist[ 3 * thirdver[0]])<<endl;
			}
			if( cosab[1] >= 1 && cosab[ 1 ] <= -1)
			{
			cout<<"pt1:"<<verlist[3*verpos[0]]<<", "<<verlist[3*verpos[0]+1]<<", "<<verlist[3*verpos[0]+2]<<endl;
			cout<<"pt2:"<<verlist[3*verpos[1]]<<", "<<verlist[3*verpos[1]+1]<<", "<<verlist[3*verpos[1]+2]<<endl;
			cout<<"pt0:"<<verlist[ 3 * thirdver[1]]<<", "<<verlist[3 * thirdver[1]+1]<<", "<<verlist[ 3 * thirdver[1]+2]<<endl;
			cout<<MyMath::getCosOfAngle(&verlist[3*verpos[0]], &verlist[3*verpos[1]], &verlist[ 3 * thirdver[1]])<<endl;
			}*/
			float sinab[2];
			sinab[ 0 ] = sqrt(1 - cosab[0]*cosab[ 0 ]);
			sinab[  1 ] = sqrt(1 -cosab[ 1 ]*cosab[ 1 ]);
			//	cout<<cosab[0]*cosab[ 0 ]<<"  "<<sinab[0]<<cosab[ 1 ]*cosab[ 1 ]<<" "<<sinab[  1 ]<<endl;
			//cout<<(cosab[ 0 ]* sinab[ 1 ] + cosab[ 1 ] * sinab[ 0 ])<<" ";
			if( (cosab[ 0 ]* sinab[ 1 ] + cosab[ 1 ] * sinab[ 0 ]) < -0.000001)	//>180
			{
				//see if the new edge going to add is in the mesh or not
				if(ver2edgehash.findInsertSort(thirdver[0], thirdver[1], -1) != -1)
				{
					//DBWindowWrite("edge already added...\n");
					continue;
				}
				again = true;
				//flip edge,change two faces, change the affected edge's edge2facelist
				//edge list and hash
				edgelist[ normedgelist[i] * 2 ] = thirdver[ 0 ];
				edgelist[ normedgelist[i] * 2 + 1] = thirdver[ 1 ];
				//-----------------//
				//	cout<<endl;
				//	cout<<"the two faces are:"<<endl;
				/*if( i == 85)
				for( int j = 0; j < 2; j ++)
				{
				cout<<facelist[ facepair[2*j]*5]<<"  "<<facelist[ facepair[2*j]*5+1]<<"  "<<facelist[ facepair[2*j]*5+2]<<"  "<<
				facelist[ facepair[2*j]*5+3]<<"  "<<facelist[ facepair[2*j]*5+4]<<endl;
				}
				cout<<"thirdver:"<<thirdver[0]<<"\t"<<thirdver[1]<<endl;*/
				//replace the old edge if it exists
				ver2edgehash.findInsertSortReplace(verpos[0], verpos[1], -1);	//edge is not in the mesh now
				ver2edgehash.findInsertSortReplace( thirdver[0], thirdver[1], normedgelist[ i ]);
				//facelist
				int edgei;
				//////////////////////////////////////////////////////////////////////////
				//	cout<<"new edge:"<<thirdver[0]<<"  "<<thirdver[1]<<endl;
				for( int j = 0; j < 2; j ++)
				{
					//for facepair[ 2*j ], change its verpos[ j ] to thirdver[ 1 - j ]
					int facei = facepair[ j * 2 ];
					int edgepos = facepair[ j*2 + 1 ];
					if( facelist[ 5*facei + facepair[ j*2 + 1 ] ] == verpos[ j ])
					{
						//cout<<"type1"<<endl;
						facelist[ 5*facei + facepair[ j*2 + 1 ] ] = thirdver[ 1 - j ];
						/*if( i == 85)
						cout<<"facepair[ j*2 + 1 ]"<<facepair[ j*2 + 1 ]<<endl;*/
						/*if( i == 85)
						cout<<"trying to find old edge:"<< thirdver[1-j]<<" "<<facelist[ 5*facei + (facepair[ j*2 + 1 ]+1)%3]<<endl;*/
						edgei = ver2edgehash.findInsertSort( thirdver[1-j], facelist[ 5*facei + (facepair[ j*2 + 1 ]+1)%3], -1);
						//////////////////////////////////////////////////////////////////////////
						//	cout<<"vers:"<<thirdver[1-j]<<" "<<facelist[ 5*facei + (facepair[ j*2 + 1 ]+1)%3]<<" edge:"<<edgei<<"edgepos:"<<edgepos<<endl;
						//	cout<<"current triangle:"<<facelist[5*facei]<<" "<<facelist[5*facei+1]<<" "<<facelist[5*facei+2]<<endl;
						facepair[ j * 2 + 1 ] = (facepair[ j * 2 + 1] + 2)%3;
						////////////////////////////////check//////////////////////////////////////////
						/*if(!(((facelist[5*facei + facepair[j*2+1]] == thirdver[0])&&(facelist[5*facei + (facepair[j*2+1]+1)%3] == thirdver[1]))
						||((facelist[5*facei + facepair[j*2+1]] == thirdver[1])&&(facelist[5*facei + (facepair[j*2+1]+1)%3] == thirdver[0]))))
						{							
						cout<<"thirdver:"<<thirdver[0]<<"\t"<<thirdver[1]<<endl;
						cout<<"face:"<<facelist[ 5*facei ]<<" "<<facelist[5*facei+1]<<" "<<facelist[5*facei+2]<<" edge:"<<facepair[2*j+1]<<endl;

						cout<<"new edge error! count:"<<count<<"edge "<<i<<"type1"<<endl;
						}*/
						//	cout<<"face:"<<facelist[5*facei]<<" "<<facelist[facei*5+1]<<"  "<<facelist[facei*5+2]<<" pos:"<<facepair[j*2+1]<<endl;
						if( edgei == -1)
						{
							cout<<"error occurs while finding the edge in the reconfigured face!"<<endl;
						}
						else
						{
							intvector& tfacelist = edge2facelist[ edgei ];	
							for(unsigned int k = 0; k < tfacelist.size()/2; k++)
							{
								if( tfacelist[ k * 2 ] == facepair[ (1-j)*2 ])
								{
									tfacelist[ k * 2 ] = facei;
									tfacelist[ k*2 + 1] = edgepos ;
									//////////////////////////////////////////////////////////////////////////
									//	 cout<<thirdver[1-j]<<" "<<
									/*if( i == 85)
									cout<<"found!"<<tfacelist[ k * 2 ]<<" "<<tfacelist[ k * 2 + 1]<<endl;*/
									break;
								}
							}
						}
					}
					else
					{
						//	cout<<"type2"<<endl;
						facelist[ 5*facei + (facepair[ j*2 + 1 ]+1)%3 ] = thirdver[ 1 - j ];
						/*if( i == 85)
						cout<<"trying to find old edge:"<<" "<< thirdver[1-j]<<facelist[ 5*facei + facepair[ j*2 + 1 ]]<<endl;*/
						edgei = ver2edgehash.findInsertSort( thirdver[1-j], facelist[ 5*facei + facepair[ j*2 + 1 ]], -1);
						//	cout<<"vers:"<<thirdver[1-j]<<" "<<facelist[ 5*facei + (facepair[ j*2 + 1 ]+1)%3]<<" edge:"<<edgei<<"edgepos:"<<edgepos<<endl;
						//	cout<<"current triangle:"<<facelist[5*facei]<<" "<<facelist[5*facei+1]<<" "<<facelist[5*facei+2]<<endl;
						facepair[ j*2 + 1] = (facepair[ j*2 + 1 ]+1)%3;
						//	cout<<"face:"<<facelist[5*facei]<<" "<<facelist[facei*5+1]<<"  "<<facelist[facei*5+2]<<" pos:"<<facepair[j*2+1]<<endl;
						////////////////////////////////check//////////////////////////////////////////
						/*if(!( ((facelist[5*facei + facepair[j*2+1]] == thirdver[0])&&(facelist[5*facei + (facepair[j*2+1]+1)%3] == thirdver[1]))
						||((facelist[5*facei + facepair[j*2+1]] == thirdver[1])&&(facelist[5*facei + (facepair[j*2+1]+1)%3] == thirdver[0]))))
						{
						cout<<"thirdver:"<<thirdver[0]<<"\t"<<thirdver[1]<<endl;
						cout<<"face:"<<facelist[ 5*facei ]<<" "<<facelist[5*facei+1]<<" "<<facelist[5*facei+2]<<" edge:"<<facepair[2*j+1]<<endl;

						cout<<"new edge error! count:"<<count<<"edge "<<i<<"type2"<<endl;
						}*/
						if( edgei == -1)
						{
							cout<<"error occurs while finding the edge in the reconfigured face!"<<endl;
						}
						else
						{
							intvector& tfacelist = edge2facelist[ edgei ];	
							for(unsigned int k = 0; k < tfacelist.size()/2; k++)
							{
								if( tfacelist[ k * 2 ] == facepair[ (1-j)*2 ])
								{
									tfacelist[ k * 2 ] = facei;
									tfacelist[ k*2 + 1] = edgepos ;
									/*if( i == 85)
									cout<<"found!"<<tfacelist[ k * 2 ]<<" "<<tfacelist[ k * 2 + 1]<<endl;*/
									break;
								}
							}
						}
					}
				}
				//	cout<<"the two faces are:"<<endl;
				/*if( i == 85)
				for( int j = 0; j < 2; j ++)
				{
				cout<<facelist[ facepair[2*j]*5]<<"  "<<facelist[ facepair[2*j]*5+1]<<"  "<<facelist[ facepair[2*j]*5+2]<<"  "<<
				facelist[ facepair[2*j]*5+3]<<"  "<<facelist[ facepair[2*j]*5+4]<<endl;
				}*/
				//	cout<<"one edge is flipped!"<<endl;
				//		break;
			}			
		}
		//	cout<<"again is "<<again<<endl;
		//	if(count == 2)
		//		break;
		//	break;
	}
	cout<<"called "<<count<<" times!";
}
void Mesh::getEdgeTypeList(vector<int>&normedgelist, vector<int>&nmedgelist, vector<int>&ctredgelist,vector<int>&edgetypelist)
{
	edgetypelist.resize(ctredgelist.size()+nmedgelist.size() + normedgelist.size(),0);
	for(unsigned int i = 0; i < ctredgelist.size(); i++)	//1 - contour edge
	{
		edgetypelist[ ctredgelist[ i ] ] = 1;
	}
	for(unsigned int i = 0; i < nmedgelist.size(); i ++)
	{
		edgetypelist[ nmedgelist[ i ]] = 2;
	}
}
bool Mesh::splitTriangle(vector<float>&verlist, vector<float>&verattrilist, HashMap& ver2edgehash2,
						 vector<int>&edgelist, vector<int>&normedgelist, vector<intvector>&edge2facelist,
						 vector<int>&facelist, float alpha)
{
	bool splitExist = false;
	//	bool again = true;
	//	int count = 0;
	int edgenum = edgelist.size()/2;
	//	while( again && count < MAXTIMES)
	//	{
	//		again = false;
	//		count ++;
	int oldfacenum , facenum;
	oldfacenum = facenum = facelist.size()/5;
	int iKwTestSplitNum=0;
	for(int curtri = 0; curtri < oldfacenum ; curtri++)
	{
		//centroid
		float centerpos[3] = {0,0,0};
		for( int i = 0; i < 3; i ++)
		{
			for( int j = 0; j < 3; j++)
			{
				centerpos[ i ] += verlist[ 3 * facelist[ curtri * 5 + j ] + i];
			}
			centerpos[ i ]/=3;
		}
		//distance
		float distance[3] = {0,0,0};
		bool split = true;
		float centerattr = 0;
		for( int i = 0; i < 3; i ++)
		{
			distance[ i ] = MyMath::vectorlen( centerpos, &verlist[ 3 * facelist[ curtri*5 + i ]]);
			if( alpha * distance[ i ] < verattrilist[ facelist[curtri*5+i]] )
			{
				split = false;
				break;
			}
			centerattr += verattrilist[ facelist[curtri*5+i]];
		}
		if( !split )continue;

		iKwTestSplitNum++;
		//////////////////////////////////////////////////////////////////////////
		/*cout<<"faces vertex:";
		for( int i = 0; i < 5; i ++)
		{
		cout<<facelist[5*curtri + i]<<" "; 
		}
		cout<<endl;
		cout<<"edges : ";
		for( int i = 0; i< 3; i ++)
		{
		cout<<ver2edgehash2.findInsertSort(facelist[5*curtri + i], facelist[5*curtri + (i+1)%3], -1)<<"  ";
		}
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////
		splitExist = true;
		//split
		//add new vertex
		verlist.push_back( centerpos[0] );
		verlist.push_back( centerpos[1]);
		verlist.push_back( centerpos[2]);
		centerattr/=3;
		verattrilist.push_back( centerattr );
		int verlen = verlist.size()/3;
		//new edge
		for( int i = 0; i< 3; i ++)
		{
			edgelist.push_back(verlen - 1);
			edgelist.push_back(facelist[curtri*5 + i]);
			int tedge = 	ver2edgehash2.findInsertSort( verlen - 1, facelist[ curtri*5 + i ], edgenum + i);
			if( tedge != edgenum + i)
			{
				cout<<"tedge is :"<<tedge<<"should be:"<<edgenum+i<<endl;
			}
			normedgelist.push_back( edgenum + i);
		}			
		edgenum += 3;			

		//face
		//add new
		for( int i = 1; i< 3; i ++)
		{
			facelist.push_back(facelist[curtri*5+ i ]);
			facelist.push_back( facelist[ curtri*5 + (i+1)%3]);
			facelist.push_back( verlen - 1);
			facelist.push_back( facelist[ curtri * 5 + 3]);
			facelist.push_back( facelist[ curtri*5 + 4]);				
		}
		facelist[ 5* curtri + 2] = verlen - 1;
		facenum += 2;

		//edge2facelist
		int ver1;
		int ver2;
		int edgei;
		//for the two old face edges
		for( int i = 1; i <=2; i++)
		{
			ver1 = facelist[ 5* (facenum + i - 3)];
			ver2 = facelist[ 5* (facenum + i - 3) + 1];
			edgei = ver2edgehash2.findInsertSort(ver1, ver2, -1);
			if( edgei == -1)
			{
				cout<<"Error occurs when trying to find the old edge!"<<endl;
			}
			else
			{
				//////////////////////////////////////////////////////////////////////////
				//	cout<<"ver1:"<<ver1<<" ver2:"<<ver2<<" edgei:"<<edgei<<endl;
				//////////////////////////////////////////////////////////////////////////
				vector<int>& tfacelist = edge2facelist[ edgei ];
				for(unsigned int j = 0; j < tfacelist.size()/2; j ++)
				{
					if( tfacelist[ j * 2 ] == curtri )
					{
						//////////////////////////////////////////////////////////////////////////
						//			cout<<"curtri is found in the list!"<<endl;
						//////////////////////////////////////////////////////////////////////////
						tfacelist[ j * 2 ] = facenum + i - 3;
						tfacelist[ j*2 + 1] = 0;
						break;
					}
				}
			}
		}
		//for the three new face edges
		edge2facelist.resize( edgenum );
		intvector* tfacelist = &edge2facelist[ edgenum - 3];
		tfacelist->push_back( curtri );
		tfacelist->push_back( 2 );
		tfacelist->push_back( facenum - 1);
		tfacelist->push_back(1);
		tfacelist = &edge2facelist[ edgenum - 2];
		tfacelist->push_back( curtri );
		tfacelist->push_back( 1 );
		tfacelist->push_back( facenum - 2 );
		tfacelist->push_back( 2 );
		tfacelist = &edge2facelist[ edgenum - 1];
		tfacelist->push_back( facenum - 2);
		tfacelist->push_back( 1 ) ;
		tfacelist->push_back( facenum - 1);
		tfacelist->push_back( 2 );

		//////////////////////////////////////////////////////////////////////////
		/*	cout<<"edgenum:"<<edgenum<<endl;
		for( int i = 0; i < normedgelist.size(); i ++)
		{
		//cout<<edge2facelist[ normedgelist[i] ].size()<<" " ;
		if(edge2facelist[ normedgelist[i] ].size()!=4)
		cout<<"edge "<<normedgelist[i]<<edge2facelist[ normedgelist[i] ].size()<<" ";
		}
		cout<<endl;*/
		//////////////////////////////////////////////////////////////////////////
		//	break;	////
		//propagate the swap

	}
	cout<<iKwTestSplitNum<<" triangles splitted"<<endl;
	//	}
	return splitExist;
}
void Mesh::LiepaRefine(float alpha)
{
	/*edge*/
	//kw: each pair represents the indices of two end vertices of an edge
	vector<int> edgelist;
	//	edgelist.clear();
	//	edge2facelist.clear();
	vector<intvector> edge2facelist;
	//kw: indices of contour edges
	vector<int> ctredgelist;
	//kw: indices of non-manifold edges
	vector<int>	nmedgelist;
	//	nmedgelist.clear();
	//kw: indices of plain edges (not contour, not non-manifold)
	vector<int> normedgelist;
	//kw: lengths of edges
	vector<float> edgelenlist;
	cout<<"gather edge information...."<<"\t";
	HashMap ver2edgehash;
	gatherEdgeInfo(edgelist, edge2facelist,ver2edgehash,ctredgelist,nmedgelist,normedgelist,edgelenlist);
	cout<<"done!"<<endl;
	cout<<"ctredgelist len:"<<ctredgelist.size()<<endl;

	/*ver*/
	cout<<"gather vertex information......"<<"\t";
	//kw: positions of vertices (3 elements (x,y,z) for one vertex)
	vector<float> verlist;
	//kw: average edge length for each vertex
	vector<float> verattrlist;
	//0 - normal ver 1 - ver on contour edge 2 - ver on non manifold edge not on contour edge
	vector<int> vermark;
	gatherVerInfo( verlist,verattrlist,vermark, edgelist, edgelenlist,nmedgelist, normedgelist,ctredgelist );
	int oldedgenum = edgelist.size()/2;		//before splitting the non-manifold edge, for the splitting those non-contour edges with two contour vertices
	//////////////////////////////////////////////////////////////////////////
	cout<<"nmedgelist.size()"<<nmedgelist.size()<<endl;
	cout<<"done!"<<endl;
	//cout<<"normedgelist.size"<<normedgelist.size()<<endl;

	/*face*/
	//kw: indices of three vertices and two material parameters for each triangle
	vector<int> facelist;
	gatherFaceInfo(facelist);
	//	cout<<"facelist.size()"<<facelist.size()/5<<endl;

	/*---debug--*/
	//dbedgelist.clear();
	//dbnmedgelist.clear();
	//dbedgelist.resize(edgelist.size());
	//for( int i =0; i < edgelist.size(); i ++)
	//{
	//	dbedgelist[ i ] = edgelist[ i ];
	//}
	//dbnmedgelist.resize( nmedgelist.size());
	//for( int i = 0; i < nmedgelist.size(); i ++)
	//{
	//	dbnmedgelist[ i ] = nmedgelist[ i ];
	//}
	////find the edge that connects the third vertex and nmvertex together
	//int dbedgelen = edgelist.size()/2;
	//for( int i = 0; i < nmedgelist.size(); i ++)
	//{
	//	int edgei = nmedgelist[ i ];
	//	int verpos[ 2 ] = {edgelist[ edgei * 2 ], edgelist[ edgei * 2  + 1 ]};
	//	intvector& curfacelist = edge2facelist[ edgei ];
	//	for( int j = 0; j < curfacelist.size()/2; j ++)
	//	{
	//		int facei = curfacelist[ j * 2  ];
	//		int edgepos = curfacelist[ j * 2 + 1 ];
	//		int thirdver = facelist[ 5*facei + (edgepos + 2)%3 ];
	//		for( int k = 0; k < 2; k ++)
	//		{                
	//			int edgeindex = ver2edgehash.findInsertSort( thirdver, verpos[ k ], dbedgelen );
	//			if( edgeindex == dbedgelen )
	//			{
	//				cout<<"HASH IS NOT CORRECTLY BUILT!!!!"<<endl;
	//			}
	//			else
	//				dbnmedgelist.push_back( edgeindex);
	//		}
	//	}
	//}
	/*------*/
	/*-----------debug----------*/
	/*dbnmedgelist.resize(nmedgelist.size()*2);
	for( int i = 0; i < nmedgelist.size(); i++)
	{
	dbnmedgelist[ i * 2] = edgelist[nmedgelist[ i ]*2];
	dbnmedgelist[ i * 2 + 1] = edgelist[nmedgelist[ i ]*2 + 1];
	}*/
	/*------------------------------*/



	/*split non-manifold edge*/
	cout<<"splitting non-manifold edge.....\t";
	splitNMEdge(verlist,verattrlist,edgelist,edge2facelist,nmedgelist,normedgelist,facelist,ver2edgehash);
	cout<<"done!"<<endl;

	/*split edges with two convex vertices*/
	//	splitNCtrEdgeTwoCtrVer( verlist, verattrlist,vermark, edgelist, edge2facelist,nmedgelist, normedgelist,facelist,ver2edgehash, oldedgenum);
	//
	//	//-------------------///
	//	/*for( int i  = 10; i < 25; i ++)
	//	{
	//		cout<<"edge "<<normedgelist[i]<<"\n";
	//		for( int j = 0; j < 2; j ++)
	//		{
	//			int facei = edge2facelist[ normedgelist[ i ] ][ 2*j ];
	//			cout<<facei<<"\t"<<edge2facelist[ normedgelist[ i ] ][ 2*j + 1]<<"\t";
	//			for( int k = 0; k < 5; k ++)
	//			{
	//				cout<<facelist[ 5*facei + k ]<<"\t";
	//			}
	//			cout<<endl;
	//		}
	//	}*/
	//
	/*swap edges*/
	HashMap ver2edgehash2;
	for(unsigned int i = 0; i < edgelist.size()/2; i ++)
	{
		if( ver2edgehash2.findInsertSort( edgelist[ i * 2 ], edgelist[ i * 2 + 1], i ) != i)
			cout<<"Error occurs when building hash map for edges!"<<endl;
	}
	cout<<"swap edges ....."<<"\t";

	swapEdge(verlist,edgelist,edge2facelist,normedgelist,facelist,ver2edgehash2);
	cout<<"done!"<<endl;

	/*edge type*/
	//	getEdgeTypeList(normedgelist,nmedgelist,ctredgelist,edgetypelist);

	/*insert new vertex and swap*/
	//kw: test commented
	cout<<"split triangles ....."<<"\t";
	int count = 0;
	while( count <= MAXTIMES )
		//while( count < 1)
	{
		count++;
		if(splitTriangle(verlist, verattrlist,ver2edgehash2,edgelist, normedgelist,edge2facelist,facelist, alpha))
		{
			swapEdge(verlist,edgelist,edge2facelist,normedgelist,facelist,ver2edgehash2);
		}
		else
			break;
	}
	cout<<"done!"<<endl;
	//kw: test commented

	//	//////////////////////////////////////////////////////////////////////////
	//	/*dbnmedgelist.clear();
	//	for( int i =  0;  i < normedgelist.size(); i ++)
	//	{
	//		if( edge2facelist[ normedgelist[ i ]].size() < 4 )
	//		{
	//			dbnmedgelist.push_back( edgelist[normedgelist[ i ]*2]);
	//			dbnmedgelist.push_back( edgelist[normedgelist[ i ]*2 + 1]);
	//		}
	//	}*/
	//
	/*put the new mesh into array*/
	//vertex ctredge face facemat
	cout<<"refresh mesh...."<<"\t";
	sufvernum = verlist.size()/3;
	delete []sufver;
	sufver = new float[ sufvernum*3];
	for( int i = 0; i < sufvernum*3; i++)
		sufver[ i ] = verlist[i];
	suffacenum = facelist.size()/5;
	delete []sufface;
	delete []sufmat;
	delete []suffacenorm;
	sufface = new int[ suffacenum * 3];
	suffacenorm = new float[suffacenum*3];
	sufmat = new int[ suffacenum * 2];
	float vec1[3],vec2[3];
	for( int i = 0; i < suffacenum; i ++)
	{
		for( int j = 0; j < 3; j ++)
		{
			sufface[ i*3 + j] = facelist[ i * 5 + j];
		}
		for( int j = 0; j < 2; j ++)
		{
			sufmat[ i * 2 + j ] = facelist[i * 5 + 3 + j];
		}
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}

	/*	sufctredgenum = ctredgelist.size();
	delete []sufctredge;
	sufctredge = new int[ sufctredgenum*2 ];
	for( int i = 0; i < sufctredgenum; i ++)
	{
	sufctredge[ i * 2 ] = edgelist[ ctredgelist[ i ] * 2 ];
	sufctredge[ i * 2 + 1] = edgelist[ ctredgelist[ i ] * 2 + 1];
	} */
	cout<<"done!"<<endl;  

	//debug
	//	dbnmedgelist.clear();
	//	dbnmedgelist.resize( nmedgelist.size());
	//for( int i = 0; i < nmedgelist.size(); i++)
	//{
	//	dbnmedgelist.push_back(edgelist[nmedgelist[i]*2]);
	//	dbnmedgelist.push_back(edgelist[nmedgelist[i]*2+1]);
	//}

	/*release the memory*/
	for(unsigned int i = 0; i < edge2facelist.size(); i ++)
		edge2facelist[i].clear();
	edge2facelist.clear();
	edgelist.clear();
	ctredgelist.clear();
	nmedgelist.clear();
	normedgelist.clear();
	edgelenlist.clear();
	verlist.clear();
	verattrlist.clear();
	vermark.clear();
}
void Mesh::gatherInfoForFair(vector<int>& vermark, vector<intvector>& verneighbr, HashMap& ver2edgehash, 
							 vector<int>&edgelist)
{
	//vector<int> edgelist;
	vector<intvector> edge2facelist;
	//edgelist and edge2facelist and ver2edgehash
	int edgelen = 0;
	int edgeindex;
	for( int i = 0;  i < suffacenum; i++)
	{
		for( int j = 0; j < 3; j ++)
		{
			edgeindex = ver2edgehash.findInsertSort(sufface[ i*3 + j ], sufface[ i*3 + (j+1)%3 ], edgelen);
			if(edgeindex == edgelen)
			{
				edgelen ++;
				edgelist.push_back(sufface[ i*3 + j ]);
				edgelist.push_back(sufface[ i*3 + (j+1)%3 ]);
				intvector facelist;
				facelist.push_back( i );	//i face
				edge2facelist.push_back( facelist );	//in c++, copy and push back
				facelist.clear();		//clear temp variable, avoid memory leakage
			}
			else
			{
				edge2facelist[ edgeindex ].push_back( i );	//face i
			}
		}
	}

	//edgemark
	int* edgemark = new int[ edgelen ];		// 0 - normal edge 1 - contour edge 2 - nonmanifold edge
	for( int i= 0; i < edgelen; i++)
		edgemark[ i ] = 0;
	for( int i = 0; i < sufctredgenum; i++)
	{
		edgeindex = ver2edgehash.findInsertSort( sufctredge[ i * 2], sufctredge[i * 2 +1], edgelen);
		//cout<<"edgeindex:"<<edgeindex<<endl;
		if( edgeindex == edgelen )
		{
			cout<<sufctredge[ i * 2]<<"\t"<<sufctredge[i * 2 +1]<<endl;
			cout<<"not valid input!"<<endl;
			continue;
		}
		edgemark[edgeindex] = 1;
	}

	//debug
	//	dbnmedgelist.clear();
	for( int i = 0; i < edgelen; i ++)
	{
		if( edge2facelist[ i ].size() > 2)
		{
			edgemark[ i ] = 2;

			//debug
			//			dbnmedgelist.push_back( edgelist[i*2]);
			//			dbnmedgelist.push_back( edgelist[i*2+1]);
		}
	}
	vermark.resize( sufvernum );
	for( int i = 0; i < sufvernum; i ++)	//0-normal vertex 1 - contour vertex 2 - non-manifold vertex 3 - contour and non-manifold
	{
		vermark[ i ] = 0;
	}
	for( int i = 0; i < edgelen; i ++)
	{
		if(edgemark[ i ] == 1)	//contour edge
		{
			if( vermark[edgelist[ i * 2 ]] == 2 )
				vermark[edgelist[ i * 2 ]] = 3;
			else
				vermark[edgelist[ i * 2 ]] = 1;
			if(vermark[ edgelist[i*2 + 1]] == 2)
				vermark[edgelist[ i * 2 + 1 ]] = 3;
			else
				vermark[ edgelist[i*2 + 1]] = 1;
		}
		else if(edgemark[ i ] == 2)	//non-manifold edge
		{
			if( vermark[edgelist[ i * 2 ]] == 1 )
				vermark[edgelist[ i * 2 ]] = 3;
			else
				vermark[edgelist[ i * 2 ]] = 2;
			if(vermark[ edgelist[i*2 + 1]] == 1)
				vermark[edgelist[ i * 2 + 1 ]] = 3;
			else
				vermark[ edgelist[i*2 + 1]] = 2;
		}
	}

	//verneighbr
	verneighbr.resize( sufvernum );
	int veri[2];
	for( int i = 0; i < edgelen; i ++)
	{
		veri[ 0 ] = edgelist[ 2*i ];
		veri[ 1 ] = edgelist[ 2*i + 1];
		for( int j = 0; j < 2; j ++)
		{
			if( vermark[veri[ j ]] == 2 || vermark[ veri[j]] == 3) //non-manifold vertex
			{
				if ( edgemark[ i ] == 2 ) //non-manifold edge
					verneighbr[ veri[ j ]].push_back( veri[ 1 - j ]);
			}
			else	//either contour vertex or normal vertex
			{
				verneighbr[ veri[ j ]].push_back( veri[ 1 - j ]);
			}
			//if( vermark[veri[ j ]] == 2 ) //non-manifold vertex
			//{
			//	if ( vermark[veri[ 1 - j ]] == 2 ) //non-manifold vertex too!
			//		verneighbr[ veri[ j ]].push_back( veri[ 1 - j ]);
			//}
			//else	if( vermark[veri[ j ]] == 0 ) //either contour vertex or normal vertex
			//{
			//	if ((vermark[veri[ 1 - j ]] == 2) || (vermark[ veri[1-j]] == 0)) //non-manifold vertex or normal vertex
			//		verneighbr[ veri[ j ]].push_back( veri[ 1 - j ]);
			//}
		}		
	}
	delete []edgemark;
	//edgelist.clear();
	for(unsigned int i = 0; i < edge2facelist.size(); i ++)
		edge2facelist[ i ].clear();
	edge2facelist.clear();
}
void inline Mesh::computeWeightList(vector<int>&edgelist, vector<float>& wlist)
{
	//inverseedgelenslist
	int edgelen = edgelist.size()/2;
	////wlist.resize(edgelen);
	//float temp;
	//for( int i = 0; i < edgelen; i ++)
	//{
	//	temp = MyMath::vectorlen(&sufver[ 3 * edgelist[ i * 2 ]], &sufver[ 3* edgelist[ i * 2 + 1]]);;
	//	if( temp == 0)
	//	{
	//		wlist[ i ] = 1000000;
	//		cout<<"Edge length is 0 when computing the weight!"<<endl;
	//	}
	//	else
	//		wlist[ i ] = 1/temp;
	//}
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < edgelen; i ++)
	{
		wlist[ i ] = 1;
	}
}
void inline Mesh::computeDiffer(float* oldval, float* differ, vector<float>& wlist, vector<int>&vermark,
								vector<intvector>&verneighbr, HashMap& ver2edgehash)
{
	memset( differ, 0, sizeof(float)*3*sufvernum);
	//only compute difference for non-manifold and normal vertex
	for( int i = 0 ; i < sufvernum ; i++)
	{
		float wsum = 0;
		for(unsigned int j = 0; j < verneighbr[i].size(); j++)	//all neighbors
		{
			int ver2 = verneighbr[ i ][ j ];
			if( (vermark[ver2] == 1) || (vermark[ver2] == 3))	//contour vertex 
				continue;
			int edgei = ver2edgehash.findInsertSort( i, ver2, -1);
			if( edgei == -1 )
			{
				cout<<"Error in SDUmbraFair when finding current edge!"<<endl;
				continue;
			}
			float curw = wlist[ edgei ];
			wsum += curw;
			for( int k = 0; k < 3; k ++)	//for three coord
			{
				differ[ 3 * i + k ] += (curw * oldval[ 3 * ver2+ k]);
			}
		}
		for( int k = 0; k < 3; k ++)
			differ[ 3 * i + k ] = differ[ 3 * i + k ]/wsum - oldval[ 3 * i + k ];
	}
}
void inline Mesh::computeDiffer(float* oldval, float* differ, vector<float>& wlist, 
								vector<intvector>&verneighbr, HashMap& ver2edgehash)
{
	memset( differ, 0, sizeof(float)*3*sufvernum);
	for( int i = 0 ; i < sufvernum ; i++)
	{
		//	for( int k = 0; k < 3; k ++)
		//		differ[ 3*i + k ] = 0;
		float wsum = 0;
		for(unsigned int j = 0; j < verneighbr[i].size(); j++)	//all neighbors
		{
			int ver2 = verneighbr[ i ][ j ];
			int edgei = ver2edgehash.findInsertSort( i, ver2, -1);
			if( edgei == -1 )
			{
				cout<<"Error in SDUmbraFair when finding current edge!"<<endl;
				continue;
			}
			float curw = wlist[ edgei ];
			wsum += curw;
			for( int k = 0; k < 3; k ++)	//for three coord
			{
				differ[ 3 * i + k ] += (curw * oldval[ 3 * ver2+ k]);
			}
		}
		for( int k = 0; k < 3; k ++)
			differ[ 3 * i + k ] = differ[ 3 * i + k ]/wsum - oldval[ 3 * i + k ];
	}
}
void inline Mesh::computeDiffer(vector<float>& oldval, vector<float>& differ, vector<float>& wlist, 
								vector<intvector>&verneighbr, HashMap& ver2edgehash)
{
	for( int i = 0 ; i < sufvernum ; i++)
	{
		for( int k = 0; k < 3; k ++)
			differ[ 3*i + k ] = 0;
		float wsum = 0;
		for(unsigned int j = 0; j < verneighbr.size(); j++)	//all neighbors
		{
			int ver2 = verneighbr[ i ][ j ];
			int edgei = ver2edgehash.findInsertSort( i, ver2, -1);
			if( edgei == -1 )
			{
				cout<<"Error in SDUmbraFair when finding current edge!"<<endl;
				continue;
			}
			float curw = wlist[ edgei ];
			wsum += curw;
			for( int k = 0; k < 3; k ++)	//for three coord
			{
				differ[ 3 * i + k ] += (curw * oldval[ 3 * ver2+ k]);
			}
		}
		for( int k = 0; k < 3; k ++)
			differ[ 3 * i + k ] = differ[ 3 * i + k ]/wsum - oldval[ 3 * i + k ];

	}
}
void inline Mesh::refreshByDiffer(float* oldval, float* newval, float* differ, float ratio)
{
	for( int i = 0; i < sufvernum*3; i ++)
	{
		newval[i] = oldval[i] + differ[ i ]*ratio;
	}
}
void inline Mesh::computeRefreshWeightList(vector<intvector>&verneighbr, vector<float>& w2list)
{
	// 1/(1 + 1/ni(signma /1nij))
	int* valence = new int[ sufvernum ];
	for( int i = 0; i < sufvernum; i ++)
		valence[ i ] = verneighbr[ i ].size();
	//////////////////////////////////////////////////////////////////////////
	//	for( int i = 0; i < sufvernum; i++)
	//		cout<<valence[i]<<" ";
	//	cout<<endl;
	//////////////////////////////////////////////////////////////////////////
	for( int i = 0; i < sufvernum; i ++)
	{
		w2list[ i ] = 0;
		for(unsigned int j = 0; j < verneighbr[ i ].size(); j++)
		{
			w2list[ i ] += (1/(float)valence[verneighbr[i][j]]);
		}
		w2list[ i ] /= valence[ i ];
		w2list[ i ] += 1;
		w2list[ i ] = 1/w2list[ i ];
		if( w2list[ i ] > 0.4)
			w2list[ i ] = 0.4;
		//	cout<<w2list[i]<<" ";
	}
	//	cout<<endl;
}
void inline Mesh::SDUmbraFair(float* oldver, float* newver,  float* oldfirstdiffer, float* newfirstdiffer, 
							  float* seconddiffer, vector<int>&vermark, vector<intvector>& verneighbr,
							  HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist)
{
	//	vector<float> wlist;
	computeWeightList(edgelist, wlist);
	//compute oldfirstdiffer
	computeDiffer(oldver, oldfirstdiffer, wlist, verneighbr, ver2edgehash);
	//compute second differ
	computeDiffer(oldfirstdiffer, seconddiffer, wlist, verneighbr, ver2edgehash);
	//	computeDiffer(oldfirstdiffer, seconddiffer, wlist, vermark, verneighbr, ver2edgehash);

	//weight list for refresh
	vector<float> w2list;
	w2list.resize(sufvernum);
	computeRefreshWeightList(verneighbr, w2list);

	//////////////////////////////////////////////////////////////////////////
	//for the selected vertices
	//(1) first difference, (2) second difference (3) weight (4)
	int curver;
	for(unsigned int i = 0; i < selectVerList.size(); i ++)
	{
		cout<<"-------------------------------------------------"<<endl;
		curver = selectVerList[ i ];
		//weight
		cout <<"weight:"<< w2list[ curver ] <<endl;
		//difference
		cout<<"difference:"<<endl;
		cout<<"ver:"<<endl;
		cout<<"first:("<<oldfirstdiffer[ curver*3 ]<<","<<oldfirstdiffer[curver*3+1]<<","<<oldfirstdiffer[curver*3+2]<<")\t";
		cout<<"second:("<<seconddiffer[curver*3]<<","<<seconddiffer[curver*3+1]<<","<<seconddiffer[curver*3+2]<<")\n";
		cout<<"neighbr:"<<endl;
		for(unsigned int j = 0; j < verneighbr[ curver ].size(); j++)
		{
			cout<<"first:("<<oldfirstdiffer[ verneighbr[curver][ j ] * 3 ]<<","<<oldfirstdiffer[ verneighbr[curver][ j ] * 3 + 1 ]<<","<<oldfirstdiffer[ verneighbr[curver][ j ] * 3 + 2 ]<<")\t";
			cout<<"second:("<<seconddiffer[ verneighbr[curver][ j ] * 3 ]<<","<<seconddiffer[ verneighbr[curver][ j ] * 3 + 1 ]<<","<<seconddiffer[ verneighbr[curver][ j ] * 3 + 2 ]<<")\n";
		}		
		cout<<"-------------------------------------------------"<<endl;
	}

	//compute newfirstdiffer
	/*	for( int i = 0; i < sufvernum*3; i ++)
	{
	newfirstdiffer[i] = oldfirstdiffer[i] + seconddiffer[ i ]*ratio2;
	}*/
	//	refreshByDiffer(oldfirstdiffer, newfirstdiffer, seconddiffer, ratio2);
	//compute newver
	/*------------------------------------*/

	for( int i = 0; i < sufvernum; i ++)
	{
		if( vermark[ i ] == 1 || vermark[ i ] == 3)
		{
			for( int j = 0; j< 3; j++)
				newver[ 3*i + j] = oldver[ 3*i + j];
			continue;
		}
		for( int j = 0; j< 3; j++)
			newver[ 3*i + j] = oldver[ 3*i + j] - seconddiffer[ 3*i + j ]*w2list[ i ];
			//by kw
			//newver[ 3*i + j] = oldver[ 3*i + j] + seconddiffer[ 3*i + j ]*w2list[ i ];

		//if( vermark[ i ] == 1 || vermark[ i ] == 3 || vermark[ i ]==2)
		/*	if( vermark[ i ] == 0 ||vermark[ i ] == 1 || vermark[ i ] == 3 )
		{
		for( int j = 0; j< 3; j++)
		newver[ 3*i + j] = oldver[ 3*i + j];
		continue;
		}
		for( int j = 0; j< 3; j++)	//only refresh the vertices on nonmanifold edge
		newver[ 3*i + j] = oldver[ 3*i + j] - seconddiffer[ 3*i + j ]*w2list[ i ];*/
	}
	/*------------------------------------*/
	/*	for( int i = 0; i < sufvernum; i ++)
	{
	if( vermark[ i ] == 1 || vermark[ i ] == 3)
	{
	for( int j = 0; j< 3; j++)
	newver[ 3*i + j] = oldver[ 3*i + j];
	continue;
	}
	for( int j = 0; j< 3; j++)
	newver[ 3*i + j] = oldver[ 3*i + j] + newfirstdiffer[ 3*i + j ]*ratio1;
	}*/	
	//	refreshByDiffer(oldver, newver, newfirstdiffer, ratio1);

}

void Mesh::SDUmbraFair(int times)
{
	cout<<"smoothing....\t";
	//initialization
	float* oldver;
	float* newver;
	float* oldfirstdiffer;
	float* newfirstdiffer;
	float* seconddiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	oldfirstdiffer = new float[sufvernum*3];
	newfirstdiffer = new float[sufvernum*3];
	seconddiffer = new float[sufvernum*3];
	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		temp = oldver;
		oldver = newver;
		newver = temp;
		temp = NULL;
		SDUmbraFair(oldver, newver, oldfirstdiffer, newfirstdiffer,seconddiffer, 
			vermark, verneighbr, ver2edgehash, edgelist,wlist);
	}	
	cout<<"done!"<<endl;

	//debug
	/*for( int i = 0; i < verneighbors.size(); i ++)
	verneighbors[i].clear();
	verneighbors.clear();
	verneighbors.resize( verneighbr.size() );
	for( int i = 0; i < verneighbr.size(); i ++)
	{
	verneighbors[ i ] = verneighbr[ i ];
	}
	curdebugpt = -1;
	movedebugpt();*/
	debugpts.clear();
	if( selectVerList.size() > 0 ){
		cout<<"current ver:"<<selectVerList[ 0 ]<<"neighbors number:"<<verneighbr[ selectVerList[ 0 ]].size()<<endl;
		for(unsigned int i = 0;  i < verneighbr[selectVerList[0]].size(); i++)
		{
			debugpts.push_back( verneighbr[ selectVerList[ 0 ]][ i ]);
			cout<<verneighbr[ selectVerList[ 0 ]][ i ]<<"  " ;
		}
		cout<<endl;
	}

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []oldfirstdiffer;
	delete []newfirstdiffer;
	delete []seconddiffer;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}
}
void Mesh::splitsmooth(float alpha0, float alphan, int times, int stimes)
{
	float delta = (alphan - alpha0)/times;
	float alpha = alpha0;
	for( int i = 0; i < times; i ++)
	{
		SDUmbraFair(stimes);
		LiepaRefine(alphan);//alpha
		//SDUmbraFair(stimes);
		alpha += delta;
	}
}
void Mesh::splitsmooth2(float alpha0, float alphan, int times, int stimes, float ratio)
{
	cout<<"in splitsmooth2! Great!"<<"alpha0"<<alpha0<<"alphan"<<alphan<<"times"<<times<<"stimes"<<stimes<<"ratio"<<ratio<<endl;
	float delta = (alphan - alpha0)/times;
	float alpha = alpha0;

	//JUFair( ratio, times);//times

	for( int i = 0; i < times; i ++)
	{
		JUFair( ratio, times);//times
		LiepaRefine(alpha);
		//JUFair( ratio, times);//times
		alpha += delta;
	}
}

void Mesh::splitsmooth3(float alpha0, float alphan, int times, int stimes, float ratio)
{
	times=10;
	//ratio=0.8;

	float delta = (alphan - alpha0)/times;
	float alpha = alpha0;
	for( int i = 0; i < times; i ++)
	{
		//LaplacianSmooth(ratio,stimes);//ratio times
		//TaubinSmooth(0.5,-0.53,40);
		LiepaRefine(alpha);//alpha
		//AverageSmooth(ratio,stimes);//stimes
		//LaplacianSmooth(ratio,40);//ratio times
		//TaubinSmooth(0.5,-0.53,40);
		alpha += delta;
	}

	//TaubinSmooth(0.3,-0.8,40);//0.5,-0.53

	//ratio=0.1;
	//FBFair( ratio,10);//times
}

void inline Mesh::computeNorm(float* oldval, float* norms, float* mags )
{
	memset( norms, 0, sizeof(float)*3*sufvernum);

	for( int i = 0 ; i < suffacenum ; i++)
	{
		float vec1[3], vec2[3], nm[3];
		MyMath::getVec(&(oldval[ sufface[ i * 3] * 3 ]), &(oldval[ sufface[ i * 3 + 1] * 3 ]), vec1 );
		MyMath::getVec(&(oldval[ sufface[ i * 3] * 3 ]), &(oldval[ sufface[ i * 3 + 2] * 3 ]), vec2 );
		MyMath::crossProductNotNorm( vec1, vec2, nm);

		if ( sufmat[i*2] > sufmat[i*2 + 1] )
		{
			nm[0] = -nm[0] ;
			nm[1] = -nm[1] ;
			nm[2] = -nm[2] ;
		}

		for ( int j = 0 ; j < 3 ; j ++ )
		{
			int ind = sufface[ i * 3 + j ] ;
			// for three vertices of the triangle
			// if ( vermark[ ind ] < 2 )
			{
				for( int k = 0; k < 3; k ++)	// for three coord
				{
					norms[ 3 * ind + k ] += nm[ k ] ;
				}
			}
		}
		// printf("%d %d\n", sufmat[ i * 2 ], sufmat[ i * 2 + 1 ] );
	}
	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		mags[ i ] = sqrt( MyMath::vectorlen( &(norms[ 3 * i ]) ) );
		// printf("%f\n", mags[i]) ;
	}

}

void inline Mesh::JUFairCenter(float ratio, float* oldver, float* newver,  float* oldfirstdiffer, float* newfirstdiffer, 
							   float* seconddiffer, vector<int>&vermark, vector<intvector>& verneighbr,
							   HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist,
							   float* norms, float* mags)
{
	float* dis = new float[ 3 * sufvernum ] ;
	float* vecs = new float[ 3 * sufvernum ] ;

	//	vector<float> wlist;
	computeWeightList(edgelist, wlist);
	//kw: compute oldfirstdiffer (uniform Laplacian, pointing from the vertex to the center of neighbors)
	computeDiffer(oldver, oldfirstdiffer, wlist, verneighbr, ver2edgehash);

	//compute local coordinates 
	//(normal at each point,average of the adjacent facet normals, 
	//kw: norms: normal vector(sum of incident triangle normals,not normalized), mags: magnitude of normal vector
	computeNorm( oldver, norms, mags );

	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		if ( vermark[i] < 2 )
		{
			// manifold points
			float s = mags[ i ] ;
			float m = s * s ;
			dis[3 * i] = MyMath::dotProduct( &(oldfirstdiffer[ 3 * i ]), &(norms[ 3 * i ]) ) / ( m * s ) ;
			for ( int k = 0 ; k < 3 ; k ++ )
			{
				//kw: normalize the normal
				norms[ 3 * i + k ] *= ( s / m ) ;
				//kw: tangential component vector=uniform laplacian-normal
				vecs[ 3 * i + k ] = oldfirstdiffer[ 3 * i + k ] - dis[3 * i] * norms[ 3 * i + k ] ;
			}
		}
		else
		{
			// non-manifold
			//dis[3*i] = 0 ;
			dis[ 3*i ] = oldfirstdiffer[ 3* i];
			dis[ 3*i + 1 ] = oldfirstdiffer[ 3*i + 1];
			dis[ 3*i + 2 ] = oldfirstdiffer[ 3*i + 2];
		}
	}

	//averaging orthogonal distances
	computeDiffer(dis, seconddiffer, wlist, verneighbr, ver2edgehash);

	vector<float> w2list;
	w2list.resize(sufvernum);
	computeRefreshWeightList(verneighbr, w2list);

	// finally, compute orthogonal and tangential movements
	//kw: ratio==0.5
	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		if( interpolate )
		{
			if( vermark[ i ] == 1 || vermark[ i ] == 3)
			{
				// Contour points: don't move
				for( int j = 0; j< 3; j++)
					newver[ 3*i + j] = oldver[ 3*i + j];
				continue;
			}
		}
		if ( vermark[ i ] == 2 || vermark[ i ] == 3 )
		{
			// Non-manifold points: Laplacian
			for ( int k = 0 ; k < 3 ; k ++ )
			{
				newver[ 3*i + k] = oldver[ 3*i + k] - seconddiffer[ 3*i + k ]*w2list[ i ];
			//	newver[ 3 * i + k ] = oldver[ 3 * i + k ] + ratio * oldfirstdiffer[ 3 * i + k ] ;
			}
		}
		else
		{
			// Manifold points: new averaging
			for ( int k = 0 ; k < 3 ; k ++ )
			{
				newver[ 3 * i + k ] = oldver[ 3 * i + k ] - ratio * seconddiffer[ 3 * i ] * norms[ 3 * i + k ] + ratio * vecs[ 3 * i + k ] ;
			}
		}
	}

	delete vecs ;
	delete dis ;
	w2list.clear();
}

void Mesh::JUFair(float ratio, int times)
{
	cout<<"In JuFair! GOOD!"<<"ratio:"<<ratio<<"times:"<<times<<endl;
	cout<<"smoothing....\t";
	//initialization
	float* oldver;
	float* newver;
	float* oldfirstdiffer;
	float* newfirstdiffer;
	float* seconddiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	oldfirstdiffer = new float[sufvernum*3];
	newfirstdiffer = new float[sufvernum*3];
	seconddiffer = new float[sufvernum*3];

	/* Added by tao */
	float* mags = new float[ sufvernum ] ;
	float* norms = new float[ sufvernum * 3 ] ;
	/* end adding */


	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		temp = oldver;
		oldver = newver;
		newver = temp;
		temp = NULL;
		JUFairCenter(ratio, oldver, newver, oldfirstdiffer, newfirstdiffer,seconddiffer, 
			vermark, verneighbr, ver2edgehash, edgelist,wlist,norms,mags);
	}	
	cout<<"done!"<<endl;

	//debug
	/*for( int i = 0; i < verneighbors.size(); i ++)
	verneighbors[i].clear();
	verneighbors.clear();
	verneighbors.resize( verneighbr.size() );
	for( int i = 0; i < verneighbr.size(); i ++)
	{
	verneighbors[ i ] = verneighbr[ i ];
	}
	curdebugpt = -1;
	movedebugpt();*/
	debugpts.clear();
	if( selectVerList.size() > 0 ){
		cout<<"current ver:"<<selectVerList[ 0 ]<<"neighbors number:"<<verneighbr[ selectVerList[ 0 ]].size()<<endl;
		for(unsigned int i = 0;  i < verneighbr[selectVerList[0]].size(); i++)
		{
			debugpts.push_back( verneighbr[ selectVerList[ 0 ]][ i ]);
			cout<<verneighbr[ selectVerList[ 0 ]][ i ]<<"  " ;
		}
		cout<<endl;
	}

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []oldfirstdiffer;
	delete []newfirstdiffer;
	delete []seconddiffer;
	delete []mags;
	delete []norms;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}

	/*
	//initialization
	float* oldver;
	float* newver;
	float* vdir;	//vertical direction for each vertex
	float* hdir;	//horizontal direction for each vertex
	float* oldvdis;	//vertical distance from the vertex to the center of the 1-ring neighbors
	float* oldhdis;	//horizontal distance from the vertex to the center of the 1-ring neighbors.
	float* newvdis;
	float* newhdis;
	int* vermark;
	oldver = new float[ sufvernum * 3 ];
	newver = new float[ sufvernum * 3 ];
	vermark = new int[ sufvernum ];
	vdir = new float[ sufvernum * 3 ];
	hdir = new float[ sufvernum * 3] ;
	oldvdis = new float[ sufvernum ];
	oldhdis = new float[ sufvernum ];
	newvdis = new float[ sufvernum ];
	newhdis = new float[ sufvernum ];

	//
	delete []oldver;
	delete []newver;
	delete []vdir;
	delete []hdir;
	delete []oldvdis;
	delete []newvdis;
	delete []newhdis;
	delete []vermark;
	*/

}

void Mesh::FBFair(float ratio, int times)
{
	//initialization
	float* oldver;
	float* newver;
	float* oldfirstdiffer;
	float* newfirstdiffer;
	float* seconddiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	oldfirstdiffer = new float[sufvernum*3];
	newfirstdiffer = new float[sufvernum*3];
	seconddiffer = new float[sufvernum*3];

	float* mags = new float[ sufvernum ] ;
	float* norms = new float[ sufvernum * 3 ] ;


	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		temp = oldver;
		oldver = newver;
		newver = temp;
		temp = NULL;
		FBFairCenter(ratio, oldver, newver, oldfirstdiffer, newfirstdiffer,seconddiffer, 
			vermark, verneighbr, ver2edgehash, edgelist,wlist,norms,mags);
	}	
	cout<<"done!"<<endl;

	debugpts.clear();
	if( selectVerList.size() > 0 ){
		cout<<"current ver:"<<selectVerList[ 0 ]<<"neighbors number:"<<verneighbr[ selectVerList[ 0 ]].size()<<endl;
		for(unsigned int i = 0;  i < verneighbr[selectVerList[0]].size(); i++)
		{
			debugpts.push_back( verneighbr[ selectVerList[ 0 ]][ i ]);
			cout<<verneighbr[ selectVerList[ 0 ]][ i ]<<"  " ;
		}
		cout<<endl;
	}

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []oldfirstdiffer;
	delete []newfirstdiffer;
	delete []seconddiffer;
	delete []mags;
	delete []norms;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}
}

void Mesh::FBFairCenter(float ratio, float* oldver, float* newver, float* oldfirstdiffer, float* newfirstdiffer, float* seconddiffer, 
						vector<int>&vermark, vector<intvector>& verneighbr,HashMap& ver2edgehash, vector<int>&edgelist, vector<float>&wlist, float* norms, float* mags)
{
	computeWeightList(edgelist, wlist);
	//kw: compute oldfirstdiffer (uniform Laplacian, pointing from the vertex to the center of neighbors)
	computeDiffer(oldver, oldfirstdiffer, wlist, verneighbr, ver2edgehash);
	float* fLapMags=new float[sufvernum];
	//compute the magnitude of laplacian
	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		fLapMags[ i ] = sqrt( MyMath::vectorlen( &(oldfirstdiffer[ 3 * i ]) ) );
	}
	//average magnitude of neighbor Laplacians
	float* fAveLapMags=new float[sufvernum];
	memset(fAveLapMags, 0, sizeof(float)*sufvernum);
	for (int i=0;i<sufvernum;i++)
	{
		float wsum=0;
		for(unsigned int j=0;j<verneighbr[i].size();j++)	//all neighbors
		{
			int ver2 = verneighbr[ i ][ j ];
			int edgei = ver2edgehash.findInsertSort( i, ver2, -1);
			if( edgei == -1 )
			{
				cout<<"Error in SDUmbraFair when finding current edge!"<<endl;
				continue;
			}
			float curw = wlist[ edgei ];
			wsum += curw;
			fAveLapMags[i]=fAveLapMags[i]+fLapMags[ver2];
		}
		fAveLapMags[i]=fAveLapMags[i]/wsum;
	}

	//compute local coordinates 
	//(normal at each point,average of the adjacent facet normals, 
	//kw: norms: normal vector(sum of incident triangle normals,not normalized), mags: magnitude of normal vector
	computeNorm( oldver, norms, mags );
	//normalize normal vector
	for(int i=0;i<sufvernum;i++)
	{
		if (vermark[i]<2)// manifold points
		{
			for (int k=0;k<3;k++)
			{
				norms[3*i+k]=norms[3*i+k]/mags[i];
			}
		}
		else
		{
			cout<<"error! non-manifold vertex"<<endl;
		}
	}
	// finally, move along the normal taking averages Laplacian magnitude as distance
	//kw: ratio==0.5
	for ( int i = 0 ; i < sufvernum ; i ++ )
	{
		if( interpolate )
		{
			if( vermark[ i ] == 1 || vermark[ i ] == 3)
			{
				// Contour points: don't move
				for( int j = 0; j< 3; j++)
					newver[ 3*i + j] = oldver[ 3*i + j];
				continue;
			}
		}
		if ( vermark[ i ] == 2 || vermark[ i ] == 3 )
		{
			cout<<"error! non-manifold vertex"<<endl;
		}
		else
		{
			// Manifold points: new averaging
			for ( int k = 0 ; k < 3 ; k ++ )
			{
				newver[ 3 * i + k ] = oldver[ 3 * i + k ] - ratio*fAveLapMags[i]* norms[ 3 * i + k ];
			}
		}
	}

	delete [] fLapMags;
	delete [] fAveLapMags;
}

void Mesh::LaplacianSmooth(float ratio, int times)
{
	//initialization
	float* oldver;
	float* newver;
	float* firstdiffer;
	//	float* oldfirstdiffer;
	//	float* newfirstdiffer;
	//	float* seconddiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	firstdiffer = new float[sufvernum*3];
	//	oldfirstdiffer = new float[sufvernum*3];
	//	newfirstdiffer = new float[sufvernum*3];
	//	seconddiffer = new float[sufvernum*3];
	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		temp = oldver;
		oldver = newver;
		newver = temp;
		temp = NULL;

		computeWeightList(edgelist, wlist);
		//compute firstdiffer
		computeDiffer(oldver, firstdiffer, wlist, verneighbr, ver2edgehash);
		//compute newver
		//	vector<float> w2list;
		//	w2list.resize(sufvernum);
		//	computeRefreshWeightList(verneighbr, w2list);
		for( int i = 0; i < sufvernum; i ++)
		{
			if( interpolate )
			{
				if( vermark[ i ] == 1 || vermark[ i ] == 3)
				{
					for( int j = 0; j< 3; j++)
						newver[ 3*i + j] = oldver[ 3*i + j];
					continue;
				}
			}
			for( int j = 0; j< 3; j++)
				newver[ 3*i + j] = oldver[ 3*i + j] + firstdiffer[ 3*i + j ]*ratio;

			//if( vermark[ i ] == 1 || vermark[ i ] == 3 || vermark[ i ]==2)
			/*	if( vermark[ i ] == 0 ||vermark[ i ] == 1 || vermark[ i ] == 3 )
			{
			for( int j = 0; j< 3; j++)
			newver[ 3*i + j] = oldver[ 3*i + j];
			continue;
			}
			for( int j = 0; j< 3; j++)
			newver[ 3*i + j] = oldver[ 3*i + j] - seconddiffer[ 3*i + j ]*w2list[ i ];*/
		}
		/*	for( int i = 0; i < sufvernum; i ++)
		{
		if( vermark[ i ] == 1 || vermark[ i ] == 3)
		{
		for( int j = 0; j< 3; j++)
		newver[ 3*i + j] = oldver[ 3*i + j];
		continue;
		}
		for( int j = 0; j< 3; j++)
		newver[ 3*i + j] = oldver[ 3*i + j] + newfirstdiffer[ 3*i + j ]*ratio1;
		}*/	
		//	refreshByDiffer(oldver, newver, newfirstdiffer, ratio1);
	}	

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []firstdiffer;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}
}

void Mesh::AverageSmooth(float ratio, int times)
{
	//initialization
	float* oldver;
	float* newver;
	float* firstdiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	firstdiffer = new float[sufvernum*3];
	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}
	float* avefirstdiffer=new float[sufvernum*3];
	memset(avefirstdiffer,0x00,sizeof(float)*sufvernum*3);

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		temp = oldver;
		oldver = newver;
		newver = temp;
		temp = NULL;

		computeWeightList(edgelist, wlist);
		//compute firstdiffer
		computeDiffer(oldver, firstdiffer, wlist, verneighbr, ver2edgehash);

		//average neighbor Laplacians
		for (int i=0;i<sufvernum;i++)
		{
			float wsum=0;
			for(unsigned int j=0;j<verneighbr[i].size();j++)	//all neighbors
			{
				int ver2 = verneighbr[ i ][ j ];
				for (int k=0;k<3;k++)
				{
					avefirstdiffer[3*i+k]=avefirstdiffer[3*i+k]+firstdiffer[3*ver2+k];
				}
			}
			for (int k=0;k<3;k++)
			{
				avefirstdiffer[3*i+k]=avefirstdiffer[3*i+k]/verneighbr[i].size();
			}
		}
		//compute newver
		//	vector<float> w2list;
		//	w2list.resize(sufvernum);
		//	computeRefreshWeightList(verneighbr, w2list);
		for( int i = 0; i < sufvernum; i ++)
		{
			if( interpolate )
			{
				if( vermark[ i ] == 1 || vermark[ i ] == 3)
				{
					for( int j = 0; j< 3; j++)
						newver[ 3*i + j] = oldver[ 3*i + j];
					continue;
				}
			}
			for( int j = 0; j< 3; j++)
				//newver[ 3*i + j] = oldver[ 3*i + j] + firstdiffer[ 3*i + j ]*ratio;
				//kw update
				newver[ 3*i + j] = oldver[ 3*i + j] + avefirstdiffer[ 3*i + j ]*ratio;
		}
	}	

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []firstdiffer;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	delete [] avefirstdiffer; 

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}
}

void Mesh::TaubinSmooth(float fLambda,float fMu,int times)
{
	//initialization
	float* oldver;
	float* newver;
	float* firstdiffer;
	oldver = new float[ sufvernum*3 ];
	newver = new float[ sufvernum*3 ];
	firstdiffer = new float[sufvernum*3];
	for( int i = 0; i < sufvernum*3; i++)
	{	
		newver[ i ] = sufver[ i ];
	}

	//mark all the types of the vertices	0 - normal vertex 1 - contour vertex 2 - manifold vertex 3-both
	//find all the neighbors for each vertex
	vector<int> vermark;
	vector<intvector> verneighbr;
	vector<int> edgelist;
	HashMap ver2edgehash;
	gatherInfoForFair(vermark, verneighbr, ver2edgehash, edgelist);
	vector<float>wlist;
	wlist.resize( edgelist.size()/2);
	for( int i = 0; i < times; i ++)
	{	
		float* temp;
		for (int iIter=0;iIter<2;iIter++)
		{
			temp = oldver;
			oldver = newver;
			newver = temp;
			temp = NULL;

			computeWeightList(edgelist, wlist);
			//compute firstdiffer
			computeDiffer(oldver, firstdiffer, wlist, verneighbr, ver2edgehash);
			//compute newver
			//	vector<float> w2list;
			//	w2list.resize(sufvernum);
			//	computeRefreshWeightList(verneighbr, w2list);
			for( int i = 0; i < sufvernum; i ++)
			{
				if( interpolate )
				{
					if( vermark[ i ] == 1 || vermark[ i ] == 3)
					{
						for( int j = 0; j< 3; j++)
							newver[ 3*i + j] = oldver[ 3*i + j];
						continue;
					}
				}
				for( int j = 0; j< 3; j++)
				{
					if (iIter==0)
					{
						newver[ 3*i + j] = oldver[ 3*i + j] + firstdiffer[ 3*i + j ]*fLambda;
					}
					else if (iIter==1)
					{
						newver[ 3*i + j] = oldver[ 3*i + j] + firstdiffer[ 3*i + j ]*fMu;
					}
				}
			}
		}

	}	

	//deallocate
	for( int i = 0; i < sufvernum*3; i ++)
		sufver[ i ] = newver[ i ];
	delete []oldver;
	delete []newver;
	delete []firstdiffer;
	vermark.clear();
	for(unsigned int i = 0; i < verneighbr.size(); i++)
		verneighbr[ i ].clear();
	verneighbr.clear();
	edgelist.clear();
	wlist.clear();

	//reset face normal
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}
}

void Mesh::loadStd( const char* fname)
{
	FILE* fin = fopen( fname ,"r");
	if( fin == NULL)
	{
//		fl_alert("Not a valid file, open fails!\n");
		return;
	}
	cout<<"------------------------------"<<endl;
	cout<<"Reading standard mesh...."<<endl;
	//read verlen, facelen
//	int tfacenum;
	fscanf(fin, "%d %d\r\n", &stdvernum, &stdfacenum);

	if( stdverlist != NULL)
		delete []stdverlist;
	if( stdfacelist != NULL )
		delete []stdfacelist;
	stdverlist = new float[ stdvernum * 3];
	stdfacelist = new int[ stdfacenum * 3 ];

	//	float min[ 3 ], max[ 3 ] , pos[ 3 ];
	for( int i = 0; i < stdvernum; i++)
	{
		fscanf(fin, "%f %f %f\r\n",&stdverlist[ 3*i ],&stdverlist[ 3*i + 1],&stdverlist[ 3*i + 2]);
		/*memcpy( pos,stdverlist + 3*i, sizeof(float)* 3  );
		if( i == 0 )
		{
		min[ 0 ] = max[ 0 ] = pos[ 0 ];
		min[ 1 ] = max[ 1 ] = pos[ 1 ];
		min[ 2 ] = max[ 2 ] = pos[ 2 ];
		}
		else
		{
		for( int j = 0; j < 3; j ++)
		{
		if ( min[ j ] > pos[ j ])
		min[ j ] = pos[ j ];
		else if ( max[ j ] < pos [ j ] )
		max[ j ] = pos[ j ];
		}
		}*/
	}
	int mat[ 2 ];
	for( int i = 0; i < stdfacenum; i ++)
	{
		fscanf(fin, "%d %d %d %d %d\r\n",&stdfacelist[ 3*i ],&stdfacelist[ 3*i + 1],&stdfacelist[ 3*i + 2], mat, mat+1);
	}

	//	float tcenter[ 3 ];
	//	MyMath::center(min, max, tcenter);
	//	float tunitlen = max[ 0 ] - min[ 0 ] > max[ 1 ] - min[ 1 ]?max[ 0 ] - min[ 0 ]:max[ 1 ] - min[ 1 ];
	//	tunitlen = tunitlen > max[ 2 ] - min[ 2 ]?tunitlen:max[ 2 ] - min[ 2 ];
	//float tunitlen = MyMath::vectorlen(min, max);
	//normalize all the vers
	//	unitlen = MyMath::vectorlen(min, max);
	//	MyMath::center(min, max, center);

	//	printf("minx:%f,%f,%f\n",min[0],min[1],min[2]);
	//	printf("max:%f,%f,%f\n",max[0],max[1],max[2]);
	//	printf("center:%f,%f,%f\n",center[0],center[1],center[2]);
	for( int i = 0; i < stdvernum; i ++)
	{
		MyMath::getrelativepos( &stdverlist[ 3 * i ], center, unitlen);
	}
	fclose(fin);
	cout<<"Reading done!"<<endl;
	cout<<"------------------------------"<<endl;	
}
void Mesh::compareDiff( const char* fname)
{
	cout<<"----------------------------"<<endl;
	cout<<"Start computing difference between the two meshes!"<<endl;
	if( stdvernum == 0 || sufver == NULL)
	{
		cout<<"One of the two meshes to compare does not exist!"<<endl;
		return;
	}

	float* sufverdiff = new float[ sufvernum ];
	float* stdverdiff = new float[ stdvernum ];

	//for each std vertex, compute difference for it
	bool first = true;
	float maxdiff ;
	for( int i = 0; i < sufvernum; i ++)
	{
		//sufverdiff[ i ] = MyMath::computeDiff( &sufver[ 3*i], stdverlist, stdvernum );
		sufverdiff[ i ] = MyMath::computeDiff( &sufver[ 3*i ], stdverlist, stdfacelist, stdfacenum);

		sufverdiff[ i ] /= 3;

		if( first )
		{
			first = false;
			maxdiff = sufverdiff[ i ];
		}
		else
		{
			if( sufverdiff[ i ] > maxdiff )
				maxdiff = sufverdiff[ i ];
		}
	}
	cout<<"Half is done!"<<endl;

	//for each sufver, compute the difference for it
	for( int i = 0; i < stdvernum; i ++)
	{
		//stdverdiff[ i ] = MyMath::computeDiff( &stdverlist[ 3*i ], sufver, sufvernum);
		stdverdiff[ i ] = MyMath::computeDiff( &stdverlist[ 3*i ], sufver, sufface, suffacenum);

		stdverdiff[ i ] /= 3;

		if ( stdverdiff[ i ] > maxdiff )
			maxdiff = stdverdiff[ i ];
	}

	cout<<"Computing is done! Max difference is:"<<maxdiff<<endl;

	//write out all the difference to file
	FILE* fout = fopen( fname,"w" );
	if(fout == NULL )
	{
		cout<<"Unable to open file :"<<fname<<"to write!"<<endl;
		return;
	}

	fprintf(fout,"{%f,", maxdiff);
	//write the two difference out
	fprintf(fout,"{{%f", sufverdiff[ 0 ]);
	for( int i  = 1; i < sufvernum; i ++)
	{
		fprintf(fout, ",%f", sufverdiff[ i ]);
	}
	fprintf(fout, "},{%f", stdverdiff[ 0 ] );
	for( int i = 1; i < stdvernum; i ++)
		fprintf( fout, ",%f", stdverdiff[ i ]);
	fprintf( fout, "}}}");
	fclose( fout );
	cout<<"Writing to file is done!"<<endl;
	cout<<"------------------------------------"<<endl;
}
Mesh::Mesh( floatvector mver, intvector mface, intvector ctrmedge, 
	 const float center2[ 3 ], const float unitlen2, const float PROCSIZE )
{
	//color
	for( int i = 0; i< 18; i++)
	{
		for( int j = 0; j < 3; j++)
			fcols[i][j] = cols[i][j]/255;
	}

	//data
	memcpy( center, center2, sizeof( float ) * 3);
	//modified by kw
	//unitlen = unitlen2/1.5;
	unitlen = unitlen2/DIM;

	ctrplanenum = 0;
	ctrplaneparam = NULL;
	ctrver = NULL;
	ctrvernum = NULL;
	ctredge = NULL;
	ctredgenum = NULL;
	ctrtriconfig = NULL;
	ctrtrinum = NULL;

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

	cout<<"Starting setting the generated mesh....................."<<endl;
	sufvernum = mver.size()/3;
	suffacenum = mface.size()/5;

	sufver = new float[ sufvernum * 3];
	sufface = new int[ suffacenum * 3 ];
	suffacenorm = new float[ suffacenum * 3];
	sufmat = new int[ suffacenum * 2 ];
	
	int matMark[18];
	for( int i = 0; i < 18; i ++)
		matMark[ i ] = 0;
	//read ver
//	float min[3];
//	float max[3];
//	bool first = true;
//	float pos[3];
	int ind = 0;
	for( int i = 0; i < sufvernum; i++)
	{
	//	fscanf(fin, "%f %f %f\r\n",&pos[0],&pos[1],&pos[2]);
	//	memcpy( &sufver[ 3*i ], pos, sizeof( float )*3);
		for( int j= 0; j < 3; j ++ )
		{
			//pos[ j ] = 
			//modified by kw
//            sufver[ ind ] = mver[ ind ] * 1.5/ PROCSIZE;
			sufver[ ind ] = mver[ ind ] * DIM/ PROCSIZE;
			ind ++;
		}		
		
		//if( first )
		//{
		//	first = false;
		//	min[0] = max[0] = pos[ 0 ];
		//	min[1] = max[1] = pos[ 1 ];
		//	min[2] = max[2] = pos[ 2 ];
		//	continue;
		//}
		//if( pos[0] < min[0])
		//	min[0] = pos[0];
		//else if( pos[0] > max[0])
		//	max[0] = pos[0];
		//if( pos[1] < min[1]) 
		//	min[1] = pos[1];
		//else if ( pos[1 ] > max[1])
		//	max[1] = pos[1];
		//if( pos[2] < min[2]) 
		//	min[2] = pos[2];
		//else if( pos[2] > max[2]) 
		//	max[2] =pos[2];
	}
	
	//unitlen = max[ 0 ] - min[ 0 ] > max[ 1 ] - min[ 1 ]?max[ 0 ] - min[ 0 ]:max[ 1 ] - min[ 1 ];
	//unitlen = unitlen > max[ 2 ] - min[ 2 ]?unitlen:max[ 2 ] - min[ 2 ];
	//unitlen /= 3;
	//MyMath::center(min, max, center);

	/*printf("minx:%f,%f,%f\n",min[0],min[1],min[2]);
	printf("max:%f,%f,%f\n",max[0],max[1],max[2]);*/
	printf("center:%f,%f,%f\n",center[0],center[1],center[2]);
	/*for( int i = 0; i < sufvernum; i ++)
	{
		MyMath::getrelativepos( &sufver[ 3 * i ], center, unitlen);
	}*/

	//read face in
	float vec1[3];
	float vec2[3];
	ind = 0;
	int ind2 = 0;
	int ind3 = 0;
	for( int i = 0; i< suffacenum; i++)
	{
		for( int j= 0; j < 3; j++ )
		{
			sufface[ ind2 ] = mface[ ind ++ ];
			ind2 ++;
		}
		for( int j = 0; j < 2; j ++)
		{
			sufmat[ ind3 ] = mface[ ind ++ ];
			ind3 ++;

			//////////////////////////////////////////////////////////////////////////
			//cout<<sufmat[ ind3 -1 ]<<" ";
			//////////////////////////////////////////////////////////////////////////

		}
		//////////////////////////////////////////////////////////////////////////
		//cout<<" , ";
		//////////////////////////////////////////////////////////////////////////
	
	//	fscanf(fin, "%d %d %d %d %d\r\n", &sufface[i * 3], &sufface[i * 3 + 1],&sufface[i * 3  + 2], &sufmat[i * 2], &sufmat[i * 2 + 1]);
		matMark[sufmat[i * 2]] = 1;
		matMark[sufmat[i * 2 + 1]] = 1;
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}

	//////////////////////////////////////////////////////////////////////////
	/*cout<<endl;
	for( int i = 0; i < suffacenum; i ++ )
	{
		cout<<sufmat[ i * 2]<<","<<sufmat[ i * 2 + 1 ]<<"\t";
	}
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////
	matlist.clear();
	curmat = 0;
	for( int i = 0; i < 18;i ++ )
	{
		if( matMark[ i ] == 1)
		{
			matlist.push_back( i );
		}
	}

	//read contour edge in
	sufctredgenum = ctrmedge.size()/2;
//	fscanf(fin, "%d\r\n",&sufctredgenum);
	cout<<"contour edge number:"<<sufctredgenum<<endl;
	if( sufctredgenum != 0 )
		sufctredge = new int[ sufctredgenum * 2 ];
	ind = 0;
	for( int i = 0; i < sufctredgenum * 2 ; i ++)
	{
		sufctredge[ i ] = ctrmedge[ i ];
		//fscanf(fin, "%d %d\r\n", &sufctredge[ i * 2 ], &sufctredge[ i * 2 + 1]);
	}
	//fclose(fin);
	//cout<<"Reading done!"<<endl;
	cout<<"------------------------------"<<endl;
}

bool Mesh::readMesh(const char* fname)
{
	FILE* fin = fopen( fname ,"r");
	if( fin == NULL)
	{
//		fl_alert("Not a valid file, open fails!\n");
		return false;
	}
	cout<<"------------------------------"<<endl;
	cout<<"Reading mesh...."<<endl;
	//read verlen, facelen
	fscanf(fin, "%d %d\r\n", &sufvernum, &suffacenum);
	if( sufver != NULL)
	{
		delete []sufver;
		delete []sufface;
		delete []sufmat;
		delete []suffacenorm;
	}
	sufver = new float[ sufvernum * 3];
	sufface = new int[ suffacenum * 3 ];
	suffacenorm = new float[ suffacenum * 3];
	sufmat = new int[ suffacenum * 2 ];
	int matMark[18];
	for( int i = 0; i < 18; i ++)
		matMark[ i ] = 0;
	//read ver
	float min[3];
	float max[3];
	bool first = true;
	float pos[3];
	for( int i = 0; i < sufvernum; i++)
	{
		fscanf(fin, "%f %f %f\r\n",&pos[0],&pos[1],&pos[2]);
		memcpy( &sufver[ 3*i ], pos, sizeof( float )*3);
		//////////////////////////////////////////////////////////////////////////
		//	cout<<(verlist[i]->getPos())[0]<<" "<<(verlist[i]->getPos())[1]<<" "<<(verlist[i]->getPos())[2]<<endl;
		if( first )
		{
			first = false;
			min[0] = max[0] = pos[0];
			min[1] = max[1] = pos[1];
			min[2] = max[2] = pos[2];
			continue;
		}
		if( pos[0] < min[0])
			min[0] = pos[0];
		else if( pos[0] > max[0])
			max[0] = pos[0];
		if( pos[1] < min[1]) 
			min[1] = pos[1];
		else if ( pos[1 ] > max[1])
			max[1] = pos[1];
		if( pos[2] < min[2]) 
			min[2] = pos[2];
		else if( pos[2] > max[2]) 
			max[2] =pos[2];
	}
	//normalize all the vers
	//unitlen = MyMath::vectorlen(min, max);
	unitlen = max[ 0 ] - min[ 0 ] > max[ 1 ] - min[ 1 ]?max[ 0 ] - min[ 0 ]:max[ 1 ] - min[ 1 ];
	unitlen = unitlen > max[ 2 ] - min[ 2 ]?unitlen:max[ 2 ] - min[ 2 ];
	unitlen /= 3;
	MyMath::center(min, max, center);

	printf("minx:%f,%f,%f\n",min[0],min[1],min[2]);
	printf("max:%f,%f,%f\n",max[0],max[1],max[2]);
	printf("center:%f,%f,%f\n",center[0],center[1],center[2]);
	for( int i = 0; i < sufvernum; i ++)
	{
		MyMath::getrelativepos( &sufver[ 3 * i ], center, unitlen);
	}

	//read face in
	float vec1[3];
	float vec2[3];
	for( int i = 0; i< suffacenum; i++)
	{
		fscanf(fin, "%d %d %d %d %d\r\n", &sufface[i * 3], &sufface[i * 3 + 1],&sufface[i * 3  + 2], &sufmat[i * 2], &sufmat[i * 2 + 1]);
		matMark[sufmat[i * 2]] = 1;
		matMark[sufmat[i * 2 + 1]] = 1;
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 1] * 3 ], vec1 );
		MyMath::getVec(&sufver[ sufface[ i * 3] * 3 ], &sufver[ sufface[ i * 3 + 2] * 3 ], vec2 );
		MyMath::crossProduct( vec1, vec2, &suffacenorm[ i * 3 ]);
	}

	matlist.clear();
	curmat = 0;
	for( int i = 0; i < 18;i ++ )
	{
		if( matMark[ i ] == 1)
		{
			matlist.push_back( i );
		}
	}

	//read contour edge in
	fscanf(fin, "%d\r\n",&sufctredgenum);
	cout<<"contour edge number:"<<sufctredgenum<<endl;
	sufctredge = new int[ sufctredgenum * 2 ];
	for( int i = 0; i < sufctredgenum; i ++)
	{
		fscanf(fin, "%d %d\r\n", &sufctredge[ i * 2 ], &sufctredge[ i * 2 + 1]);
	}
	fclose(fin);
	cout<<"Reading done!"<<endl;
	cout<<"------------------------------"<<endl;


	/*---------------------*/
	selectVerList.clear();
	//	dbnmedgelist.clear();
	/*	edgelist.clear();
	for(int i = 0; i < edge2facelist.size(); i ++)
	edge2facelist[ i ].clear();
	edge2facelist.clear();
	ctredgelist.clear();
	nmedgelist.clear();
	normedgelist.clear();
	edgelenlist.clear();
	edgetypelist.clear();*/
	/*---------------------*/	
	return true;
}
void Mesh::writeMeshSuf(const char* fname)

{
	FILE* fout = fopen( fname ,"w");
	if( fout == NULL)
	{
//		fl_alert("Unable to write the file!\n");
		return;
	}
	cout<<"------------------------------"<<endl;
	cout<<"writing mesh...."<<endl;

	//write verlen, facelen
	fprintf(fout, "%d %d\r\n", sufvernum, suffacenum);

	//write ver
	for( int i = 0; i < sufvernum; i++)
	{
		fprintf(fout, "%f %f %f\r\n",sufver[3*i]*unitlen+center[ 0 ],sufver[3*i+1]*unitlen+center[ 1 ],sufver[3*i+2] *unitlen+center[2]);
	}

	//write face
	for( int i = 0; i< suffacenum; i++)
	{
		fprintf(fout, "%d %d %d %d %d\r\n", sufface[i * 3], sufface[i * 3 + 1],sufface[i * 3  + 2], sufmat[i * 2], sufmat[i * 2 + 1]);
	}

	//write contour edge
	fprintf(fout, "%d\r\n",sufctredgenum);
	//cout<<"contour edge number:"<<sufctredgenum<<endl;
	for( int i = 0; i < sufctredgenum; i ++)
	{
		fprintf(fout, "%d %d\r\n", sufctredge[ i * 2 ], sufctredge[ i * 2 + 1]);
	}
	fclose(fout);
	cout<<"Writing done!"<<endl;
	cout<<"------------------------------"<<endl;
}
void Mesh::writeMeshPly(const char* fname)
{
	//Open file
	FILE* fout = fopen ( fname, "wb" ) ;
	if( fout == NULL )
	{
		cout<<"Unable the file to write!"<<endl;
		return;
	}
	cout<<"Start writing.......\t";
	//Write header
	printf("Vertices counted: %d Polys counted: %d \n", sufvernum, suffacenum) ;
	PLYWriter::writeHeader( fout, sufvernum, suffacenum ) ;

	//Write all vertices
	float pts[3];
	for( int i = 0; i < sufvernum; i ++)
	{
		memcpy( pts, &sufver[ 3 * i ], sizeof( float )* 3  );
		for( int j = 0; j < 3; j++)
		{
			pts[ j ] = pts[ j ]*unitlen + center[ j ];
		}
		PLYWriter::writeVertex( fout, pts) ;
	}


	//Write all triangles
	int face[ 3 ];
	for( int i = 0; i < suffacenum; i ++)
	{
		memcpy( face,  &sufface[3 * i], sizeof (int ) * 3);
		PLYWriter::writeFace( fout, 3,  face) ;
	}

	//Close file
	fclose( fout ) ;
	cout<<"done!"<<endl;

	/*
	* Temporaray, write one point on the contour into another file, to get the plane info.
	*/
	char fname2[20] = "plane.txt\0";
	fout = fopen( fname2, "w");
	if( fout == NULL )
	{
		cout<<"Unable open the file to write out the plane informaiton"<<endl;
		return;
	}
	fprintf(fout,"1 %d\n", ctrplanenum);	//type 1, #planes

	for( int i = 0; i < ctrplanenum; i ++)
	{
		fprintf( fout, "%f %f %f\n", ctrver[i][0], ctrver[ i ][ 1 ], ctrver[ i ][ 2 ]);
	}
	fclose( fout );
	//////////////////////////////////////////////////////////////////////////
}
bool Mesh::readContour(const char* fname)
{
	FILE* fin = fopen(fname, "r");
	if( fin == NULL)
	{
//		fl_alert( "Reading contour fails! Invalid .ctr file!");
		return false;
	}
	cout<<"------------------------------"<<endl;
	cout<<"Reading contour in...."<<endl;
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
		delete []ctredgenum;
		delete []ctrtrinum;
		delete []ctrtriconfig;
		ctrplanenum = 0;
	}

	//read the number of contours
	fscanf( fin,"%d", &ctrplanenum);	
	ctrplaneparam = new float[ 4 * ctrplanenum];
	ctrver = new float*[ ctrplanenum ];
	ctrvernum = new int[ ctrplanenum ];
	ctredge = new int*[ ctrplanenum ];
	ctredgenum = new int[ ctrplanenum ];
	ctrtrinum = new int[ctrplanenum];
	ctrtriconfig = new int*[ctrplanenum];

	for( int i = 0; i < ctrplanenum; i ++)
	{
		//read param in
		fscanf(fin, "%f %f %f %f\r\n",&ctrplaneparam[ i * 4],&ctrplaneparam[ i * 4 + 1],
			&ctrplaneparam[ i * 4 + 2],&ctrplaneparam[ i * 4 + 3]);
		//read vernum and edgenum
		fscanf(fin, "%d %d %d\r\n",&ctrvernum[ i ], &ctredgenum[ i ], &ctrtrinum[ i ]);

		//read vers and edges and triangles
		ctrver[ i ] = new float[ 3 * ctrvernum[ i ] ];
		ctredge[ i ] = new int[ 2 * ctredgenum[ i ]];
		ctrtriconfig[ i ] = new int[ 4*ctrtrinum[ i ]];

		for( int j = 0; j < ctrvernum[i]; j++)
		{
			fscanf(fin, "%f %f %f\r\n",&ctrver[ i ][j*3], &ctrver[ i ][j*3 + 1], &ctrver[ i ][j*3 + 2]);
			//normalize all the positions
			MyMath::getrelativepos( &ctrver[ i ][j*3], center, unitlen);
		}

		for( int j = 0; j < ctredgenum[ i ]; j++)
		{
			fscanf(fin, "%d %d\r\n",&ctredge[ i ][j * 2], &ctredge[ i ][j * 2 + 1]);
		}

		for( int j = 0; j < ctrtrinum[ i ]; j ++)
		{
			fscanf(fin, "%d %d %d %d\r\n", &ctrtriconfig[ i ][ 4 * j ], &ctrtriconfig[ i ][ 4*j + 1],
				&ctrtriconfig[ i ][ 4 * j + 2],&ctrtriconfig[ i ][ 4 * j + 3]);
		}
	}
	fclose( fin );

	cout<<"Reading contour done!"<<endl;
	cout<<"------------------------------"<<endl;
	return true;

}