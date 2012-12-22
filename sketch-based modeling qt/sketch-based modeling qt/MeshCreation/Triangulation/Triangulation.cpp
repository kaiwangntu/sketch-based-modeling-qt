#include "../Triangulation/Triangulation.h"

//check if the three vertices are in CCW order
bool Triangulation::checkCCW(
			  floatvector& ver2d,
			  vector<intvector>& cycles,
			  int veri[ 6 ]
			  )
{
	int vi[ 3 ];
	for( int i = 0; i < 3; i ++ )
	{
		vi[ i ] = cycles[ veri[ 2*i ] ][ veri[ 2*i + 1 ]];
	}

	float dir1[ 2 ], dir2[ 2 ];
	for( int i = 0; i < 2; i ++ )
	{
		dir1[ i ] = ver2d[ vi[ 1 ] * 2 + i ] - ver2d[ vi[ 0 ] * 2 + i ];
		dir2[ i ] = ver2d[ vi[ 2 ] * 2 + i ] - ver2d[ vi[ 0 ] * 2 + i ];
	}
	float val = dir1[ 0 ] * dir2[ 1 ] - dir1[ 1 ] * dir2[ 0 ];
	float len1 = sqrt( (float)( dir1[ 0 ]*dir1[ 0 ] + dir1[ 1 ] *dir1[ 1 ] ));
	val /= len1;
	len1 = sqrt( ( float )( dir2[ 0 ] * dir2[ 0 ] + dir2[ 1 ] * dir2[ 1 ] ));
	val /= len1; 
	if( val > TOLERANCE_THREE )
		return true;
	return false;
}
bool Triangulation::checkCCW(  floatvector& ver2d,  int vi[ 3 ]  )
{
	float dir1[ 2 ], dir2[ 2 ];
	for( int i = 0; i < 2; i ++ )
	{
		dir1[ i ] = ver2d[ vi[ 0 ] * 2 + i ] - ver2d[ vi[ 2 ] * 2 + i ];
		dir2[ i ] = ver2d[ vi[ 1 ] * 2 + i ] - ver2d[ vi[ 2 ] * 2 + i ];
	}

	float val = dir1[ 0 ] * dir2[ 1 ] - dir1[ 1 ] * dir2[ 0 ];
	float len1 = sqrt( (float)( dir1[ 0 ]*dir1[ 0 ] + dir1[ 1 ] *dir1[ 1 ] ));
	val /= len1;
	len1 = sqrt( ( float )( dir2[ 0 ] * dir2[ 0 ] + dir2[ 1 ] * dir2[ 1 ] ));
	val /= len1; 
	if( val > -TOLERANCE_FOUR )	//for check vertex in triangle, require the vertex lies exactly right of one edge
		return true;
	return false;
}

//check if v1 v2 lies between the two edge incident with v1
bool Triangulation::checkInBetween(
	floatvector& ver2d,
	vector<intvector>& cycles,
	int cycle1, int v1, int cycle2, int v2)
{
	int len = cycles[ cycle1 ].size();
	int prev = cycles[ cycle1 ][(v1 + len - 1)%len];
	int nextv = cycles[ cycle1 ][ (v1 + 1)%len ] ;
	int curv = cycles[ cycle1 ][ v1 ];
	int othv = cycles[ cycle2 ][ v2 ];

	if( (othv == prev) || ( othv == nextv ) )return true;

	float xaxis[ 2 ];
	float vec[ 2 ];
	float lvec[ 2 ];
	for( int i = 0; i < 2; i ++ )
	{
		xaxis[ i ] = ver2d[ 2*nextv + i ] - ver2d[ 2*curv + i ];
		vec[ i ] = ver2d[ 2*othv + i ] - ver2d[ 2*curv + i ];
		lvec[ i ] = ver2d[ 2*prev + i ] - ver2d[ 2*curv + i ];
	}
	float xaxislen = sqrt( (float ) ( xaxis[ 0 ] * xaxis[ 0 ] + xaxis[ 1 ] * xaxis[ 1 ]));
	xaxis[ 0 ] = xaxis[ 0 ]/xaxislen;
	xaxis[ 1 ] = xaxis[ 1 ]/xaxislen;
	
	float ang = getAngleVecX2D( vec, xaxis);
	float lang = getAngleVecX2D( lvec, xaxis);
	if( ang < lang )
		return true;
	return false;
}

bool Triangulation::checkEdgeInter(
		floatvector& ver2d,
		int rveri[ 3 ],	//the three vertices of the triangle
		int evi[ 2 ]
					)
{
	//check if the edge intersects with the edges of triangles
	for( int i = 0; i < 3; i ++ )
	{
		int vs[ 2 ] = { rveri[ i ], rveri [ (i + 1) % 3 ]};
		//check if the two edges have common vertex
		if( (vs[ 0 ] == evi[ 0 ]) || ( vs[ 0 ] == evi[ 1 ] ) 
			|| ( vs[ 1 ] == evi[ 0 ]) ||( vs[ 1 ] == evi[ 1 ]))
		{
			continue;	//no intersection at all
		}

		bool checkover = false;
		//check if the two vertices of edge lie on different sides of the triangle edge
		float dir[ 2 ];
		dir[ 0 ] = ver2d[ 2*vs[ 0 ] + 1] - ver2d[ 2*vs[ 1 ] + 1];
		dir[ 1 ] = ver2d[ 2*vs[ 1 ] ] - ver2d[ 2*vs[ 0 ] ];
		float val = dir[ 0 ] * ver2d[ 2*vs[ 0 ]] + dir[ 1 ]*ver2d[ 2*vs[ 0 ]+1 ];
		float vals[ 2 ];
		vals[ 0 ] = dir[ 0 ] * ver2d[ 2*evi[ 0 ]] + dir[ 1 ] * ver2d[ 2*evi[ 0 ] + 1];
		vals[ 1 ] = dir[ 0 ] * ver2d[ 2*evi[ 1 ]] + dir[ 1 ] * ver2d[ 2*evi[ 1 ] + 1];
		int signs[ 2 ];
		signs[ 0 ] = MyMath::getSign( vals[ 0 ] - val);
		signs[ 1 ] = MyMath::getSign( vals[ 1 ] - val) ;
		if( signs[ 0 ] * signs[ 1 ] < 0 )
			checkover = false;
		else
			checkover = true;
		if( checkover )
			continue;
		

		//check if the two vertices of triangle edge lie on different sides of the edge
		dir[ 0 ] = ver2d[ 2*evi[ 0 ] + 1] - ver2d[ 2*evi[ 1] +1];
		dir[ 1 ] = ver2d[ 2*evi[ 1 ]] - ver2d[ 2*evi[ 0 ] ];
		val = dir[ 0 ] * ver2d[ 2*evi[ 0 ]] + dir[ 1 ] * ver2d[ 2*evi[ 0 ] + 1];
		vals[ 0 ] = dir[ 0 ]*ver2d[ 2*vs[ 0 ]] + dir[ 1 ] * ver2d[ 2*vs[ 0 ] + 1];
		vals[ 1 ] = dir[ 0 ] * ver2d[ 2*vs[ 1 ]] + dir[ 1 ] * ver2d[ 2*vs[ 1 ] + 1];
		signs[ 0 ] = MyMath::getSign( vals[ 0 ] - val );
		signs[ 1 ] = MyMath::getSign( vals[ 1 ] - val);
		if( signs[ 0 ] * signs[ 1 ] == -1)
			return true;	//intersects!
	}
	return false;
}
//check if the vertex lies in the triangle or not
//bool checkInTri(floatvector& ver2d,  int rveri[ 3 ], int vi  )
//{
//	//check if vi is one of the vertex
//	for( int i = 0; i < 3; i ++ )
//	{
//		if( vi == rveri[ i ]) return true;
//	}
//
//	//check if vi lies on the left of all the vertices
//
//
//}

//check if current triangle can be cut or not
//v1, v2 are the two first vertices of the first cycle
bool Triangulation::checkTri(
			  floatvector& ver2d,
			  vector<intvector>& cycles,
			  int veri[ 6 ]
			  )
{
	bool suc = true;
	//step1.check if the triangle is counter clock wise
	suc = checkCCW(ver2d, cycles,veri);
	if( !suc )return false;

	//////////////////////////////////////////////////////////////////////////
	//cout<<"checkccw pass!"<<endl;
	//////////////////////////////////////////////////////////////////////////
	//step2. check if v1 v3 lies between two incident edges of v1
	//		check if v2v3 lies between two incident edges of v2
	//		check if v3v1 and v3v2 lies between two incident edges of v3
	suc = checkInBetween(ver2d,cycles,veri[ 0 ], veri[ 1 ], veri[ 4 ], veri[ 5 ]);
	if( !suc )return false;
	suc =  checkInBetween(ver2d,cycles,veri[ 2 ], veri[ 3 ], veri[ 4 ], veri[ 5 ]);
	if( !suc ) return false;
	suc = checkInBetween(ver2d,cycles,veri[ 4 ], veri[ 5 ], veri[ 0 ], veri[ 1 ]);
	if( !suc ) return false;
	suc = checkInBetween(ver2d,cycles,veri[ 4 ], veri[ 5 ], veri[ 2 ], veri[ 3 ]);
	if( !suc ) return false;

	//step3. check if there is any vertex lying in the triangle
	int rveri[ 3 ];	//real vertex index in ver2d, not in cycles
	for( int i = 0; i < 3; i ++ )
		rveri[ i ] = cycles[ veri[ 2*i ]][ veri[ 2*i + 1]];
	int cyclenum = cycles.size();
	for( int i = 0; i < cyclenum; i ++  )
	{
		int vnum = cycles[ i ].size();
		for( int j = 0; j < vnum ;j ++ )
		{
			int vi = cycles[ i ][ j ];
			int k = 0;
			for(; k < 3; k ++ )
			{
				if( vi == rveri[ k ] )
					break;
			}
					
			if( k < 3)
				continue;
			int tvi[ 3 ];
			tvi[ 2 ] = vi;
			bool intri = false;
			for( int k = 0; k < 3; k ++ )
			{
				tvi[ 0 ] = rveri[ k ];
				tvi[ 1 ] = rveri[ (k+1)%3];
				intri = checkCCW( ver2d,tvi ) ;
				if( !intri )	//for one edge, the vertex lies on the right side of it
					break;
			}
			if( intri )	//current vertex lies in the triangle
				return false;
		}
	}
		
	//step4. check every edge of the cycle, if intersec with any edge of current triangle
	for( int i = 0; i < cyclenum; i ++ )
	{
		int vnum = cycles[ i ].size();
		for( int j = 0;  j < vnum; j ++ )
		{
			int evs[ 2 ];
			evs[ 0 ] = cycles[ i ][ j ];
			evs[ 1 ] = cycles[ i ][ (j+1)%vnum ];
			if( checkEdgeInter(ver2d,rveri, evs) )
				return false;	//intersects!
		}
	}

	//when coming here, check succeeds, return true
	return true;
}
void Triangulation::triangulate( floatvector& ver2d,
	 vector<intvector>& cycles,
	 intvector& tri_vec	//resulting triangles
	 )
{
	bool getone = false;
	int veri[ 6 ] = {0,0,0,1,0,2};
	intvector::iterator iter;
	vector<intvector>::iterator iter2;

	int cyclehead = cycles[ 0 ][ 0 ];

	//////////////////////////////////////////////////////////////////////////
	/*cout<<"cyclehead:"<<cyclehead<<endl;
	cout<<"cyclehead is:"<<cyclehead<<" cycles number:"<<cycles.size()<<endl;

	for( int i= 0; i < cycles.size(); i++ )
	{
		for( int j = 0;j < cycles[ i ].size(); j ++ )
		{
			cout<<cycles[ i ][ j ]<<", ";
		}
		cout<<endl;
	}*/
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	int tcount = 0;
	int tind = 1;
//	writeVerCycle( ver2d, cycles, tind, tind);
	//////////////////////////////////////////////////////////////////////////
	while( cycles[ 0 ].size() != 2 )
	{
		//only one cycle, and it is a triangle!
		if( (cycles.size() == 1) &&(cycles[ 0 ].size() == 3))
		{
			tri_vec.push_back( cycles[ 0 ][ 0 ]);
			tri_vec.push_back( cycles[ 0 ][ 1 ]);
			tri_vec.push_back( cycles[ 0 ][ 2 ]);
			break;
		}

		tcount ++;
		if( tcount == 500000)
		{
		//	cout<<"in not normal exit!"<<endl;
		//	writeVerCycle( ver2d, cycles, 100,100);
			cout<<"Unable to triangulate the current region! Exit anyway! Region information is in vercycle_100_100.txt! "<<endl;
			tri_vec.clear();
			writeVerCycle( ver2d, cycles, 100,100);
			//////////////////////////////////////////////////////////////////////////
			cout<<"cyclehead is:"<<cyclehead<<" cycles number:"<<cycles.size()<<endl;

			for(unsigned int i= 0; i < cycles.size(); i++ )
			{
				for(unsigned int j = 0;j < cycles[ i ].size(); j ++ )
				{
					cout<<cycles[ i ][ j ]<<", ";
				}
				cout<<endl;
			}
			//////////////////////////////////////////////////////////////////////////
			break;
		}
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		//writeVerCycle( ver2d, cycles, 1000,1000);
		//////////////////////////////////////////////////////////////////////////
		getone = false;

		//case1 corner can be cut from current cycle
		veri[ 4 ] = 0;
		veri[ 5 ] = 2;
		if( checkTri(ver2d, cycles,veri) )	//ok, current triangle can be cut off!
		{
		//	getone = true;
            
			//remove the triangle from the cycles, and add it into trilist
			tri_vec.push_back( cycles[ veri[ 0 ]][ veri[ 1 ]] );
			tri_vec.push_back( cycles[ veri[ 2 ]][ veri[ 3 ]]);
			tri_vec.push_back( cycles[ veri[ 4 ]][ veri[ 5 ]]);
			
			iter = cycles[ 0 ].begin();
			iter ++;
			cycles[ 0 ].erase( iter );
			//////////////////////////////////////////////////////////////////////////
			tind ++;
//			writeVerCycle( ver2d, cycles, tind, tind);
			//////////////////////////////////////////////////////////////////////////
			cyclehead = cycles[ 0 ][ 0 ];
			continue;	//have already got one triangle, continue
		}

		//case1 not exists, case2 check two vertices of current cycle and one from another triangle
		int cyclenum = cycles.size();
		for( int i = 1; i < cyclenum; i ++ )
		{
	//		cout<<"checking cycle:"<<i<<endl;
			int vnum = cycles[ i ].size();
			for( int j = 0; j < vnum; j ++ )
			{
				veri[ 4 ] = i;
				veri[ 5 ] = j;
				if(	checkTri( ver2d, cycles, veri ))	//current triangle can be cut off
				{
					tri_vec.push_back( cycles[ veri[ 0 ]][ veri[ 1 ]] );
					tri_vec.push_back( cycles[ veri[ 2 ]][ veri[ 3 ]]);
					tri_vec.push_back( cycles[ veri[ 4 ]][ veri[ 5 ]]);

					//change the cycles
					cycles[ 0 ].push_back( cycles[ 0 ][ 0 ]);
					cyclehead = cycles[ 0 ][ 0 ] = cycles[ i ][ j ];
					for( int k = j; k < vnum; k ++ )
						cycles[ 0 ].push_back( cycles[ i ][ k ]);
					for( int k = 0; k < j ; k ++)
					{
						cycles[ 0 ].push_back( cycles[ i ][ k ]);
					}
					iter2 = cycles.begin();
					for( int k = 0; k < i; k++ )
						iter2 ++;					
					cycles.erase( iter2 );
                    getone = true;
					//////////////////////////////////////////////////////////////////////////
					//tind ++;
					//writeVerCycle( ver2d, cycles, tind, tind);
					//////////////////////////////////////////////////////////////////////////
					break;
				}
			}
			if( getone )
				break;
		}

		//otherwise, rotate current cycle
		if( getone )continue;	//has already found one triangle
        //rotate the current cycle
		int v0 = cycles[ 0 ][ 0 ];
		int vnum = cycles[ 0 ].size();
		for( int j = 1; j < vnum; j ++ )
		{
			cycles[ 0 ][ j - 1 ] = cycles[ 0 ][ j ];
		}
        cycles[ 0 ][ vnum - 1 ] = v0;
		//////////////////////////////////////////////////////////////////////////
		//tind ++;
		//writeVerCycle( ver2d, cycles, tind, tind);
		//////////////////////////////////////////////////////////////////////////
		//rotate one loop of the first cycle, still didn't find a good position to cut!
		if( cycles[ 0 ][ 0 ] == cyclehead )
		{
			//////////////////// //////////////////////////////////////////////////////
			/*
			*	This is added later
			*	Now it is only for one cycle region
			*  actually, it can be extended to multiple cycles.
			*  I do not care if two triangles overlap. if apply smoothing, everything will be puffed out
			*/
			if( cycles.size() == 1 )
			{
				int v1 = cycles[ 0 ][ 0 ];
				int vnum = cycles[ 0 ].size() -1;
				for( int i = 1; i < vnum; i ++ )
				{
					tri_vec.push_back( v1 );
					tri_vec.push_back( cycles[ 0][ i ] );
					tri_vec.push_back( cycles[ 0][ i + 1 ]);
				}
				return;
			}
		}
	}
}

void Triangulation::writeVerCycle(
								  floatvector& ver2d, 
								  vector<intvector>& cycles, 
								  int sheeti, int regioni)
{
	char fname[ 20 ] = "vercycle";
	char strnum[ 5 ];
	itoa( sheeti, strnum, 10);
	strcat( fname, strnum );
	itoa( regioni, strnum, 10 );
	strcat( fname, "_");
	strcat( fname, strnum);
	strcat( fname, ".txt");

	FILE* fout = fopen( fname, "w");

	if( fout == NULL )
	{
		cout<<"unable to open file "<<fname<<" to write!"<<endl;
		return;
	}

	//write out vertice first
	int vnum = ver2d.size()/2;
	for( int i = 0; i< vnum; i ++)
	{
		if( i == 0 )
		{
			fprintf( fout, "{{{%f,%f}", ver2d[ 2*i ], ver2d[ 2* i + 1]);
		}
		else
		{
			fprintf( fout, "{%f,%f}", ver2d[ 2*i ], ver2d[ 2*i + 1]);
		}
		if( i != vnum - 1)
			fprintf( fout, ",");
	}

	int cyclenum = cycles.size();
	for( int i = 0; i < cyclenum; i ++ )
	{
		if( i == 0 )
		{
			fprintf( fout, "},{");
		}

		int vnum = cycles[i].size();
		//////////////////////////////////////////////////////////////////////////
		//cout<<"in write: vnum: "<<vnum<<endl;
		//////////////////////////////////////////////////////////////////////////
		for( int j = 0; j < vnum ;j ++)
		{
			if( j == 0 )
				fprintf( fout, "{%d", cycles[ i ][ j ] + 1);
			else
				fprintf( fout, "%d", cycles[ i ][ j ] + 1);
			if( j != vnum - 1)
				fprintf( fout, ",");
		}

		if( i != cyclenum - 1)
			fprintf( fout, "},");
		else
			fprintf( fout, "}}}");
	}

	fclose( fout );
}