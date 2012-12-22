#include "../Ctr2SufManager/Ctr2SufManager.h"

void Ctr2SufManager::partition()
{
	clearFaceContour();
	//clear the old result, a new generation start from here!
	//because other data may be dependent on the value of partition result, 
	//so before clearpartition, other data must be cleared too!
	clearPartition();

	//temporary data
	floatvector tssver;
	intvector tssedge;
	vector<intvector> tssface;
	intvector tssface_planeindex;
	vector<intvector> tssspace;
	vector<intvector> tssspace_planeside;

	//partition!
	SpacePartitioner partitioner;

	if( !isComnCase )
		partitioner.partition( planenum, pparam, pbbox, enlargeratio,
		tssver, tssedge, tssface, tssface_planeindex, tssspace, tssspace_planeside);
	else
	{
		partitioner.partition_ComnLine(
			planenum, pparam,
		 pbbox, enlargeratio,
		tssver, tssedge,  comnedgei,
		tssface, tssface_planeindex, 
		tssspace, tssspace_planeside,
		 comndir, comnpt //common line of the cutting planes
		);

		//////////////////////////////////////////////////////////////////////////
		cout<<"common edge is:"<<comnedgei<<endl;
		//////////////////////////////////////////////////////////////////////////
		comnveri[ 0 ] = tssedge[ 2 * comnedgei ];
		comnveri[ 1 ] = tssedge[ 2 * comnedgei + 1];
	}

	//copy them out
	int versize = tssver.size();
	ssvernum = versize/3;
	ssver = new float[ versize ];
	for( int i = 0; i < versize; i ++)
	{
		ssver[ i ] = tssver[ i ];
	}
	int edgesize = tssedge.size();
	ssedgenum = edgesize/2;
	ssedge = new int[ edgesize ];
	for( int i = 0; i < edgesize; i ++)
		ssedge[ i ] = tssedge[ i ];
    
	ssfacenum = tssface.size();
	ssface = new int* [ ssfacenum ];
	ssfaceedgenum = new int [ ssfacenum ];
	ssface_planeindex = new int[ ssfacenum ];
	for( int i = 0; i < ssfacenum ;i ++ )
	{
		ssface_planeindex[ i ] = tssface_planeindex[ i ];
		ssfaceedgenum [ i ] = tssface[ i ].size();
		ssface[ i ] = new int[ ssfaceedgenum[ i ]];
		for(int j = 0;  j < ssfaceedgenum[ i ]; j ++)
		{
			ssface[ i ][ j ] = tssface[ i ][ j ];
		}
	}
	
	ssspacenum = tssspace.size();
	ssspace = new int* [ ssspacenum];
	ssspace_planeside = new int* [ ssspacenum] ;
	ssspacefacenum = new int[ ssspacenum ];
	for( int i = 0; i < ssspacenum; i ++)
	{
		ssspacefacenum[ i ] = tssspace[ i ].size();
		ssspace[ i ] = new int[ ssspacefacenum[ i ]];
		ssspace_planeside[ i ] = new int[ssspacefacenum[ i ]];
		for( int j = 0; j < ssspacefacenum[ i ]; j ++) 
		{
			ssspace[ i ][ j ] = tssspace[ i ][ j ];
			ssspace_planeside[ i ][ j ] = tssspace_planeside[ i ][ j ];
		}
	}

	//save info for rendering before clearing
	vector<int> vecSSfaceedgenum;
	for (int i=0;i<ssfacenum;i++)
	{
		vecSSfaceedgenum.push_back(tssface.at(i).size());
	}
	vector<int> vecSSspacefacenum;
	for (int i=0;i<this->ssspacenum;i++)
	{
		vecSSspacefacenum.push_back(tssspace.at(i).size());
	}
	SortFaceInfo(tssver,tssedge,tssface.size(),tssface,tssface_planeindex,ssspacenum,vecSSfaceedgenum,vecSSspacefacenum,tssspace);

	//clear the temporary data   
	tssver.clear();
	tssedge.clear();

	int size = tssface.size();
	for( int i = 0; i < size; i ++ )
		tssface[ i ].clear();
	tssface.clear();
	tssface_planeindex.clear();

	size = tssspace.size();
	for( int i = 0; i < size; i ++)
	{
		tssspace[ i ].clear();
		tssspace_planeside[ i ].clear();
	}
	tssspace.clear();
	tssspace_planeside.clear();	
}

void Ctr2SufManager::SortFaceInfo(vector<float> vecSSver,vector<int> vecSSedge,int iSSfacenum, vector<vector<int>> vecvecSSface,vector<int> vecSSface_planeindex, 
							  int iSSspacenum,vector<int> vecSSfaceedgenum,vector<int> vecSSspacefacenum, vector<vector<int>> vecvecSSspace)
{
	//fill in basic info for each face
	for (int i=0;i<iSSfacenum;i++)
	{
		ResortedFace currentFace;
		if (vecSSface_planeindex.at(i)<0)//face on the bounding box
		{
			currentFace.bBoundaryFace=true;
		}
		else
		{
			currentFace.bBoundaryFace=false;
			currentFace.iFacePlaneID=vecSSface_planeindex.at(i);
			//get the four vertices of the face
			set<Point_3> setCurrentFace;
			assert(vecSSfaceedgenum.at(i)==4);
			for (int j=0;j<vecSSfaceedgenum[i];j++)//ssfaceedgenum[i] should == 4
			{
				int iEdgeInd=vecvecSSface.at(i).at(j);
				int iVerBeginInd=vecSSedge.at(2*iEdgeInd);
				int iVerEndInd=vecSSedge.at(2*iEdgeInd+1);
				Point_3 VerBegin=Point_3(vecSSver.at(3*iVerBeginInd),vecSSver.at(3*iVerBeginInd+1),vecSSver.at(3*iVerBeginInd+2));
				Point_3 VerEnd=Point_3(vecSSver.at(3*iVerEndInd),vecSSver.at(3*iVerEndInd+1),vecSSver.at(3*iVerEndInd+2));
				setCurrentFace.insert(VerBegin);
				setCurrentFace.insert(VerEnd);
			}
			vector<Point_3> tempFaceVer;
			tempFaceVer.resize(setCurrentFace.size());
			copy(setCurrentFace.begin(),setCurrentFace.end(),tempFaceVer.begin());
			GetSquareFace(tempFaceVer);
			currentFace.vecFaceVertex=tempFaceVer;
		}
		this->vecResortFace.push_back(currentFace);
	}
	//fill in the properties related to subspace for each face
	for (int i=0;i<iSSspacenum;i++)
	{
		for (int j=0;j<vecSSspacefacenum.at(i);j++)
		{
			int iFaceId=vecvecSSspace.at(i).at(j);
			//if boundary face,do not calculate at all
			if (this->vecResortFace.at(iFaceId).bBoundaryFace)
			{
				continue;
			}
			this->vecResortFace.at(iFaceId).vecSubspaceId.push_back(i);
			bool bOrient=false;
			Vector_3 HeightVec=CGAL::NULL_VECTOR;
			GetFacePara(iFaceId,i,bOrient,HeightVec,vecSSspacefacenum,vecvecSSspace,vecSSfaceedgenum,vecvecSSface,vecSSedge,vecSSver);
			this->vecResortFace.at(iFaceId).vecOrient.push_back(bOrient);
			this->vecResortFace.at(iFaceId).vecHeightVect.push_back(HeightVec);
		}

	}
	assert(this->vecResortFace.size()==iSSfacenum);
	for (unsigned int i=0;i<vecResortFace.size();i++)
	{
		if (!vecResortFace.at(i).bBoundaryFace)
		{
			assert(this->vecResortFace.at(i).vecFaceVertex.size()==4);
			assert(this->vecResortFace.at(i).vecSubspaceId.size()==2);
			assert(this->vecResortFace.at(i).vecOrient.size()==2);
			assert(this->vecResortFace.at(i).vecHeightVect.size()==2);
		}
	}
}

void Ctr2SufManager::GetFacePara(int iFaceId,int iSubspaceId,bool& bOrient,Vector_3& HeightVec,vector<int> vecSSspacefacenum,vector<vector<int>> vecvecSSspace,
								 vector<int> vecSSfaceedgenum,vector<vector<int>> vecvecSSface,vector<int> vecSSedge,vector<float> vecSSver)
{
	//get the four vertices of the current face
	vector<Point_3> vecCurrentFace=this->vecResortFace.at(iFaceId).vecFaceVertex;

	//get an arbitrary edge of the subspace which has only one point on the face[iFaceId]
	//record the length of the edge(the height w.r.t. the face in the subspace) and the vertex which is not on the face
	Point_3 ArbitPoint;
	//iterate other faces of the subspace 
	for (int i=0;i<vecSSspacefacenum.at(iSubspaceId);i++)
	{
		int iOtherFaceId=vecvecSSspace.at(iSubspaceId).at(i);
		if (iOtherFaceId==iFaceId)//if it is the current face
		{
			continue;
		}
		for (int j=0;j<vecSSfaceedgenum.at(iOtherFaceId);j++)//ssfaceedgenum[i] should == 4
		{
			int iEdgeInd=vecvecSSface.at(iOtherFaceId).at(j);
			int iVerBeginInd=vecSSedge.at(2*iEdgeInd);
			int iVerEndInd=vecSSedge.at(2*iEdgeInd+1);
			Point_3 VerBegin=Point_3(vecSSver.at(3*iVerBeginInd),vecSSver.at(3*iVerBeginInd+1),vecSSver.at(3*iVerBeginInd+2));
			Point_3 VerEnd=Point_3(vecSSver.at(3*iVerEndInd),vecSSver.at(3*iVerEndInd+1),vecSSver.at(3*iVerEndInd+2));
			//judge if one is on the face and the other is not
			vector<Point_3>::iterator pFind0=find(vecCurrentFace.begin(),vecCurrentFace.end(),VerBegin);
			vector<Point_3>::iterator pFind1=find(vecCurrentFace.begin(),vecCurrentFace.end(),VerEnd);
			if (pFind0==vecCurrentFace.end()&&pFind1!=vecCurrentFace.end())
			{
				ArbitPoint=VerBegin;
				HeightVec=VerBegin-VerEnd;
				break;
			}
			else if (pFind0!=vecCurrentFace.end()&&pFind1==vecCurrentFace.end())
			{
				ArbitPoint=VerEnd;
				HeightVec=VerEnd-VerBegin;
				break;
			}
		}
		if (HeightVec!=CGAL::NULL_VECTOR)
		{
			break;
		}
	}
	assert(HeightVec!=CGAL::NULL_VECTOR);
	//judge the type of the plane of the current face
	//since the bounding box is also considered,vector<CurveNetwork> vecTempCN cannot be used for the calculation
	Point_3 Point0=vecCurrentFace.at(0);
	Point_3 Point1=vecCurrentFace.at(1);
	Point_3 Point2=vecCurrentFace.at(2);
	Point_3 Point3=vecCurrentFace.at(3);
	if (Point0.x()==Point1.x() && Point1.x()==Point2.x() && Point2.x()==Point3.x())//yoz plane
	{
		if (ArbitPoint.x()<Point0.x())
		{
			bOrient=false;
		}
		else
		{
			bOrient=true;
		}
	}
	else if (Point0.y()==Point1.y() && Point1.y()==Point2.y() && Point2.y()==Point3.y())//xoz plane
	{
		if (ArbitPoint.y()<Point0.y())
		{
			bOrient=false;
		}
		else
		{
			bOrient=true;
		}
	}
	else if (Point0.z()==Point1.z() && Point1.z()==Point2.z() && Point2.z()==Point3.z())//xoy plane
	{
		if (ArbitPoint.z()<Point0.z())
		{
			bOrient=false;
		} 
		else
		{
			bOrient=true;
		}
	}
	else
	{
		cout<<"plane type judge error"<<endl;
		cout<<"X: "<<Point0.x()<<" "<<Point1.x()<<" "<<Point2.x()<<" "<<Point3.x()<<endl;
		cout<<"Y: "<<Point0.y()<<" "<<Point1.y()<<" "<<Point2.y()<<" "<<Point3.y()<<endl;
		cout<<"Z: "<<Point0.z()<<" "<<Point1.z()<<" "<<Point2.z()<<" "<<Point3.z()<<endl;
	}
}


void Ctr2SufManager::clearPartition()
{
	if( ssver == NULL )return;

	delete []ssver;
	delete []ssedge;
	
	for( int i = 0; i < ssfacenum; i ++ )
	{
		delete []ssface[ i ];
		
	}	
	delete []ssface;
	delete []ssfaceedgenum;
	delete []ssface_planeindex;

	for( int i = 0; i < ssspacenum; i ++)
	{
		delete []ssspace[ i ];
		delete []ssspace_planeside[ i ];
	}
	delete []ssspace_planeside;
	delete []ssspace;
	delete []ssspacefacenum;

	ssvernum = ssedgenum = ssfacenum = ssspacenum = 0;

	//kw added,clear ma info
	this->ClearMAInfo();

	this->vecResortFace.clear();
}

void Ctr2SufManager::renderPartition()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glLineWidth( 2 );
	glColor3f( 0, 0, 1);

	//render all the edges
//	int edgelen = ssedge.size();
//	for( int i = 0; i < ssedge.size(); i ++)
//	{

//	}
	
	glEnd();
	glEnable(GL_LIGHTING);
}
void Ctr2SufManager::writePartitionOut(const char* fname)
{
	SpacePartitioner partitioner;
	partitioner.wfilePartition(fname,ssvernum,ssver,ssedgenum, ssedge,ssfacenum,ssfaceedgenum,
		 ssface,ssface_planeindex,ssspacenum, ssspacefacenum, ssspace,ssspace_planeside);
}