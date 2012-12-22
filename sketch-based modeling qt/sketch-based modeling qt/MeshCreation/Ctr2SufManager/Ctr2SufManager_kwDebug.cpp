#include "Ctr2SufManager.h"

void Ctr2SufManager::SetOldCenter(Point_3 OldCenterIn)
{
	this->OldCenter=OldCenterIn;
}

void Ctr2SufManager::SaveMAInfo(int majptnum,float* majpt,int maseamnum,int* maseam)
{
	//bounding points of MA planes
	vector<Point_3> vecNewMAPoint;
	for (int i=0;i<majptnum;i++)
	{
		vecNewMAPoint.push_back(Point_3(majpt[3*i]*DIM/PROCSIZE+this->OldCenter.x(),
			majpt[3*i+1]*DIM/PROCSIZE+this->OldCenter.y(),
			majpt[3*i+2]*DIM/PROCSIZE+this->OldCenter.z()));
	}
	this->vecvecMAPoint.push_back(vecNewMAPoint);
	//seams of MA planes
	vector<Int_Int_Pair> vecNewMASeam;
	for (int i=0;i<maseamnum;i++)
	{
		vecNewMASeam.push_back(make_pair(maseam[2*i],maseam[2*i+1]));
	}
	this->vecvecMASeam.push_back(vecNewMASeam);
	assert(this->vecvecMAPoint.size()==this->vecvecMASeam.size());
}

void Ctr2SufManager::ClearMAInfo()
{
	this->vecvecMAPoint.clear();
	this->vecvecMASeam.clear();
}

void Ctr2SufManager::GetSquareFace(vector<Point_3>& InOutPoints)
{
	//max distance from the first point
	double dMaxDist=0;
	//index of point which has the max distance from the first point
	int iMaxDistInd=0;
	for (unsigned int i=1;i<InOutPoints.size();i++)
	{
		double dDist=CGAL::squared_distance(InOutPoints.front(),InOutPoints.at(i));
		if (dDist>dMaxDist)
		{
			dDist=dMaxDist;
			iMaxDistInd=i;
		}
	}
	//the furthest point is not the third point
	//switch that point with the third point
	if (iMaxDistInd!=2)
	{
		swap(InOutPoints.at(2),InOutPoints.at(iMaxDistInd));
	}
}

void Ctr2SufManager::CheckCCW()
{
	int iTotal=0;
	for( int i = 0; i < this->mesh->suffacenum; i ++)
	{
		if (this->mesh->sufmat[i*2]==1)
		{
			int temp=this->mesh->sufface[3*i+0];
			this->mesh->sufface[3*i+0]=this->mesh->sufface[3*i+1];
			this->mesh->sufface[3*i+1]=temp;
			iTotal++;
		}
	}

	cout<<iTotal<<" triangles permuted when checking ccw"<<endl;

	return;
}

//void Ctr2SufManager::SaveCtr2FaceInfo()
//{
//	this->ctrfverposvec.clear();
//	this->ctrfvertypevec.clear();
//	this->ctrfedgevec.clear();
//	this->ctrfedgetypevec.clear();
//	for (int i=0;i<this->ssfacenum;i++)
//	{
//		floatvector CurrentCtrVer;
//		intvector CurrentCtrVerType;
//		for (int j=0;j<this->ctrfvernum[i];j++)
//		{
//			CurrentCtrVer.push_back(this->ctrfverpos[i][3*j]*DIM/PROCSIZE);
//			CurrentCtrVer.push_back(this->ctrfverpos[i][3*j+1]*DIM/PROCSIZE);
//			CurrentCtrVer.push_back(this->ctrfverpos[i][3*j+2]*DIM/PROCSIZE);
//			CurrentCtrVerType.push_back(this->ctrfvertype[i][j]);
//		}
//		this->ctrfverposvec.push_back(CurrentCtrVer);
//		this->ctrfvertypevec.push_back(CurrentCtrVerType);
//
//		intvector CurrentCtrEdge;
//		intvector CurrentCtrEdgeType;
//		for (int j=0;j<this->ctrfedgenum[i];j++)
//		{
//			CurrentCtrEdge.push_back(this->ctrfedge[i][4*j]);
//			CurrentCtrEdge.push_back(this->ctrfedge[i][4*j+1]);
//			CurrentCtrEdgeType.push_back(this->ctrfedgetype[i][j]);
//		}
//		this->ctrfedgevec.push_back(CurrentCtrEdge);
//		this->ctrfedgetypevec.push_back(CurrentCtrEdgeType);
//	}
//}

void Ctr2SufManager::Render()
{
	if (this->bRenderSS)
	{
		RenderSubspace();
		RenderMAPlane();
	}
	//RenderCtrToFace();
}

void Ctr2SufManager::RenderSubspace()
{
	if (this->ssvernum==0 || this->ssedgenum==0)
	{
		return;
	}

	glLineWidth(2);
	//draw the frame of the plane
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);	
	//glColor3f(0,0,1);
	glColor3f(1,0,0);
	//for each subspace
	for (int i=0;i<this->ssspacenum;i++)
	{
		//for each face in the subspace
		for (int j=0;j<this->ssspacefacenum[i];j++)
		{
			int iFaceID=this->ssspace[i][j];
			ResortedFace FaceInfo=this->vecResortFace.at(iFaceID);
			if (FaceInfo.bBoundaryFace)
			{
				continue;
			}
			glBegin(GL_QUADS);
			for (unsigned int k=0;k<FaceInfo.vecFaceVertex.size();k++)
			{
				glVertex3d(FaceInfo.vecFaceVertex.at(k).x()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).y()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).z()*DIM/PROCSIZE);
			}
			glEnd();
		}
	}
	//draw the transparent face of the plane(2 faces)
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_LIGHTING);
	glLineWidth(1);


	glDepthMask(FALSE);
	//	glDisable(GL_DEPTH_TEST);

	const GLfloat red_color[] = {1.0f, 0.0f, 0.0f, 0.2f};
	const GLfloat mat_red_emission[] = {1.0f, 0.0f, 0.0f, 1.0f};
	const GLfloat green_color[] = {0.0f, 1.0f, 0.0f, 0.2f};
	const GLfloat mat_green_emission[] = {0.0f, 1.0f, 0.0f, 1.0f};
	const GLfloat blue_color[] = {0.0f, 0.0f, 0.6f, 0.2f};
	const GLfloat mat_blue_emission[] = {0.0f, 0.0f, 1.0f, 1.0f};

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  mat_red_emission);

	//for each subspace
	for (int i=0;i<this->ssspacenum;i++)
	{
		//for each face in the subspace
		for (int j=0;j<this->ssspacefacenum[i];j++)
		{
			int iFaceID=this->ssspace[i][j];
			ResortedFace FaceInfo=this->vecResortFace.at(iFaceID);
			if (FaceInfo.bBoundaryFace)
			{
				continue;
			}
			glBegin(GL_QUADS);
			glNormal3f(FaceInfo.vecHeightVect.front().x(),FaceInfo.vecHeightVect.front().y(),FaceInfo.vecHeightVect.front().z());
			for (unsigned int k=0;k<FaceInfo.vecFaceVertex.size();k++)
			{
				glVertex3d(FaceInfo.vecFaceVertex.at(k).x()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).y()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).z()*DIM/PROCSIZE);
			}
			glNormal3f(FaceInfo.vecHeightVect.back().x(),FaceInfo.vecHeightVect.back().y(),FaceInfo.vecHeightVect.back().z());
			for (unsigned int k=0;k<FaceInfo.vecFaceVertex.size();k++)
			{
				glVertex3d(FaceInfo.vecFaceVertex.at(k).x()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).y()*DIM/PROCSIZE,
					FaceInfo.vecFaceVertex.at(k).z()*DIM/PROCSIZE);
			}
			glEnd();
		}
	}
	//	glEnable(GL_DEPTH_TEST);
	glDepthMask(TRUE);

	//glColor3f(1,0,0);
	//glDisable(GL_LIGHTING);
	//for (int i=0;i<ssedgenum;i++)
	//{
	//	glLineWidth(2);
	//	glBegin(GL_LINES);
	//	int iStartPointInd=ssedge[2*i];
	//	int iEndPointInd=ssedge[2*i+1];
	//	glVertex3f(ssver[3*iStartPointInd]*DIM/PROCSIZE+this->OldCenter.x(),ssver[3*iStartPointInd+1]*DIM/PROCSIZE+this->OldCenter.y(),
	//		ssver[3*iStartPointInd+2]*DIM/PROCSIZE+this->OldCenter.z());
	//	glVertex3f(ssver[3*iEndPointInd]*DIM/PROCSIZE+this->OldCenter.x(),ssver[3*iEndPointInd+1]*DIM/PROCSIZE+this->OldCenter.y(),
	//		ssver[3*iEndPointInd+2]*DIM/PROCSIZE+this->OldCenter.z());
	//	glEnd();
	//	glLineWidth(1);
	//}
	//glEnable(GL_LIGHTING);
}

void Ctr2SufManager::RenderMAPlane()
{
	if (this->vecvecMASeam.empty())
	{
		return;
	}
	glColor3f(0,0,1);
	glDisable(GL_LIGHTING);
	for (unsigned int i=0;i<this->vecvecMASeam.size();i++)
	{
		for (unsigned int j=0;j<this->vecvecMASeam.at(i).size();j++)
		{
			glLineWidth(2);
			glBegin(GL_LINES);
			int iStartPointInd=this->vecvecMASeam.at(i).at(j).first;
			int iEndPointInd=this->vecvecMASeam.at(i).at(j).second;
			glVertex3f(this->vecvecMAPoint.at(i).at(iStartPointInd).x(),
				this->vecvecMAPoint.at(i).at(iStartPointInd).y(),
				this->vecvecMAPoint.at(i).at(iStartPointInd).z());
			glVertex3f(this->vecvecMAPoint.at(i).at(iEndPointInd).x(),
				this->vecvecMAPoint.at(i).at(iEndPointInd).y(),
				this->vecvecMAPoint.at(i).at(iEndPointInd).z());
			glEnd();
			glLineWidth(1);
		}
	}
	glEnable(GL_LIGHTING);
}

//void Ctr2SufManager::RenderCtrToFace()
//{
//	if (this->ctrfverposvec.empty())
//	{
//		return;
//	}
//	glDisable(GL_LIGHTING);
//	glPointSize(10);
//	for (unsigned int i=0;i<this->ctrfverposvec.size();i++)
//	{
//		for (unsigned int j=0;j<this->ctrfverposvec.at(i).size()/3;j++)
//		{
//			if (this->ctrfvertypevec.at(i).at(j)==2)
//			{
//				glColor3f(0,0,1);
//			}
//			else if (this->ctrfvertypevec.at(i).at(j)==3)
//			{
//				glColor3f(1,0,0);
//			}
//			else
//			{
//				glColor3f(0,1,0);
//			}
//			glBegin(GL_POINTS);
//			glVertex3f(this->ctrfverposvec.at(i).at(3*j),this->ctrfverposvec.at(i).at(3*j+1),this->ctrfverposvec.at(i).at(3*j+2));
//			glEnd();
//		}
//	}
//	glPointSize(1);
//
//	glLineWidth(5);
//	for (unsigned int i=0;i<this->ctrfedgevec.size();i++)
//	{
//		for (unsigned int j=0;j<this->ctrfedgevec.at(i).size()/2;j++)
//		{
//			if (this->ctrfedgetypevec.at(i).at(j)==2)
//			{
//				glColor3f(0,0,1);
//			}
//			else if (this->ctrfedgetypevec.at(i).at(j)==3)
//			{
//				glColor3f(0,1,0);
//			}
//			else
//			{
//				glColor3f(1,0,0);
//			}
//			glBegin(GL_LINES);
//			int iStartPointInd=this->ctrfedgevec.at(i).at(2*j);
//			int iEndPointInd=this->ctrfedgevec.at(i).at(2*j+1);
//			glVertex3f(this->ctrfverposvec.at(i).at(3*iStartPointInd),
//				this->ctrfverposvec.at(i).at(3*iStartPointInd+1),
//				this->ctrfverposvec.at(i).at(3*iStartPointInd+2));
//			glVertex3f(this->ctrfverposvec.at(i).at(3*iEndPointInd),
//				this->ctrfverposvec.at(i).at(3*iEndPointInd+1),
//				this->ctrfverposvec.at(i).at(3*iEndPointInd+2));
//			glEnd();
//		}
//	}
//	glLineWidth(1);
//	glEnable(GL_LIGHTING);
//}