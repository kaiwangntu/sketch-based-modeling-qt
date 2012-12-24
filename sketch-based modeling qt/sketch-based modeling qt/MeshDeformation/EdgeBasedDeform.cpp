#include "EdgeBasedDeform.h"

CEdgeBasedDeform::CEdgeBasedDeform(void)
{
}

CEdgeBasedDeform::~CEdgeBasedDeform(void)
{
}

void CEdgeBasedDeform::GetEdgeInfo(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices,
								   vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle>& ROIEdges)
{

	//get edges of handle vertices
	for (unsigned int i=0;i<vecHandleNb.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=vecHandleNb.at(i)->vertex_begin();
		do 
		{
			vector<Halfedge_handle>::iterator pFind=find(ROIEdges.begin(),ROIEdges.end(),Havc);
			vector<Halfedge_handle>::iterator pFindOpp=find(ROIEdges.begin(),ROIEdges.end(),Havc->opposite());
			if ((pFind==ROIEdges.end())&&(pFindOpp==ROIEdges.end()))
			{
				ROIEdges.push_back(Havc);
			}
			Havc++;
		} while(Havc!=vecHandleNb.at(i)->vertex_begin());
	}

	//get edges of roi vertices
	for (unsigned int i=0;i<ROIVertices.size();i++)
	{
		Halfedge_around_vertex_circulator Havc=ROIVertices.at(i)->vertex_begin();
		do 
		{
			vector<Halfedge_handle>::iterator pFind=find(ROIEdges.begin(),ROIEdges.end(),Havc);
			vector<Halfedge_handle>::iterator pFindOpp=find(ROIEdges.begin(),ROIEdges.end(),Havc->opposite());
			if ((pFind==ROIEdges.end())&&(pFindOpp==ROIEdges.end()))
			{
				ROIEdges.push_back(Havc);
			}
			Havc++;
		} while(Havc!=ROIVertices.at(i)->vertex_begin());
	}

	////the following part is useless
	////the anchor vertex edges,whose another end vertex is handle or roi,are also roi vertex
	//for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	//{
	//	Halfedge_around_vertex_circulator Havc=vecAnchorVertices.at(i)->vertex_begin();
	//	do 
	//	{
	//		Vertex_handle VerOpp=Havc->opposite()->vertex();
	//		vector<Vertex_handle>::iterator pFindHandle=find(vecHandleNb.begin(),vecHandleNb.end(),VerOpp);
	//		vector<Vertex_handle>::iterator pFindROI=find(ROIVertices.begin(),ROIVertices.end(),VerOpp);
	//		if ((pFindHandle!=vecHandleNb.end())||(pFindROI!=ROIVertices.end()))//another end vertex is handle or roi
	//		{
	//			vector<Halfedge_handle>::iterator pFind=find(ROIEdges.begin(),ROIEdges.end(),Havc);
	//			vector<Halfedge_handle>::iterator pFindOpp=find(ROIEdges.begin(),ROIEdges.end(),Havc->opposite());
	//			if ((pFind==ROIEdges.end())&&(pFindOpp==ROIEdges.end()))
	//			{
	//				ROIEdges.push_back(Havc);
	//			}
	//		}
	//		Havc++;
	//	} while(Havc!=vecAnchorVertices.at(i)->vertex_begin());
	//}
}

void CEdgeBasedDeform::GetEdgeTopo(vector<Halfedge_handle> ROIEdges, vector<vector<Halfedge_handle>>& NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<Halfedge_handle> CurrentNbEdges;
		Vertex_handle EndVertex0=ROIEdges.at(i)->vertex();
		Vertex_handle EndVertex1=ROIEdges.at(i)->opposite()->vertex();

		vector<Halfedge_handle> Nb0;
		Halfedge_around_vertex_circulator Havc=EndVertex0->vertex_begin();
		do 
		{
			Nb0.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex0->vertex_begin());
		reverse(Nb0.begin(),Nb0.end());
		vector<Halfedge_handle>::iterator pFind=find(Nb0.begin(),Nb0.end(),ROIEdges.at(i));
		int iIndex=pFind-Nb0.begin();
		rotate(Nb0.begin(),Nb0.begin()+iIndex,Nb0.end());
		Nb0.erase(Nb0.begin());

		vector<Halfedge_handle> Nb1;
		Havc=EndVertex1->vertex_begin();
		do 
		{
			Nb1.push_back(Havc);
			Havc++;
		} while(Havc!=EndVertex1->vertex_begin());
		reverse(Nb1.begin(),Nb1.end());
		pFind=find(Nb1.begin(),Nb1.end(),ROIEdges.at(i)->opposite());
		iIndex=pFind-Nb1.begin();
		rotate(Nb1.begin(),Nb1.begin()+iIndex,Nb1.end());
		Nb1.erase(Nb1.begin());

		//there may be more vertices who connect both EndVertex0 and EndVertex1 !
		assert(Nb0.front()->opposite()->vertex()==Nb1.back()->opposite()->vertex());
		assert(Nb0.back()->opposite()->vertex()==Nb1.front()->opposite()->vertex());
		assert(Nb0.size()+Nb1.size()==EndVertex0->vertex_degree()+EndVertex1->vertex_degree()-2);

		CurrentNbEdges=Nb0;
		CurrentNbEdges.insert(CurrentNbEdges.end(),Nb1.begin(),Nb1.end());
		NeighborEdges.push_back(CurrentNbEdges);
	}
}

void CEdgeBasedDeform::ComputeEdgeLaplacian(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Point_3> NbPoints;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			NbPoints.push_back(NbPoint);
		}
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		if (iType==1)
		{
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				dLaplacian[0]=dLaplacian[0]+CurrentPoint.x()-NbPoints.at(j).x();
				dLaplacian[1]=dLaplacian[1]+CurrentPoint.y()-NbPoints.at(j).y();
				dLaplacian[2]=dLaplacian[2]+CurrentPoint.z()-NbPoints.at(j).z();
			}
			ROIEdges.at(i)->SetUniformLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
		}
		else if (iType==3)
		{
			vector<double> EdgeWeights;
			double dPosSumWeight,dNegSumWeight;
			dPosSumWeight=dNegSumWeight=0;
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				Point_3 Vertex1=NbPoints.at(j);
				double dCurrentWeight=0;
				Point_3 Vertex1Prev,Vertex1Next;
				if (j==0)
				{
					Vertex1Prev=NbPoints.back();
					Vertex1Next=NbPoints.at(j+1);
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				else if (j==NbPoints.size()-1)
				{
					Vertex1Prev=NbPoints.at(j-1);
					Vertex1Next=NbPoints.front();
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				else
				{
					Vertex1Prev=NbPoints.at(j-1);
					Vertex1Next=NbPoints.at(j+1);
					dCurrentWeight=GeometryAlgorithm::GetWeightForWeightedLaplacian(CurrentPoint,Vertex1,Vertex1Prev,Vertex1Next,iType);
				}
				dLaplacian[0]=dLaplacian[0]+dCurrentWeight*Vertex1.x();
				dLaplacian[1]=dLaplacian[1]+dCurrentWeight*Vertex1.y();
				dLaplacian[2]=dLaplacian[2]+dCurrentWeight*Vertex1.z();
				EdgeWeights.push_back(dCurrentWeight);
				if (j<ROIEdges.at(i)->vertex()->vertex_degree()-1)
				{
					dPosSumWeight=dPosSumWeight+dCurrentWeight;
				}
				else
				{
					dNegSumWeight=dNegSumWeight+dCurrentWeight;
				}
			}
			dLaplacian[0]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.x()-dLaplacian[0];
			dLaplacian[1]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.y()-dLaplacian[1];
			dLaplacian[2]=(dPosSumWeight+dNegSumWeight)*CurrentPoint.z()-dLaplacian[2];

			ROIEdges.at(i)->SetWeightedLaplacian(Vector_3(dLaplacian[0],dLaplacian[1],dLaplacian[2]));
			ROIEdges.at(i)->SetEdgeWeights(EdgeWeights);
			ROIEdges.at(i)->SetPosSumWeight(dPosSumWeight);
			ROIEdges.at(i)->SetNegSumWeight(dNegSumWeight);
		}
	}

}

void CEdgeBasedDeform::GetOneMoreLayerAnchor(vector<Vertex_handle> vecHandleNb,vector<Vertex_handle> ROIVertices, 
											 vector<Vertex_handle> vecAnchorVertices,vector<Vertex_handle>& OneMoreLayerAnchor)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Vertex_handle CurrentAnchor=vecAnchorVertices.at(i);
		Halfedge_around_vertex_circulator Havc=CurrentAnchor->vertex_begin();
		do 
		{
			Vertex_handle AnchorNeighbor=Havc->opposite()->vertex();
			vector<Vertex_handle>::iterator pFind=find(OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end(),AnchorNeighbor);
			if (pFind==OneMoreLayerAnchor.end())
			{
				vector<Vertex_handle>::iterator pFind2=find(vecAllVertex.begin(),vecAllVertex.end(),AnchorNeighbor);
				if (pFind2==vecAllVertex.end())
				{
					OneMoreLayerAnchor.push_back(AnchorNeighbor);
				}
			}
			Havc++;
		} while(Havc!=CurrentAnchor->vertex_begin());
	}
}

void CEdgeBasedDeform::GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
										  vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
										  vector<vector<Halfedge_handle>> NeighborEdges,vector<vector<double> >& vecvecLaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<double> CurrentRow;
		Vertex_handle EndVer0,EndVer1;//two end points of the edge
		EndVer0=ROIEdges.at(i)->vertex();
		EndVer1=ROIEdges.at(i)->opposite()->vertex();
		
		for (unsigned int j=0;j<vecAllVertex.size();j++)
		{
			if (iType==1)//uniform weight
			{
				if (vecAllVertex.at(j)==EndVer0)
				{
					CurrentRow.push_back(0.5*(EndVer1->vertex_degree()-1));
				}
				else if (vecAllVertex.at(j)==EndVer1)
				{
					CurrentRow.push_back(0.5*(EndVer0->vertex_degree()-1));
				}
				else if (GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j))
					&&(GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j))))
				{
					CurrentRow.push_back(-1.0);
				}
				else if (GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j))
					||(GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j))))
				{
					CurrentRow.push_back(-0.5);
				}
				else
				{
					CurrentRow.push_back(0);
				}
			}
			else if (iType==3)//weighted
			{
				if (vecAllVertex.at(j)==EndVer0)
				{
					CurrentRow.push_back(0.5*ROIEdges.at(i)->GetNegSumWeight());
				}
				else if (vecAllVertex.at(j)==EndVer1)
				{
					CurrentRow.push_back(0.5*ROIEdges.at(i)->GetPosSumWeight());
				}
				else
				{
					int iNbIndex0=GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j));
					int iNbIndex1=GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j));
					if (iNbIndex0!=0||iNbIndex1!=0)
					{
						CurrentRow.push_back(0);
						for (unsigned int k=0;k<NeighborEdges.at(i).size();k++)
						{
							if (NeighborEdges.at(i).at(k)->opposite()->vertex()==vecAllVertex.at(j))
							{
								CurrentRow.back()=CurrentRow.back()-0.5*(ROIEdges.at(i)->GetEdgeWeights().at(k));
							}
						}
					}
					else
					{
						CurrentRow.push_back(0);
					}
				}
			}
		}
		vecvecLaplacianMatrix.push_back(CurrentRow);
	}
}

void CEdgeBasedDeform::GetLaplacianMatrix(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
										  vector<Vertex_handle> vecAnchorVertices,vector<Halfedge_handle> ROIEdges,
										  vector<vector<Halfedge_handle>> NeighborEdges,SparseMatrix& LaplacianMatrix)
{
	vector<Vertex_handle> vecAllVertex=vecHandleNb;
	vecAllVertex.insert(vecAllVertex.end(),ROIVertices.begin(),ROIVertices.end());
	vecAllVertex.insert(vecAllVertex.end(),vecAnchorVertices.begin(),vecAnchorVertices.end());
	
	LaplacianMatrix.m=vecAllVertex.size();

	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Vertex_handle EndVer0,EndVer1;//two end points of the edge
		EndVer0=ROIEdges.at(i)->vertex();
		EndVer1=ROIEdges.at(i)->opposite()->vertex();

		for (unsigned int j=0;j<vecAllVertex.size();j++)
		{
			if (iType==1)//uniform weight
			{
				if (vecAllVertex.at(j)==EndVer0)
				{
					LaplacianMatrix[i][j]=0.5*(EndVer1->vertex_degree()-1);
				}
				else if (vecAllVertex.at(j)==EndVer1)
				{
					LaplacianMatrix[i][j]=0.5*(EndVer0->vertex_degree()-1);
				}
				else if (GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j))
					&&(GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j))))
				{
					LaplacianMatrix[i][j]=-1.0;
				}
				else if (GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j))
					||(GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j))))
				{
					LaplacianMatrix[i][j]=-0.5;
				}
				else
				{
					//0, so do nothing
				}
			}
			else if (iType==3)//weighted
			{
				if (vecAllVertex.at(j)==EndVer0)
				{
					LaplacianMatrix[i][j]=0.5*ROIEdges.at(i)->GetNegSumWeight();
				}
				else if (vecAllVertex.at(j)==EndVer1)
				{
					LaplacianMatrix[i][j]=0.5*ROIEdges.at(i)->GetPosSumWeight();
				}
				else
				{
					int iNbIndex0=GeometryAlgorithm::JudgeIfNeighbors(EndVer0,vecAllVertex.at(j));
					int iNbIndex1=GeometryAlgorithm::JudgeIfNeighbors(EndVer1,vecAllVertex.at(j));
					if (iNbIndex0!=0||iNbIndex1!=0)
					{
						//may contain bugs!!
						float fTemp=0.0;
						for (unsigned int k=0;k<NeighborEdges.at(i).size();k++)
						{
							if (NeighborEdges.at(i).at(k)->opposite()->vertex()==vecAllVertex.at(j))
							{
								fTemp=fTemp-0.5*(ROIEdges.at(i)->GetEdgeWeights().at(k));
							}
						}
						if (fTemp!=0.0)
						{
							LaplacianMatrix[i][j]=fTemp;
						}
					}
					else
					{
						//0, so do nothing
					}
				}
			}
		}
	}
}

//get the constraint matrix of anchor and handle vertices
void CEdgeBasedDeform::GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint, 
																 vector<Vertex_handle> vecHandleNb,
																 vector<Vertex_handle> ROIVertices, 
																 vector<Vertex_handle> vecAnchorVertices,
																 vector<vector<double> >& AnchorConstraintMatrix, 
																 vector<vector<double> >& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	int iAnchorRow=(int)vecAnchorVertices.size();
	for (int i=0;i<iAnchorRow;i++)
	{
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			if (j-i==iColumn-iAnchorRow)
			{
				CurrentRow.push_back(1);
			} 
			else
			{
				CurrentRow.push_back(0);
			}
		}
		AnchorConstraintMatrix.push_back(CurrentRow);
	}

	int iHandleRow=(int)vecHandlePoint.size();
	for (int i=0;i<iHandleRow;i++)
	{
		HandlePointStruct CurrentHandlePoint=vecHandlePoint.at(i);
		vector<double> CurrentRow;
		for (int j=0;j<iColumn;j++)
		{
			CurrentRow.push_back(0);
		}
		for (unsigned int j=0;j<CurrentHandlePoint.vecVertexIndex.size();j++)
		{
			CurrentRow.at(CurrentHandlePoint.vecVertexIndex.at(j))=CONSTRAINED_HANDLE_WEIGHT*CurrentHandlePoint.vecPara.at(j);
		}
		HandleConstraintMatrix.push_back(CurrentRow);
	}
}

void CEdgeBasedDeform::GetConstraintsMatrixToNaiveLaplacian(vector<HandlePointStruct> vecHandlePoint, 
															vector<Vertex_handle> vecHandleNb,
															vector<Vertex_handle> ROIVertices, 
															vector<Vertex_handle> vecAnchorVertices,
															SparseMatrix& AnchorConstraintMatrix, 
															SparseMatrix& HandleConstraintMatrix)
{
	int iColumn=(int)(vecHandleNb.size()+ROIVertices.size()+vecAnchorVertices.size());
	AnchorConstraintMatrix.m=HandleConstraintMatrix.m=iColumn;
	int iAnchorRow=(int)vecAnchorVertices.size();
	for (int i=0;i<iAnchorRow;i++)
	{
		for (int j=0;j<iColumn;j++)
		{
			if (j-i==iColumn-iAnchorRow)
			{
				AnchorConstraintMatrix[i][j]=1;
			} 
			else
			{
				//0, do nothing
			}
		}
	}

	int iHandleRow=(int)vecHandlePoint.size();
	for (int i=0;i<iHandleRow;i++)
	{
		HandlePointStruct CurrentHandlePoint=vecHandlePoint.at(i);
		for (unsigned int j=0;j<CurrentHandlePoint.vecVertexIndex.size();j++)
		{
			HandleConstraintMatrix[i][CurrentHandlePoint.vecVertexIndex.at(j)]=CONSTRAINED_HANDLE_WEIGHT*CurrentHandlePoint.vecPara.at(j);
		}
	}
}

void CEdgeBasedDeform::ComputeNaiveLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
														  vector<Vertex_handle> vecAnchorVertices, vector<Point_3> vecDeformCurvePoint3d,
														  vector<Halfedge_handle> ROIEdges, vector<vector<double> >& LaplacianRightHandSide, 
														  vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//edge laplacian
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		if (iType==1)
		{
			Laplacian[0].push_back(ROIEdges.at(i)->GetUniformLaplacian().x());
			Laplacian[1].push_back(ROIEdges.at(i)->GetUniformLaplacian().y());
			Laplacian[2].push_back(ROIEdges.at(i)->GetUniformLaplacian().z());
		}
		else if (iType==3)
		{
			Laplacian[0].push_back(ROIEdges.at(i)->GetWeightedLaplacian().x());
			Laplacian[1].push_back(ROIEdges.at(i)->GetWeightedLaplacian().y());
			Laplacian[2].push_back(ROIEdges.at(i)->GetWeightedLaplacian().z());
		}
	}

	//anchor and handle constraint
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		Anchor[0].push_back(vecAnchorVertices.at(i)->point().x());
		Anchor[1].push_back(vecAnchorVertices.at(i)->point().y());
		Anchor[2].push_back(vecAnchorVertices.at(i)->point().z());
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIEdges.size()+vecAnchorVertices.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CEdgeBasedDeform::IterativeEdgeBasedDeform(int iType,int iIterNum,KW_Mesh& Mesh, vector<HandlePointStruct>& vecHandlePoint,
												vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
												vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints)
{
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges;
	GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges;
	GetEdgeTopo(ROIEdges,NeighborEdges);
	
	//compute edge laplacian
	ComputeEdgeLaplacian(iType,ROIEdges,NeighborEdges);

	//test
	//testPoints.clear();
	//for (unsigned int i=0;i<HandleEdges.size();i++)
	//{
	//	Point_3 MidPoint=CGAL::midpoint(HandleEdges.at(i)->vertex()->point(),HandleEdges.at(i)->opposite()->vertex()->point());
	//	testPoints.push_back(MidPoint);
	//}
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	Point_3 MidPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
	//	testPoints.push_back(MidPoint);
	//}
	cout<<"related edge num: "<<ROIEdges.size()<<endl;

	//get one more layer anchor vertices
	vector<Vertex_handle> OneMoreLayerAnchor;
	GetOneMoreLayerAnchor(vecHandleNb,ROIVertices,vecAnchorVertices,OneMoreLayerAnchor);
	vecAnchorVertices.insert(vecAnchorVertices.end(),OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end());
	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;


	//get Laplacian matrix
	vector<vector<double> > vecvecLaplacianMatrix;
	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,vecvecLaplacianMatrix);

	vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	clock_t TotalBegin=clock();   

	CMath TAUCSSolver;
	vector<vector<double>> AT;
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		clock_t IterationBegin=clock();   

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			IterativeUpdateLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,AnchorPosConstraints,vecDeformCurvePoint3d,
				ROIEdges,NeighborEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		clock_t RHS=clock();   
		cout<<"time for computing RHS: "<<float(RHS-IterationBegin)<<endl;

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
//		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);
		
		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

		if (bResult)
		{
			//vecDeformCurvePoint3d.clear();
			//calculated result of handle 
			for (unsigned int i=0;i<vecHandleNb.size();i++)
			{
				vecHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<ROIVertices.size();i++)
			{
				ROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+i),
					Result.at(1).at(vecHandleNb.size()+i),
					Result.at(2).at(vecHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecAnchorVertices.size();i++)
			{
				vecAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+i));
			}
		}

		clock_t LSE=clock();   
//		DBWindowWrite("time for computing LSE: %f\n",float(LSE-RHS));

	}

	TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();   
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;

}

void CEdgeBasedDeform::IterativeUpdateLaplacianRightHandSide(int iType,vector<Vertex_handle> vecHandleNb, 
															 vector<Vertex_handle> ROIVertices,vector<Point_3> AnchorConstraints,
															 vector<Point_3> vecDeformCurvePoint3d,vector<Halfedge_handle> ROIEdges,
															 vector<vector<Halfedge_handle>> NeighborEdges, vector<vector<double> >& LaplacianRightHandSide,
															 vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//edge laplacian
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Point_3> NbPoints;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			NbPoints.push_back(NbPoint);
		}
		double dLaplacian[3];
		dLaplacian[0]=dLaplacian[1]=dLaplacian[2]=0;
		//compute new laplacian
		if (iType==1)
		{
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				dLaplacian[0]=dLaplacian[0]+CurrentPoint.x()-NbPoints.at(j).x();
				dLaplacian[1]=dLaplacian[1]+CurrentPoint.y()-NbPoints.at(j).y();
				dLaplacian[2]=dLaplacian[2]+CurrentPoint.z()-NbPoints.at(j).z();
			}
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);
			Vector_3 NewNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);
			double dOldMagnitude=std::sqrt(ROIEdges.at(i)->GetUniformLaplacian()*ROIEdges.at(i)->GetUniformLaplacian());
			dLaplacian[0]=NewNormal.x()*dOldMagnitude;
			dLaplacian[1]=NewNormal.y()*dOldMagnitude;
			dLaplacian[2]=NewNormal.z()*dOldMagnitude;
		}
		else if (iType==3)
		{
			for (unsigned int j=0;j<NbPoints.size();j++)
			{
				dLaplacian[0]=dLaplacian[0]+ROIEdges.at(i)->GetEdgeWeights().at(j)*(CurrentPoint.x()-NbPoints.at(j).x());
				dLaplacian[1]=dLaplacian[1]+ROIEdges.at(i)->GetEdgeWeights().at(j)*(CurrentPoint.y()-NbPoints.at(j).y());
				dLaplacian[2]=dLaplacian[2]+ROIEdges.at(i)->GetEdgeWeights().at(j)*(CurrentPoint.z()-NbPoints.at(j).z());
			}
			double dTemp=std::sqrt(dLaplacian[0]*dLaplacian[0]+dLaplacian[1]*dLaplacian[1]+dLaplacian[2]*dLaplacian[2]);
			Vector_3 NewNormal(dLaplacian[0]/dTemp,dLaplacian[1]/dTemp,dLaplacian[2]/dTemp);
			double dOldMagnitude=std::sqrt(ROIEdges.at(i)->GetWeightedLaplacian()*ROIEdges.at(i)->GetWeightedLaplacian());
			dLaplacian[0]=NewNormal.x()*dOldMagnitude;
			dLaplacian[1]=NewNormal.y()*dOldMagnitude;
			dLaplacian[2]=NewNormal.z()*dOldMagnitude;
		}
		Laplacian[0].push_back(dLaplacian[0]);
		Laplacian[1].push_back(dLaplacian[1]);
		Laplacian[2].push_back(dLaplacian[2]);
	}

	//anchor and handle constraint
	for (unsigned int i=0;i<AnchorConstraints.size();i++)
	{
		Anchor[0].push_back(AnchorConstraints.at(i).x());
		Anchor[1].push_back(AnchorConstraints.at(i).y());
		Anchor[2].push_back(AnchorConstraints.at(i).z());
	}
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIEdges.size()+AnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		LaplacianRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}

void CEdgeBasedDeform::EdgeBasedDeform(double dLamda,int iType,int iIterNum,KW_Mesh& Mesh, vector<HandlePointStruct>& vecHandlePoint,
									   vector<Vertex_handle>& vecHandleNb, vector<Vertex_handle>& ROIVertices,
									   vector<Vertex_handle>& vecAnchorVertices, vector<Point_3>& vecDeformCurvePoint3d,vector<Point_3>& testPoints)
{
	//collect deform related edge info first
	vector<Halfedge_handle> ROIEdges;
	GetEdgeInfo(vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges);

	//get edge topo
	vector<vector<Halfedge_handle>> NeighborEdges;
	GetEdgeTopo(ROIEdges,NeighborEdges);

	//compute edge laplacian
	ComputeEdgeLaplacian(iType,ROIEdges,NeighborEdges);

	//test
	//testPoints.clear();
	//for (unsigned int i=0;i<ROIEdges.size();i++)
	//{
	//	Point_3 MidPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
	//	testPoints.push_back(MidPoint);
	//}
	cout<<"related edge num: "<<ROIEdges.size()<<endl;
	//get one more layer anchor vertices
	vector<Vertex_handle> OneMoreLayerAnchor;
	GetOneMoreLayerAnchor(vecHandleNb,ROIVertices,vecAnchorVertices,OneMoreLayerAnchor);
	vecAnchorVertices.insert(vecAnchorVertices.end(),OneMoreLayerAnchor.begin(),OneMoreLayerAnchor.end());
	cout<<"HandleNb Vertex num: "<<vecHandleNb.size()<<endl;
	cout<<"ROI Vertex num: "<<ROIVertices.size()<<endl;
	cout<<"Anchor Vertex num: "<<vecAnchorVertices.size()<<endl;

	//get Laplacian matrix
//	vector<vector<double> > vecvecLaplacianMatrix;
//	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,vecvecLaplacianMatrix);
	SparseMatrix LaplacianMatrix(ROIEdges.size());
	GetLaplacianMatrix(iType,vecHandleNb,ROIVertices,vecAnchorVertices,ROIEdges,NeighborEdges,LaplacianMatrix);

	//vector<vector<double> > AnchorConstraintMatrix,HandleConstraintMatrix;
	//GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
	//	AnchorConstraintMatrix,HandleConstraintMatrix);
	//vector<vector<double> > LeftHandMatrixA=vecvecLaplacianMatrix;
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	//LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());
	SparseMatrix AnchorConstraintMatrix(vecAnchorVertices.size()),HandleConstraintMatrix(vecHandlePoint.size());
	GetConstraintsMatrixToNaiveLaplacian(vecHandlePoint,vecHandleNb,ROIVertices,vecAnchorVertices,
		AnchorConstraintMatrix,HandleConstraintMatrix);
	SparseMatrix LeftHandMatrixA=LaplacianMatrix;
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),AnchorConstraintMatrix.begin(),AnchorConstraintMatrix.end());
	LeftHandMatrixA.insert(LeftHandMatrixA.end(),HandleConstraintMatrix.begin(),HandleConstraintMatrix.end());

	//the anchor constraints must keep fixed during iterations,so store them first
	vector<Point_3> AnchorPosConstraints;
	for (unsigned int i=0;i<vecAnchorVertices.size();i++)
	{
		AnchorPosConstraints.push_back(vecAnchorVertices.at(i)->point());
	}

	clock_t TotalBegin=clock();   

	CMath TAUCSSolver;
	SparseMatrix AT(LeftHandMatrixA.NCols());
	TAUCSSolver.TAUCSFactorize(LeftHandMatrixA,AT);

	for (int iCurrent=0;iCurrent<=iIterNum;iCurrent++)
	{
		clock_t IterationBegin=clock();   

		vector<vector<double> > LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide;
		if (iCurrent==0)
		{
			BackUpEdgeVectorsForRigidDeform(ROIEdges,NeighborEdges);
			ComputeNaiveLaplacianRightHandSide(iType,vecHandleNb,ROIVertices,vecAnchorVertices,
				vecDeformCurvePoint3d,ROIEdges,LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}
		else
		{
			ComputeFlexibleRightHandSide(dLamda,iType,vecHandleNb,ROIVertices,AnchorPosConstraints,
					vecDeformCurvePoint3d,ROIEdges,NeighborEdges,
					LaplacianRightHandSide,AnchorRightHandSide,HandleRightHandSide);
		}

		clock_t RHS=clock();   
		cout<<"time for computing RHS: "<<float(RHS-IterationBegin)<<endl;

		vector<vector<double> > RightHandSide=LaplacianRightHandSide;
		for (int i=0;i<3;i++)
		{
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),AnchorRightHandSide.at(i).begin(),
				AnchorRightHandSide.at(i).end());
			RightHandSide.at(i).insert(RightHandSide.at(i).end(),HandleRightHandSide.at(i).begin(),
				HandleRightHandSide.at(i).end());
		}
		vector<vector<double> > Result;
//		bool bResult=CMath::ComputeLSE(LeftHandMatrixA,RightHandSide,Result);

		bool bResult=TAUCSSolver.TAUCSComputeLSE(AT,RightHandSide,Result);

		if (bResult)
		{
			//vecDeformCurvePoint3d.clear();
			//calculated result of handle 
			for (unsigned int i=0;i<vecHandleNb.size();i++)
			{
				vecHandleNb.at(i)->point()=Point_3(Result.at(0).at(i),Result.at(1).at(i),Result.at(2).at(i));
			}
			//calculated result of ROI 
			for (unsigned int i=0;i<ROIVertices.size();i++)
			{
				ROIVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+i),
					Result.at(1).at(vecHandleNb.size()+i),
					Result.at(2).at(vecHandleNb.size()+i));
			}
			//calculated result of anchor 
			for (unsigned int i=0;i<vecAnchorVertices.size();i++)
			{
				vecAnchorVertices.at(i)->point()=Point_3(Result.at(0).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(1).at(vecHandleNb.size()+ROIVertices.size()+i),
					Result.at(2).at(vecHandleNb.size()+ROIVertices.size()+i));
			}
		}

		clock_t LSE=clock();   
		//DBWindowWrite("time for computing LSE: %f\n",float(LSE-RHS));

		//compute Rotation for Handle+ROI+Anchor
		if (iCurrent!=iIterNum)
		{
			ComputeRotationForRigidDeform(iType,ROIEdges,NeighborEdges);
			ComputeScaleFactor(iType,ROIEdges,NeighborEdges);
		}
	}

	TAUCSSolver.TAUCSClear();

	clock_t TotalEnd=clock();   
	cout<<"total time: "<<float(TotalEnd-TotalBegin)<<endl;
}

void CEdgeBasedDeform::BackUpEdgeVectorsForRigidDeform(vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),ROIEdges.at(i)->opposite()->vertex()->point());
		vector<Vector_3> EdgeVectors;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 CurrentEdge=CurrentPoint-NbPoint;
			EdgeVectors.push_back(CurrentEdge);
		}
		ROIEdges.at(i)->SetOldEdgeVectors(EdgeVectors);
	}
}

void CEdgeBasedDeform::ComputeRotationForRigidDeform(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<vector<double> > TotalMatrix,IdentityMatrix;
		vector<double> tempRow;
		for (int j=0;j<3;j++)
		{
			tempRow.push_back(0);
		}
		for (int j=0;j<3;j++)
		{
			TotalMatrix.push_back(tempRow);
		}
		IdentityMatrix=TotalMatrix;

		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),
			ROIEdges.at(i)->opposite()->vertex()->point());

		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 NewEdge=CurrentPoint-NbPoint;
			//get corresponding old edge
			Vector_3 OldEdge=ROIEdges.at(i)->GetOldEdgeVectors().at(j);
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}

			//construct convariance matrix S
			vector<vector<double> > CurrentMatrix=IdentityMatrix;
			double dCurrentMatrix[9];
			dCurrentMatrix[0]=dCurrentWeight*OldEdge.x()*NewEdge.x();
			dCurrentMatrix[1]=dCurrentWeight*OldEdge.x()*NewEdge.y();
			dCurrentMatrix[2]=dCurrentWeight*OldEdge.x()*NewEdge.z();
			dCurrentMatrix[3]=dCurrentWeight*OldEdge.y()*NewEdge.x();
			dCurrentMatrix[4]=dCurrentWeight*OldEdge.y()*NewEdge.y();
			dCurrentMatrix[5]=dCurrentWeight*OldEdge.y()*NewEdge.z();
			dCurrentMatrix[6]=dCurrentWeight*OldEdge.z()*NewEdge.x();
			dCurrentMatrix[7]=dCurrentWeight*OldEdge.z()*NewEdge.y();
			dCurrentMatrix[8]=dCurrentWeight*OldEdge.z()*NewEdge.z();
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					CurrentMatrix.at(j).at(k)=dCurrentMatrix[3*j+k];
				}
			}
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					TotalMatrix.at(j).at(k)=TotalMatrix.at(j).at(k)+CurrentMatrix.at(j).at(k);
				}
			}
		}

		vector<vector<double> > OutPutU,OutPutSigma,OutPutV;
		CMath::ComputeSVD(TotalMatrix,OutPutU,OutPutSigma,OutPutV);

		//change the sign of the column of U which corresponds to the smallest singular value
		int iMinSVIndex=0;
		for (int j=0;j<3;j++)
		{
			if (OutPutSigma.at(j).at(j)!=0)
			{
				iMinSVIndex=j;
			}
			else
			{
				break;
			}
		}

		vector<double> RotationMatrix;
		bool bSign=false;
		while (true)//ensure that det(RotationMatrix)==1
		{
			if (bSign)
			{
				for (int j=0;j<3;j++)
				{
					OutPutU.at(j).at(iMinSVIndex)=-OutPutU.at(j).at(iMinSVIndex);
				}
			}
			//convert U to UT
			vector<vector<double> > UT=OutPutU;
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					UT.at(j).at(k)=OutPutU.at(k).at(j);
				}
			}
			//multiply V and UT
			for (int j=0;j<3;j++)
			{
				for (int k=0;k<3;k++)
				{
					double dCurrentValue=0;
					for (int m=0;m<3;m++)
					{
						dCurrentValue=dCurrentValue+OutPutV.at(j).at(m)*UT.at(m).at(k);
					}
					RotationMatrix.push_back(dCurrentValue);
				}
			}
			double dDeterminant=RotationMatrix.at(0)*RotationMatrix.at(4)*RotationMatrix.at(8)
				+RotationMatrix.at(1)*RotationMatrix.at(5)*RotationMatrix.at(6)
				+RotationMatrix.at(2)*RotationMatrix.at(7)*RotationMatrix.at(3)
				-RotationMatrix.at(2)*RotationMatrix.at(4)*RotationMatrix.at(6)
				-RotationMatrix.at(1)*RotationMatrix.at(3)*RotationMatrix.at(8)
				-RotationMatrix.at(0)*RotationMatrix.at(7)*RotationMatrix.at(5);
			//DBWindowWrite("%f\t",dDeterminant);
			if (dDeterminant>0)
			{
				//DBWindowWrite("\n");
				break;
			}
			else
			{
				RotationMatrix.clear();
				bSign=true;
			}
		}

		//store rotation matrix
		ROIEdges.at(i)->SetRigidDeformRotationMatrix(RotationMatrix);
	}
}

void CEdgeBasedDeform::ComputeScaleFactor(int iType,vector<Halfedge_handle>& ROIEdges,vector<vector<Halfedge_handle>> NeighborEdges)
{
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<double> RotationMatrix=ROIEdges.at(i)->GetRigidDeformRotationMatrix();
		int iOldEdgeIndex=0;
		double dNumerator[3],dDenominator[3];
		for (int j=0;j<3;j++)
		{
			dNumerator[j]=dDenominator[j]=0;
		}

		Point_3 CurrentPoint=CGAL::midpoint(ROIEdges.at(i)->vertex()->point(),
			ROIEdges.at(i)->opposite()->vertex()->point());

		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			//compute new edge
			Point_3 NbPoint=CGAL::midpoint(NeighborEdges.at(i).at(j)->vertex()->point(),
				NeighborEdges.at(i).at(j)->opposite()->vertex()->point());
			Vector_3 NewEdge=CurrentPoint-NbPoint;
			//get corresponding old edge
			Vector_3 OldEdge=ROIEdges.at(i)->GetOldEdgeVectors().at(j);
			//get RotatedOldEdge=RotationMatrix*OldEdge
			double dRotatedOldEdge[3];
			dRotatedOldEdge[0]=RotationMatrix.at(0)*OldEdge.x()+
				RotationMatrix.at(1)*OldEdge.y()+
				RotationMatrix.at(2)*OldEdge.z();
			dRotatedOldEdge[1]=RotationMatrix.at(3)*OldEdge.x()+
				RotationMatrix.at(4)*OldEdge.y()+
				RotationMatrix.at(5)*OldEdge.z();
			dRotatedOldEdge[2]=RotationMatrix.at(6)*OldEdge.x()+
				RotationMatrix.at(7)*OldEdge.y()+
				RotationMatrix.at(8)*OldEdge.z();
			//get weight for the edge
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			}
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}
			//get numerator
			dNumerator[0]=dNumerator[0]+dCurrentWeight*dRotatedOldEdge[0]*NewEdge.x();
			dNumerator[1]=dNumerator[1]+dCurrentWeight*dRotatedOldEdge[1]*NewEdge.y();
			dNumerator[2]=dNumerator[2]+dCurrentWeight*dRotatedOldEdge[2]*NewEdge.z();
			//get denominator
			dDenominator[0]=dDenominator[0]+dCurrentWeight*dRotatedOldEdge[0]*dRotatedOldEdge[0];
			dDenominator[1]=dDenominator[1]+dCurrentWeight*dRotatedOldEdge[1]*dRotatedOldEdge[1];
			dDenominator[2]=dDenominator[2]+dCurrentWeight*dRotatedOldEdge[2]*dRotatedOldEdge[2];
		}
		double dScale[3];
		for (int j=0;j<3;j++)
		{
			dScale[j]=(double)(dNumerator[j]/dDenominator[j]);
		}

//		double dUniformScale=(dScale[0]+dScale[1]+dScale[2])/3;
//		dScale[0]=dScale[1]=dScale[2]=dUniformScale;
		//store rotation matrix
		ROIEdges.at(i)->SetScaleFactor(Vector_3(dScale[0],dScale[1],dScale[2]));
	}
}

void CEdgeBasedDeform::ComputeFlexibleRightHandSide(double dLamda,int iType,vector<Vertex_handle> vecHandleNb, vector<Vertex_handle> ROIVertices,
													vector<Point_3> AnchorConstraints, vector<Point_3> vecDeformCurvePoint3d,
													vector<Halfedge_handle> ROIEdges, vector<vector<Halfedge_handle>> NeighborEdges, 
													vector<vector<double> >& RigidRightHandSide,vector<vector<double> >& AnchorRightHandSide,vector<vector<double> >& HandleRightHandSide)
{
	vector<double> Laplacian[3],Anchor[3],Handle[3];
	//edge laplacian
	for (unsigned int i=0;i<ROIEdges.size();i++)
	{
		vector<double> CurrentRotationMatrix=ROIEdges.at(i)->GetRigidDeformRotationMatrix();
		//compute rotated laplacian
		double CurrentRotatedLaplacian[3];
		for (int j=0;j<3;j++)
		{
			if (iType==1)
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIEdges.at(i)->GetUniformLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIEdges.at(i)->GetUniformLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIEdges.at(i)->GetUniformLaplacian().z();
			}
			else
			{
				CurrentRotatedLaplacian[j]=
					CurrentRotationMatrix.at(3*j+0)*ROIEdges.at(i)->GetWeightedLaplacian().x()
					+CurrentRotationMatrix.at(3*j+1)*ROIEdges.at(i)->GetWeightedLaplacian().y()
					+CurrentRotationMatrix.at(3*j+2)*ROIEdges.at(i)->GetWeightedLaplacian().z();
			}
		}
		Vector_3 ScaleFactor=ROIEdges.at(i)->GetScaleFactor();
		CurrentRotatedLaplacian[0]=CurrentRotatedLaplacian[0]*ScaleFactor.x();
		CurrentRotatedLaplacian[1]=CurrentRotatedLaplacian[1]*ScaleFactor.y();
		CurrentRotatedLaplacian[2]=CurrentRotatedLaplacian[2]*ScaleFactor.z();

		//compute rigid
		double CurrentRigid[3];
		CurrentRigid[0]=CurrentRigid[1]=CurrentRigid[2]=0;
		for (unsigned int j=0;j<NeighborEdges.at(i).size();j++)
		{
			vector<double> NbRotationMatrix=NeighborEdges.at(i).at(j)->GetRigidDeformRotationMatrix();
			//assume edges other than handle and roi are fixed
			if (NbRotationMatrix.empty())
			{
				NbRotationMatrix.push_back(1);NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(0);
				NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(1);NbRotationMatrix.push_back(0);
				NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(0);NbRotationMatrix.push_back(1);
			}
			double dCurrentWeight;
			if (iType==1)
			{
				dCurrentWeight=1;
			} 
			else
			{
				dCurrentWeight=ROIEdges.at(i)->GetEdgeWeights().at(j);
			}
			double dProduct[3];
			for (int k=0;k<3;k++)
			{
				dProduct[k]=
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+0)+NbRotationMatrix.at(3*k+0))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).x())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+1)+NbRotationMatrix.at(3*k+1))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).y())
					+
					dCurrentWeight*0.5*(CurrentRotationMatrix.at(3*k+2)+NbRotationMatrix.at(3*k+2))*
					(ROIEdges.at(i)->GetOldEdgeVectors().at(j).z());
				CurrentRigid[k]=CurrentRigid[k]+dProduct[k];
			}
		}
		for (int j=0;j<3;j++)
		{
			double dValue=dLamda*CurrentRigid[j]+(1-dLamda)*CurrentRotatedLaplacian[j];
			Laplacian[j].push_back(dValue);
		}
	}

	for (unsigned int i=0;i<AnchorConstraints.size();i++)
	{
		Anchor[0].push_back(AnchorConstraints.at(i).x());
		Anchor[1].push_back(AnchorConstraints.at(i).y());
		Anchor[2].push_back(AnchorConstraints.at(i).z());
	}
	
	for (unsigned int i=0;i<vecDeformCurvePoint3d.size();i++)
	{
		Handle[0].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).x());
		Handle[1].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).y());
		Handle[2].push_back(CONSTRAINED_HANDLE_WEIGHT*vecDeformCurvePoint3d.at(i).z());
	}

	assert(Laplacian[0].size()+Anchor[0].size()+Handle[0].size()==
		ROIEdges.size()+AnchorConstraints.size()+vecDeformCurvePoint3d.size());

	for (int i=0;i<3;i++)
	{
		RigidRightHandSide.push_back(Laplacian[i]);
		AnchorRightHandSide.push_back(Anchor[i]);
		HandleRightHandSide.push_back(Handle[i]);
	}
}